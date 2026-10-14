/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "driver_ssd1306_interface.h"
#include "oled_hz.h"
#include "voice_text.h"
#include "siren_tab.h"
#include "net_secret.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NET_Q 8
#define NET_RXQ 512

static volatile uint8_t net_rx[NET_RXQ];
static volatile uint16_t net_rx_head;
static volatile uint16_t net_rx_tail;
static uint8_t net_rx8;

typedef enum {/*state_type*/
	offline,
	standby,
	checking,
	pre_alarm,
	alarm,
	alarm_cancel,
} state_type;

typedef enum {/*button_type*/
	idle,
	debounce,
	pressed,
	triggered,
} button_type;

typedef enum {/*led_mode_type*/
	led_on,
	led_off,
} led_mode_type;

typedef enum {/*warn_mode_type*/
	warn_off,
	warn_on,
} warn_mode_type;

typedef enum {/*buzz_mode_type*/
	buzz_off,
	buzz_siren,
} buzz_mode_type;

typedef enum {/*screen_type*/
	scr_offline,
	scr_online,
	scr_checking,
	scr_prealarm,
	scr_alarm,
	scr_cancel,
} screen_type;

typedef enum {/*voice_type*/
	voice_none,
	voice_prealarm,
	voice_alarm,
	voice_cancel,
} voice_type;

typedef enum {/*net_type*/
	net_none,
	net_offline,
	net_online,
	net_alarm,
	net_cancel,
} net_type;

typedef struct {/*state_handle*/
	state_type state;
	uint32_t tick;
	bool flag;
	bool standby_enter;
	bool checking_enter;
	bool pre_alarm_enter;
	bool alarm_enter;
	bool alarm_cancel_enter;
	bool offline_enter;
} state_handle;

typedef struct {/*signle_handle*/
	uint8_t windows[5];
	uint8_t output;
	bool flag;
} signle_handle;

typedef struct {/*button_handle*/
	button_type button;
	uint32_t tick;
} button_handle;

typedef struct {/*net_handle*/
	uint8_t state;
	uint8_t hit;
	uint8_t stage;
	uint8_t retry;
	uint8_t head;
	uint8_t tail;
	uint8_t dirty;
	uint32_t timeout;
	uint32_t backoff;
	uint32_t tick;
	const char *last;
	const char *box[NET_Q];
	uint8_t frame[64];
	uint8_t frame_len;
} net_handle;

typedef struct {/*hardware_handle*/
	warn_mode_type light_mode;
	buzz_mode_type buzz_mode;
	ssd1306_handle_t oled;
} hardware_handle;
	
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NET_TOPIC "SilverGuard_3Xu"
#define MQTT_CLIENT "sg-module-01"
#define BKP_PRE   0xA501
#define BKP_ALARM 0xA502

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t now;
state_handle st;
signle_handle sg;
button_handle bt;
net_handle nt;
hardware_handle hd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/*状态机流转*/
static void main_state_machine(void);
static uint8_t analyze_data(uint8_t content[], uint8_t len);
static void detect_button(void);
static void distribute(void);
/*实现层*/
static void on_board_light(led_mode_type mode);
static void oled(screen_type screen);
static void warning_light(warn_mode_type mode);
static void buzzer(buzz_mode_type mode);
static void SYN6288(voice_type voice);
static void ESP8266(net_type net);
/*中断回调函数*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/*辅助函数*/
/*板载灯（PC13）*/
static void led_lighted(led_mode_type led);
/*报警灯（PB0）*/
static void light_lighted(led_mode_type led);
/*OLED（I2C1）*/
static void draw_hz16(uint8_t x, uint8_t y, const uint8_t *glyph);
static void draw_hz16_line(uint8_t x, uint8_t y, const uint8_t *const *glyphs, uint8_t count);
static void oled_init(void);
/*时间片调制（蜂鸣器/报警灯）*/
static void modulate(void);
/*语音（USART3）*/
static void voice_send(const uint8_t *text, uint16_t len);
/*网络（USART1）*/
static void net_flush(void);
static void net_send(const char *cmd);
static uint8_t net_poll(const char *token);
static uint8_t net_len(uint8_t *p, uint16_t v);
static void net_frame_connect(void);
static void net_frame_pub(const char *payload);
static void net_raw_start(void);
static uint8_t net_raw_task(void);
static void net_start(uint8_t step);
static void net_put(const char *msg);
static void net_fail(void);
static void net_task(void);
/*报警记忆（BKP）*/
static void alarm_keep(state_type state);
/*状态簿记*/
static void set_state(state_type state);
/*初始化*/
static void init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  init();
  oled_init();
  HAL_UART_Receive_IT(&huart2, &sg.windows[0], 1);
  HAL_UART_Receive_IT(&huart1, &net_rx8, 1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    now = HAL_GetTick();
    detect_button();
    main_state_machine();
    distribute();
    modulate();
    net_task();
    HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 332;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(On_board_light_GPIO_Port, On_board_light_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Light_GPIO_Port, Light_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : On_board_light_Pin */
  GPIO_InitStruct.Pin = On_board_light_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(On_board_light_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Light_Pin */
  GPIO_InitStruct.Pin = Light_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Light_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*状态机流转*/

static void main_state_machine(void)
{
	if (st.flag == 1 && (st.state == standby || st.state == checking))
		set_state(offline);
	switch(st.state){
	case standby:
		if (bt.button == triggered)
			set_state(alarm);
		else if (sg.output == 1)
			set_state(checking);
		break;
	case checking:
		if (bt.button == triggered)
			set_state(alarm);
		else if (sg.output == 0)
			set_state(standby);
		else if (now - st.tick >= 5000)
			set_state(pre_alarm);
		break;
	case pre_alarm:
		if (bt.button == triggered)
			set_state(alarm_cancel);
		else if (now - st.tick >= 10000)
			set_state(alarm);
		break;
	case alarm:
		if (bt.button == triggered)
			set_state(alarm_cancel);
		break;
	case alarm_cancel:
		if (now - st.tick >= 3000)
			set_state((st.flag == 1) ? offline : standby);
		break;
	case offline:
		if (bt.button == triggered)
			set_state(alarm);
		else if (st.flag == 0){
			if (sg.output == 1)
				set_state(checking);
			else if (sg.output == 0)
				set_state(standby);
		}
	}
}

static uint8_t analyze_data(uint8_t content[], uint8_t len)
{
	uint8_t i;
	for(i = 1; i < len; i++){
		if (content[i] != content[0]){
			return sg.output;}
		}
	return content[0];
}

static void detect_button(void)
{
	uint8_t level = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
	switch (bt.button){
	case idle:
		if (level == GPIO_PIN_RESET){
			bt.tick = HAL_GetTick();
			bt.button = debounce;}
		break;
	case debounce:
		if (now - bt.tick >= 20){
			if (level == GPIO_PIN_RESET){
				bt.button = pressed;
				bt.tick = HAL_GetTick();}
			else {
				bt.button = idle;}
			}
			break;
	case pressed:
		if (now - bt.tick >= 1000){
			bt.button = triggered;}
		else if (level == GPIO_PIN_SET){
			bt.button = idle;}
		break;
	case triggered:
		bt.button = idle;
		break;
	}
}

static void distribute(void)
{
	if (st.standby_enter){
		st.standby_enter = 0;
		oled(scr_online);
		warning_light(warn_off);
		buzzer(buzz_off);
		SYN6288(voice_none);
		ESP8266(net_online);}
	if (st.checking_enter){
		st.checking_enter = 0;
		oled(scr_checking);
		warning_light(warn_off);
		buzzer(buzz_off);
		SYN6288(voice_none);
		ESP8266(net_none);}
	if (st.pre_alarm_enter){
		st.pre_alarm_enter = 0;
		oled(scr_prealarm);
		warning_light(warn_on);
		buzzer(buzz_off);
		SYN6288(voice_prealarm);
		ESP8266(net_none);}
	if (st.alarm_enter){
		st.alarm_enter = 0;
		oled(scr_alarm);
		warning_light(warn_on);
		buzzer(buzz_siren);
		SYN6288(voice_alarm);
		ESP8266(net_alarm);}
	if (st.alarm_cancel_enter){
		st.alarm_cancel_enter = 0;
		oled(scr_cancel);
		warning_light(warn_off);
		buzzer(buzz_off);
		SYN6288(voice_cancel);
		ESP8266(net_cancel);}
	if (st.offline_enter){
		st.offline_enter = 0;
		memset(sg.windows, 0xFF, sizeof(sg.windows));
		sg.output = 0xFF;
		oled(scr_offline);
		warning_light(warn_off);
		buzzer(buzz_off);
		SYN6288(voice_none);
		ESP8266(net_offline);}
	on_board_light((st.state == offline || st.flag == 1) ? led_off : led_on);
}

/*实现层*/

static void on_board_light(led_mode_type mode)
{
	static led_mode_type before = led_on;

	if (mode == before)
		return;
	before = mode;
	led_lighted(mode);
}

static void oled(screen_type screen)
{
	static uint8_t before_screen = 0xFF;
	static uint8_t before_flag = 0xFF;
	uint8_t flag = (st.flag == 1) ? 1 : 0;

	if ((uint8_t)screen == before_screen && flag == before_flag)
		return;
	before_screen = (uint8_t)screen;
	before_flag = flag;

	ssd1306_clear(&hd.oled);

	if (screen == scr_offline || st.flag == 1)
		ssd1306_gram_write_string(&hd.oled, 0, 0, "offline", 7, 1, SSD1306_FONT_16);
	else
		ssd1306_gram_write_string(&hd.oled, 0, 0, "online", 6, 1, SSD1306_FONT_16);

	switch(screen){
	case scr_checking:
		draw_hz16_line(0, 16, hz_checking_a, sizeof(hz_checking_a) / sizeof(hz_checking_a[0]));
		draw_hz16_line(0, 32, hz_checking_b, sizeof(hz_checking_b) / sizeof(hz_checking_b[0]));
		break;
	case scr_prealarm:
		draw_hz16_line(0, 16, hz_prealarm_a, sizeof(hz_prealarm_a) / sizeof(hz_prealarm_a[0]));
		draw_hz16_line(0, 32, hz_prealarm_b, sizeof(hz_prealarm_b) / sizeof(hz_prealarm_b[0]));
		draw_hz16_line(0, 48, hz_prealarm_c, sizeof(hz_prealarm_c) / sizeof(hz_prealarm_c[0]));
		break;
	case scr_alarm:
		draw_hz16_line(0, 16, hz_alarm_a, sizeof(hz_alarm_a) / sizeof(hz_alarm_a[0]));
		draw_hz16_line(0, 32, hz_alarm_b, sizeof(hz_alarm_b) / sizeof(hz_alarm_b[0]));
		break;
	case scr_cancel:
		draw_hz16_line(0, 16, hz_cancel_a, sizeof(hz_cancel_a) / sizeof(hz_cancel_a[0]));
		break;
	default:
		break;
	}

	ssd1306_gram_update(&hd.oled);
}

static void warning_light(warn_mode_type mode)
{
	hd.light_mode = mode;
}

static void buzzer(buzz_mode_type mode)
{
	hd.buzz_mode = mode;
	if (mode == buzz_off)
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}

static void SYN6288(voice_type voice)
{
	switch(voice){
	case voice_prealarm:
		voice_send(txt_pre_alarm, sizeof(txt_pre_alarm));
		break;
	case voice_alarm:
		voice_send(txt_alarm, sizeof(txt_alarm));
		break;
	case voice_cancel:
		voice_send(txt_cancel, sizeof(txt_cancel));
		break;
	default:
		break;
	}
}

static void ESP8266(net_type net)
{
	const char *msg;

	switch(net){
	case net_offline:
		msg = "offline";
		break;
	case net_online:
		msg = "online";
		break;
	case net_alarm:
		msg = "alarm";
		break;
	case net_cancel:
		msg = "cancel";
		break;
	default:
		return;
	}

	if (msg == nt.last && nt.dirty == 0)
		return;

	nt.last = msg;
	net_put(msg);
}

/*中断回调函数*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2){
		sg.flag = 1;
		sg.output = analyze_data(sg.windows, 5);
		uint8_t i;
		for(i = 4; i > 0; i--){
			sg.windows[i] = sg.windows[i-1];}
		sg.windows[0] = 0xFF;
		HAL_UART_Receive_IT(&huart2, &sg.windows[0], 1);
		}
	else if (huart->Instance == USART1){
		uint16_t next = (uint16_t)((net_rx_head + 1) % NET_RXQ);
		if (next != net_rx_tail){
			net_rx[net_rx_head] = net_rx8;
			net_rx_head = next;
		}
		HAL_UART_Receive_IT(&huart1, &net_rx8, 1);
		}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1){
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_PEFLAG(huart);
		HAL_UART_Receive_IT(&huart1, &net_rx8, 1);
	}
	else if (huart->Instance == USART2){
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_PEFLAG(huart);
		HAL_UART_Receive_IT(&huart2, &sg.windows[0], 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == TIM2){
		static uint8_t i2 = 0;
		i2++;
		if (i2 >= 200){
			i2 = 0;
			if (sg.flag == 1){
				sg.flag = 0;
				st.flag = 0;}
			else {
				st.flag = 1;}
		}
	}
}

/*辅助函数*/

/*板载灯（PC13）*/

static void led_lighted(led_mode_type led_mode)
{
	if (led_mode == led_on)
		HAL_GPIO_WritePin(On_board_light_GPIO_Port, On_board_light_Pin, GPIO_PIN_RESET);
	else 
		HAL_GPIO_WritePin(On_board_light_GPIO_Port, On_board_light_Pin, GPIO_PIN_SET);
}

/*报警灯（PB0）*/

static void light_lighted(led_mode_type led_mode)
{
	static led_mode_type before = led_off;

	if (led_mode == before)
		return;
	before = led_mode;

	if (led_mode == led_on)
		HAL_GPIO_WritePin(Light_GPIO_Port, Light_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Light_GPIO_Port, Light_Pin, GPIO_PIN_RESET);
}

/*OLED（I2C1）*/

static void draw_hz16(uint8_t x, uint8_t y, const uint8_t *glyph)
{
	uint8_t i, bit;

	if ((x + 15 > 127) || (y + 15 > 63)) return;

	for (i = 0; i < 16; i++){
		for (bit = 0; bit < 8; bit++){
			if (glyph[i] & (1 << bit))
				ssd1306_gram_write_point(&hd.oled, x + i, y + bit, 1);
			if (glyph[16 + i] & (1 << bit))
				ssd1306_gram_write_point(&hd.oled, x + i, y + 8 + bit, 1);
		}
	}
}

static void draw_hz16_line(uint8_t x, uint8_t y, const uint8_t *const *glyphs, uint8_t count)
{
	uint8_t i;

	for (i = 0; i < count; i++)
		draw_hz16(x + i * 16, y, glyphs[i]);
}

static void oled_init(void)
{
	DRIVER_SSD1306_LINK_INIT(&hd.oled, ssd1306_handle_t);
	DRIVER_SSD1306_LINK_IIC_INIT(&hd.oled, ssd1306_interface_iic_init);
	DRIVER_SSD1306_LINK_IIC_DEINIT(&hd.oled, ssd1306_interface_iic_deinit);
	DRIVER_SSD1306_LINK_IIC_WRITE(&hd.oled, ssd1306_interface_iic_write);
	DRIVER_SSD1306_LINK_DELAY_MS(&hd.oled, ssd1306_interface_delay_ms);
	DRIVER_SSD1306_LINK_DEBUG_PRINT(&hd.oled, ssd1306_interface_debug_print);
	DRIVER_SSD1306_LINK_SPI_INIT(&hd.oled, ssd1306_interface_spi_init);
	DRIVER_SSD1306_LINK_SPI_DEINIT(&hd.oled, ssd1306_interface_spi_deinit);
	DRIVER_SSD1306_LINK_SPI_WRITE_COMMAND(&hd.oled, ssd1306_interface_spi_write_cmd);
	DRIVER_SSD1306_LINK_SPI_COMMAND_DATA_GPIO_INIT(&hd.oled, ssd1306_interface_spi_cmd_data_gpio_init);
	DRIVER_SSD1306_LINK_SPI_COMMAND_DATA_GPIO_DEINIT(&hd.oled, ssd1306_interface_spi_cmd_data_gpio_deinit);
	DRIVER_SSD1306_LINK_SPI_COMMAND_DATA_GPIO_WRITE(&hd.oled, ssd1306_interface_spi_cmd_data_gpio_write);
	DRIVER_SSD1306_LINK_RESET_GPIO_INIT(&hd.oled, ssd1306_interface_reset_gpio_init);
	DRIVER_SSD1306_LINK_RESET_GPIO_DEINIT(&hd.oled, ssd1306_interface_reset_gpio_deinit);
	DRIVER_SSD1306_LINK_RESET_GPIO_WRITE(&hd.oled, ssd1306_interface_reset_gpio_write);

	ssd1306_set_interface(&hd.oled, SSD1306_INTERFACE_IIC);
	ssd1306_set_addr_pin(&hd.oled, SSD1306_ADDR_SA0_0);
	ssd1306_init(&hd.oled);

	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xAE}, 1);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xD5, 0x80}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xA8, 0x3F}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xD3, 0x00}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0x40}, 1);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0x8D, 0x14}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0x20, 0x02}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xA1}, 1);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xC8}, 1);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xDA, 0x12}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0x81, 0xCF}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xD9, 0xF1}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xDB, 0x40}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xA4, 0xA6}, 2);
	ssd1306_write_cmd(&hd.oled, (uint8_t[]){0xAF}, 1);

	ssd1306_clear(&hd.oled);
	ssd1306_gram_update(&hd.oled);
}

/*时间片调制（蜂鸣器/报警灯）*/

static void modulate(void)
{
	static uint16_t last_arr = 0;
	uint16_t arr;

	if (hd.light_mode == warn_off)
		light_lighted(led_off);
	else
		light_lighted(led_on);

	if (hd.buzz_mode == buzz_off){
		if (last_arr != 0){
			last_arr = 0;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);}
		return;
	}

	arr = siren_tab[(now % SIREN_T) * SIREN_N / SIREN_T];

	if (arr != last_arr){
		last_arr = arr;
		__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr / 2);}
}

/*语音（USART3）*/

static void voice_send(const uint8_t *text, uint16_t len)
{
	uint8_t frame[48];
	uint16_t n;
	uint16_t i;
	uint8_t ecc = 0;

	if (len > 40)
		return;

	n = len + 3;
	frame[0] = 0xFD;
	frame[1] = (uint8_t)(n >> 8);
	frame[2] = (uint8_t)n;
	frame[3] = 0x01;
	frame[4] = 0x00;
	for (i = 0; i < len; i++)
		frame[5 + i] = text[i];
	for (i = 0; i < 5 + len; i++)
		ecc ^= frame[i];
	frame[5 + len] = ecc;

	HAL_UART_Transmit(&huart3, frame, len + 6, 100);
}

/*网络（USART1）*/

static void net_flush(void)
{
	net_rx_tail = net_rx_head;
}

static void net_send(const char *cmd)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);
	HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 100);
}

static uint8_t net_poll(const char *token)
{
	uint8_t c;
	uint8_t len = (uint8_t)strlen(token);

	while (net_rx_tail != net_rx_head){
		c = net_rx[net_rx_tail];
		net_rx_tail = (uint16_t)((net_rx_tail + 1) % NET_RXQ);
		if (c == token[nt.hit]){
			nt.hit++;
			if (nt.hit >= len){
				nt.hit = 0;
				return 1;
			}
		}
		else {
			nt.hit = (c == token[0]) ? 1 : 0;
		}
	}
	return 0;
}

static uint8_t net_len(uint8_t *p, uint16_t v)
{
	uint8_t n = 0;

	do {
		uint8_t b = (uint8_t)(v % 128);

		v = (uint16_t)(v / 128);
		if (v > 0)
			b |= 0x80;
		p[n] = b;
		n++;
	} while (v > 0);
	return n;
}

static void net_frame_connect(void)
{
	uint8_t n = (uint8_t)strlen(MQTT_CLIENT);
	uint8_t i = 0;

	nt.frame[i] = 0x10;
	i++;
	i += net_len(&nt.frame[i], (uint16_t)(12 + n));
	nt.frame[i] = 0x00; i++;
	nt.frame[i] = 0x04; i++;
	memcpy(&nt.frame[i], "MQTT", 4); i += 4;
	nt.frame[i] = 0x04; i++;
	nt.frame[i] = 0x02; i++;
	nt.frame[i] = 0x00; i++;
	nt.frame[i] = 60; i++;
	nt.frame[i] = 0x00; i++;
	nt.frame[i] = n; i++;
	memcpy(&nt.frame[i], MQTT_CLIENT, n);
	i += n;
	nt.frame_len = i;
}

static void net_frame_pub(const char *payload)
{
	uint16_t topic_len = (uint16_t)strlen(NET_TOPIC);
	uint16_t payload_len = (uint16_t)strlen(payload);
	uint16_t rem = (uint16_t)(topic_len + payload_len + 2);
	uint8_t i = 0;

	nt.frame[i] = 0x30;
	i++;
	i += net_len(&nt.frame[i], rem);
	nt.frame[i] = (uint8_t)(topic_len >> 8); i++;
	nt.frame[i] = (uint8_t)(topic_len & 0xFF); i++;
	memcpy(&nt.frame[i], NET_TOPIC, topic_len);
	i += (uint8_t)topic_len;
	memcpy(&nt.frame[i], payload, payload_len);
	i += (uint8_t)payload_len;
	nt.frame_len = i;
}

static void net_raw_start(void)
{
	char cmd[16];
	uint8_t v = nt.frame_len;
	uint8_t i = 0;

	memcpy(cmd, "AT+CIPSEND=", 11);
	i = 11;
	if (v >= 10){
		cmd[i] = (char)('0' + v / 10);
		i++;
		v = (uint8_t)(v % 10);}
	cmd[i] = (char)('0' + v);
	i++;
	cmd[i] = 0;

	net_flush();
	nt.hit = 0;
	net_send(cmd);
	nt.tick = now;
	nt.stage = 1;
}

static uint8_t net_raw_task(void)
{
	if (nt.stage == 1){
		if (net_poll(">")){
			HAL_UART_Transmit(&huart1, nt.frame, nt.frame_len, 100);
			nt.hit = 0;
			nt.tick = now;
			nt.stage = 2;
			return 0;
		}
		if (now - nt.tick > 1000){
			nt.stage = 0;
			net_fail();
			return 2;}
		return 0;
	}

	if (nt.stage == 2){
		if (net_poll("SEND OK")){
			nt.stage = 0;
			nt.tick = now;
			return 1;
		}
		if (now - nt.tick > 2000){
			nt.stage = 0;
			net_fail();
			return 2;}
	}
	return 0;
}

static void net_start(uint8_t step)
{
	static const char *const cmd[4] = {
		"AT+CWMODE=1",
		"AT+CWJAP=\"" NET_SSID "\",\"" NET_PASS "\"",
		"AT+CIPMUX=0",
		"AT+CIPSTART=\"TCP\",\"broker.emqx.io\",1883",
	};
	static const uint32_t limit[4] = {1000, 20000, 1000, 8000};

	net_flush();
	nt.hit = 0;
	net_send(cmd[step - 1]);
	nt.timeout = limit[step - 1];
	nt.tick = now;
	nt.state = step;
}

static void net_put(const char *msg)
{
	uint8_t next = (nt.tail + 1) % NET_Q;

	if (next == nt.head)
		nt.head = (nt.head + 1) % NET_Q;
	nt.box[nt.tail] = msg;
	nt.tail = next;
	nt.dirty = 1;
}

static void net_fail(void)
{
	nt.retry++;
	nt.backoff = (nt.retry <= 5) ? (2000u << (nt.retry - 1)) : 60000u;
	nt.state = 6;
	nt.tick = now;
}

static void net_task(void)
{
	if (nt.state == 0){
		net_start(1);
		return;
	}

	if (nt.state <= 4){
		if (net_poll((nt.state == 4) ? "CONNECT" : "OK")){
			if (nt.state == 4){
				net_frame_connect();
				net_raw_start();
				nt.state = 7;}
			else
				net_start(nt.state + 1);
			return;
		}
		if (now - nt.tick > nt.timeout)
			net_fail();
		return;
	}

	if (nt.state == 6){
		if (now - nt.tick > nt.backoff)
			net_start((nt.retry >= 3) ? 1 : 3);
		return;
	}

	if (nt.state == 7){
		if (net_raw_task() == 1){
			nt.retry = 0;
			nt.state = 5;
			if (nt.dirty != 0 && nt.last != 0)
				net_put(nt.last);}
		return;
	}

	if (nt.stage != 0){
		if (net_raw_task() == 1)
			nt.retry = 0;
		return;
	}

	if (nt.head != nt.tail){
		net_frame_pub(nt.box[nt.head]);
		nt.head = (nt.head + 1) % NET_Q;
		net_raw_start();
		return;
	}

	if (nt.dirty != 0)
		nt.dirty = 0;

	if (now - nt.tick > 30000){
		nt.frame[0] = 0xC0;
		nt.frame[1] = 0x00;
		nt.frame_len = 2;
		net_raw_start();}
}

/*报警记忆（BKP）*/

static void alarm_keep(state_type state)
{
	if (state == pre_alarm)
		BKP->DR1 = BKP_PRE;
	else if (state == alarm)
		BKP->DR1 = BKP_ALARM;
	else if (state == alarm_cancel)
		BKP->DR1 = 0;
}

/*状态簿记*/

static void set_state(state_type state)
{
	st.state = state;
	st.tick = HAL_GetTick();
	switch(state){
	case standby:
		st.standby_enter = 1;
		break;
	case checking:
		st.checking_enter = 1;
		break;
	case pre_alarm:
		st.pre_alarm_enter = 1;
		break;
	case alarm:
		st.alarm_enter = 1;
		break;
	case alarm_cancel:
		st.alarm_cancel_enter = 1;
		break;
	case offline:
		st.offline_enter = 1;
		break;
	}
	alarm_keep(state);
}

/*初始化*/

static void init(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_BKP_CLK_ENABLE();
	PWR->CR |= PWR_CR_DBP;
	__HAL_RCC_CLEAR_RESET_FLAGS();

	if (BKP->DR1 == BKP_PRE)
		set_state(pre_alarm);
	else if (BKP->DR1 == BKP_ALARM)
		set_state(alarm);
	else
		set_state(offline);

	st.flag = 0;
	memset(sg.windows, 0xFF, sizeof(sg.windows));
	sg.output = 0xFF;
	sg.flag = 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
