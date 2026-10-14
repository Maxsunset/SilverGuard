/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define On_board_light_Pin GPIO_PIN_13
#define On_board_light_GPIO_Port GPIOC
#define Button_Pin GPIO_PIN_0
#define Button_GPIO_Port GPIOA
#define Host_computer_TX_Pin GPIO_PIN_2
#define Host_computer_TX_GPIO_Port GPIOA
#define Host_computer_RX_Pin GPIO_PIN_3
#define Host_computer_RX_GPIO_Port GPIOA
#define buzzer_Pin GPIO_PIN_6
#define buzzer_GPIO_Port GPIOA
#define Light_Pin GPIO_PIN_0
#define Light_GPIO_Port GPIOB
#define SYN6288_TX_Pin GPIO_PIN_10
#define SYN6288_TX_GPIO_Port GPIOB
#define SYN6288_RX_Pin GPIO_PIN_11
#define SYN6288_RX_GPIO_Port GPIOB
#define ESP8266_TX_Pin GPIO_PIN_9
#define ESP8266_TX_GPIO_Port GPIOA
#define ESP8266_RX_Pin GPIO_PIN_10
#define ESP8266_RX_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
