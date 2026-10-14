/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_ssd1306_interface.c
 * @brief     SilverGuard 工程专用的 ssd1306 接口适配层（硬件 I2C1）
 * @version   2.0.0
 * @author    Shifeng Li (LibDriver 上游模板) / SilverGuard 本地适配
 * @date      2021-03-30
 *
 * 与上游模板的区别：
 *   上游用 GPIO 翻转模拟 I2C（软件 I2C，引脚写死在 PB10/PB11）。
 *   本工程改用 STM32F103 的硬件 I2C1：PB6 = I2C1_SCL，PB7 = I2C1_SDA，100 kHz。
 *   这样就与 USART3(SYN6288，占用 PB10/PB11) 彻底解耦。
 *
 * 引脚与总线由 CubeMX 生成的 MX_I2C1_Init() 配置，本文件不碰任何 GPIO 寄存器，
 * 遵循“一个引脚只有一个配置所有者”的原则。
 */

#include "driver_ssd1306_interface.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief I2C1 句柄由 CubeMX 在 main.c 中定义，这里只引用，不重复初始化
 */
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief 单次硬件 I2C 事务的超时时间，单位 ms
 * @note  屏幕未接或线松时，每次写会阻塞到这里超时返回，所以不要设得过大
 */
#define SSD1306_IIC_TIMEOUT_MS       10U

/**
 * @brief 单个 I2C 事务中携带的数据字节数上限
 * @note  控制字节 + 数据不超过 SSD1306_IIC_CHUNK_SIZE + 1 字节，
 *        SSD1306 允许把一次长写拆成多个事务（每个事务都以控制字节开头），
 *        因此这里分块发送：栈占用小，也不会出现单次超长传输。
 */
#define SSD1306_IIC_CHUNK_SIZE      32U

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   总线已由 CubeMX 初始化，这里只确认它确实已经就绪
 */
uint8_t ssd1306_interface_iic_init(void)
{
    return (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) ? 0 : 1;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 * @note   I2C1 是工程级共享资源，这里不做反初始化
 */
uint8_t ssd1306_interface_iic_deinit(void)
{
    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address（LibDriver 传 0x78，即 SA0 接地的 8 位写地址）
 * @param[in] reg iic register address（SSD1306 控制字节：0x00 = 命令，0x40 = 数据）
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t ssd1306_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t  tx[SSD1306_IIC_CHUNK_SIZE + 1U];
    uint16_t sent = 0U;

    if ((buf == NULL) && (len != 0U))
    {
        return 1;
    }

    while (sent < len)
    {
        uint16_t n = (uint16_t)(len - sent);

        if (n > SSD1306_IIC_CHUNK_SIZE)
        {
            n = SSD1306_IIC_CHUNK_SIZE;
        }

        tx[0] = reg;
        (void)memcpy(&tx[1], &buf[sent], n);

        /* HAL 要求 DevAddress 是“7 位地址左移一位”，0x78 恰好就是这个形式，可直接传。
           详细约定见 stm32f1xx_hal_i2c.c 中 HAL_I2C_Master_Transmit 的 @param 说明。 */
        if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)addr, tx,
                                    (uint16_t)(n + 1U), SSD1306_IIC_TIMEOUT_MS) != HAL_OK)
        {
            return 1;
        }

        sent = (uint16_t)(sent + n);
    }

    return 0;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 * @note   本工程使用 I2C 接口，SPI 相关回调保留为空实现
 */
uint8_t ssd1306_interface_spi_init(void)
{
    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 * @note   本工程使用 I2C 接口
 */
uint8_t ssd1306_interface_spi_deinit(void)
{
    return 0;
}

/**
 * @brief     interface spi bus write
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 * @note      本工程使用 I2C 接口
 */
uint8_t ssd1306_interface_spi_write_cmd(uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;

    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void ssd1306_interface_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      当前工程尚未重定向 printf，输出会落空；
 *            将来若要看到调试信息，需要实现 _write/fputc 重定向。
 */
void ssd1306_interface_debug_print(const char *const fmt, ...)
{
    (void)fmt;
}

/**
 * @brief  interface command && data gpio init
 * @return status code
 *         - 0 success
 * @note   本工程使用 I2C 接口
 */
uint8_t ssd1306_interface_spi_cmd_data_gpio_init(void)
{
    return 0;
}

/**
 * @brief  interface command && data gpio deinit
 * @return status code
 *         - 0 success
 * @note   本工程使用 I2C 接口
 */
uint8_t ssd1306_interface_spi_cmd_data_gpio_deinit(void)
{
    return 0;
}

/**
 * @brief     interface command && data gpio write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 * @note      本工程使用 I2C 接口
 */
uint8_t ssd1306_interface_spi_cmd_data_gpio_write(uint8_t value)
{
    (void)value;

    return 0;
}

/**
 * @brief  interface reset gpio init
 * @return status code
 *         - 0 success
 * @note   本工程使用 I2C 接口，模块无独立复位脚
 */
uint8_t ssd1306_interface_reset_gpio_init(void)
{
    return 0;
}

/**
 * @brief  interface reset gpio deinit
 * @return status code
 *         - 0 success
 * @note   本工程使用 I2C 接口
 */
uint8_t ssd1306_interface_reset_gpio_deinit(void)
{
    return 0;
}

/**
 * @brief     interface reset gpio write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 * @note      本工程使用 I2C 接口
 */
uint8_t ssd1306_interface_reset_gpio_write(uint8_t value)
{
    (void)value;

    return 0;
}
