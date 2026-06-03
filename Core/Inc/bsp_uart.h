#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含头文件 */
#include "main.h"       // 包含 HAL 库定义 (如 UART_HandleTypeDef)
#include "stm32f4xx_hal.h"

/* 函数声明 */

/**
 * @brief 配置串口接收 (开启DMA接收和IDLE中断)
 * @note  需要在 main.c 的 MX_USART1_UART_Init() 之后调用
 */
void UART_Config(void);

/**
 * @brief 串口IDLE中断处理函数
 * @note  需要在 stm32f4xx_it.c 的 USART1_IRQHandler 中调用
 */
void UART_IDLE_IRQHandler(void);

/**
 * @brief 使用DMA发送数据
 * @param data 要发送的数据指针
 * @param len  数据长度
 */
void UART_DMA_Send(uint8_t *data, uint16_t len);


/**
 * @brief 使用DMA发送数据
 * @param send debug information
 * @param len  数据长度
 */

//void DMA_Printf1(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */




