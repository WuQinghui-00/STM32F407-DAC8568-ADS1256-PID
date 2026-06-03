#include "bsp_uart.h"
#include "protocol.h"
#include <stdarg.h>

// 硬件句柄 (CubeMX生成)
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

// 缓冲区定义
#define RX_BUF_SIZE 256 
uint8_t Uart_RxBuffer[RX_BUF_SIZE];
//volatile uint8_t Rx_Data_Ready = 0;



//// 自定义 DMA 发送函数，用法类似 printf
//// 例如: DMA_Printf1("Temp: %.2f\n", 23.5);
//#define TX_BUFFER_SIZE1 256 
//uint8_t TxBuffer1[TX_BUFFER_SIZE1];
//void DMA_Printf1(const char *format, ...)
//{
//    va_list args;
//		int len;

//    // 1. 等待上一次 DMA 发送完成 (保护 TxBuffer)
//    // 如果发送频率极高，这里会短暂阻塞；否则直接通过
//    while (huart1.gState != HAL_UART_STATE_READY);

//    // 2. 格式化字符串到 TxBuffer
//    va_start(args, format);
//    // vsnprintf 比较安全，防止缓冲区溢出
//    len = vsnprintf((char *)TxBuffer1, TX_BUFFER_SIZE1, format, args);
//    va_end(args);

//    // 3. 启动 DMA 发送
//    if (len > 0)
//    {
//        // 这里的 &huart1 需要根据实际情况引用
//        HAL_UART_Transmit_DMA(&huart1, TxBuffer1, len);
//    }
//}




/**
  * @brief 开启串口接收 (在main初始化中调用)
  */
void UART_Config(void) {
    // 开启IDLE中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // 开启DMA接收
    HAL_UART_Receive_DMA(&huart1, Uart_RxBuffer, RX_BUF_SIZE);
}

/**
  * @brief 串口中断服务函数 (放入 stm32f4xx_it.c 的 USART1_IRQHandler 中)
  */
void UART_IDLE_IRQHandler(void) {
    uint32_t tmp_flag = 0;
    uint32_t temp;
    uint16_t rx_len;

    // 1. 获取并清除 IDLE 标志位
    tmp_flag = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE);
    if ((tmp_flag != RESET)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        // 2. 【核心修改】不要调用 HAL_UART_DMAStop
        // 直接读取当前 DMA 剩余数据量
        // 使用 huart1.hdmarx 确保获取的是挂载到该串口的正确 DMA 句柄
        // HAL_DMA_GetState check is optional but good for debug

        // --- 关键步骤：先暂停 DMA Stream 以确保读数稳定 (针对 STM32F4/F7) ---
        __HAL_DMA_DISABLE(huart1.hdmarx);

        // 3. 读取剩余数量 (NDTR寄存器)
        temp = __HAL_DMA_GET_COUNTER(huart1.hdmarx);

        // 4. 计算实际接收长度
        // 务必确保这里的 RX_BUF_SIZE 与 main() 中启动接收时的长度完全一致！
        rx_len = RX_BUF_SIZE - temp;
				

        // 5. 只有收到数据才处理
        if (rx_len == RX_FRAME_LEN) {
            // 处理数据
            // 注意：此时数据就在 Uart_RxBuffer[0] 到 Uart_RxBuffer[rx_len-1]
            Protocol_ProcessPacket(Uart_RxBuffer); 
        }

        // 6. 重启 DMA 接收 (这会自动重新使能 DMA Stream)
        // 务必检查返回值，防止重启失败导致后续无法接收
        if (HAL_UART_Receive_DMA(&huart1, Uart_RxBuffer, RX_BUF_SIZE) != HAL_OK) {
            // 如果出错（比如因为Uart忙），可以在这里做错误处理，例如复位Uart
            // HAL_UART_ErrorCallback(&huart1);
        }
    }
}

/**
  * @brief DMA发送函数
  */
void UART_DMA_Send(uint8_t *data, uint16_t len) {
    // 等待上一次发送完成 (简单阻塞，实际可优化为环形缓冲)
    while(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY);
    HAL_UART_Transmit_DMA(&huart1, data, len);
}


