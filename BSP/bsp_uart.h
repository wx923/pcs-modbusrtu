/*!
    \file    bsp_uart.h
    \brief   UART设备描述符与BSP层API
*/

#ifndef BSP_UART_H
#define BSP_UART_H

#include "gd32f30x.h"
#include "gd32f30x_libopt.h"
#include "FreeRTOS.h"
#include "semphr.h"

//定义缓冲区大小
#define BSP_UART_RX_BUF_SIZE  256
#define BSP_UART_TX_BUF_SIZE  256

//定义RS485传输方向
typedef enum {
    UART_DIR_RECV = 0,
    UART_DIR_SEND
} uart_dir_t;

//定义UART设备结构体
typedef struct {
    uint32_t usart_periph; //UART外设
    uint32_t baudrate; //波特率

    dma_channel_enum dma_rx_ch; //DMA接收通道
    uint8_t *rx_buf; //接收缓冲区
    uint16_t rx_buf_size;
    uint16_t rx_len; //接收长度

    dma_channel_enum dma_tx_ch; //DMA发送通道   
    uint8_t *tx_buf; //发送缓冲区   
    uint16_t tx_buf_size; //发送缓冲区大小  

    SemaphoreHandle_t frame_sem; //帧信号量
    SemaphoreHandle_t tx_done_sem; //发送完成信号量

    uint32_t de_gpio_port; //RS485方向控制引脚端口
    uint16_t de_gpio_pin; //RS485方向控制引脚
    uart_dir_t curr_dir; //当前传输方向
} UartDevice;

extern UartDevice uart1_dev; //UART1设备
extern UartDevice uart2_dev; //UART2设备

void bsp_uart1_init(void); //UART1初始化
void bsp_uart2_init(void); //UART2初始化
int bsp_uart_send(UartDevice *dev, const uint8_t *data, uint16_t len); //UART发送

#endif /* BSP_UART_H */ //UART头文件    
