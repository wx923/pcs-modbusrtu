/*!
    \file    DSP_MasterTask.c
    \brief   DSP 主站任务（USART2，接 DSP）

             主循环（每 20ms 执行一次）：
               轮询一个寄存器段（03 读），最多 3 次重试，收到回复后更新寄存器镜像

             注意：上位机的写指令（FC06/10）由 MB_SlaveTask 直接转发给 DSP，
             不再通过此任务的写队列。
*/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

#include "bsp_uart.h"
#include "modbus_rtu.h"
#include "dsp_mirror.h"

extern SemaphoreHandle_t g_rs485_2_sem;
extern SemaphoreHandle_t g_uart2_mutex;

#define DSP_SLAVE_ID         0x08
#define RETRY_MAX            3
#define REPLY_TIMEOUT_MS      200

typedef struct { uint16_t addr; uint16_t count; } poll_item_t;

static const poll_item_t s_poll[] = {
    {0x3000, 7}, {0x3080, 30}, {0x3180, 7}, {0x3200, 8},
    {0x3300, 6}, {0x3500, 12}, {0x3580, 9}, {0x3680, 9},
    {0x3700, 10}, {0x3900, 32}, {0x3920, 32}, {0x3940, 32},
    {0x3960, 32}, {0x3980, 32}, {0x3A00, 5}, {0x4500, 48},
    {0x4580, 8}, {0x4600, 16},
};
#define POLL_LEN (sizeof(s_poll) / sizeof(s_poll[0]))

static uint8_t  s_tx_buf[BSP_UART_TX_BUF_SIZE];
static uint8_t  s_rx_buf[BSP_UART_RX_BUF_SIZE];

/* 等待 USART2 收到一帧 */
static int wait_reply(void)
{
    return (xSemaphoreTake(g_rs485_2_sem, pdMS_TO_TICKS(REPLY_TIMEOUT_MS)) == pdTRUE) ? 0 : -1;
}

/* 重启 USART2 DMA 接收 */
static void restart_uart2_dma(void)
{
    dma_channel_disable(DMA0, uart2_dev.dma_rx_ch);
    dma_memory_address_config(DMA0, uart2_dev.dma_rx_ch, (uint32_t)uart2_dev.rx_buf);
    dma_transfer_number_config(DMA0, uart2_dev.dma_rx_ch, uart2_dev.rx_buf_size);
    dma_channel_enable(DMA0, uart2_dev.dma_rx_ch);
    uart2_dev.rx_len = 0;
}

/* 发送一帧并等待回复（最多 RETRY_MAX 次） */
static int send_and_wait(const uint8_t *tx, uint8_t tx_len)
{
    xSemaphoreTake(g_uart2_mutex, portMAX_DELAY);

    for (int retry = 0; retry < RETRY_MAX; retry++) {
        if (bsp_uart_send(&uart2_dev, tx, tx_len) != 0) continue;
        if (wait_reply() != 0) continue;

        uint16_t len = uart2_dev.rx_len;
        if (len == 0 || len > BSP_UART_RX_BUF_SIZE) {
            restart_uart2_dma();
            continue;
        }
        memcpy(s_rx_buf, uart2_dev.rx_buf, len);
        restart_uart2_dma();

        if (mb_check_crc(s_rx_buf, len) != 0) continue;

        xSemaphoreGive(g_uart2_mutex);
        return (int)len;
    }

    xSemaphoreGive(g_uart2_mutex);
    return -1;
}

/* 轮询一个寄存器段 */
static void poll_one(const poll_item_t *item)
{
    int len = mb_build_req_read(s_tx_buf, sizeof(s_tx_buf),
                                 DSP_SLAVE_ID, item->addr, item->count);
    if (len < 0) return;

    int rsp_len = send_and_wait(s_tx_buf, (uint8_t)len);
    if (rsp_len < 0) return;

    uint16_t vals[32];
    uint16_t n = 0;
    if (mb_parse_rsp_read(s_rx_buf, (uint16_t)rsp_len, &n, vals) < 0) return;

    for (uint16_t i = 0; i < n; i++) {
        dsp_mirror_write_reg((uint16_t)(item->addr + i), vals[i]);
    }
}

void DSP_MasterTask(void *arg)
{
    (void)arg;
    uint16_t poll_idx = 0;

    while (1) {
        poll_one(&s_poll[poll_idx]);
        poll_idx = (poll_idx + 1) % POLL_LEN;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
