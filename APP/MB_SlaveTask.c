/*!
    \file    MB_SlaveTask.c
    \brief   Modbus RTU 从机任务（USART1，接上位机）

             处理上位机指令：
               FC 03 读寄存器 -> 直接查 g_dsp_mirror_reg 并回复
               FC 06/10 写寄存器 -> 转发给 DSP_MasterTask（通过 dsp_push_write），
                                   DSP 回复后再回复上位机
*/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

#include "bsp_uart.h"
#include "modbus_rtu.h"
#include "dsp_mirror.h"

extern SemaphoreHandle_t g_rs485_1_sem;
extern SemaphoreHandle_t g_rs485_2_sem;
extern SemaphoreHandle_t g_uart2_mutex;

#define ARM_SLAVE_ID      0x08
#define DSP_SLAVE_ID      0x01
#define TX_BUF_SIZE       256
#define DSP_REPLY_TIMEOUT_MS 300

typedef struct {
    int result;   /* 0=ok, -1=timeout */
    uint16_t ack_addr;
    uint16_t ack_count;
} dsp_reply_t;

/* DSP 同步等待用的本地信号量（由 MB_SlaveTask 自行管理） */
static SemaphoreHandle_t s_dsp_reply_sem;
static StaticSemaphore_t s_dsp_reply_sem_buf;
static StaticSemaphore_t s_dsp_reply_mutex_buf;
static StaticSemaphore_t s_dsp_reply_data_buf;
static StaticQueue_t     s_dsp_reply_queue_handle;
static uint8_t           s_dsp_reply_queue_storage[1];
static dsp_reply_t       s_dsp_reply_data;

typedef struct {
    uint8_t fc;
    uint16_t addr;
    uint16_t count;
    uint16_t vals[8];
} dsp_wr_item_t;

void dsp_push_write(uint8_t fc, uint16_t addr, uint16_t count, const uint16_t *vals);

static uint8_t  s_rx_buf[BSP_UART_RX_BUF_SIZE];
static uint8_t  s_tx_buf[TX_BUF_SIZE];
static uint8_t  s_dsp_tx_buf[BSP_UART_TX_BUF_SIZE];
static uint8_t  s_dsp_rx_buf[BSP_UART_RX_BUF_SIZE];

static void restart_uart1_dma(void)
{
    dma_channel_disable(DMA0, uart1_dev.dma_rx_ch);
    dma_memory_address_config(DMA0, uart1_dev.dma_rx_ch, (uint32_t)uart1_dev.rx_buf);
    dma_transfer_number_config(DMA0, uart1_dev.dma_rx_ch, uart1_dev.rx_buf_size);
    dma_channel_enable(DMA0, uart1_dev.dma_rx_ch);
    uart1_dev.rx_len = 0;
}

static void restart_uart2_dma(void)
{
    dma_channel_disable(DMA0, uart2_dev.dma_rx_ch);
    dma_memory_address_config(DMA0, uart2_dev.dma_rx_ch, (uint32_t)uart2_dev.rx_buf);
    dma_transfer_number_config(DMA0, uart2_dev.dma_rx_ch, uart2_dev.rx_buf_size);
    dma_channel_enable(DMA0, uart2_dev.dma_rx_ch);
    uart2_dev.rx_len = 0;
}

/* ── 等待 DSP 回复 ── */
static int wait_dsp_reply(uint16_t *ack_addr, uint16_t *ack_count)
{
    if (xSemaphoreTake(s_dsp_reply_sem, pdMS_TO_TICKS(DSP_REPLY_TIMEOUT_MS)) != pdTRUE) {
        return -1;
    }
    if (ack_addr  != NULL) *ack_addr  = s_dsp_reply_data.ack_addr;
    if (ack_count != NULL) *ack_count = s_dsp_reply_data.ack_count;
    return s_dsp_reply_data.result;
}

/* ── 发送一帧给 DSP 并等待回复 ── */
static int send_to_dsp_and_wait(const uint8_t *tx, uint8_t tx_len)
{
    xSemaphoreTake(g_uart2_mutex, portMAX_DELAY);

    for (int retry = 0; retry < 3; retry++) {
        if (bsp_uart_send(&uart2_dev, tx, tx_len) != 0) continue;
        if (wait_dsp_reply(NULL, NULL) != 0) continue;

        uint16_t len = uart2_dev.rx_len;
        if (len == 0 || len > BSP_UART_RX_BUF_SIZE) {
            restart_uart2_dma();
            continue;
        }
        memcpy(s_dsp_rx_buf, uart2_dev.rx_buf, len);
        restart_uart2_dma();

        if (mb_check_crc(s_dsp_rx_buf, len) != 0) continue;

        xSemaphoreGive(g_uart2_mutex);
        return (int)len;
    }

    xSemaphoreGive(g_uart2_mutex);
    return -1;
}

/* ── 处理 DSP 回复回调（由 DSP_MasterTask 调用） ── */
void MB_SlaveTask_on_dsp_reply(int result, uint16_t ack_addr, uint16_t ack_count)
{
    s_dsp_reply_data.result    = result;
    s_dsp_reply_data.ack_addr  = ack_addr;
    s_dsp_reply_data.ack_count = ack_count;
    xSemaphoreGive(s_dsp_reply_sem);
}

void MB_SlaveTask(void *arg)
{
    (void)arg;

    /* 创建 DSP 同步信号量 */
    s_dsp_reply_sem = xSemaphoreCreateBinaryStatic(&s_dsp_reply_sem_buf);

    if (s_dsp_reply_sem == NULL) {
        while (1) {}
    }

    while (1) {
        if (xSemaphoreTake(g_rs485_1_sem, portMAX_DELAY) != pdTRUE) continue;

        uint16_t len = uart1_dev.rx_len;
        if (len == 0 || len > BSP_UART_RX_BUF_SIZE) {
            restart_uart1_dma();
            continue;
        }

        memcpy(s_rx_buf, uart1_dev.rx_buf, len);
        restart_uart1_dma();

        if (mb_check_crc(s_rx_buf, len) != 0) continue;

        uint8_t  id   = s_rx_buf[0];
        uint8_t  fc   = s_rx_buf[1];
        uint16_t addr = (uint16_t)((s_rx_buf[2] << 8) | s_rx_buf[3]);
        uint16_t val  = (uint16_t)((s_rx_buf[4] << 8) | s_rx_buf[5]);
        int tx_len = -1;

        /* 只响应本机地址 0x08 */
        if (id != ARM_SLAVE_ID) {
            continue;
        }

        if (fc == MB_FC_READ_HOLDING) {
            uint16_t count = (uint16_t)((s_rx_buf[4] << 8) | s_rx_buf[5]);
            if (count == 0 || count > 125) {
                tx_len = mb_build_rsp_exc(s_tx_buf, sizeof(s_tx_buf), id, MB_EXC_ILLEGAL_VAL);
            } else {
                uint16_t vals[125];
                for (uint16_t i = 0; i < count; i++) {
                    dsp_mirror_read_reg((uint16_t)(addr + i), &vals[i]);
                }
                tx_len = mb_build_rsp_read(s_tx_buf, sizeof(s_tx_buf), id, vals, count);
            }
        }
        else if (fc == MB_FC_WRITE_SINGLE) {
            /* 转发给 DSP，等 DSP 回复后再回上位机 */
            int rsp_len = mb_build_req_write_single(s_dsp_tx_buf, sizeof(s_dsp_tx_buf),
                                                     DSP_SLAVE_ID, addr, val);
            if (rsp_len > 0) {
                int ok = send_to_dsp_and_wait(s_dsp_tx_buf, (uint8_t)rsp_len);
                if (ok > 0) {
                    uint16_t ack_addr = 0, ack_val = 0;
                    if (mb_parse_rsp_write_single(s_dsp_rx_buf, (uint16_t)ok, &ack_addr, &ack_val) == 0 &&
                        ack_addr == addr && ack_val == val) {
                        dsp_mirror_write_reg(addr, val);
                        tx_len = mb_build_rsp_write_single(s_tx_buf, sizeof(s_tx_buf), id, addr, val);
                    } else {
                        tx_len = mb_build_rsp_exc(s_tx_buf, sizeof(s_tx_buf), id, MB_EXC_SLAVE_FAIL);
                    }
                } else {
                    tx_len = mb_build_rsp_exc(s_tx_buf, sizeof(s_tx_buf), id, MB_EXC_SLAVE_FAIL);
                }
            } else {
                tx_len = mb_build_rsp_exc(s_tx_buf, sizeof(s_tx_buf), id, MB_EXC_SLAVE_FAIL);
            }
        }
        else if (fc == MB_FC_WRITE_MULTI) {
            uint16_t count = (uint16_t)((s_rx_buf[4] << 8) | s_rx_buf[5]);
            uint8_t  bc    = s_rx_buf[6];
            if (count == 0 || count > 125 || bc != count * 2) {
                tx_len = mb_build_rsp_exc(s_tx_buf, sizeof(s_tx_buf), id, MB_EXC_ILLEGAL_VAL);
            } else {
                uint16_t vals[125];
                for (uint16_t i = 0; i < count; i++) {
                    vals[i] = (uint16_t)((s_rx_buf[7 + i * 2] << 8) | s_rx_buf[7 + i * 2 + 1]);
                }
                int rsp_len = mb_build_req_write_multi(s_dsp_tx_buf, sizeof(s_dsp_tx_buf),
                                                       DSP_SLAVE_ID, addr, vals, count);
                if (rsp_len > 0) {
                    int ok = send_to_dsp_and_wait(s_dsp_tx_buf, (uint8_t)rsp_len);
                    if (ok > 0) {
                        uint16_t ack_addr = 0, ack_count = 0;
                        if (mb_parse_rsp_write_multi(s_dsp_rx_buf, (uint16_t)ok, &ack_addr, &ack_count) == 0 &&
                            ack_addr == addr && ack_count == count) {
                            for (uint16_t i = 0; i < count; i++) {
                                dsp_mirror_write_reg((uint16_t)(addr + i), vals[i]);
                            }
                            tx_len = mb_build_rsp_write_multi(s_tx_buf, sizeof(s_tx_buf), id, addr, count);
                        } else {
                            tx_len = mb_build_rsp_exc(s_tx_buf, sizeof(s_tx_buf), id, MB_EXC_SLAVE_FAIL);
                        }
                    } else {
                        tx_len = mb_build_rsp_exc(s_tx_buf, sizeof(s_tx_buf), id, MB_EXC_SLAVE_FAIL);
                    }
                } else {
                    tx_len = mb_build_rsp_exc(s_tx_buf, sizeof(s_tx_buf), id, MB_EXC_SLAVE_FAIL);
                }
            }
        }
        else {
            tx_len = mb_build_rsp_exc(s_tx_buf, sizeof(s_tx_buf), id, MB_EXC_ILLEGAL_FUNC);
        }

        if (tx_len > 0) {
            bsp_uart_send(&uart1_dev, s_tx_buf, (uint16_t)tx_len);
        }
    }
}
