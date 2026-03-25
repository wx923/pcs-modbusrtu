/*!
    \file    bsp_uart.c
    \brief   UART BSP层实现：DMA+空闲中断实现Modbus RTU通信
*/

#include "bsp_uart.h"
#include <string.h>

static uint8_t rx_buf1[BSP_UART_RX_BUF_SIZE];
static uint8_t tx_buf1[BSP_UART_TX_BUF_SIZE];
static uint8_t rx_buf2[BSP_UART_RX_BUF_SIZE];
static uint8_t tx_buf2[BSP_UART_TX_BUF_SIZE];

/* 发送完成信号量（内部创建） */
static StaticSemaphore_t s_uart1_tx_sem_buf;
static StaticSemaphore_t s_uart2_tx_sem_buf;

/* 帧信号量（由 main.c 创建，此处引用） */
extern SemaphoreHandle_t g_rs485_1_sem;
extern SemaphoreHandle_t g_rs485_2_sem;

UartDevice uart1_dev = {
    .usart_periph  = USART1,
    .baudrate      = 9600,
    .dma_rx_ch     = DMA_CH4,
    .rx_buf        = rx_buf1,
    .rx_buf_size   = BSP_UART_RX_BUF_SIZE,
    .rx_len        = 0,
    .dma_tx_ch     = DMA_CH3,
    .tx_buf        = tx_buf1,
    .tx_buf_size   = BSP_UART_TX_BUF_SIZE,
    .tx_done_sem   = NULL,
    .de_gpio_port  = GPIOA,
    .de_gpio_pin   = GPIO_PIN_8,
    .curr_dir      = UART_DIR_RECV,
};

UartDevice uart2_dev = {
    .usart_periph  = USART2,
    .baudrate      = 9600,
    .dma_rx_ch     = DMA_CH5,
    .rx_buf        = rx_buf2,
    .rx_buf_size   = BSP_UART_RX_BUF_SIZE,
    .rx_len        = 0,
    .dma_tx_ch     = DMA_CH6,
    .tx_buf        = tx_buf2,
    .tx_buf_size   = BSP_UART_TX_BUF_SIZE,
    .tx_done_sem   = NULL,
    .de_gpio_port  = GPIOA,
    .de_gpio_pin   = GPIO_PIN_1,
    .curr_dir      = UART_DIR_RECV,
};

void bsp_uart1_init(void)
{
    dma_parameter_struct dma_init_struct;

    uart1_dev.tx_done_sem = xSemaphoreCreateBinaryStatic(&s_uart1_tx_sem_buf);
    if (uart1_dev.tx_done_sem == NULL) {
        while (1) {}
    }
    xSemaphoreGive(uart1_dev.tx_done_sem);

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_USART1);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_bit_reset(GPIOA, GPIO_PIN_8);

    /* DMA RX 初始化 */
    dma_deinit(DMA0, uart1_dev.dma_rx_ch);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.periph_addr  = (uint32_t)&USART_DATA(USART1);
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.memory_addr  = (uint32_t)uart1_dev.rx_buf;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number       = uart1_dev.rx_buf_size;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init(DMA0, uart1_dev.dma_rx_ch, &dma_init_struct);
    dma_circulation_enable(DMA0, uart1_dev.dma_rx_ch);
    dma_channel_enable(DMA0, uart1_dev.dma_rx_ch);

    /* DMA TX 初始化 */
    dma_deinit(DMA0, uart1_dev.dma_tx_ch);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.periph_addr  = (uint32_t)&USART_DATA(USART1);
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.memory_addr  = (uint32_t)uart1_dev.tx_buf;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number       = 0;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init(DMA0, uart1_dev.dma_tx_ch, &dma_init_struct);
    dma_flag_clear(DMA0, uart1_dev.dma_tx_ch, DMA_FLAG_FTF);
    dma_interrupt_enable(DMA0, uart1_dev.dma_tx_ch, DMA_INT_FTF);

    usart_baudrate_set(USART1, uart1_dev.baudrate);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_interrupt_enable(USART1, USART_INT_IDLE);
    usart_dma_receive_config(USART1, USART_RECEIVE_DMA_ENABLE);
    usart_dma_transmit_config(USART1, USART_TRANSMIT_DMA_ENABLE);
    usart_enable(USART1);

    nvic_irq_enable(USART1_IRQn, 5, 0);
    nvic_irq_enable(DMA0_Channel3_IRQn, 5, 0);
    nvic_irq_enable(DMA0_Channel4_IRQn, 5, 0);
}

void bsp_uart2_init(void)
{
    dma_parameter_struct dma_init_struct;

    uart2_dev.tx_done_sem = xSemaphoreCreateBinaryStatic(&s_uart2_tx_sem_buf);
    if (uart2_dev.tx_done_sem == NULL) {
        while (1) {}
    }
    xSemaphoreGive(uart2_dev.tx_done_sem);

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_USART2);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_bit_reset(GPIOA, GPIO_PIN_1);

    /* DMA RX 初始化 */
    dma_deinit(DMA0, uart2_dev.dma_rx_ch);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.periph_addr  = (uint32_t)&USART_DATA(USART2);
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.memory_addr  = (uint32_t)uart2_dev.rx_buf;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number       = uart2_dev.rx_buf_size;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init(DMA0, uart2_dev.dma_rx_ch, &dma_init_struct);
    dma_circulation_enable(DMA0, uart2_dev.dma_rx_ch);
    dma_channel_enable(DMA0, uart2_dev.dma_rx_ch);

    /* DMA TX 初始化 */
    dma_deinit(DMA0, uart2_dev.dma_tx_ch);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.periph_addr  = (uint32_t)&USART_DATA(USART2);
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.memory_addr  = (uint32_t)uart2_dev.tx_buf;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number       = 0;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init(DMA0, uart2_dev.dma_tx_ch, &dma_init_struct);
    dma_flag_clear(DMA0, uart2_dev.dma_tx_ch, DMA_FLAG_FTF);
    dma_interrupt_enable(DMA0, uart2_dev.dma_tx_ch, DMA_INT_FTF);

    usart_baudrate_set(USART2, uart2_dev.baudrate);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_interrupt_enable(USART2, USART_INT_IDLE);
    usart_dma_receive_config(USART2, USART_RECEIVE_DMA_ENABLE);
    usart_dma_transmit_config(USART2, USART_TRANSMIT_DMA_ENABLE);
    usart_enable(USART2);

    nvic_irq_enable(USART2_IRQn, 5, 0);
    nvic_irq_enable(DMA0_Channel5_IRQn, 5, 0);
    nvic_irq_enable(DMA0_Channel6_IRQn, 5, 0);
}

int bsp_uart_send(UartDevice *dev, const uint8_t *data, uint16_t len)
{
    if (xSemaphoreTake(dev->tx_done_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
        return -1;
    }

    memcpy(dev->tx_buf, data, len);

    gpio_bit_set(dev->de_gpio_port, dev->de_gpio_pin);
    dev->curr_dir = UART_DIR_SEND;

    dma_channel_disable(DMA0, dev->dma_tx_ch);
    dma_memory_address_config(DMA0, dev->dma_tx_ch, (uint32_t)dev->tx_buf);
    dma_transfer_number_config(DMA0, dev->dma_tx_ch, len);
    dma_channel_enable(DMA0, dev->dma_tx_ch);

    return 0;
}

void USART1_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART1, USART_INT_FLAG_IDLE) != RESET) {
        /* 清除 IDLE 标志：先读 STAT0，再读 RDATA */
        (void)USART_STAT0(USART1);
        (void)USART_DATA(USART1);

        uart1_dev.rx_len = uart1_dev.rx_buf_size - dma_transfer_number_get(DMA0, uart1_dev.dma_rx_ch);
        dma_channel_disable(DMA0, uart1_dev.dma_rx_ch);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_rs485_1_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void USART2_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_IDLE) != RESET) {
        /* 清除 IDLE 标志：先读 STAT0，再读 RDATA */
        (void)USART_STAT0(USART2);
        (void)USART_DATA(USART2);

        uart2_dev.rx_len = uart2_dev.rx_buf_size - dma_transfer_number_get(DMA0, uart2_dev.dma_rx_ch);
        dma_channel_disable(DMA0, uart2_dev.dma_rx_ch);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_rs485_2_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void DMA0_Channel3_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF) != RESET) {
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);

        while (usart_flag_get(USART1, USART_FLAG_TC) == RESET);

        gpio_bit_reset(uart1_dev.de_gpio_port, uart1_dev.de_gpio_pin);
        uart1_dev.curr_dir = UART_DIR_RECV;

        dma_channel_disable(DMA0, uart1_dev.dma_rx_ch);
        dma_memory_address_config(DMA0, uart1_dev.dma_rx_ch, (uint32_t)uart1_dev.rx_buf);
        dma_transfer_number_config(DMA0, uart1_dev.dma_rx_ch, uart1_dev.rx_buf_size);
        dma_channel_enable(DMA0, uart1_dev.dma_rx_ch);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(uart1_dev.tx_done_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void DMA0_Channel6_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA0, DMA_CH6, DMA_INT_FLAG_FTF) != RESET) {
        dma_interrupt_flag_clear(DMA0, DMA_CH6, DMA_INT_FLAG_FTF);

        while (usart_flag_get(USART2, USART_FLAG_TC) == RESET);

        gpio_bit_reset(uart2_dev.de_gpio_port, uart2_dev.de_gpio_pin);
        uart2_dev.curr_dir = UART_DIR_RECV;

        dma_channel_disable(DMA0, uart2_dev.dma_rx_ch);
        dma_memory_address_config(DMA0, uart2_dev.dma_rx_ch, (uint32_t)uart2_dev.rx_buf);
        dma_transfer_number_config(DMA0, uart2_dev.dma_rx_ch, uart2_dev.rx_buf_size);
        dma_channel_enable(DMA0, uart2_dev.dma_rx_ch);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(uart2_dev.tx_done_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
