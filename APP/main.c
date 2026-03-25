/*!
    \file    main.c
    \brief   ARM 应用层入口：系统初始化、RTOS 内核对象创建、任务启动
*/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include "gd32f30x_libopt.h"
#include "system_gd32f30x.h"
#include "dsp_mirror.h"

extern void bsp_init(void);

extern void MB_SlaveTask(void *arg);
extern void DSP_MasterTask(void *arg);

static void FaultLed_BlinkLoop(void)
{
    while (1);
}

/* =====================================================================
 * RTOS 内核对象（静态分配）
 * ===================================================================== */

/* USART1/2 帧信号量：分别由各自空闲中断释放 */
static StaticSemaphore_t s_rs485_1_sem_buf;
static StaticSemaphore_t s_rs485_2_sem_buf;
static StaticSemaphore_t s_uart2_mutex_buf;
static StaticSemaphore_t s_reg_mutex_buf;

/* 任务栈 */
static StackType_t s_mb_slave_task_stack[160];
static StackType_t s_dsp_task_stack[256];

static StaticTask_t s_mb_slave_task_tcb;
static StaticTask_t s_dsp_task_tcb;

/* =====================================================================
 * 全局句柄
 * ===================================================================== */
SemaphoreHandle_t g_rs485_1_sem;
SemaphoreHandle_t g_rs485_2_sem;
SemaphoreHandle_t g_uart2_mutex;
SemaphoreHandle_t g_reg_mutex;

/* =====================================================================
 * main
 * ===================================================================== */
int main(void)
{
    __disable_irq();

    bsp_init();

    memset(&g_dsp_mirror_reg, 0, sizeof(g_dsp_mirror_reg));

    /* 创建信号量 */
    g_rs485_1_sem  = xSemaphoreCreateBinaryStatic(&s_rs485_1_sem_buf);
    g_rs485_2_sem  = xSemaphoreCreateBinaryStatic(&s_rs485_2_sem_buf);
    g_uart2_mutex  = xSemaphoreCreateMutexStatic(&s_uart2_mutex_buf);
    g_reg_mutex    = xSemaphoreCreateMutexStatic(&s_reg_mutex_buf);

    if (!g_rs485_1_sem || !g_rs485_2_sem || !g_uart2_mutex || !g_reg_mutex) {
        FaultLed_BlinkLoop();
    }

    /* 任务优先级：MB_SlaveTask(7) > DSP_MasterTask(6) */
    xTaskCreateStatic(MB_SlaveTask,
                      "MB_Slave",
                      160,
                      NULL,
                      tskIDLE_PRIORITY + 7,
                      s_mb_slave_task_stack,
                      &s_mb_slave_task_tcb);

    xTaskCreateStatic(DSP_MasterTask,
                      "DSP_Master",
                      256,
                      NULL,
                      tskIDLE_PRIORITY + 6,
                      s_dsp_task_stack,
                      &s_dsp_task_tcb);

    __enable_irq();
    vTaskStartScheduler();

    FaultLed_BlinkLoop();
}
