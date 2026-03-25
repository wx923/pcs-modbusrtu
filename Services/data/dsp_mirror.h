//头文件定义
#ifndef __DSP_MIRROR_H
#define __DSP_MIRROR_H

#include <stdint.h>


//定义寄存器段的数据个

#define DSP_BOARD_3000_REGLEN    7
#define DSP_BOARD_3080_REGLEN    30
#define DSP_BOARD_3180_REGLEN    7
#define DSP_BOARD_3200_REGLEN    8
#define DSP_BOARD_3300_REGLEN    6
#define DSP_BOARD_3500_REGLEN    12
#define DSP_BOARD_3580_REGLEN    9     
#define DSP_BOARD_3680_REGLEN    9    
#define DSP_BOARD_3700_REGLEN    10
#define DSP_BOARD_3900_REGLEN    32    
#define DSP_BOARD_3920_REGLEN    32    
#define DSP_BOARD_3940_REGLEN    32    
#define DSP_BOARD_3960_REGLEN    32    
#define DSP_BOARD_3980_REGLEN    32     
#define DSP_BOARD_3A00_REGLEN    5     
#define DSP_BOARD_4500_REGLEN    48
#define DSP_BOARD_4580_REGLEN    8
#define DSP_BOARD_4600_REGLEN    16
#define DSP_BOARD_5000_REGLEN    255
#define DSP_BOARD_5400_REGLEN    255
#define DSP_BOARD_5800_REGLEN    255
#define DSP_BOARD_5B00_REGLEN    255
#define DSP_BOARD_6000_REGLEN    255
#define DSP_BOARD_6400_REGLEN    255

//定义寄存器数据段
typedef struct{
    uint16_t seg_3000_val[DSP_BOARD_3000_REGLEN];
    uint16_t seg_3080_val[DSP_BOARD_3080_REGLEN];
    uint16_t seg_3180_val[DSP_BOARD_3180_REGLEN];
    uint16_t seg_3200_val[DSP_BOARD_3200_REGLEN];
    uint16_t seg_3300_val[DSP_BOARD_3300_REGLEN];
    uint16_t seg_3500_val[DSP_BOARD_3500_REGLEN];
    uint16_t seg_3580_val[DSP_BOARD_3580_REGLEN];
    uint16_t seg_3680_val[DSP_BOARD_3680_REGLEN];
    uint16_t seg_3700_val[DSP_BOARD_3700_REGLEN];
    uint16_t seg_3900_val[DSP_BOARD_3900_REGLEN];
    uint16_t seg_3920_val[DSP_BOARD_3920_REGLEN];
    uint16_t seg_3940_val[DSP_BOARD_3940_REGLEN];
    uint16_t seg_3960_val[DSP_BOARD_3960_REGLEN];
    uint16_t seg_3980_val[DSP_BOARD_3980_REGLEN];
    uint16_t seg_3A00_val[DSP_BOARD_3A00_REGLEN];
    uint16_t seg_4500_val[DSP_BOARD_4500_REGLEN];
    uint16_t seg_4580_val[DSP_BOARD_4580_REGLEN];
    uint16_t seg_4600_val[DSP_BOARD_4600_REGLEN];
    uint16_t seg_5000_val[DSP_BOARD_5000_REGLEN];
    uint16_t seg_5400_val[DSP_BOARD_5400_REGLEN];
    uint16_t seg_5800_val[DSP_BOARD_5800_REGLEN];
    uint16_t seg_5B00_val[DSP_BOARD_5B00_REGLEN];
    uint16_t seg_6000_val[DSP_BOARD_6000_REGLEN];
    uint16_t seg_6400_val[DSP_BOARD_6400_REGLEN];
} dsp_mirror_reg_t;

//声明可供外部使用的
extern dsp_mirror_reg_t g_dsp_mirror_reg;

//读取一个寄存器数据
void dsp_mirror_read_reg(uint16_t reg_addr, uint16_t *reg_data);
//写入一个寄存器数据
void dsp_mirror_write_reg(uint16_t reg_addr, uint16_t reg_data);

#endif