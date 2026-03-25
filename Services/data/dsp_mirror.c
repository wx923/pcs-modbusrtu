//头文件
#include "dsp_mirror.h"

//定义全局变量
dsp_mirror_reg_t g_dsp_mirror_reg;

//定义寄存器段映射表结构体
typedef struct{
    uint16_t reg_addr;
    uint16_t count;
    uint16_t *reg_data;
}dsp_mirror_reg_map_t;

//定义寄存器段映射表数组
static dsp_mirror_reg_map_t g_dsp_mirror_reg_map[] = {
    {0x3000, DSP_BOARD_3000_REGLEN, g_dsp_mirror_reg.seg_3000_val},
    {0x3080, DSP_BOARD_3080_REGLEN, g_dsp_mirror_reg.seg_3080_val},
    {0x3180, DSP_BOARD_3180_REGLEN, g_dsp_mirror_reg.seg_3180_val},
    {0x3200, DSP_BOARD_3200_REGLEN, g_dsp_mirror_reg.seg_3200_val},
    {0x3300, DSP_BOARD_3300_REGLEN, g_dsp_mirror_reg.seg_3300_val},
    {0x3500, DSP_BOARD_3500_REGLEN, g_dsp_mirror_reg.seg_3500_val},
    {0x3580, DSP_BOARD_3580_REGLEN, g_dsp_mirror_reg.seg_3580_val},
    {0x3680, DSP_BOARD_3680_REGLEN, g_dsp_mirror_reg.seg_3680_val},
    {0x3700, DSP_BOARD_3700_REGLEN, g_dsp_mirror_reg.seg_3700_val},
    {0x3900, DSP_BOARD_3900_REGLEN, g_dsp_mirror_reg.seg_3900_val},
    {0x3920, DSP_BOARD_3920_REGLEN, g_dsp_mirror_reg.seg_3920_val},
    {0x3940, DSP_BOARD_3940_REGLEN, g_dsp_mirror_reg.seg_3940_val},
    {0x3960, DSP_BOARD_3960_REGLEN, g_dsp_mirror_reg.seg_3960_val},
    {0x3980, DSP_BOARD_3980_REGLEN, g_dsp_mirror_reg.seg_3980_val},
    {0x3A00, DSP_BOARD_3A00_REGLEN, g_dsp_mirror_reg.seg_3A00_val},
    {0x4500, DSP_BOARD_4500_REGLEN, g_dsp_mirror_reg.seg_4500_val},
    {0x4580, DSP_BOARD_4580_REGLEN, g_dsp_mirror_reg.seg_4580_val},
    {0x4600, DSP_BOARD_4600_REGLEN, g_dsp_mirror_reg.seg_4600_val},
    {0x5000, DSP_BOARD_5000_REGLEN, g_dsp_mirror_reg.seg_5000_val},
    {0x5400, DSP_BOARD_5400_REGLEN, g_dsp_mirror_reg.seg_5400_val},
    {0x5800, DSP_BOARD_5800_REGLEN, g_dsp_mirror_reg.seg_5800_val},
    {0x5B00, DSP_BOARD_5B00_REGLEN, g_dsp_mirror_reg.seg_5B00_val},
    {0x6000, DSP_BOARD_6000_REGLEN, g_dsp_mirror_reg.seg_6000_val},
    {0x6400, DSP_BOARD_6400_REGLEN, g_dsp_mirror_reg.seg_6400_val},
};

//定义寄存器段映射表数组长度
#define DSP_MIRROR_REG_MAP_LEN sizeof(g_dsp_mirror_reg_map) / sizeof(dsp_mirror_reg_map_t)

//创建寄存器数据表

//读取一个寄存器数据
void dsp_mirror_read_reg(uint16_t reg_addr, uint16_t *reg_data){
    *reg_data = 0xFFFF;
    for(int i = 0; i < DSP_MIRROR_REG_MAP_LEN; i++){
        uint16_t seg_start = g_dsp_mirror_reg_map[i].reg_addr;
        uint16_t seg_end   = seg_start + g_dsp_mirror_reg_map[i].count;
        if(reg_addr >= seg_start && reg_addr < seg_end){
            *reg_data = g_dsp_mirror_reg_map[i].reg_data[reg_addr - seg_start];
            break;
        }
    }
}

//写入一个寄存器数据
void dsp_mirror_write_reg(uint16_t reg_addr, uint16_t reg_data){
    for(int i = 0; i < DSP_MIRROR_REG_MAP_LEN; i++){
        uint16_t seg_start = g_dsp_mirror_reg_map[i].reg_addr;
        uint16_t seg_end   = seg_start + g_dsp_mirror_reg_map[i].count;
        if(reg_addr >= seg_start && reg_addr < seg_end){
            g_dsp_mirror_reg_map[i].reg_data[reg_addr - seg_start] = reg_data;
            break;
        }
    }
}