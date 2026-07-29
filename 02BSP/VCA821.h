#ifndef __VCA821_H
#define __VCA821_H

#include "main.h"
#include "dac.h"

// VCA821 的控制电压输出使用 DAC1 通道 1。
#define VCA_DAC_CHANNEL  DAC_CHANNEL_1

// 当前前端线性增益，FFT 和幅值换算会直接用到。
extern float g_vca821_linear_gain;

// 初始化前端增益控制链路。
void VCA821_Init(void);
// 按 dB 设置增益，内部会换算成 DAC 控制电压。
void VCA821_SetGain(float gain_dB);
// 返回当前线性增益值，供幅值补偿使用。
float VCA821_GetLinearGain(void);

// 保留旧版慢速 AGC 接口，兼容已有工程调用。
void Run_Slow_AGC(void);
// 200ms 周期的慢速 AGC。
void Run_Slow_AGC_200ms(void);
// 直接按电压设置增益控制值。
void VGA_SetVoltage(float voltage);

#endif


