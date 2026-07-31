#ifndef __TASKS_H
#define __TASKS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define SERIAL_SCREEN_WAVE_POINTS 800U

// 串口屏一帧所需的全部测量与绘图数据。
typedef struct {
	float f1_base_freq;
	float Vpp_val;
	float Vrms_val;

	uint8_t harmonic_count;
	float harmonic_freq[4];
	float harmonic_amp[4];

	uint8_t wave_disp_mode;
	uint16_t wave_pixels[SERIAL_SCREEN_WAVE_POINTS];
	uint16_t spec_bar_x[4];
	uint16_t spec_bar_y[4];
} SerialScreenData_t;

// 只发布完整帧；显示层直接读取。
extern SerialScreenData_t g_screen_data;

// 完成一次采样、频谱分析和显示数据更新。
void Task_Signal_Processing(void);
// 请求刷新 1 或 3 个周期；中断里只发请求，不直接跑 ADC/FFT。
void Task_RequestWaveRefresh(uint8_t wave_mode);
// 请求刷新一次频谱页。
void Task_RequestSpectrumRefresh(void);
// USART1 初始化后调用一次。
void SerialScreen_Init(void);
// 按当前页面发送一帧显示数据。
void SerialScreen_Render_Task(const SerialScreenData_t *data);
// 逐字节接收串口屏控制命令。
void SerialScreen_Rx_Handler(uint8_t rx_byte);

#endif /* __TASKS_H */
