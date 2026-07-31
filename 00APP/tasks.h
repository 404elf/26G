#ifndef __TASKS_H
#define __TASKS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define SERIAL_SCREEN_WAVE_POINTS 400U

// 这一组数据就是屏幕最终要显示的内容，包含频率、幅值、波形和柱状图。
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

// 任务层把处理结果写到这个全局结构里，显示层直接读取即可。
extern SerialScreenData_t g_screen_data;

// 信号处理任务：采样、FFT、谐波识别和显示数据整理都在这里完成。
void Task_Signal_Processing(void);
// 请求刷新一次静态波形；wave_mode 取 1 或 3。
// 串口屏触摸回调只需调用本接口，真正的 ADC/FFT 处理仍在主循环完成。
void Task_RequestWaveRefresh(uint8_t wave_mode);
// 请求重新测量一帧，并在 page1 显示离散频谱和各分量定量数据。
void Task_RequestSpectrumRefresh(void);
// 初始化淘晶驰串口屏接收链路；在 USART1 初始化完成后调用一次。
void SerialScreen_Init(void);
// 串口显示发送入口，当前先保留接口，后续由协议层实现。
void SerialScreen_Render_Task(const SerialScreenData_t *data);
// 串口命令入口，用来切换显示模式等简单控制项。
void SerialScreen_Rx_Handler(uint8_t rx_byte);

#endif /* __TASKS_H */
