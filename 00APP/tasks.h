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

// 同一帧的 Vpp 对比值，全部使用 mV；展开该结构即可截图。
typedef struct {
	float rank1_mv;
	float rank3_mv;
	float rank5_mv;
	float rank7_mv;
	float rank9_mv;
	float rank13_mv;
	float rank17_mv;
	float rank21_mv;
	float rank25_mv;
	float rank31_mv;
	float rank35_mv;
	float rank41_mv;
	float rank45_mv;
	float rank51_mv;
	float rank61_mv;
	float rank81_mv;
	float rank101_mv;
	float rank121_mv;
	float rank141_mv;
	float rank161_mv;
	float rank181_mv;
	float rank200_mv;
	float avg3_8_mv;
	float avg5_12_mv;
	float avg9_24_mv;
	float avg17_32_mv;
	float avg25_48_mv;
	float avg33_64_mv;
	float avg41_80_mv;
	float avg61_120_mv;
	float avg81_160_mv;
	uint16_t cycle_count;
	float cycle_mean_mv;
	float cycle_median_mv;
	float cycle_trim10_mv;
	float cycle_trim20_mv;
	float cycle_trim30_mv;
	float cycle_auto_mv;
	float backup_mv;
	float model_mv;
	float ols_residual_mv;
	float ols_residual_pct;
	float ols_amp_error_pct;
	float ols_pivot_ratio;
	float ols_interference_mv;
	uint8_t ols_used;
} TaskVppDebug_t;

// OLS 联调精简视图；不参与正式计算。
typedef struct {
	uint8_t solved;
	uint8_t used;
	uint8_t held;
	uint8_t confirm_count;
	uint32_t stable_signature;
	uint32_t candidate_signature;
	float display_mv;
	float candidate_mv;
	float backup_mv;
	float model_mv;
	float corrected_mv;
	float correction_gain;
	float residual_mv;
	float residual_pct;
	float amp_error_pct;
	float pivot_ratio;
	float interference_mv;
	float interference_detected_mv;
	float phase2_deg;
	float phase3_deg;
} TaskOlsDebug_t;

extern volatile TaskVppDebug_t g_vpp_debug;
extern volatile TaskOlsDebug_t g_ols_debug;
extern volatile float g_rel_phase_deg[4];

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
