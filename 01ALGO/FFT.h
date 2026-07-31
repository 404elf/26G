#ifndef _FFT_H
#define _FFT_H

#include "main.h"
#include "arm_math.h"

// 4096 个实采样点对应 2048 个 DMA 双样本字。
#define FFT_LENGTH      4096U
// 实数 FFT 只保留正频率半谱。
#define FFT_BIN_COUNT   (FFT_LENGTH / 2U)
// 三重 ADC 等效采样率；改定时器后记得同步这里。
#define FFT_SAMPLE_RATE_HZ  7200000.0f
#define FFT_BIN_WIDTH_HZ     (FFT_SAMPLE_RATE_HZ / (float32_t)FFT_LENGTH)

// FFT 实例与共享缓冲区。
extern arm_rfft_fast_instance_f32 g_rfft_instance;
// 加 Hann 窗后的 FFT 输入。
extern float32_t rfft_input[FFT_LENGTH];
// 去直流后的原始时域帧，供相位估计使用。
extern float32_t rfft_time_domain[FFT_LENGTH];
// CMSIS-DSP 复数输出暂存区。
extern float32_t rfft_output_complex[FFT_LENGTH];
// 正频率幅度谱。
extern float32_t rfft_mag[FFT_BIN_COUNT];

// 初始化 FFT 并预生成窗函数。
void FFT_Init(void);
// 解包 DMA、去直流、加窗并执行 FFT。
void FFT_Process_Unpack(const uint32_t *dma_buf, float linear_gain);
// 用 Rife 插值细化指定峰的频率和幅值。
void FFT_Rife_Interpolate(int bin_center, float32_t *out_freq, float32_t *out_amp, float linear_gain);
// 返回指定范围内的最大幅值 bin。
int FFT_Find_Max_Bin(const float32_t *mag, int start_bin, int end_bin);
// 局部峰搜索接口，语义上用于谐波窗口。
int FFT_Find_Local_Max_Bin(const float32_t *mag, int start_bin, int end_bin);

#endif

