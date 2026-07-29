#ifndef _FFT_H
#define _FFT_H

#include "main.h"
#include "arm_math.h"

// FFT 长度固定为 4096 点，便于和当前采样帧对齐。
#define FFT_LENGTH      4096U
// 实数 FFT 的有效幅值点数为 N/2。
#define FFT_BIN_COUNT   (FFT_LENGTH / 2U)
// 采样率由前端采样链路决定，这里用于把 bin 换算成频率。
#define FFT_SAMPLE_RATE_HZ  7200000.0f
#define FFT_BIN_WIDTH_HZ     (FFT_SAMPLE_RATE_HZ / (float32_t)FFT_LENGTH)

// FFT 实例和缓冲区都放在全局，方便任务层直接调用。
extern arm_rfft_fast_instance_f32 g_rfft_instance;
// 加窗后的时域输入（供 FFT 使用），放在 CCM 中。
extern float32_t rfft_input[FFT_LENGTH];
// 去直流、未加窗的纯交流时域信号（供过零触发和波形显示使用）。
extern float32_t rfft_time_domain[FFT_LENGTH];
// CMSIS-DSP 输出的复数域临时缓冲。
extern float32_t rfft_output_complex[FFT_LENGTH];
// 每个频点的幅值谱，供基波和谐波搜索使用。
extern float32_t rfft_mag[FFT_BIN_COUNT];

// 初始化 FFT 运行环境。
void FFT_Init(void);
// 把 DMA 数据展开成时域信号，再做窗函数和 FFT。
void FFT_Process_Unpack(const uint32_t *dma_buf, float linear_gain);
// 对当前峰值附近做细化插值，得到更准确的频率和幅值。
void FFT_Rife_Interpolate(int bin_center, float32_t *out_freq, float32_t *out_amp);
// 在指定 bin 范围内找幅值最大的点。
int FFT_Find_Max_Bin(const float32_t *mag, int start_bin, int end_bin);
// 谐波搜索时使用的局部峰值搜索接口，便于语义区分。
int FFT_Find_Local_Max_Bin(const float32_t *mag, int start_bin, int end_bin);

#endif

