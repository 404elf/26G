#ifndef __SIG_UTIL_H
#define __SIG_UTIL_H

#include "main.h"

// 根据频率对幅值做前端补偿，让不同频段的结果更接近真实值。
float SIG_Apply_LUT_Compensation(float freq_Hz, float raw_amp);
// 用谐波幅值估算总真有效值。
float SIG_Calculate_TrueRMS(const float *harmonic_amps, uint8_t count);
// 从 DMA 数据中估算峰峰值，结果会结合前端增益做修正。
float SIG_Calculate_Vpp(const uint32_t *dma_buf, float linear_gain);
// 用简单迟滞阈值找到波形的起始零交点，便于重采样显示。
int SIG_Find_Hysteresis_ZeroCrossing(const float *unpacked_v, float V_avg);
// 把采样点重采样成适合屏幕显示的波形像素值。
void SIG_CubicSpline_Resample(const float *in_pts, uint16_t in_len, uint16_t *out_pixels, uint16_t out_len);

#endif /* __SIG_UTIL_H */
