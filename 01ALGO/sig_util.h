#ifndef __SIG_UTIL_H
#define __SIG_UTIL_H

#include "main.h"

// 按频率补偿前端幅频响应。
float SIG_Apply_LUT_Compensation(float freq_Hz, float raw_amp);
// 由各正弦分量峰值计算总有效值。
float SIG_Calculate_TrueRMS(const float *harmonic_amps, uint8_t count);
// 剔除极端样本后计算峰峰值。
float SIG_Calculate_Vpp(const uint32_t *dma_buf, float linear_gain);
// 查找带迟滞的上升过零点。
int SIG_Find_Hysteresis_ZeroCrossing(const float *unpacked_v, float V_avg);
// 从原始帧估相位，再按谐波参数合成 1 或 3 个周期。
uint8_t SIG_ReconstructHarmonicWave(const float *time_domain,
                                    uint16_t sample_count,
                                    float sample_rate_hz,
                                    const float *component_freq_hz,
                                    const float *component_amp,
                                    uint8_t component_count,
                                    uint8_t period_count,
                                    uint16_t *out_pixels,
                                    uint16_t out_len);

#endif /* __SIG_UTIL_H */
