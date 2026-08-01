#ifndef __SIG_UTIL_H
#define __SIG_UTIL_H

#include "main.h"

// 按频率补偿前端幅频响应。
float SIG_Apply_LUT_Compensation(float freq_Hz, float raw_amp);
// 由各正弦分量峰值计算总有效值。
float SIG_Calculate_TrueRMS(const float *harmonic_amps, uint8_t count);
// 全帧排序与区间平均候选，结果单位均为 V。
typedef struct {
	float rank1_v;
	float rank3_v;
	float rank5_v;
	float rank7_v;
	float rank9_v;
	float rank13_v;
	float rank17_v;
	float rank21_v;
	float rank25_v;
	float rank31_v;
	float rank35_v;
	float rank41_v;
	float rank45_v;
	float rank51_v;
	float rank61_v;
	float rank81_v;
	float rank101_v;
	float rank121_v;
	float rank141_v;
	float rank161_v;
	float rank181_v;
	float rank200_v;
	float avg3_8_v;
	float avg5_12_v;
	float avg9_24_v;
	float avg17_32_v;
	float avg25_48_v;
	float avg33_64_v;
	float avg41_80_v;
	float avg61_120_v;
	float avg81_160_v;
} SIG_VppSet_t;

// 按完整基波周期统计极值，所有候选来自同一帧。
typedef struct {
	uint16_t cycle_count;
	float mean_v;
	float median_v;
	float trim10_v;
	float trim20_v;
	float trim30_v;
	float auto_v;
} SIG_CycleVppSet_t;

uint8_t SIG_Calculate_VppSet(const uint32_t *dma_buf, float linear_gain, SIG_VppSet_t *out);
uint8_t SIG_Calculate_CycleVppSet(const float *time_domain,
                                  uint16_t sample_count,
                                  float sample_rate_hz,
                                  float base_freq_hz,
                                  float linear_gain,
                                  SIG_CycleVppSet_t *out);
float SIG_Calculate_Vpp(const uint32_t *dma_buf, float linear_gain);
float SIG_Calculate_CycleVpp(const float *time_domain,
                             uint16_t sample_count,
                             float sample_rate_hz,
                             float base_freq_hz,
                             float linear_gain);
// 查找带迟滞的上升过零点。
int SIG_Find_Hysteresis_ZeroCrossing(const float *unpacked_v, float V_avg);
// 联合最小二乘拟合结果；电压量统一使用 V。
typedef struct {
	float model_vpp_v;
	float residual_rms_v;
	float residual_ratio;
	float pivot_ratio;
	float component_amp_v[4];
	float relative_phase_rad[4];
	float interference_amp_v;
} SIG_HarmonicFitResult_t;

// 旧投影法仅作为拟合失败时的显示后备。
uint8_t SIG_ReconstructHarmonicWave(const float *time_domain,
                                    uint16_t sample_count,
                                    float sample_rate_hz,
                                    const float *component_freq_hz,
                                    const float *component_amp,
                                    uint8_t component_count,
                                    uint8_t period_count,
                                    uint16_t *out_pixels,
                                    uint16_t out_len,
                                    float *out_model_vpp,
                                    float *out_relative_phase);

// 同时拟合直流、基波、有效谐波和可选干扰，再由拟合模型重构波形。
uint8_t SIG_ReconstructHarmonicWaveOLS(const float *time_domain,
                                       uint16_t sample_count,
                                       float sample_rate_hz,
                                       const float *component_freq_hz,
                                       uint8_t component_count,
                                       float interference_freq_hz,
                                       uint8_t period_count,
                                       uint16_t *out_pixels,
                                       uint16_t out_len,
                                       SIG_HarmonicFitResult_t *out_fit);

#endif /* __SIG_UTIL_H */
