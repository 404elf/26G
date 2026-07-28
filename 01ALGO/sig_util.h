#ifndef __SIG_UTIL_H
#define __SIG_UTIL_H

#include "main.h"

/* ---- 过零检测最大点数 ---- */
#define MAX_ZC  32

/* ================================================================
 *  通用信号处理工具函数
 * ================================================================ */

/* 标准差 (RMS) */
float Calc_StdDiv(float* data, uint32_t len);

/* 多周期过零测频（迟滞门限可配） */
float Calc_Freq_ZC(float* signal, uint32_t len, float fs, float hysteresis);

/* 调幅度 ma = (Amax-Amin)/(Amax+Amin)，同时输出包络 Vpp (V) */
float Calc_MA(float* envelope, uint32_t len, float* env_vpp);

/* 最大频偏 Delta_fm (kHz)，用标准差(RMS)替代峰峰值，免疫尖峰噪声 */
float Calc_DeltaFm(float* fm_dev, uint32_t len);

/* 将原始测频值归口到标准频率 1/2/3/4/5 kHz */
float Freq_RoundToStandard(float raw_f);

/* ---- Task D 数字解调专用 ---- */

/* 16点滑动平均低通滤波 */
void LPF_16pt(float* in, float* out, uint32_t len);

/* 自适应迟滞二值化 */
void Adaptive_Binarize(float* signal, uint8_t* bit_stream, uint32_t len);

/* 估计码速率并归口到 6k/8k/10k bps */
float Estimate_CodeRate(uint8_t* bit_stream, uint32_t len, float fs);

/* 2PSK 二倍相角载波恢复 + 星座扶正到 I 轴 */
void PSK_CarrierRecovery(float* I, float* Q, float* I_rot, uint32_t len);

#endif /* __SIG_UTIL_H */
