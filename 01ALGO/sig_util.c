#include "sig_util.h"
#include "arm_math.h"

/**
  * @brief：计算数组标准差 (RMS)
  * @param：data  输入数据数组
  * @param：len   数组长度
  * @retval：标准差
  */
float Calc_StdDiv(float* data, uint32_t len) {
    float mean = 0.0f;
    for (uint32_t i = 0; i < len; i++) mean += data[i];
    mean /= (float)len;
    float sum_sq = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        float d = data[i] - mean;
        sum_sq += d * d;
    }
    return sqrtf(sum_sq / (float)len);
}

/**
  * @brief：多周期过零测频（带迟滞抗噪）
  * @param：signal     输入信号数组
  * @param：len        数组长度
  * @param：fs         采样率 (Hz)
  * @param：hysteresis 迟滞门限，用于抑制均值线附近的噪声抖动
  * @retval：测得的频率 (Hz)，失败返回 0.0f
  */
float Calc_Freq_ZC(float* signal, uint32_t len, float fs, float hysteresis) {
    float mean = 0.0f;
    for (uint32_t i = 0; i < len; i++) mean += signal[i];
    mean /= (float)len;

    uint32_t zc_idx[MAX_ZC];
    uint32_t zc_cnt = 0;
    uint8_t state = 0;

    for (uint32_t i = 1; i < len && zc_cnt < MAX_ZC; i++) {
        if (state == 0 && (signal[i] > mean + hysteresis)) {
            state = 1;
            zc_idx[zc_cnt++] = i;
        } else if (state == 1 && (signal[i] < mean - hysteresis)) {
            state = 0;
        }
    }
    if (zc_cnt < 2) return 0.0f;

    uint32_t total_samples = zc_idx[zc_cnt - 1] - zc_idx[0];
    float period = (float)total_samples / (float)(zc_cnt - 1) / fs;
    return 1.0f / period;
}

/**
  * @brief：计算调幅度 ma = (Amax-Amin)/(Amax+Amin)，同时输出包络峰峰值
  * @param：envelope  AM 包络数组
  * @param：len       数组长度
  * @param：env_vpp   输出包络峰峰值 Vpp (V)，满量程 2V
  * @retval：调幅度 ma (0~1)
  */
float Calc_MA(float* envelope, uint32_t len, float* env_vpp) {
    float amax = envelope[0], amin = envelope[0];
    for (uint32_t i = 1; i < len; i++) {
        if (envelope[i] > amax) amax = envelope[i];
        if (envelope[i] < amin) amin = envelope[i];
    }
    float ma;
    if (amax + amin < 0.001f)
        ma = 0.0f;
    else
        ma = (amax - amin) / (amax + amin);
    *env_vpp = 3.3f * ma;
    return ma;
}

/**
  * @brief：计算最大频偏 Δfm (kHz)，用标准差(RMS)替代峰峰值，免疫尖峰噪声
  * @param：fm_dev   FM 瞬时频偏数组 (Hz)
  * @param：len      数组长度
  * @param：mod_vpp  输出调制信号峰峰值 Vpp (V)
  * @retval：最大频偏 Δfm (kHz)
  */
float Calc_DeltaFm(float* fm_dev, uint32_t len) {
    // 1. 计算均值
    float mean = 0.0f;
    for (uint32_t i = 0; i < len; i++) mean += fm_dev[i];
    mean /= (float)len;

    // 2. 计算标准差 (RMS)
    float sum_sq = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        float d = fm_dev[i] - mean;
        sum_sq += d * d;
    }
    float S_freq = sqrtf(sum_sq / (float)len);

    // 3. 正弦波振幅 A = sqrt(2) * S_freq，即峰值频偏 (Hz)
    float dfm_peak_hz = S_freq * 1.414214f * 1.0f;

    return dfm_peak_hz / 1000.0f;   // Delta_fm (kHz)
}

/**
  * @brief：将原始测频值归口到标准频率 1/2/3/4/5 kHz
  * @param：raw_f  原始频率 (Hz)
  * @retval：归口后的标准频率 (Hz)，无效返回 0.0f
  * @note：归口区间以中点 ±500Hz 为界，防止边界抖动
  */
float Freq_RoundToStandard(float raw_f) {
    if (raw_f >= 0.0f && raw_f < 1500.0f) return 1000.0f;
    if (raw_f >= 1500.0f && raw_f < 2500.0f) return 2000.0f;
    if (raw_f >= 2500.0f && raw_f < 3500.0f) return 3000.0f;
    if (raw_f >= 3500.0f && raw_f < 4500.0f) return 4000.0f;
    if (raw_f >= 4500.0f && raw_f <= 6500.0f) return 5000.0f;
    return 1000.0f;  // 题目无 0 挡，最小值 1kHz
}

/* ================================================================
 *  Task D — 数字解调工具函数
 * ================================================================ */

/**
  * @brief：16点滑动平均低通滤波
  * @param：in   输入数组
  * @param：out  输出数组（可与 in 相同，原地滤波）
  * @param：len  数组长度
  */
void LPF_16pt(float* in, float* out, uint32_t len) {
    const uint32_t W = 16;
    if (len < W) return;
    float sum = 0.0f;
    for (uint32_t i = 0; i < W; i++) sum += in[i];
    for (uint32_t i = 0; i < len - W; i++) {
        out[i] = sum / (float)W;
        sum += in[i + W] - in[i];
    }
    float last = out[len - W - 1];
    for (uint32_t i = len - W; i < len; i++) out[i] = last;
}

/**
  * @brief：自适应迟滞二值化（NRZ 码流恢复）
  * @param：signal     输入模拟信号
  * @param：bit_stream 输出 0/1 码流
  * @param：len        数组长度
  * @note：迟滞带 = ±15% 均值，免疫均值线附近噪声抖动
  */
void Adaptive_Binarize(float* signal, uint8_t* bit_stream, uint32_t len) {
    // 计算均值作为判决基准
    float mean = 0.0f;
    for (uint32_t i = 0; i < len; i++) mean += signal[i];
    mean /= (float)len;
    if (mean < 0.001f && mean > -0.001f) mean = 1.0f;  // 防零均值
    float abs_mean = (mean > 0.0f) ? mean : -mean;

    float hi_thr = mean + 0.15f * abs_mean;   // 上门限
    float lo_thr = mean - 0.15f * abs_mean;   // 下门限

    uint8_t state = (signal[0] > mean) ? 1 : 0;
    for (uint32_t i = 0; i < len; i++) {
        if (state == 0 && signal[i] > hi_thr) state = 1;
        if (state == 1 && signal[i] < lo_thr) state = 0;
        bit_stream[i] = state;
    }
}

/**
  * @brief：估计 NRZ 码速率并归口到 6k/8k/10k bps
  * @param：bit_stream  0/1 码流
  * @param：len         数组长度
  * @param：fs          采样率 (Hz)
  * @retval：归口后的码速率 (bps)，失败返回 0
  * @note：通过寻找最小连续脉冲长度推算位周期
  */
float Estimate_CodeRate(uint8_t* bit_stream, uint32_t len, float fs) {
    uint32_t run_len = 1;
    uint8_t  prev = bit_stream[0];

    // 收集所有有效跳变间隔，并映射到单比特等效长度
    uint32_t sum_L = 0;
    uint32_t cnt_L = 0;

    for (uint32_t i = 1; i < len; i++) {
        if (bit_stream[i] == prev) {
            run_len++;
        } else {
            // 剔除极小值（高频毛刺 < 4 个样点）
            if (run_len >= 4) {
                // 将多比特间隔归一化为单比特等效长度
                uint32_t L_single = run_len;
                if      (run_len >= 231) L_single = run_len / 3;
                else if (run_len >= 136) L_single = run_len / 2;

                // 只收集落在合理区间内的值 (6k~10k 对应 80~133 点)
                if (L_single >= 60 && L_single <= 150) {
                    sum_L += L_single;
                    cnt_L++;
                }
            }
            run_len = 1;
            prev = bit_stream[i];
        }
    }
    // 最后一段边界处理
    if (run_len >= 4) {
        uint32_t L_single = run_len;
        if      (run_len >= 231) L_single = run_len / 3;
        else if (run_len >= 136) L_single = run_len / 2;
        if (L_single >= 60 && L_single <= 150) {
            sum_L += L_single;
            cnt_L++;
        }
    }

    // 无有效数据则兜底 10k
    if (cnt_L == 0) return 10000.0f;

    // 取所有有效间隔的平均值作为单比特周期估算
    float avg_L = (float)sum_L / (float)cnt_L;
    float Rc_raw = fs / avg_L;

    // 归口到 6000 / 8000 / 10000 bps
    if      (Rc_raw <  7000.0f) return 6000.0f;
    else if (Rc_raw <  9000.0f) return 8000.0f;
    else                         return 10000.0f;
}

/**
  * @brief：2PSK 二倍相角载波恢复 + 星座扶正到 I 轴
  * @param：I       I 路基带信号
  * @param：Q       Q 路基带信号
  * @param：I_rot   输出：相位旋转后的 I 路（正值→bit 1，负值→bit 0）
  * @param：len     数组长度
  * @note：二倍角消除 180° 模糊，均值求 φ0，旋转恢复纯净基带
  */
void PSK_CarrierRecovery(float* I, float* Q, float* I_rot, uint32_t len) {
    // 1. 计算二倍相角并累加平均
    float sum_cos2 = 0.0f, sum_sin2 = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        float theta = atan2f(Q[i], I[i]);
        sum_cos2 += cosf(2.0f * theta);
        sum_sin2 += sinf(2.0f * theta);
    }

    // 2. 恢复载波相位 φ0
    float phi0 = 0.5f * atan2f(sum_sin2 / (float)len, sum_cos2 / (float)len);

    // 3. 旋转星座图扶正到 I 轴
    float c = cosf(phi0);
    float s = sinf(phi0);
    for (uint32_t i = 0; i < len; i++) {
        I_rot[i] = I[i] * c + Q[i] * s;
    }
}
