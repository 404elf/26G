#include "sig_util.h"

#include <math.h>

#include "FFT.h"
#include "adc_measure.h"

// 频率补偿 LUT：根据频率修正前端幅值衰减。
#define SIG_LUT_POINTS 51U

static const float32_t lut_freq_gain[SIG_LUT_POINTS] = {
	1.000f, 1.008f, 1.016f, 1.024f, 1.032f, 1.040f, 1.048f, 1.056f, 1.064f, 1.072f,
	1.080f, 1.088f, 1.096f, 1.104f, 1.112f, 1.120f, 1.128f, 1.136f, 1.144f, 1.152f,
	1.160f, 1.168f, 1.176f, 1.184f, 1.192f, 1.200f, 1.208f, 1.216f, 1.224f, 1.232f,
	1.240f, 1.248f, 1.256f, 1.264f, 1.272f, 1.280f, 1.288f, 1.296f, 1.304f, 1.312f,
	1.320f, 1.328f, 1.336f, 1.344f, 1.352f, 1.360f, 1.368f, 1.376f, 1.384f, 1.392f,
	1.400f
};

static float32_t SIG_ClampFloat(float32_t value, float32_t min_value, float32_t max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

// 有序插入一个新值，用来维护最小/最大样本集合。
// 用整数版替代原来的 float 版，避免循环内反复做浮点运算。
static void SIG_InsertAscending_s32(int32_t *sorted_values, uint8_t count, int32_t value)
{
	uint8_t pos = count;
	while (pos > 0U && sorted_values[pos - 1U] > value) {
		sorted_values[pos] = sorted_values[pos - 1U];
		pos--;
	}
	sorted_values[pos] = value;
}

static void SIG_InsertDescending_s32(int32_t *sorted_values, uint8_t count, int32_t value)
{
	uint8_t pos = count;
	while (pos > 0U && sorted_values[pos - 1U] < value) {
		sorted_values[pos] = sorted_values[pos - 1U];
		pos--;
	}
	sorted_values[pos] = value;
}

float SIG_Apply_LUT_Compensation(float freq_Hz, float raw_amp)
{
    // 低频和高频直接落到 LUT 两端，中间做线性插值。
	if (freq_Hz <= 10000.0f) {
		return raw_amp * lut_freq_gain[0];
	}

	if (freq_Hz >= 500000.0f) {
		return raw_amp * lut_freq_gain[SIG_LUT_POINTS - 1U];
	}

	float32_t step_hz = 10000.0f;
	float32_t position = (freq_Hz - 10000.0f) / step_hz;
	uint32_t index = (uint32_t)position;
	float32_t fraction = position - (float32_t)index;
	if (index >= (SIG_LUT_POINTS - 1U)) {
		index = SIG_LUT_POINTS - 2U;
		fraction = 1.0f;
	}

	float32_t gain = lut_freq_gain[index] + (lut_freq_gain[index + 1U] - lut_freq_gain[index]) * fraction;
	return raw_amp * gain;
}

float SIG_Calculate_TrueRMS(const float *harmonic_amps, uint8_t count)
{
	if (harmonic_amps == NULL || count == 0U) {
		return 0.0f;
	}

	float32_t sum = 0.0f;
	for (uint8_t i = 0; i < count; i++) {
		// (Vp / √2)? = Vp? / 2，用乘法代替除法更快。
		sum += harmonic_amps[i] * harmonic_amps[i] * 0.5f;
	}
	return sqrtf(sum);
}

float SIG_Calculate_Vpp(const uint32_t *dma_buf, float linear_gain)
{
	if (dma_buf == NULL) {
		return 0.0f;
	}

	// 先用 ADC 原始码值（12 位整数）做 Top-5 极值统计，避免循环内大量浮点除法。
	int32_t smallest[5] = { 9999, 9999, 9999, 9999, 9999 };
	int32_t largest[5] = { -1, -1, -1, -1, -1 };

	for (uint32_t word_index = 0; word_index < ADC_DMA_WORDS; word_index++) {
		uint32_t packed = dma_buf[word_index];
		int32_t s0 = (int32_t)(packed & 0x0FFFU);
		int32_t s1 = (int32_t)((packed >> 16U) & 0x0FFFU);

		if (s0 < smallest[4]) { SIG_InsertAscending_s32(smallest, 4U, s0); }
		if (s0 > largest[4])  { SIG_InsertDescending_s32(largest, 4U, s0); }
		if (s1 < smallest[4]) { SIG_InsertAscending_s32(smallest, 4U, s1); }
		if (s1 > largest[4])  { SIG_InsertDescending_s32(largest, 4U, s1); }
	}

	// 只在最后把第 5 大 / 第 5 小的码值换算成电压，再做增益还原。
	float32_t v_max = (float32_t)largest[4] * 3.3f / 4096.0f;
	float32_t v_min = (float32_t)smallest[4] * 3.3f / 4096.0f;
	float32_t gain = (linear_gain > 0.0f) ? linear_gain : 1.0f;
	return (v_max - v_min) / gain;
}

int SIG_Find_Hysteresis_ZeroCrossing(const float *unpacked_v, float V_avg)
{
	if (unpacked_v == NULL) {
		return -1;
	}

	const float32_t v_hyst = 0.02f;
	// 用迟滞条件找上升沿零交点，避免噪声导致误触发。
	for (uint32_t index = 0; index + 1U < FFT_LENGTH; index++) {
		float32_t current = unpacked_v[index];
		float32_t next = unpacked_v[index + 1U];
		if (current < (V_avg - v_hyst) && next >= (V_avg + v_hyst)) {
			return (int)index;
		}
	}
	return -1;
}

void SIG_CubicSpline_Resample(const float *in_pts, uint16_t in_len, uint16_t *out_pixels, uint16_t out_len)
{
	if (in_pts == NULL || out_pixels == NULL || in_len < 2U || out_len == 0U) {
		return;
	}

	// 1. 寻找当前波形段的极值
	float32_t min_value = in_pts[0];
	float32_t max_value = in_pts[0];
	for (uint16_t i = 1; i < in_len; i++) {
		if (in_pts[i] < min_value) {
			min_value = in_pts[i];
		}
		if (in_pts[i] > max_value) {
			max_value = in_pts[i];
		}
	}

	float32_t range = max_value - min_value;
	if (range < 1e-6f) {
		for (uint16_t i = 0; i < out_len; i++) {
			out_pixels[i] = 120U;
		}
		return;
	}

	// 2. Catmull-Rom 四点样条插值：高频波峰圆滑无折角
	for (uint16_t out_index = 0; out_index < out_len; out_index++) {
		float32_t source_pos;
		if (out_len == 1U) {
			source_pos = 0.0f;
		} else {
			source_pos = ((float32_t)out_index * (float32_t)(in_len - 1U)) / (float32_t)(out_len - 1U);
		}

		int i1 = (int)source_pos;
		int i0 = (i1 > 0) ? (i1 - 1) : 0;
		int i2 = (i1 + 1 < (int)in_len) ? (i1 + 1) : (in_len - 1);
		int i3 = (i1 + 2 < (int)in_len) ? (i1 + 2) : (in_len - 1);

		float32_t t = source_pos - (float32_t)i1;
		float32_t t2 = t * t;
		float32_t t3 = t2 * t;

		float32_t p0 = in_pts[i0];
		float32_t p1 = in_pts[i1];
		float32_t p2 = in_pts[i2];
		float32_t p3 = in_pts[i3];

		float32_t interpolated = 0.5f * ((2.0f * p1) +
		                                 (-p0 + p2) * t +
		                                 (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
		                                 (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);

		// 3. 归一化并映射到 20~220 像素区间（留上下边距，避免贴屏边）
		float32_t normalized = (interpolated - min_value) / range;
		normalized = SIG_ClampFloat(normalized, 0.0f, 1.0f);
		float32_t pixel = 220.0f - normalized * 200.0f;
		out_pixels[out_index] = (uint16_t)(pixel + 0.5f);
	}
}


