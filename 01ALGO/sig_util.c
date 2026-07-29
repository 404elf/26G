#include "sig_util.h"

#include <math.h>

#include "FFT.h"

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
static void SIG_InsertAscending(float32_t *sorted_values, uint8_t count, float32_t value)
{
	uint8_t pos = count;
	while (pos > 0U && sorted_values[pos - 1U] > value) {
		sorted_values[pos] = sorted_values[pos - 1U];
		pos--;
	}
	sorted_values[pos] = value;
}

// 反向插入，维护最大的几个样本。
static void SIG_InsertDescending(float32_t *sorted_values, uint8_t count, float32_t value)
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
		// 每个谐波按正弦波换算成 RMS 后再平方叠加。
		float32_t component = harmonic_amps[i] / 1.41421356f;
		sum += component * component;
	}
	return sqrtf(sum);
}

float SIG_Calculate_Vpp(const uint32_t *dma_buf, float linear_gain)
{
	if (dma_buf == NULL) {
		return 0.0f;
	}

	float32_t smallest[5] = { 1e9f, 1e9f, 1e9f, 1e9f, 1e9f };
	float32_t largest[5] = { -1e9f, -1e9f, -1e9f, -1e9f, -1e9f };

	for (uint32_t word_index = 0; word_index < ADC_DMA_WORDS; word_index++) {
		// 双 ADC 采样值是打包在一个 32 位字里的，这里拆开后分别统计。
		uint32_t packed = dma_buf[word_index];
		uint16_t samples[2];
		samples[0] = (uint16_t)(packed & 0xFFFFU) & 0x0FFFU;
		samples[1] = (uint16_t)((packed >> 16U) & 0xFFFFU) & 0x0FFFU;

		for (uint8_t i = 0; i < 2U; i++) {
			float32_t voltage = (float32_t)samples[i] * 3.3f / 4096.0f;

			if (voltage < smallest[4]) {
				SIG_InsertAscending(smallest, 4U, voltage);
			}
			if (voltage > largest[4]) {
				SIG_InsertDescending(largest, 4U, voltage);
			}
		}
	}

	float32_t gain = (linear_gain > 0.0f) ? linear_gain : 1.0f;
	return (largest[4] - smallest[4]) / gain;
}

int SIG_Find_Hysteresis_ZeroCrossing(const float *unpacked_v, float V_avg)
{
	if (unpacked_v == NULL) {
		return -1;
	}

	const float32_t v_hyst = 0.02f;
	// 用迟滞条件找上升沿零交点，避免噪声导致误触发。
	for (uint32_t index = 0; index + 1U < FFT_BIN_COUNT; index++) {
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

	float32_t min_value = in_pts[0];
	float32_t max_value = in_pts[0];
	// 先找出当前段波形的最大最小值，再映射到屏幕像素高度。
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

	for (uint16_t out_index = 0; out_index < out_len; out_index++) {
		// 屏幕上的每个像素点都来自原始波形的一段插值结果。
		float32_t source_pos;
		if (out_len == 1U) {
			source_pos = 0.0f;
		} else {
			source_pos = ((float32_t)out_index * (float32_t)(in_len - 1U)) / (float32_t)(out_len - 1U);
		}

		uint16_t left_index = (uint16_t)source_pos;
		uint16_t right_index = (left_index + 1U < in_len) ? (left_index + 1U) : left_index;
		float32_t fraction = source_pos - (float32_t)left_index;
		float32_t interpolated = in_pts[left_index] + (in_pts[right_index] - in_pts[left_index]) * fraction;
		float32_t normalized = (interpolated - min_value) / range;
		float32_t pixel = (1.0f - SIG_ClampFloat(normalized, 0.0f, 1.0f)) * 240.0f;
		if (pixel < 0.0f) {
			pixel = 0.0f;
		} else if (pixel > 240.0f) {
			pixel = 240.0f;
		}
		out_pixels[out_index] = (uint16_t)(pixel + 0.5f);
	}
}


