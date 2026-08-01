#include "sig_util.h"

#include <math.h>

#include "FFT.h"
#include "adc_measure.h"

// 10~500 kHz 幅值补偿表，每 10 kHz 一个点。
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

// 维护固定长度的极值队列，供抗毛刺 Vpp 统计使用。
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
    // 超出表范围时钳位，中间线性插值。
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
		// 输入是峰值，不是 RMS；每项功率为 Vp*Vp/2。
		sum += harmonic_amps[i] * harmonic_amps[i] * 0.5f;
	}
	return sqrtf(sum);
}

float SIG_Calculate_Vpp(const uint32_t *dma_buf, float linear_gain)
{
	if (dma_buf == NULL) {
		return 0.0f;
	}

	// 丢掉两端各 4 个毛刺，用第 5 大减第 5 小。
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
	// 20 mV 迟滞，避免零点附近来回触发。
	for (uint32_t index = 0; index + 1U < FFT_LENGTH; index++) {
		float32_t current = unpacked_v[index];
		float32_t next = unpacked_v[index + 1U];
		if (current < (V_avg - v_hyst) && next >= (V_avg + v_hyst)) {
			return (int)index;
		}
	}
	return -1;
}

#define SIG_MODEL_VPP_POINTS 4096U

static float32_t SIG_CalculateModelVpp(const float32_t *component_freq_hz,
                                       const float32_t *component_amp,
                                       const float32_t *relative_phase,
                                       uint8_t component_count)
{
    const float32_t two_pi = 6.28318530717958647692f;
    float32_t osc_cos[4] = { 0.0f };
    float32_t osc_sin[4] = { 0.0f };
    float32_t step_cos[4] = { 0.0f };
    float32_t step_sin[4] = { 0.0f };

    for (uint8_t component = 0U; component < component_count; component++) {
        uint32_t order = (uint32_t)(component_freq_hz[component] / component_freq_hz[0] + 0.5f);
        if (order < 1U) {
            return 0.0f;
        }
        float32_t step = two_pi * (float32_t)order / (float32_t)SIG_MODEL_VPP_POINTS;
        osc_cos[component] = arm_cos_f32(relative_phase[component]);
        osc_sin[component] = arm_sin_f32(relative_phase[component]);
        step_cos[component] = arm_cos_f32(step);
        step_sin[component] = arm_sin_f32(step);
    }

    float32_t model_min = 0.0f;
    float32_t model_max = 0.0f;
    for (uint16_t sample = 0U; sample < SIG_MODEL_VPP_POINTS; sample++) {
        float32_t value = 0.0f;
        for (uint8_t component = 0U; component < component_count; component++) {
            value += component_amp[component] * osc_cos[component];
        }

        if (sample == 0U) {
            model_min = value;
            model_max = value;
        } else {
            if (value < model_min) { model_min = value; }
            if (value > model_max) { model_max = value; }
        }

        for (uint8_t component = 0U; component < component_count; component++) {
            float32_t next_cos = osc_cos[component] * step_cos[component] -
                                 osc_sin[component] * step_sin[component];
            osc_sin[component] = osc_sin[component] * step_cos[component] +
                                 osc_cos[component] * step_sin[component];
            osc_cos[component] = next_cos;
        }
    }

    return model_max - model_min;
}
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
                                    float *out_relative_phase)
{
    const float32_t two_pi = 6.28318530717958647692f;
    float32_t raw_phase[4] = { 0.0f };
    float32_t relative_phase[4] = { 0.0f };

    if (out_model_vpp != NULL) {
        *out_model_vpp = 0.0f;
    }
    if (out_relative_phase != NULL) {
        for (uint8_t component = 0U; component < 4U; component++) {
            out_relative_phase[component] = 0.0f;
        }
    }

    if (time_domain == NULL || component_freq_hz == NULL || component_amp == NULL ||
        out_pixels == NULL || sample_count < 2U || out_len == 0U ||
        component_count == 0U || component_count > 4U ||
        sample_rate_hz <= 0.0f || component_freq_hz[0] <= 0.0f) {
        return 0U;
    }

    // Hann 加权正交投影只估相位；显示点不再直接复用带噪 ADC 样本。
    float32_t window_step = two_pi / (float32_t)(sample_count - 1U);
    float32_t window_step_cos = arm_cos_f32(window_step);
    float32_t window_step_sin = arm_sin_f32(window_step);

    for (uint8_t component = 0U; component < component_count; component++) {
        float32_t omega = two_pi * component_freq_hz[component] / sample_rate_hz;
        float32_t step_cos = arm_cos_f32(omega);
        float32_t step_sin = arm_sin_f32(omega);
        float32_t osc_cos = 1.0f;
        float32_t osc_sin = 0.0f;
        float32_t window_cos = 1.0f;
        float32_t window_sin = 0.0f;
        float32_t in_phase = 0.0f;
        float32_t quadrature = 0.0f;

        for (uint16_t sample = 0U; sample < sample_count; sample++) {
            float32_t window = 0.5f * (1.0f - window_cos);
            float32_t weighted = time_domain[sample] * window;
            in_phase += weighted * osc_cos;
            quadrature += weighted * osc_sin;

            float32_t next_osc_cos = osc_cos * step_cos - osc_sin * step_sin;
            osc_sin = osc_sin * step_cos + osc_cos * step_sin;
            osc_cos = next_osc_cos;

            float32_t next_window_cos = window_cos * window_step_cos - window_sin * window_step_sin;
            window_sin = window_sin * window_step_cos + window_cos * window_step_sin;
            window_cos = next_window_cos;
        }

        // 对 x=A·cos(ωt+φ)，I/Q 对应 A·cosφ 与 -A·sinφ。
        raw_phase[component] = atan2f(-quadrature, in_phase);
    }

    // 消掉任意帧起点，只保留各谐波相对基波的相位。
    relative_phase[0] = 0.0f;
    for (uint8_t component = 1U; component < component_count; component++) {
        uint32_t order = (uint32_t)(component_freq_hz[component] / component_freq_hz[0] + 0.5f);
        if (order < 2U) {
            return 0U;
        }
        float32_t phase = raw_phase[component] - (float32_t)order * raw_phase[0];
        while (phase > 3.14159265358979323846f) { phase -= two_pi; }
        while (phase < -3.14159265358979323846f) { phase += two_pi; }
        relative_phase[component] = phase;
    }

    if (out_relative_phase != NULL) {
        for (uint8_t component = 0U; component < component_count; component++) {
            out_relative_phase[component] = relative_phase[component];
        }
    }
    if (out_model_vpp != NULL) {
        *out_model_vpp = SIG_CalculateModelVpp(component_freq_hz,
                                               component_amp,
                                               relative_phase,
                                               component_count);
    }

    uint8_t periods = (period_count == 3U) ? 3U : 1U;
    float32_t max_abs = 0.0f;
    for (uint16_t out_index = 0U; out_index < out_len; out_index++) {
        float32_t fraction = (out_len == 1U) ? 0.0f : (float32_t)out_index / (float32_t)(out_len - 1U);
        float32_t base_angle = two_pi * (float32_t)periods * fraction;
        float32_t value = 0.0f;
        for (uint8_t component = 0U; component < component_count; component++) {
            uint32_t order = (uint32_t)(component_freq_hz[component] / component_freq_hz[0] + 0.5f);
            value += component_amp[component] * arm_cos_f32((float32_t)order * base_angle + relative_phase[component]);
        }
        float32_t abs_value = fabsf(value);
        if (abs_value > max_abs) { max_abs = abs_value; }
    }

    if (max_abs < 1e-6f) {
        for (uint16_t out_index = 0U; out_index < out_len; out_index++) { out_pixels[out_index] = 120U; }
        return 1U;
    }

    for (uint16_t out_index = 0U; out_index < out_len; out_index++) {
        float32_t fraction = (out_len == 1U) ? 0.0f : (float32_t)out_index / (float32_t)(out_len - 1U);
        float32_t base_angle = two_pi * (float32_t)periods * fraction;
        float32_t value = 0.0f;
        for (uint8_t component = 0U; component < component_count; component++) {
            uint32_t order = (uint32_t)(component_freq_hz[component] / component_freq_hz[0] + 0.5f);
            value += component_amp[component] * arm_cos_f32((float32_t)order * base_angle + relative_phase[component]);
        }
        float32_t pixel = 120.0f - (value / max_abs) * 100.0f;
        pixel = SIG_ClampFloat(pixel, 20.0f, 220.0f);
        out_pixels[out_index] = (uint16_t)(pixel + 0.5f);
    }

    return 1U;
}
