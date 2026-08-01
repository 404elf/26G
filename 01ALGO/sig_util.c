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
		// 输入是各分量峰值，每项均方贡献为 Vp*Vp/2。
		sum += harmonic_amps[i] * harmonic_amps[i] * 0.5f;
	}
	return sqrtf(sum);
}

#define SIG_VPP_RANK_COUNT 200U
#define SIG_VPP_MAX_CYCLES 160U

static int32_t g_vpp_smallest[SIG_VPP_RANK_COUNT];
static int32_t g_vpp_largest[SIG_VPP_RANK_COUNT];
static float32_t g_cycle_max[SIG_VPP_MAX_CYCLES];
static float32_t g_cycle_min[SIG_VPP_MAX_CYCLES];

static float32_t SIG_RankVpp_s32(const int32_t *largest,
                                 const int32_t *smallest,
                                 uint16_t rank,
                                 float32_t code_to_v)
{
	uint16_t index = rank - 1U;
	return ((float32_t)largest[index] - (float32_t)smallest[index]) * code_to_v;
}

static float32_t SIG_MeanRank_s32(const int32_t *values, uint16_t first, uint16_t last)
{
	int32_t sum = 0;
	for (uint16_t index = first; index <= last; index++) {
		sum += values[index];
	}
	return (float32_t)sum / (float32_t)(last - first + 1U);
}

static float32_t SIG_MeanWindowVpp_s32(const int32_t *largest,
                                       const int32_t *smallest,
                                       uint16_t first_rank,
                                       uint16_t last_rank,
                                       float32_t code_to_v)
{
	uint16_t first = first_rank - 1U;
	uint16_t last = last_rank - 1U;
	return (SIG_MeanRank_s32(largest, first, last) -
	        SIG_MeanRank_s32(smallest, first, last)) * code_to_v;
}

uint8_t SIG_Calculate_VppSet(const uint32_t *dma_buf, float linear_gain, SIG_VppSet_t *out)
{
	if (dma_buf == NULL || out == NULL) {
		return 0U;
	}

	for (uint16_t index = 0U; index < SIG_VPP_RANK_COUNT; index++) {
		g_vpp_smallest[index] = 9999;
		g_vpp_largest[index] = -1;
	}

	for (uint32_t word_index = 0U; word_index < ADC_DMA_WORDS; word_index++) {
		uint32_t packed = dma_buf[word_index];
		int32_t samples[2] = {
			(int32_t)(packed & 0x0FFFU),
			(int32_t)((packed >> 16U) & 0x0FFFU)
		};

		for (uint8_t sample = 0U; sample < 2U; sample++) {
			if (samples[sample] < g_vpp_smallest[SIG_VPP_RANK_COUNT - 1U]) {
				SIG_InsertAscending_s32(g_vpp_smallest, SIG_VPP_RANK_COUNT - 1U, samples[sample]);
			}
			if (samples[sample] > g_vpp_largest[SIG_VPP_RANK_COUNT - 1U]) {
				SIG_InsertDescending_s32(g_vpp_largest, SIG_VPP_RANK_COUNT - 1U, samples[sample]);
			}
		}
	}

	float32_t gain = (linear_gain > 0.0f) ? linear_gain : 1.0f;
	float32_t code_to_v = 3.3f / 4096.0f / gain;
	out->rank1_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 1U, code_to_v);
	out->rank3_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 3U, code_to_v);
	out->rank5_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 5U, code_to_v);
	out->rank7_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 7U, code_to_v);
	out->rank9_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 9U, code_to_v);
	out->rank13_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 13U, code_to_v);
	out->rank17_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 17U, code_to_v);
	out->rank21_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 21U, code_to_v);
	out->rank25_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 25U, code_to_v);
	out->rank31_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 31U, code_to_v);
	out->rank35_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 35U, code_to_v);
	out->rank41_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 41U, code_to_v);
	out->rank45_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 45U, code_to_v);
	out->rank51_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 51U, code_to_v);
	out->rank61_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 61U, code_to_v);
	out->rank81_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 81U, code_to_v);
	out->rank101_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 101U, code_to_v);
	out->rank121_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 121U, code_to_v);
	out->rank141_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 141U, code_to_v);
	out->rank161_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 161U, code_to_v);
	out->rank181_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 181U, code_to_v);
	out->rank200_v = SIG_RankVpp_s32(g_vpp_largest, g_vpp_smallest, 200U, code_to_v);
	out->avg3_8_v = SIG_MeanWindowVpp_s32(g_vpp_largest, g_vpp_smallest, 3U, 8U, code_to_v);
	out->avg5_12_v = SIG_MeanWindowVpp_s32(g_vpp_largest, g_vpp_smallest, 5U, 12U, code_to_v);
	out->avg9_24_v = SIG_MeanWindowVpp_s32(g_vpp_largest, g_vpp_smallest, 9U, 24U, code_to_v);
	out->avg17_32_v = SIG_MeanWindowVpp_s32(g_vpp_largest, g_vpp_smallest, 17U, 32U, code_to_v);
	out->avg25_48_v = SIG_MeanWindowVpp_s32(g_vpp_largest, g_vpp_smallest, 25U, 48U, code_to_v);
	out->avg33_64_v = SIG_MeanWindowVpp_s32(g_vpp_largest, g_vpp_smallest, 33U, 64U, code_to_v);
	out->avg41_80_v = SIG_MeanWindowVpp_s32(g_vpp_largest, g_vpp_smallest, 41U, 80U, code_to_v);
	out->avg61_120_v = SIG_MeanWindowVpp_s32(g_vpp_largest, g_vpp_smallest, 61U, 120U, code_to_v);
	out->avg81_160_v = SIG_MeanWindowVpp_s32(g_vpp_largest, g_vpp_smallest, 81U, 160U, code_to_v);
	return 1U;
}

static void SIG_SortAscending_f32(float32_t *values, uint16_t count)
{
	for (uint16_t index = 1U; index < count; index++) {
		float32_t value = values[index];
		uint16_t pos = index;
		while (pos > 0U && values[pos - 1U] > value) {
			values[pos] = values[pos - 1U];
			pos--;
		}
		values[pos] = value;
	}
}

static float32_t SIG_Median_f32(const float32_t *values, uint16_t count)
{
	uint16_t middle = count / 2U;
	if ((count & 1U) != 0U) {
		return values[middle];
	}
	return 0.5f * (values[middle - 1U] + values[middle]);
}

static float32_t SIG_TrimmedMean_f32(const float32_t *values, uint16_t count, uint16_t trim)
{
	uint16_t first = trim;
	uint16_t last = count - trim;
	float32_t sum = 0.0f;
	for (uint16_t index = first; index < last; index++) {
		sum += values[index];
	}
	return sum / (float32_t)(last - first);
}

static float32_t SIG_CycleRange_f32(const float32_t *max_values,
                                    const float32_t *min_values,
                                    uint16_t count,
                                    uint16_t trim)
{
	return SIG_TrimmedMean_f32(max_values, count, trim) -
	       SIG_TrimmedMean_f32(min_values, count, trim);
}

uint8_t SIG_Calculate_CycleVppSet(const float *time_domain,
                                  uint16_t sample_count,
                                  float sample_rate_hz,
                                  float base_freq_hz,
                                  float linear_gain,
                                  SIG_CycleVppSet_t *out)
{
	if (time_domain == NULL || out == NULL || sample_count < 2U ||
	    sample_rate_hz <= 0.0f || base_freq_hz <= 0.0f) {
		return 0U;
	}

	float32_t samples_per_cycle = sample_rate_hz / base_freq_hz;
	uint16_t cycle_count = (uint16_t)((float32_t)(sample_count - 1U) / samples_per_cycle);
	if (cycle_count > SIG_VPP_MAX_CYCLES) {
		cycle_count = SIG_VPP_MAX_CYCLES;
	}
	if (cycle_count < 2U) {
		return 0U;
	}

	for (uint16_t cycle = 0U; cycle < cycle_count; cycle++) {
		uint16_t start = (uint16_t)ceilf((float32_t)cycle * samples_per_cycle);
		uint16_t end = (uint16_t)ceilf((float32_t)(cycle + 1U) * samples_per_cycle);
		if (end > sample_count) {
			end = sample_count;
		}
		if (start >= end) {
			return 0U;
		}

		float32_t high = time_domain[start];
		float32_t low = time_domain[start];
		for (uint16_t sample = (uint16_t)(start + 1U); sample < end; sample++) {
			if (time_domain[sample] > high) { high = time_domain[sample]; }
			if (time_domain[sample] < low) { low = time_domain[sample]; }
		}
		g_cycle_max[cycle] = high;
		g_cycle_min[cycle] = low;
	}

	SIG_SortAscending_f32(g_cycle_max, cycle_count);
	SIG_SortAscending_f32(g_cycle_min, cycle_count);

	uint16_t trim10 = cycle_count / 10U;
	uint16_t trim20 = cycle_count / 5U;
	uint16_t trim30 = (uint16_t)(((uint32_t)cycle_count * 3U) / 10U);
	float32_t gain = (linear_gain > 0.0f) ? linear_gain : 1.0f;
	out->cycle_count = cycle_count;
	out->mean_v = SIG_CycleRange_f32(g_cycle_max, g_cycle_min, cycle_count, 0U) / gain;
	out->median_v = (SIG_Median_f32(g_cycle_max, cycle_count) -
	                 SIG_Median_f32(g_cycle_min, cycle_count)) / gain;
	out->trim10_v = SIG_CycleRange_f32(g_cycle_max, g_cycle_min, cycle_count, trim10) / gain;
	out->trim20_v = SIG_CycleRange_f32(g_cycle_max, g_cycle_min, cycle_count, trim20) / gain;
	out->trim30_v = SIG_CycleRange_f32(g_cycle_max, g_cycle_min, cycle_count, trim30) / gain;
	out->auto_v = (cycle_count < 8U) ? out->median_v : out->trim20_v;
	return 1U;
}

float SIG_Calculate_CycleVpp(const float *time_domain,
                             uint16_t sample_count,
                             float sample_rate_hz,
                             float base_freq_hz,
                             float linear_gain)
{
	SIG_CycleVppSet_t result;
	if (!SIG_Calculate_CycleVppSet(time_domain, sample_count, sample_rate_hz,
	                              base_freq_hz, linear_gain, &result)) {
		return 0.0f;
	}
	return result.auto_v;
}

float SIG_Calculate_Vpp(const uint32_t *dma_buf, float linear_gain)
{
	if (dma_buf == NULL) {
		return 0.0f;
	}

	// Top-5 极值统计：剔除偶发毛刺，用第 5 大减第 5 小算 Vpp。
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

#define SIG_OLS_MAX_TONES 4U
#define SIG_OLS_MAX_PARAMS 9U
#define SIG_OLS_PIVOT_MIN 1e-6f
#define SIG_OLS_RENORMALIZE_MASK 0xFFU

// 规格很小，静态存放可避免在任务栈上放 9x10 矩阵。
static float32_t g_sig_ols_aug[SIG_OLS_MAX_PARAMS][SIG_OLS_MAX_PARAMS + 1U];

static uint8_t SIG_IsUsableFloat(float32_t value)
{
    return ((value == value) && (fabsf(value) < 1e20f)) ? 1U : 0U;
}

// 带部分主元的高斯消元，直接解方程，不显式求逆。
static uint8_t SIG_SolveOlsSystem(uint8_t size, float32_t *solution, float32_t *out_pivot_ratio)
{
    float32_t min_pivot = 1e20f;
    float32_t max_pivot = 0.0f;

    if (solution == NULL || out_pivot_ratio == NULL || size == 0U || size > SIG_OLS_MAX_PARAMS) {
        return 0U;
    }

    for (uint8_t column = 0U; column < size; column++) {
        uint8_t pivot_row = column;
        float32_t pivot_abs = fabsf(g_sig_ols_aug[column][column]);
        for (uint8_t row = (uint8_t)(column + 1U); row < size; row++) {
            float32_t candidate_abs = fabsf(g_sig_ols_aug[row][column]);
            if (candidate_abs > pivot_abs) {
                pivot_abs = candidate_abs;
                pivot_row = row;
            }
        }
        if (!SIG_IsUsableFloat(pivot_abs) || pivot_abs < SIG_OLS_PIVOT_MIN) {
            return 0U;
        }

        if (pivot_row != column) {
            for (uint8_t item = column; item <= size; item++) {
                float32_t temp = g_sig_ols_aug[column][item];
                g_sig_ols_aug[column][item] = g_sig_ols_aug[pivot_row][item];
                g_sig_ols_aug[pivot_row][item] = temp;
            }
        }

        if (pivot_abs < min_pivot) { min_pivot = pivot_abs; }
        if (pivot_abs > max_pivot) { max_pivot = pivot_abs; }

        float32_t pivot = g_sig_ols_aug[column][column];
        for (uint8_t row = (uint8_t)(column + 1U); row < size; row++) {
            float32_t factor = g_sig_ols_aug[row][column] / pivot;
            g_sig_ols_aug[row][column] = 0.0f;
            for (uint8_t item = (uint8_t)(column + 1U); item <= size; item++) {
                g_sig_ols_aug[row][item] -= factor * g_sig_ols_aug[column][item];
            }
        }
    }

    for (int row = (int)size - 1; row >= 0; row--) {
        float32_t value = g_sig_ols_aug[row][size];
        for (uint8_t column = (uint8_t)(row + 1); column < size; column++) {
            value -= g_sig_ols_aug[row][column] * solution[column];
        }
        float32_t pivot = g_sig_ols_aug[row][row];
        if (fabsf(pivot) < SIG_OLS_PIVOT_MIN) {
            return 0U;
        }
        solution[row] = value / pivot;
        if (!SIG_IsUsableFloat(solution[row])) {
            return 0U;
        }
    }

    *out_pivot_ratio = (max_pivot > 0.0f) ? (min_pivot / max_pivot) : 0.0f;
    return 1U;
}

static void SIG_AdvanceOlsOscillators(float32_t *osc_cos,
                                      float32_t *osc_sin,
                                      const float32_t *step_cos,
                                      const float32_t *step_sin,
                                      uint8_t tone_count,
                                      uint16_t sample_index)
{
    for (uint8_t tone = 0U; tone < tone_count; tone++) {
        float32_t old_cos = osc_cos[tone];
        float32_t next_cos = old_cos * step_cos[tone] - osc_sin[tone] * step_sin[tone];
        osc_sin[tone] = osc_sin[tone] * step_cos[tone] + old_cos * step_sin[tone];
        osc_cos[tone] = next_cos;

        // 长帧递推会积累舍入误差，定期拉回单位圆。
        if ((sample_index & SIG_OLS_RENORMALIZE_MASK) == SIG_OLS_RENORMALIZE_MASK) {
            float32_t norm = sqrtf(osc_cos[tone] * osc_cos[tone] + osc_sin[tone] * osc_sin[tone]);
            if (norm > 0.0f) {
                osc_cos[tone] /= norm;
                osc_sin[tone] /= norm;
            }
        }
    }
}

uint8_t SIG_ReconstructHarmonicWaveOLS(const float *time_domain,
                                       uint16_t sample_count,
                                       float sample_rate_hz,
                                       const float *component_freq_hz,
                                       uint8_t component_count,
                                       float interference_freq_hz,
                                       uint8_t period_count,
                                       uint16_t *out_pixels,
                                       uint16_t out_len,
                                       SIG_HarmonicFitResult_t *out_fit)
{
    const float32_t two_pi = 6.28318530717958647692f;
    // 单任务调用，工作数组放 BSS，给中断栈留足余量。
    static float32_t tone_freq_hz[SIG_OLS_MAX_TONES];
    static float32_t step_cos[SIG_OLS_MAX_TONES];
    static float32_t step_sin[SIG_OLS_MAX_TONES];
    static float32_t osc_cos[SIG_OLS_MAX_TONES];
    static float32_t osc_sin[SIG_OLS_MAX_TONES];
    static float32_t basis[SIG_OLS_MAX_PARAMS];
    static float32_t solution[SIG_OLS_MAX_PARAMS];
    uint8_t has_interference = (interference_freq_hz > 0.0f) ? 1U : 0U;
    uint8_t tone_count = (uint8_t)(component_count + has_interference);
    uint8_t parameter_count = (uint8_t)(1U + 2U * tone_count);

    if (out_fit != NULL) {
        out_fit->model_vpp_v = 0.0f;
        out_fit->residual_rms_v = 0.0f;
        out_fit->residual_ratio = 0.0f;
        out_fit->pivot_ratio = 0.0f;
        out_fit->interference_amp_v = 0.0f;
        for (uint8_t component = 0U; component < 4U; component++) {
            out_fit->component_amp_v[component] = 0.0f;
            out_fit->relative_phase_rad[component] = 0.0f;
        }
    }

    if (time_domain == NULL || component_freq_hz == NULL || out_pixels == NULL || out_fit == NULL ||
        sample_count < 2U || out_len == 0U || component_count == 0U ||
        tone_count > SIG_OLS_MAX_TONES || parameter_count > SIG_OLS_MAX_PARAMS ||
        sample_rate_hz <= 0.0f || component_freq_hz[0] <= 0.0f) {
        return 0U;
    }

    for (uint8_t tone = 0U; tone < component_count; tone++) {
        tone_freq_hz[tone] = component_freq_hz[tone];
    }
    if (has_interference) {
        tone_freq_hz[component_count] = interference_freq_hz;
    }
    for (uint8_t tone = 0U; tone < tone_count; tone++) {
        if (tone_freq_hz[tone] <= 0.0f || tone_freq_hz[tone] >= 0.5f * sample_rate_hz) {
            return 0U;
        }
        float32_t step = two_pi * tone_freq_hz[tone] / sample_rate_hz;
        step_cos[tone] = arm_cos_f32(step);
        step_sin[tone] = arm_sin_f32(step);
        osc_cos[tone] = 1.0f;
        osc_sin[tone] = 0.0f;
    }

    for (uint8_t row = 0U; row < parameter_count; row++) {
        for (uint8_t column = 0U; column <= parameter_count; column++) {
            g_sig_ols_aug[row][column] = 0.0f;
        }
    }

    float32_t signal_square_sum = 0.0f;
    for (uint16_t sample = 0U; sample < sample_count; sample++) {
        basis[0] = 1.0f;
        for (uint8_t tone = 0U; tone < tone_count; tone++) {
            basis[1U + 2U * tone] = osc_cos[tone];
            basis[2U + 2U * tone] = osc_sin[tone];
        }

        float32_t measured = time_domain[sample];
        signal_square_sum += measured * measured;
        for (uint8_t row = 0U; row < parameter_count; row++) {
            for (uint8_t column = row; column < parameter_count; column++) {
                g_sig_ols_aug[row][column] += basis[row] * basis[column];
            }
            g_sig_ols_aug[row][parameter_count] += basis[row] * measured;
        }
        SIG_AdvanceOlsOscillators(osc_cos, osc_sin, step_cos, step_sin, tone_count, sample);
    }

    float32_t inverse_count = 1.0f / (float32_t)sample_count;
    for (uint8_t row = 0U; row < parameter_count; row++) {
        for (uint8_t column = row; column < parameter_count; column++) {
            g_sig_ols_aug[row][column] *= inverse_count;
            g_sig_ols_aug[column][row] = g_sig_ols_aug[row][column];
        }
        g_sig_ols_aug[row][parameter_count] *= inverse_count;
    }

    if (!SIG_SolveOlsSystem(parameter_count, solution, &out_fit->pivot_ratio)) {
        return 0U;
    }

    static float32_t raw_phase[4];
    for (uint8_t component = 0U; component < component_count; component++) {
        float32_t cosine_coefficient = solution[1U + 2U * component];
        float32_t sine_coefficient = solution[2U + 2U * component];
        out_fit->component_amp_v[component] =
            sqrtf(cosine_coefficient * cosine_coefficient + sine_coefficient * sine_coefficient);
        raw_phase[component] = atan2f(-sine_coefficient, cosine_coefficient);
    }
    if (has_interference) {
        uint8_t tone = component_count;
        float32_t cosine_coefficient = solution[1U + 2U * tone];
        float32_t sine_coefficient = solution[2U + 2U * tone];
        out_fit->interference_amp_v =
            sqrtf(cosine_coefficient * cosine_coefficient + sine_coefficient * sine_coefficient);
    }

    out_fit->relative_phase_rad[0] = 0.0f;
    for (uint8_t component = 1U; component < component_count; component++) {
        uint32_t order = (uint32_t)(component_freq_hz[component] / component_freq_hz[0] + 0.5f);
        if (order < 2U) {
            return 0U;
        }
        float32_t phase = raw_phase[component] - (float32_t)order * raw_phase[0];
        while (phase > 3.14159265358979323846f) { phase -= two_pi; }
        while (phase < -3.14159265358979323846f) { phase += two_pi; }
        out_fit->relative_phase_rad[component] = phase;
    }

    for (uint8_t tone = 0U; tone < tone_count; tone++) {
        osc_cos[tone] = 1.0f;
        osc_sin[tone] = 0.0f;
    }
    float32_t residual_square_sum = 0.0f;
    for (uint16_t sample = 0U; sample < sample_count; sample++) {
        float32_t estimated = solution[0];
        for (uint8_t tone = 0U; tone < tone_count; tone++) {
            estimated += solution[1U + 2U * tone] * osc_cos[tone] +
                         solution[2U + 2U * tone] * osc_sin[tone];
        }
        float32_t residual = time_domain[sample] - estimated;
        residual_square_sum += residual * residual;
        SIG_AdvanceOlsOscillators(osc_cos, osc_sin, step_cos, step_sin, tone_count, sample);
    }

    out_fit->residual_rms_v = sqrtf(residual_square_sum * inverse_count);
    float32_t signal_rms_v = sqrtf(signal_square_sum * inverse_count);
    out_fit->residual_ratio = (signal_rms_v > 1e-9f)
                              ? (out_fit->residual_rms_v / signal_rms_v)
                              : 1.0f;
    out_fit->model_vpp_v = SIG_CalculateModelVpp(component_freq_hz,
                                                  out_fit->component_amp_v,
                                                  out_fit->relative_phase_rad,
                                                  component_count);

    uint8_t periods = (period_count == 3U) ? 3U : 1U;
    float32_t max_abs = 0.0f;
    for (uint16_t out_index = 0U; out_index < out_len; out_index++) {
        float32_t fraction = (out_len == 1U) ? 0.0f : (float32_t)out_index / (float32_t)(out_len - 1U);
        float32_t base_angle = two_pi * (float32_t)periods * fraction;
        float32_t value = 0.0f;
        for (uint8_t component = 0U; component < component_count; component++) {
            uint32_t order = (uint32_t)(component_freq_hz[component] / component_freq_hz[0] + 0.5f);
            value += out_fit->component_amp_v[component] *
                     arm_cos_f32((float32_t)order * base_angle + out_fit->relative_phase_rad[component]);
        }
        float32_t abs_value = fabsf(value);
        if (abs_value > max_abs) { max_abs = abs_value; }
    }

    if (max_abs < 1e-9f) {
        for (uint16_t out_index = 0U; out_index < out_len; out_index++) { out_pixels[out_index] = 120U; }
        return 1U;
    }

    for (uint16_t out_index = 0U; out_index < out_len; out_index++) {
        float32_t fraction = (out_len == 1U) ? 0.0f : (float32_t)out_index / (float32_t)(out_len - 1U);
        float32_t base_angle = two_pi * (float32_t)periods * fraction;
        float32_t value = 0.0f;
        for (uint8_t component = 0U; component < component_count; component++) {
            uint32_t order = (uint32_t)(component_freq_hz[component] / component_freq_hz[0] + 0.5f);
            value += out_fit->component_amp_v[component] *
                     arm_cos_f32((float32_t)order * base_angle + out_fit->relative_phase_rad[component]);
        }
        float32_t pixel = 120.0f - (value / max_abs) * 100.0f;
        pixel = SIG_ClampFloat(pixel, 20.0f, 220.0f);
        out_pixels[out_index] = (uint16_t)(pixel + 0.5f);
    }

    return 1U;
}
