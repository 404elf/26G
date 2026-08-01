#include "tasks.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FFT.h"

#include "adc_measure.h"
#include "sig_util.h"
#include "usart.h"

// 联调时才改为 1；交付前保持 0，避免屏幕被连续刷新。
#define TASK_DEBUG_AUTO_REFRESH 0
#define TASK_DEBUG_FIREWATER_OUTPUT 0

#define HMI_WAVE_SOURCE_POINTS SERIAL_SCREEN_WAVE_POINTS
#define HMI_WAVE_TX_POINTS 800U
#define HMI_SPECTRUM_TX_POINTS 496U
#define HMI_SPECTRUM_SIDE_MARGIN 16U
#define HMI_ADD_READY_TIMEOUT_MS 100U
#define HMI_VIEW_WAVEFORM 0U
#define HMI_VIEW_SPECTRUM 1U

// 显示帧放 IRAM1，避开 DMA 与 FFT 热区。
SerialScreenData_t g_screen_data __attribute__((section(".iram1")));

// 保留全部候选供监视窗口比较，正式显示按频段选取。
volatile TaskVppDebug_t g_vpp_debug = { 0 };
volatile TaskOlsDebug_t g_ols_debug = { 0 };
volatile float g_rel_phase_deg[4] = { 0.0f };

// 测量边界与抗噪门限，单位见变量名。
static const uint32_t TASK_SAMPLE_TIMEOUT_MS = 10U;
static const float32_t TASK_HARMONIC_NOISE_FLOOR_V = 0.002f;
static const float32_t TASK_BASE_MIN_AMP_V = 0.003f;
static const float32_t TASK_FREQ_GRID_HZ = 500.0f;
static const float32_t TASK_BASE_MIN_FREQ_HZ = 10000.0f;
static const float32_t TASK_BASE_MAX_FREQ_HZ = 250000.0f;
static const float32_t TASK_VPP_TWO_COMPONENT_SWITCH_HZ = 50000.0f;
static const float32_t TASK_COMPONENT_MAX_FREQ_HZ = 500000.0f;
static const float32_t TASK_FREQ_EDGE_TOLERANCE_HZ = 250.0f;
static const float32_t TASK_HARMONIC_FREQ_TOLERANCE_HZ = TASK_FREQ_GRID_HZ;
static const float32_t TASK_INTERFERENCE_CENTER_HZ = 1000000.0f;
static const float32_t TASK_INTERFERENCE_SEARCH_HALF_HZ = 10000.0f;
static const float32_t TASK_INTERFERENCE_MIN_AMP_V = 0.001f;

#define TASK_VPP_BATCH_FRAMES 3U
#define TASK_DECISION_CONFIRM_FRAMES 3U
#define TASK_DECISION_MAX_AUTO_RETRIES 8U
#define TASK_DECISION_OLS_BIT (1UL << 4)
#define TASK_SPECTRAL_CANDIDATE_MAX 12U

typedef struct {
	int bin;
	float32_t freq_hz;
	float32_t amp_v;
} Task_SpectralCandidate_t;

// 工作帧成功后才发布，失败时保留上一帧。
static SerialScreenData_t g_working_screen_data __attribute__((section(".iram1")));

// 中断只置请求，主循环看到 pending 后处理一次。
static volatile uint8_t g_wave_refresh_pending = 0U;
static volatile uint8_t g_requested_wave_mode = 1U;
static volatile uint8_t g_requested_view = HMI_VIEW_WAVEFORM;
static uint8_t g_render_view = HMI_VIEW_WAVEFORM;

// 短命令采用滑动缓冲，兼容 W1/W3/T1/T3。
static char g_serial_cmd_buffer[12];
static uint8_t g_serial_cmd_length = 0;
static uint8_t g_hmi_rx_byte = 0U;
static volatile uint8_t g_hmi_addt_ready = 0U;
static volatile uint8_t g_hmi_addt_finished = 0U;
static uint8_t g_hmi_wave_tx[HMI_WAVE_TX_POINTS] __attribute__((section(".iram1")));

// 发布判定防抖；只约束模式切换，不改本帧测量结果。
static uint32_t g_stable_decision_signature = 0U;
static uint32_t g_pending_decision_signature = 0U;
static uint8_t g_stable_decision_valid = 0U;
static uint8_t g_pending_decision_count = 0U;
static volatile uint8_t g_decision_retry_count = 0U;

// 每次按键单独收集 3 帧；发布后不把本轮结果带到下一次按键。
static float32_t g_vpp_batch_mv[TASK_VPP_BATCH_FRAMES] = { 0.0f };
static uint32_t g_vpp_batch_signature = 0U;
static uint8_t g_vpp_batch_count = 0U;
static volatile uint8_t g_vpp_batch_reset_pending = 0U;

static void HMISendString(const char *str);
static HAL_StatusTypeDef HMI_SendCurveBytes(uint16_t point_count);
static HAL_StatusTypeDef HMI_SendWaveform(const uint16_t *wave_pixels, uint16_t source_count);
static HAL_StatusTypeDef HMI_SendSpectrum(const SerialScreenData_t *data);

static void Task_ResetScreenData(SerialScreenData_t *data, uint8_t wave_mode)
{
	if (data == NULL) {
		return;
	}
	memset(data, 0, sizeof(*data));
	data->wave_disp_mode = (wave_mode == 3U) ? 3U : 1U;
}

static void Task_UpdateSpectrumBars(SerialScreenData_t *data, uint8_t harmonic_count)
{
	if (data == NULL) {
		return;
	}

	// 将分量序号和幅值换成柱状图坐标。
	for (uint8_t i = 0; i < 4U; i++) {
		if (i < harmonic_count) {
			data->spec_bar_x[i] = (uint16_t)(24U + (uint16_t)i * 28U);
			float amp = data->harmonic_amp[i];
			if (amp < 0.0f) {
				amp = 0.0f;
			}
			if (amp > 240.0f) {
				amp = 240.0f;
			}
			data->spec_bar_y[i] = (uint16_t)(amp + 0.5f);
		} else {
			data->spec_bar_x[i] = 0U;
			data->spec_bar_y[i] = 0U;
		}
	}
}

static uint8_t Task_IsAboveNoiseFloor(float32_t peak_mag, float32_t linear_gain)
{
	// 先做廉价幅值门限，噪声峰不进入 Rife 插值。
	float32_t gain = (linear_gain > 0.0f) ? linear_gain : 1.0f;
	float32_t estimated_amp = (peak_mag / (float32_t)FFT_BIN_COUNT) * 2.0f / gain;
	return (estimated_amp >= TASK_HARMONIC_NOISE_FLOOR_V) ? 1U : 0U;
}

static float32_t Task_SnapFrequencyToGrid(float32_t freq_hz)
{
	if (freq_hz <= 0.0f) {
		return 0.0f;
	}
	return floorf(freq_hz / TASK_FREQ_GRID_HZ + 0.5f) * TASK_FREQ_GRID_HZ;
}

// 系数按现有现场数据取误差带上沿，避免过度追零。
static float32_t Task_GetOlsVppCorrection(float32_t base_freq_hz,
                                          uint8_t component_count,
                                          uint8_t has_interference)
{
	if (component_count <= 1U) {
		return 1.0f;
	}
	if (has_interference) {
		return 0.872f;
	}
	if (component_count == 2U) {
		return 0.985f;
	}

	if (base_freq_hz <= 60000.0f) {
		return 0.960f;
	}
	if (base_freq_hz < 120000.0f) {
		return 0.960f - (base_freq_hz - 60000.0f) * (0.030f / 60000.0f);
	}
	if (base_freq_hz < 160000.0f) {
		return 0.930f - (base_freq_hz - 120000.0f) * (0.065f / 40000.0f);
	}
	return 0.865f;
}

static uint8_t Task_GetVppDecisionRegion(float32_t base_freq_hz, uint8_t component_count)
{
	// 0: rank101；1: rank3。三分量优先抗毛刺，两分量再按周期数对应的频段分类。
	if (component_count >= 3U || base_freq_hz < TASK_VPP_TWO_COMPONENT_SWITCH_HZ) {
		return 0U;
	}
	return 1U;
}

static uint32_t Task_BuildDecisionSignature(const SerialScreenData_t *data,
                                            float32_t base_freq_hz,
                                            uint8_t component_count,
                                            uint8_t has_interference,
                                            uint8_t use_ols)
{
	uint32_t signature = (uint32_t)Task_GetVppDecisionRegion(base_freq_hz, component_count);
	if (has_interference) {
		signature |= (1UL << 3);
	}
	if (use_ols) {
		signature |= TASK_DECISION_OLS_BIT;
	}

	// 基频档位和谐波阶数都参与确认，防止相邻判定来回切换。
	uint32_t base_grid = (uint32_t)(base_freq_hz / TASK_FREQ_GRID_HZ + 0.5f);
	if (base_grid > 0x1FFU) {
		base_grid = 0x1FFU;
	}
	signature |= base_grid << 17U;

	if (data != NULL && base_freq_hz > 0.0f) {
		for (uint8_t component = 1U; component < component_count && component < 3U; component++) {
			uint32_t order = (uint32_t)(data->harmonic_freq[component] * 1000.0f /
			                            base_freq_hz + 0.5f);
			if (order > 0x3FU) {
				order = 0x3FU;
			}
			signature |= order << (5U + (uint32_t)(component - 1U) * 6U);
		}
	}
	return signature;
}

static uint8_t Task_ConfirmDecision(uint32_t candidate_signature)
{
	if (g_stable_decision_valid == 0U) {
		g_stable_decision_signature = candidate_signature;
		g_pending_decision_signature = candidate_signature;
		g_pending_decision_count = 0U;
		g_stable_decision_valid = 1U;
		return 1U;
	}

	if (candidate_signature == g_stable_decision_signature) {
		g_pending_decision_signature = candidate_signature;
		g_pending_decision_count = 0U;
		return 1U;
	}

	if (candidate_signature != g_pending_decision_signature) {
		g_pending_decision_signature = candidate_signature;
		g_pending_decision_count = 1U;
	} else if (g_pending_decision_count < 0xFFU) {
		g_pending_decision_count++;
	}

	if (g_pending_decision_count >= TASK_DECISION_CONFIRM_FRAMES) {
		g_stable_decision_signature = candidate_signature;
		g_pending_decision_count = 0U;
		return 1U;
	}
	return 0U;
}

static void Task_ResetVppBatch(void)
{
	g_vpp_batch_signature = 0U;
	g_vpp_batch_count = 0U;
}

static uint8_t Task_CollectVppBatch(float32_t candidate_mv,
	                                uint32_t signature,
	                                float32_t *average_mv)
{
	if (average_mv == NULL) {
		return 0U;
	}

	if (g_vpp_batch_count >= TASK_VPP_BATCH_FRAMES ||
	    (g_vpp_batch_count > 0U && signature != g_vpp_batch_signature)) {
		Task_ResetVppBatch();
	}
	if (g_vpp_batch_count == 0U) {
		g_vpp_batch_signature = signature;
	}

	g_vpp_batch_mv[g_vpp_batch_count++] = candidate_mv;
	if (g_vpp_batch_count < TASK_VPP_BATCH_FRAMES) {
		return 0U;
	}

	float32_t sorted[TASK_VPP_BATCH_FRAMES];
	for (uint8_t i = 0U; i < TASK_VPP_BATCH_FRAMES; i++) {
		sorted[i] = g_vpp_batch_mv[i];
	}
	for (uint8_t i = 1U; i < TASK_VPP_BATCH_FRAMES; i++) {
		float32_t value = sorted[i];
		uint8_t j = i;
		while (j > 0U && sorted[j - 1U] > value) {
			sorted[j] = sorted[j - 1U];
			j--;
		}
		sorted[j] = value;
	}

	*average_mv = sorted[1];
	return 1U;
}

static void Task_InsertSpectralCandidate(Task_SpectralCandidate_t *candidates,
	                                      uint8_t *candidate_count,
	                                      Task_SpectralCandidate_t candidate)
{
	if (candidates == NULL || candidate_count == NULL) {
		return;
	}

	// 相邻搜索可能命中同一主瓣，只留较强记录。
	for (uint8_t i = 0U; i < *candidate_count; i++) {
		if (fabsf(candidates[i].freq_hz - candidate.freq_hz) < (0.5f * TASK_FREQ_GRID_HZ)) {
			if (candidate.amp_v > candidates[i].amp_v) {
				candidates[i] = candidate;
			}
			return;
		}
	}

	if (*candidate_count < TASK_SPECTRAL_CANDIDATE_MAX) {
		candidates[*candidate_count] = candidate;
		(*candidate_count)++;
		return;
	}

	uint8_t weakest = 0U;
	for (uint8_t i = 1U; i < TASK_SPECTRAL_CANDIDATE_MAX; i++) {
		if (candidates[i].amp_v < candidates[weakest].amp_v) {
			weakest = i;
		}
	}
	if (candidate.amp_v > candidates[weakest].amp_v) {
		candidates[weakest] = candidate;
	}
}

static void Task_SortCandidatesByFrequency(Task_SpectralCandidate_t *candidates, uint8_t candidate_count)
{
	for (uint8_t i = 0U; i < candidate_count; i++) {
		for (uint8_t j = (uint8_t)(i + 1U); j < candidate_count; j++) {
			if (candidates[j].freq_hz < candidates[i].freq_hz) {
				Task_SpectralCandidate_t temp = candidates[i];
				candidates[i] = candidates[j];
				candidates[j] = temp;
			}
		}
	}
}

static int Task_FindFundamentalBin(float32_t linear_gain)
{
	Task_SpectralCandidate_t candidates[TASK_SPECTRAL_CANDIDATE_MAX];
	uint8_t candidate_count = 0U;

	// 边界放宽半格，10 kHz 和 500 kHz 不能被抖动挡掉。
	int start_bin = (int)floorf(
	    (TASK_BASE_MIN_FREQ_HZ - TASK_FREQ_EDGE_TOLERANCE_HZ) / FFT_BIN_WIDTH_HZ);
	int end_bin = (int)ceilf(
	    (TASK_COMPONENT_MAX_FREQ_HZ + TASK_FREQ_EDGE_TOLERANCE_HZ) / FFT_BIN_WIDTH_HZ);
	if (start_bin < 1) {
		start_bin = 1;
	}
	if (end_bin > (int)FFT_BIN_COUNT - 2) {
		end_bin = (int)FFT_BIN_COUNT - 2;
	}

	for (int bin = start_bin; bin <= end_bin; bin++) {
		if (rfft_mag[bin] <= rfft_mag[bin - 1] || rfft_mag[bin] < rfft_mag[bin + 1]) {
			continue;
		}
		if (!Task_IsAboveNoiseFloor(rfft_mag[bin], linear_gain)) {
			continue;
		}

		float32_t freq_hz = 0.0f;
		float32_t amp_v = 0.0f;
		FFT_Rife_Interpolate(bin, &freq_hz, &amp_v, linear_gain);
		if (freq_hz < (TASK_BASE_MIN_FREQ_HZ - TASK_FREQ_EDGE_TOLERANCE_HZ) ||
		    freq_hz > (TASK_COMPONENT_MAX_FREQ_HZ + TASK_FREQ_EDGE_TOLERANCE_HZ) ||
		    amp_v < TASK_BASE_MIN_AMP_V) {
			continue;
		}

		Task_SpectralCandidate_t candidate;
		candidate.bin = bin;
		candidate.freq_hz = Task_SnapFrequencyToGrid(freq_hz);
		candidate.amp_v = amp_v;
		if (candidate.freq_hz < TASK_BASE_MIN_FREQ_HZ ||
		    candidate.freq_hz > TASK_COMPONENT_MAX_FREQ_HZ) {
			continue;
		}
		Task_InsertSpectralCandidate(candidates, &candidate_count, candidate);
	}

	if (candidate_count == 0U) {
		return -1;
	}

	Task_SortCandidatesByFrequency(candidates, candidate_count);

	int best_index = -1;
	uint8_t best_match_count = 0U;
	for (uint8_t i = 0U; i < candidate_count; i++) {
		// 250~500 kHz 只能作谐波，不能作题设基波。
		if (candidates[i].freq_hz < TASK_BASE_MIN_FREQ_HZ ||
		    candidates[i].freq_hz > TASK_BASE_MAX_FREQ_HZ) {
			continue;
		}

		uint32_t base_grid = (uint32_t)(candidates[i].freq_hz / TASK_FREQ_GRID_HZ + 0.5f);
		if (base_grid == 0U) {
			continue;
		}

		uint8_t match_count = 1U;
		for (uint8_t j = 0U; j < candidate_count; j++) {
			if (j == i || candidates[j].freq_hz <= candidates[i].freq_hz) {
				continue;
			}

			uint32_t candidate_grid = (uint32_t)(candidates[j].freq_hz / TASK_FREQ_GRID_HZ + 0.5f);
			uint32_t harmonic_order = (candidate_grid + base_grid / 2U) / base_grid;
			if (harmonic_order < 2U) {
				continue;
			}

			uint32_t expected_grid = harmonic_order * base_grid;
			uint32_t grid_error = (candidate_grid > expected_grid)
			                    ? (candidate_grid - expected_grid)
			                    : (expected_grid - candidate_grid);
			if (grid_error <= 1U) {
				match_count++;
			}
		}

		if (match_count > best_match_count) {
			best_match_count = match_count;
			best_index = (int)i;
		}
	}

	if (best_index >= 0 && best_match_count >= 2U) {
		return candidates[best_index].bin;
	}

	// 纯基波没有谐波族；此时只在题设基带内接受最强单峰。
	int strongest_base_index = -1;
	for (uint8_t i = 0U; i < candidate_count; i++) {
		if (candidates[i].freq_hz < TASK_BASE_MIN_FREQ_HZ ||
		    candidates[i].freq_hz > TASK_BASE_MAX_FREQ_HZ) {
			continue;
		}
		if (strongest_base_index < 0 ||
		    candidates[i].amp_v > candidates[strongest_base_index].amp_v) {
			strongest_base_index = (int)i;
		}
	}
	return (strongest_base_index >= 0) ? candidates[strongest_base_index].bin : -1;
}

void Task_RequestWaveRefresh(uint8_t wave_mode)
{
	g_requested_wave_mode = (wave_mode == 3U) ? 3U : 1U;
	g_requested_view = HMI_VIEW_WAVEFORM;
	g_decision_retry_count = 0U;
	g_vpp_batch_reset_pending = 1U;
	g_wave_refresh_pending = 1U;
}

void Task_RequestSpectrumRefresh(void)
{
	g_requested_view = HMI_VIEW_SPECTRUM;
	g_decision_retry_count = 0U;
	g_vpp_batch_reset_pending = 1U;
	g_wave_refresh_pending = 1U;
}

void Task_Signal_Processing(void)
{
	static uint32_t frame_cnt = 0;
#if TASK_DEBUG_AUTO_REFRESH
	// 调试刷新不改周期模式。
	g_wave_refresh_pending = 1U;
#endif
	if (g_wave_refresh_pending == 0U) {
		return;
	}

	uint8_t requested_wave_mode = (g_requested_wave_mode == 3U) ? 3U : 1U;
	uint8_t requested_view = (g_requested_view == HMI_VIEW_SPECTRUM)
	                       ? HMI_VIEW_SPECTRUM
	                       : HMI_VIEW_WAVEFORM;
	g_wave_refresh_pending = 0U;
	if (g_vpp_batch_reset_pending != 0U) {
		Task_ResetVppBatch();
		g_vpp_batch_reset_pending = 0U;
	}

	// 采样
	const float gain_snapshot = 1.0f;

	// 启动失败就退出，保留驱动层诊断现场。
	if (ADC_Measure_Start() != HAL_OK) {
		ADC_Measure_Stop();
		return;
	}

	// 等待完整 DMA 帧
	uint32_t wait_start = HAL_GetTick();
	while ((adc_buffer_ready == 0U) && ((HAL_GetTick() - wait_start) < TASK_SAMPLE_TIMEOUT_MS)) {
	}
	if (adc_buffer_ready == 0U) {
		ADC_Measure_Stop();
		return;
	}

	// 回调已冻结硬件；此处只取得帧处理权。
	if (ADC_Measure_TakeFrame() != HAL_OK) {
		ADC_Measure_Stop();
		return;
	}

	// 频域分析
	FFT_Process_Unpack(adc_dma_buf, gain_snapshot);

	// 选择能解释其他谱线的最低谐波族成员作为基波。
	int k_base = Task_FindFundamentalBin(gain_snapshot);
	if (k_base < 1) {
		// 本帧不可信，屏幕继续保留上一帧。
		ADC_Measure_ReleaseFrame();
		return;
	}

	// 整 bin 只负责定位，最终频率和幅值由 Rife 细化。
	float32_t f1_measured = 0.0f;
	float32_t A1 = 0.0f;
	FFT_Rife_Interpolate(k_base, &f1_measured, &A1, gain_snapshot);
	float32_t f1 = Task_SnapFrequencyToGrid(f1_measured);
	if (f1 < TASK_BASE_MIN_FREQ_HZ || f1 > TASK_BASE_MAX_FREQ_HZ) {
		ADC_Measure_ReleaseFrame();
		return;
	}
	// 幅值补偿暂关；启用时基波和谐波必须一起开。
	// A1 = SIG_Apply_LUT_Compensation(f1, A1);

	// 先写工作帧，全部成功后再发布。
	Task_ResetScreenData(&g_working_screen_data, requested_wave_mode);
	g_working_screen_data.f1_base_freq = f1;
	g_working_screen_data.harmonic_count = 1U;
	g_working_screen_data.harmonic_freq[0] = f1 / 1000.0f;
	g_working_screen_data.harmonic_amp[0] = A1 * 1000.0f;

	// 逐次搜索 n·f1，最多记录两个谐波且不超过 500 kHz。
	uint8_t h_count = 1U;
	for (uint8_t harmonic = 2U; ; harmonic++) {
		// 基波加两个谐波已满足题目上限。
		if (h_count >= 3U) {
			break;
		}

		// 对外频率必须是吸附后基波的整数倍。
		float32_t f_target = f1 * (float32_t)harmonic;

		// 后续次数只会更高，越过 500 kHz 即可结束。
		if (f_target > TASK_COMPONENT_MAX_FREQ_HZ) {
			break;
		}

		// 找峰用实测 f1，别用吸附值累积高次网格误差。
		float32_t f_target_measured = f1_measured * (float32_t)harmonic;
		int bin_target = (int)(f_target_measured / FFT_BIN_WIDTH_HZ + 0.5f);
		int k_h = FFT_Find_Local_Max_Bin(rfft_mag, bin_target - 2, bin_target + 2);
		if (k_h < 1) {
			// 目标附近没有峰，继续检查下一次谐波。
			continue;
		}

		// 先挡底噪，再做插值。
		if (!Task_IsAboveNoiseFloor(rfft_mag[k_h], gain_snapshot)) {
			// 太弱，不拟合。
			continue;
		}

		// 通过门限后再细化频率与幅值。
		float32_t fh = 0.0f;
		float32_t Ah = 0.0f;
		FFT_Rife_Interpolate(k_h, &fh, &Ah, gain_snapshot);
		// 与基波补偿保持同开同关。
		// Ah = SIG_Apply_LUT_Compensation(fh, Ah);

		// 细化后仍须贴近 n·f1，防止邻近干扰冒充谐波。
		if (fabsf(fh - f_target_measured) > TASK_HARMONIC_FREQ_TOLERANCE_HZ) {
			continue;
		}

		if (Ah * 1000.0f > 1.0f) {
			// 实测峰确认存在并给幅值；显示频率发布为 n·f1。
			g_working_screen_data.harmonic_freq[h_count] = f_target / 1000.0f;
			g_working_screen_data.harmonic_amp[h_count] = Ah * 1000.0f;
			h_count++;
		}
	}

	// 同一帧保留多种 Vpp，正式值在末尾按基频选择。
	g_working_screen_data.harmonic_count = h_count;
	g_working_screen_data.Vrms_val = SIG_Calculate_TrueRMS(g_working_screen_data.harmonic_amp, h_count);
	SIG_VppSet_t vpp_set;
	if (!SIG_Calculate_VppSet(adc_dma_buf, gain_snapshot, &vpp_set)) {
		ADC_Measure_ReleaseFrame();
		return;
	}
	g_vpp_debug.rank1_mv = vpp_set.rank1_v * 1000.0f;
	g_vpp_debug.rank3_mv = vpp_set.rank3_v * 1000.0f;
	g_vpp_debug.rank5_mv = vpp_set.rank5_v * 1000.0f;
	g_vpp_debug.rank7_mv = vpp_set.rank7_v * 1000.0f;
	g_vpp_debug.rank9_mv = vpp_set.rank9_v * 1000.0f;
	g_vpp_debug.rank13_mv = vpp_set.rank13_v * 1000.0f;
	g_vpp_debug.rank17_mv = vpp_set.rank17_v * 1000.0f;
	g_vpp_debug.rank21_mv = vpp_set.rank21_v * 1000.0f;
	g_vpp_debug.rank25_mv = vpp_set.rank25_v * 1000.0f;
	g_vpp_debug.rank31_mv = vpp_set.rank31_v * 1000.0f;
	g_vpp_debug.rank35_mv = vpp_set.rank35_v * 1000.0f;
	g_vpp_debug.rank41_mv = vpp_set.rank41_v * 1000.0f;
	g_vpp_debug.rank45_mv = vpp_set.rank45_v * 1000.0f;
	g_vpp_debug.rank51_mv = vpp_set.rank51_v * 1000.0f;
	g_vpp_debug.rank61_mv = vpp_set.rank61_v * 1000.0f;
	g_vpp_debug.rank81_mv = vpp_set.rank81_v * 1000.0f;
	g_vpp_debug.rank101_mv = vpp_set.rank101_v * 1000.0f;
	g_vpp_debug.rank121_mv = vpp_set.rank121_v * 1000.0f;
	g_vpp_debug.rank141_mv = vpp_set.rank141_v * 1000.0f;
	g_vpp_debug.rank161_mv = vpp_set.rank161_v * 1000.0f;
	g_vpp_debug.rank181_mv = vpp_set.rank181_v * 1000.0f;
	g_vpp_debug.rank200_mv = vpp_set.rank200_v * 1000.0f;
	g_vpp_debug.avg3_8_mv = vpp_set.avg3_8_v * 1000.0f;
	g_vpp_debug.avg5_12_mv = vpp_set.avg5_12_v * 1000.0f;
	g_vpp_debug.avg9_24_mv = vpp_set.avg9_24_v * 1000.0f;
	g_vpp_debug.avg17_32_mv = vpp_set.avg17_32_v * 1000.0f;
	g_vpp_debug.avg25_48_mv = vpp_set.avg25_48_v * 1000.0f;
	g_vpp_debug.avg33_64_mv = vpp_set.avg33_64_v * 1000.0f;
	g_vpp_debug.avg41_80_mv = vpp_set.avg41_80_v * 1000.0f;
	g_vpp_debug.avg61_120_mv = vpp_set.avg61_120_v * 1000.0f;
	g_vpp_debug.avg81_160_mv = vpp_set.avg81_160_v * 1000.0f;

	SIG_CycleVppSet_t cycle_set = { 0 };
	(void)SIG_Calculate_CycleVppSet(rfft_time_domain,
	                                FFT_LENGTH,
	                                FFT_SAMPLE_RATE_HZ,
	                                f1_measured,
	                                gain_snapshot,
	                                &cycle_set);
	g_vpp_debug.cycle_count = cycle_set.cycle_count;
	g_vpp_debug.cycle_mean_mv = cycle_set.mean_v * 1000.0f;
	g_vpp_debug.cycle_median_mv = cycle_set.median_v * 1000.0f;
	g_vpp_debug.cycle_trim10_mv = cycle_set.trim10_v * 1000.0f;
	g_vpp_debug.cycle_trim20_mv = cycle_set.trim20_v * 1000.0f;
	g_vpp_debug.cycle_trim30_mv = cycle_set.trim30_v * 1000.0f;
	g_vpp_debug.cycle_auto_mv = cycle_set.auto_v * 1000.0f;
	// 实测分类：三分量或低频两分量用 rank101，其余两分量用 rank3。
	uint8_t vpp_rank_region = Task_GetVppDecisionRegion(f1, h_count);
	g_working_screen_data.Vpp_val = (vpp_rank_region == 0U)
	                                ? g_vpp_debug.rank101_mv
	                                : g_vpp_debug.rank3_mv;
	// 先保存现场验证过的结果；拟合不可信时继续使用它。
	float32_t backup_vpp_mv = g_working_screen_data.Vpp_val;
	g_vpp_debug.backup_mv = backup_vpp_mv;
	g_vpp_debug.model_mv = 0.0f;
	g_vpp_debug.ols_residual_mv = 0.0f;
	g_vpp_debug.ols_residual_pct = 0.0f;
	g_vpp_debug.ols_amp_error_pct = 0.0f;
	g_vpp_debug.ols_pivot_ratio = 0.0f;
	g_vpp_debug.ols_interference_mv = 0.0f;
	g_vpp_debug.ols_used = 0U;
	for (uint8_t component = 0U; component < 4U; component++) {
		g_rel_phase_deg[component] = 0.0f;
	}

	// 拟合频率用实测 f1 的整数倍；吸附频率只负责对外显示。
	float32_t phase_freq_hz[4] = { 0.0f };
	for (uint8_t component = 0U; component < h_count; component++) {
		float32_t display_freq_hz = g_working_screen_data.harmonic_freq[component] * 1000.0f;
		uint32_t order = (uint32_t)(display_freq_hz / f1 + 0.5f);
		if (order < 1U) {
			order = 1U;
		}
		phase_freq_hz[component] = f1_measured * (float32_t)order;
	}

	// 第三问的 1 MHz 信号只作为干扰基函数，不参与输出波形。
	float32_t interference_freq_hz = 0.0f;
	float32_t detected_interference_amp_v = 0.0f;
	int interference_bin = (int)(TASK_INTERFERENCE_CENTER_HZ / FFT_BIN_WIDTH_HZ + 0.5f);
	int interference_peak_bin = FFT_Find_Local_Max_Bin(rfft_mag,
	                                                   interference_bin - 4,
	                                                   interference_bin + 4);
	if (interference_peak_bin > 0) {
		float32_t detected_freq_hz = 0.0f;
		FFT_Rife_Interpolate(interference_peak_bin,
		                     &detected_freq_hz,
		                     &detected_interference_amp_v,
		                     gain_snapshot);
		if (detected_interference_amp_v >= TASK_INTERFERENCE_MIN_AMP_V &&
		    fabsf(detected_freq_hz - TASK_INTERFERENCE_CENTER_HZ) <= TASK_INTERFERENCE_SEARCH_HALF_HZ) {
			interference_freq_hz = detected_freq_hz;
		}
	}

	SIG_HarmonicFitResult_t fit_result;
	uint8_t fit_solved = SIG_ReconstructHarmonicWaveOLS(rfft_time_domain,
	                                                    FFT_LENGTH,
	                                                    FFT_SAMPLE_RATE_HZ,
	                                                    phase_freq_hz,
	                                                    h_count,
	                                                    interference_freq_hz,
	                                                    requested_wave_mode,
	                                                    g_working_screen_data.wave_pixels,
	                                                    HMI_WAVE_SOURCE_POINTS,
	                                                    &fit_result);
	if (fit_solved) {
		float32_t max_amp_error_ratio = 0.0f;
		for (uint8_t component = 0U; component < h_count; component++) {
			float32_t reference_amp_v = g_working_screen_data.harmonic_amp[component] * 0.001f;
			if (reference_amp_v > 0.001f) {
				float32_t error_ratio =
				    fabsf(fit_result.component_amp_v[component] - reference_amp_v) / reference_amp_v;
				if (error_ratio > max_amp_error_ratio) {
					max_amp_error_ratio = error_ratio;
				}
			}
		}

		g_vpp_debug.model_mv = fit_result.model_vpp_v * 1000.0f;
		g_vpp_debug.ols_residual_mv = fit_result.residual_rms_v * 1000.0f;
		g_vpp_debug.ols_residual_pct = fit_result.residual_ratio * 100.0f;
		g_vpp_debug.ols_amp_error_pct = max_amp_error_ratio * 100.0f;
		g_vpp_debug.ols_pivot_ratio = fit_result.pivot_ratio;
		g_vpp_debug.ols_interference_mv = fit_result.interference_amp_v * 1000.0f;

		// OLS 只保留给波形重构和相位诊断，正式 Vpp 固定使用实测 rank 分段方案。
		g_working_screen_data.Vpp_val = backup_vpp_mv;
		g_vpp_debug.ols_used = 0U;

		for (uint8_t component = 0U; component < h_count; component++) {
			g_rel_phase_deg[component] =
			    fit_result.relative_phase_rad[component] * 57.29577951308232f;
		}
	} else {
		// 极少数病态帧仍可显示旧模型，但 Vpp 不离开保底方案。
		float32_t old_model_vpp_mv = 0.0f;
		float32_t old_relative_phase_rad[4] = { 0.0f };
		if (!SIG_ReconstructHarmonicWave(rfft_time_domain,
		                                 FFT_LENGTH,
		                                 FFT_SAMPLE_RATE_HZ,
		                                 phase_freq_hz,
		                                 g_working_screen_data.harmonic_amp,
		                                 h_count,
		                                 requested_wave_mode,
		                                 g_working_screen_data.wave_pixels,
		                                 HMI_WAVE_SOURCE_POINTS,
		                                 &old_model_vpp_mv,
		                                 old_relative_phase_rad)) {
			ADC_Measure_ReleaseFrame();
			return;
		}
		g_vpp_debug.model_mv = old_model_vpp_mv;
		g_working_screen_data.Vpp_val = backup_vpp_mv;
		for (uint8_t component = 0U; component < h_count; component++) {
			g_rel_phase_deg[component] =
			    old_relative_phase_rad[component] * 57.29577951308232f;
		}
	}

	uint8_t has_interference = (interference_freq_hz > 0.0f) ? 1U : 0U;
	float32_t candidate_vpp_mv = g_working_screen_data.Vpp_val;
	uint32_t candidate_signature = Task_BuildDecisionSignature(
	    &g_working_screen_data, f1, h_count, has_interference, g_vpp_debug.ols_used);
	uint8_t decision_accepted = Task_ConfirmDecision(candidate_signature);

	// 精简结构同时显示候选和已发布状态，便于确认防抖是否生效。
	g_ols_debug.solved = fit_solved;
	g_ols_debug.used = decision_accepted
	                   ? g_vpp_debug.ols_used
	                   : ((g_stable_decision_signature & TASK_DECISION_OLS_BIT) ? 1U : 0U);
	g_ols_debug.held = decision_accepted ? 0U : 1U;
	g_ols_debug.confirm_count = g_pending_decision_count;
	g_ols_debug.stable_signature = g_stable_decision_signature;
	g_ols_debug.candidate_signature = candidate_signature;
	g_ols_debug.display_mv = decision_accepted ? candidate_vpp_mv : g_screen_data.Vpp_val;
	g_ols_debug.candidate_mv = candidate_vpp_mv;
	g_ols_debug.backup_mv = backup_vpp_mv;
	g_ols_debug.model_mv = fit_solved ? (fit_result.model_vpp_v * 1000.0f) : 0.0f;
	g_ols_debug.correction_gain = fit_solved
	                              ? Task_GetOlsVppCorrection(f1, h_count, has_interference)
	                              : 1.0f;
	g_ols_debug.corrected_mv = g_ols_debug.model_mv * g_ols_debug.correction_gain;
	g_ols_debug.residual_mv = g_vpp_debug.ols_residual_mv;
	g_ols_debug.residual_pct = g_vpp_debug.ols_residual_pct;
	g_ols_debug.amp_error_pct = g_vpp_debug.ols_amp_error_pct;
	g_ols_debug.pivot_ratio = g_vpp_debug.ols_pivot_ratio;
	g_ols_debug.interference_mv = g_vpp_debug.ols_interference_mv;
	g_ols_debug.interference_detected_mv = detected_interference_amp_v * 1000.0f;
	g_ols_debug.phase2_deg = (fit_solved && h_count > 1U) ? g_rel_phase_deg[1] : 0.0f;
	g_ols_debug.phase3_deg = (fit_solved && h_count > 2U) ? g_rel_phase_deg[2] : 0.0f;

	if (!decision_accepted) {
		// 一次按键在后台补采；始终不稳定时停止，旧画面不变。
		if (g_decision_retry_count < TASK_DECISION_MAX_AUTO_RETRIES) {
			g_decision_retry_count++;
			g_wave_refresh_pending = 1U;
		}
		ADC_Measure_ReleaseFrame();
		return;
	}
	g_decision_retry_count = 0U;
	float32_t averaged_vpp_mv = 0.0f;
	if (!Task_CollectVppBatch(candidate_vpp_mv, candidate_signature, &averaged_vpp_mv)) {
		// 同一次按键继续补采，集满后才更新屏幕。
		g_ols_debug.display_mv = g_screen_data.Vpp_val;
		g_wave_refresh_pending = 1U;
		ADC_Measure_ReleaseFrame();
		return;
	}
	g_working_screen_data.Vpp_val = averaged_vpp_mv;
	g_ols_debug.display_mv = averaged_vpp_mv;

	// 频谱柱只表达已确认的分量。
	Task_UpdateSpectrumBars(&g_working_screen_data, h_count);

	// 到这里才提交；不按键或本帧失败都保持原画面。
	g_screen_data = g_working_screen_data;
	g_render_view = requested_view;
	SerialScreen_Render_Task(&g_screen_data);

	// FireWater 调试输出，交付版保持关闭。
#if TASK_DEBUG_FIREWATER_OUTPUT
	printf(":cnt,%lu,:f1,%.1f,:Vpp,%.3f,:Vrms,%.3f\n",
	       (unsigned long)frame_cnt,
	       g_screen_data.f1_base_freq / 1000.0f,
	       g_screen_data.Vpp_val / 1000.0f,
	       g_screen_data.Vrms_val / 1000.0f);
#endif
	frame_cnt++;
	ADC_Measure_ReleaseFrame();
}

void SerialScreen_Render_Task(const SerialScreenData_t *data)
{
	if (data == NULL) {
		return;
	}

	if (g_render_view == HMI_VIEW_SPECTRUM) {
		(void)HMI_SendSpectrum(data);
		return;
	}

	// page0：Vpp、Vrms 保留 1 位小数并带单位，基频显示整数。
	char command[40];
	uint32_t vpp_tenth_mv = (uint32_t)(data->Vpp_val * 10.0f + 0.5f);
	uint32_t vrms_tenth_mv = (uint32_t)(data->Vrms_val * 10.0f + 0.5f);
	uint32_t base_freq_hz = (uint32_t)(data->f1_base_freq + 0.5f);
	snprintf(command, sizeof(command), "t_vpp.txt=\"%lu.%lu mV\"",
	         (unsigned long)(vpp_tenth_mv / 10U),
	         (unsigned long)(vpp_tenth_mv % 10U));
	HMISendString(command);

	snprintf(command, sizeof(command), "t_vrms.txt=\"%lu.%lu mV\"",
	         (unsigned long)(vrms_tenth_mv / 10U),
	         (unsigned long)(vrms_tenth_mv % 10U));
	HMISendString(command);

	snprintf(command, sizeof(command), "t_f.txt=\"%lu Hz\"",
	         (unsigned long)base_freq_hz);
	HMISendString(command);

	(void)HMI_SendWaveform(data->wave_pixels, HMI_WAVE_SOURCE_POINTS);
}

void SerialScreen_Init(void)
{
	g_serial_cmd_length = 0U;
	g_serial_cmd_buffer[0] = '\0';
	g_hmi_addt_ready = 0U;
	g_hmi_addt_finished = 0U;
	(void)HAL_UART_Receive_IT(&huart1, &g_hmi_rx_byte, 1U);
}

void SerialScreen_Rx_Handler(uint8_t rx_byte)
{
	// addt 握手：0xFE 可发送原始字节，0xFD 表示屏幕收完。
	if (rx_byte == 0xFEU) {
		g_hmi_addt_ready = 1U;
		return;
	}
	if (rx_byte == 0xFDU) {
		g_hmi_addt_finished = 1U;
		return;
	}

	// 边收边匹配短命令。
	if (rx_byte == '\r' || rx_byte == '\n' || rx_byte == 0xFFU) {
		// 分隔符只负责清空残留命令。
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
		return;
	}

	// 未满则追加。
	if (g_serial_cmd_length < (sizeof(g_serial_cmd_buffer) - 1U)) {
		g_serial_cmd_buffer[g_serial_cmd_length++] = (char)rx_byte;
		g_serial_cmd_buffer[g_serial_cmd_length] = '\0';
	} else {
		// 满后丢最旧字节，保留最近命令。
		memmove(g_serial_cmd_buffer, &g_serial_cmd_buffer[1], sizeof(g_serial_cmd_buffer) - 2U);
		g_serial_cmd_buffer[sizeof(g_serial_cmd_buffer) - 2U] = (char)rx_byte;
		g_serial_cmd_buffer[sizeof(g_serial_cmd_buffer) - 1U] = '\0';
		g_serial_cmd_length = (uint8_t)(sizeof(g_serial_cmd_buffer) - 1U);
	}

	// 1、W1、T1：刷新单周期。
	if (rx_byte == '1' || strstr(g_serial_cmd_buffer, "W1") != NULL || strstr(g_serial_cmd_buffer, "T1") != NULL) {
		Task_RequestWaveRefresh(1U);
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
		return;
	}

	// 3、W3、T3：刷新三周期。
	if (rx_byte == '3' || strstr(g_serial_cmd_buffer, "W3") != NULL || strstr(g_serial_cmd_buffer, "T3") != NULL) {
		Task_RequestWaveRefresh(3U);
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
		return;
	}

	// S：刷新频谱页。
	if (rx_byte == 'S' || rx_byte == 's') {
		Task_RequestSpectrumRefresh();
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
	}
}

// printf 走 USART1；别在采样时序敏感区大量打印。
int fputc(int ch, FILE *f)
{
	(void)f;
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

// 陶晶驰串口屏

static void UART_SendByte(uint8_t byte)
{
	HAL_UART_Transmit(&huart1, &byte, 1, 0xFFFF);
}

// HMI 指令以三个 0xFF 结尾。
static void HMISendString(const char *str)
{
	while (*str) {
		UART_SendByte((uint8_t)*str++);
	}
	UART_SendByte(0xFF);
	UART_SendByte(0xFF);
	UART_SendByte(0xFF);
}

static HAL_StatusTypeDef HMI_SendWaveform(const uint16_t *wave_pixels, uint16_t source_count)
{
	if (wave_pixels == NULL || source_count < 2U) {
		return HAL_ERROR;
	}

	// s0 宽 800 像素；源点变化时这里仍负责重采样到 800 点。
	for (uint16_t i = 0U; i < HMI_WAVE_TX_POINTS; i++) {
		float32_t source_pos =
		    ((float32_t)i * (float32_t)(source_count - 1U)) /
		    (float32_t)(HMI_WAVE_TX_POINTS - 1U);
		uint16_t left = (uint16_t)source_pos;
		uint16_t right = (left + 1U < source_count) ? (left + 1U) : left;
		float32_t fraction = source_pos - (float32_t)left;
		float32_t pixel =
		    (float32_t)wave_pixels[left] +
		    ((float32_t)wave_pixels[right] - (float32_t)wave_pixels[left]) * fraction;

		if (pixel < 0.0f) {
			pixel = 0.0f;
		} else if (pixel > 255.0f) {
			pixel = 255.0f;
		}

		// 注意方向相反：Y 坐标越小越高，曲线数据值越大越高。
		g_hmi_wave_tx[i] = (uint8_t)(255.0f - pixel + 0.5f);
	}

	return HMI_SendCurveBytes(HMI_WAVE_TX_POINTS);
}

static HAL_StatusTypeDef HMI_SendSpectrum(const SerialScreenData_t *data)
{
	if (data == NULL) {
		return HAL_ERROR;
	}

	char command[40];
	float32_t max_amp = 0.0f;

	// page1：幅值保留 1 位小数，频率显示整数；单位由界面固定显示。
	for (uint8_t i = 0U; i < 3U; i++) {
		float32_t amp_mv = (i < data->harmonic_count) ? data->harmonic_amp[i] : 0.0f;
		float32_t freq_hz = (i < data->harmonic_count)
		                  ? data->harmonic_freq[i] * 1000.0f
		                  : 0.0f;
		uint32_t amp_tenth_mv = (uint32_t)(amp_mv * 10.0f + 0.5f);
		uint32_t freq_hz_int = (uint32_t)(freq_hz + 0.5f);

		snprintf(command, sizeof(command), "t_vp%u.txt=\"%lu.%lu\"",
		         (unsigned int)(i + 1U),
		         (unsigned long)(amp_tenth_mv / 10U),
		         (unsigned long)(amp_tenth_mv % 10U));
		HMISendString(command);

		snprintf(command, sizeof(command), "t_f%u.txt=\"%lu\"",
		         (unsigned int)(i + 1U),
		         (unsigned long)freq_hz_int);
		HMISendString(command);

		if (amp_mv > max_amp) {
			max_amp = amp_mv;
		}
	}

	// 10~500 kHz 映射到留边后的横轴；纵轴按本帧最大分量归一化。
	memset(g_hmi_wave_tx, 10, HMI_SPECTRUM_TX_POINTS);
	if (max_amp > 0.0f) {
		uint8_t component_count = (data->harmonic_count < 3U)
		                        ? data->harmonic_count
		                        : 3U;
		for (uint8_t i = 0U; i < component_count; i++) {
			float32_t freq_khz = data->harmonic_freq[i];
			if (freq_khz < 10.0f) {
				freq_khz = 10.0f;
			} else if (freq_khz > 500.0f) {
				freq_khz = 500.0f;
			}

			float32_t normalized_freq = (freq_khz - 10.0f) / 490.0f;
			uint16_t drawable_width =
			    HMI_SPECTRUM_TX_POINTS - 1U - 2U * HMI_SPECTRUM_SIDE_MARGIN;
			uint16_t x = (uint16_t)(
			    (float32_t)HMI_SPECTRUM_SIDE_MARGIN +
			    normalized_freq * (float32_t)drawable_width + 0.5f);
			float32_t ratio = data->harmonic_amp[i] / max_amp;
			if (ratio < 0.0f) {
				ratio = 0.0f;
			} else if (ratio > 1.0f) {
				ratio = 1.0f;
			}
			uint8_t peak = (uint8_t)(30.0f + ratio * 205.0f + 0.5f);
			if (x > 0U) {
				g_hmi_wave_tx[x - 1U] = peak;
			}
			g_hmi_wave_tx[x] = peak;
			if (x + 1U < HMI_SPECTRUM_TX_POINTS) {
				g_hmi_wave_tx[x + 1U] = peak;
			}
		}
	}

	return HMI_SendCurveBytes(HMI_SPECTRUM_TX_POINTS);
}

static HAL_StatusTypeDef HMI_SendCurveBytes(uint16_t point_count)
{
	if (point_count == 0U || point_count > HMI_WAVE_TX_POINTS) {
		return HAL_ERROR;
	}

	HMISendString("cle s0.id,0");

	char command[32];
	snprintf(command, sizeof(command), "addt s0.id,0,%u",
	         (unsigned int)point_count);
	g_hmi_addt_ready = 0U;
	g_hmi_addt_finished = 0U;
	HMISendString(command);

	uint32_t wait_start = HAL_GetTick();
	while ((g_hmi_addt_ready == 0U) &&
	       ((HAL_GetTick() - wait_start) < HMI_ADD_READY_TIMEOUT_MS)) {
	}
	if (g_hmi_addt_ready == 0U) {
		return HAL_TIMEOUT;
	}

	return HAL_UART_Transmit(&huart1,
	                         g_hmi_wave_tx,
	                         point_count,
	                         HMI_ADD_READY_TIMEOUT_MS);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart != NULL && huart->Instance == USART1) {
		SerialScreen_Rx_Handler(g_hmi_rx_byte);
		(void)HAL_UART_Receive_IT(&huart1, &g_hmi_rx_byte, 1U);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart != NULL && huart->Instance == USART1) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		(void)HAL_UART_Receive_IT(&huart1, &g_hmi_rx_byte, 1U);
	}
}

