#include "tasks.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FFT.h"
#include "VCA821.h"
#include "adc_measure.h"
#include "sig_util.h"

// 调试阶段设为 1：主循环每次都自动请求一帧，方便在监视窗口连续观察数据。
// 串口屏接入后改为 0：恢复为仅收到 W1/W3 时刷新一次静态帧。
#define TASK_DEBUG_AUTO_REFRESH 1

// 屏幕显示数据放在 IRAM1，减少和 DMA/FFT 缓冲区的冲突。
SerialScreenData_t g_screen_data __attribute__((section(".iram1")));

// 保命参数：采样等待超时和谐波噪声门限。
static const uint32_t TASK_SAMPLE_TIMEOUT_MS = 10U;
static const float32_t TASK_HARMONIC_NOISE_FLOOR_V = 0.002f;
static const float32_t TASK_BASE_MIN_AMP_V = 0.003f;
static const float32_t TASK_FREQ_GRID_HZ = 500.0f;
static const float32_t TASK_SIGNAL_MIN_FREQ_HZ = 10000.0f;
static const float32_t TASK_SIGNAL_MAX_FREQ_HZ = 500000.0f;

#define TASK_SPECTRAL_CANDIDATE_MAX 12U

typedef struct {
	int bin;
	float32_t freq_hz;
	float32_t amp_v;
} Task_SpectralCandidate_t;

// 屏幕只发布完整成功的一帧；处理中途失败时继续保留上一帧。
static SerialScreenData_t g_working_screen_data __attribute__((section(".iram1")));

// 未来可由串口接收中断写入。主循环只在 pending=1 时采样一次。
static volatile uint8_t g_wave_refresh_pending = 0U;
static volatile uint8_t g_requested_wave_mode = 1U;

// 串口命令缓冲：用于识别简单控制指令，例如波形显示模式切换。
static char g_serial_cmd_buffer[12];
static uint8_t g_serial_cmd_length = 0;

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

	// 这里把“第几个谐波 + 它有多强”转成屏幕上要画的柱子位置和高度。
	// X 坐标表示第几个谐波，Y 坐标表示柱子的高度。
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
	// 这一步是“防误判”的保险丝：先估算这个峰大约有多大，再决定它是不是有效谐波。
	// 如果算出来的幅值太小，就直接认为它只是噪声，不继续做精细插值。
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

static void Task_InsertSpectralCandidate(Task_SpectralCandidate_t *candidates,
	                                      uint8_t *candidate_count,
	                                      Task_SpectralCandidate_t candidate)
{
	if (candidates == NULL || candidate_count == NULL) {
		return;
	}

	// 同一个主瓣可能在邻近搜索中被重复细化到同一条谱线，只保留幅值更大的记录。
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

	int start_bin = (int)ceilf(TASK_SIGNAL_MIN_FREQ_HZ / FFT_BIN_WIDTH_HZ);
	int end_bin = (int)floorf(TASK_SIGNAL_MAX_FREQ_HZ / FFT_BIN_WIDTH_HZ);
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
		if (freq_hz < TASK_SIGNAL_MIN_FREQ_HZ || freq_hz > TASK_SIGNAL_MAX_FREQ_HZ || amp_v < TASK_BASE_MIN_AMP_V) {
			continue;
		}

		Task_SpectralCandidate_t candidate;
		candidate.bin = bin;
		candidate.freq_hz = Task_SnapFrequencyToGrid(freq_hz);
		candidate.amp_v = amp_v;
		Task_InsertSpectralCandidate(candidates, &candidate_count, candidate);
	}

	if (candidate_count == 0U) {
		return -1;
	}

	Task_SortCandidatesByFrequency(candidates, candidate_count);

	int best_index = -1;
	uint8_t best_match_count = 0U;
	for (uint8_t i = 0U; i < candidate_count; i++) {
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

	// 题目信号至少含一个谐波；若本帧不能形成谐波族，则不发布错误基波。
	if (best_index < 0 || best_match_count < 2U) {
		return -1;
	}
	return candidates[best_index].bin;
}

void Task_RequestWaveRefresh(uint8_t wave_mode)
{
	g_requested_wave_mode = (wave_mode == 3U) ? 3U : 1U;
	g_wave_refresh_pending = 1U;
}

void Task_Signal_Processing(void)
{
	static uint32_t frame_cnt = 0;
#if TASK_DEBUG_AUTO_REFRESH
	// 只补充刷新请求，不覆盖 requested_wave_mode；以后即使接入 W3，自动调试也能保持三周期模式。
	g_wave_refresh_pending = 1U;
#endif
	if (g_wave_refresh_pending == 0U) {
		return;
	}

	uint8_t requested_wave_mode = (g_requested_wave_mode == 3U) ? 3U : 1U;
	g_wave_refresh_pending = 0U;

	// ===== 第一段：准备采样 =====
	float gain_snapshot = VCA821_GetLinearGain();

	// 单次触发 DMA。启动步骤失败时直接保留诊断现场，不进入无意义的等待。
	if (ADC_Measure_Start() != HAL_OK) {
		ADC_Measure_Stop();
		return;
	}

	// ===== 第二段：等待 DMA 完成 =====
	uint32_t wait_start = HAL_GetTick();
	while ((adc_buffer_ready == 0U) && ((HAL_GetTick() - wait_start) < TASK_SAMPLE_TIMEOUT_MS)) {
	}
	if (adc_buffer_ready == 0U) {
		ADC_Measure_Stop();
		return;
	}

	// DMA 回调已经冻结硬件；这里只同步 HAL 句柄并取得这一帧的独占处理权。
	if (ADC_Measure_TakeFrame() != HAL_OK) {
		ADC_Measure_Stop();
		return;
	}

	// ===== 第三段：做频域分析 =====
	// DMA 已完成，直接处理 adc_dma_buf。
	FFT_Process_Unpack(adc_dma_buf, gain_snapshot);

	// 搜集有效谱线，并选择能解释其他谱线的最低谐波族成员作为基波。
	int k_base = Task_FindFundamentalBin(gain_snapshot);
	if (k_base < 1) {
		// 本帧不能形成可信的“基波 + 谐波族”，保留屏幕上的上一帧。
		ADC_Measure_ReleaseFrame();
		return;
	}

	// 用 Rife 插值把基波频率和幅值再细化一下。
	// 这样比直接拿整 bin 的峰值更准确。
	float32_t f1_measured = 0.0f;
	float32_t A1 = 0.0f;
	FFT_Rife_Interpolate(k_base, &f1_measured, &A1, gain_snapshot);
	float32_t f1 = Task_SnapFrequencyToGrid(f1_measured);
	if (f1 < TASK_SIGNAL_MIN_FREQ_HZ || f1 > TASK_SIGNAL_MAX_FREQ_HZ) {
		ADC_Measure_ReleaseFrame();
		return;
	}
	// 幅值补偿暂关：先用未经 LUT 修正的结果做验证。
	// A1 = SIG_Apply_LUT_Compensation(f1, A1);

	// ===== 第四段：组织显示用的基础结果 =====
	// 所有结果先写工作缓存，成功后才整体发布，失败不破坏上一帧。
	Task_ResetScreenData(&g_working_screen_data, requested_wave_mode);
	g_working_screen_data.f1_base_freq = f1;
	g_working_screen_data.harmonic_count = 1U;
	g_working_screen_data.harmonic_freq[0] = f1 / 1000.0f;
	g_working_screen_data.harmonic_amp[0] = A1 * 1000.0f;

	// ===== 第五段：搜索谐波（动态遍历，无硬编码限制） =====
	// 按 2f1, 3f1, 4f1, 5f1, ... 逐个找谐波。
	// 终止条件两个：
	//   1. f_target > 500kHz — 超出题目规定范围
	//   2. 已找满 2 个谐波（题目明确最多 2 个谐波分量）
	// 先局部寻峰，再做噪声门限判断，最后才做精细插值。
	uint8_t h_count = 1U;
	for (uint8_t harmonic = 2U; ; harmonic++) {
		// 终止条件一：已找满 2 个谐波（基波+谐波≥3），直接退出。
		if (h_count >= 3U) {
			break;
		}

		// 理论谐波频率。
		float32_t f_target = f1 * (float32_t)harmonic;

		// 终止条件二：超出题目规定的 500kHz 上限，不再继续搜索。
		if (f_target > 500000.0f) {
			break;
		}

		// 在理论位置附近找一个局部最大值。
		int bin_target = (int)(f_target / FFT_BIN_WIDTH_HZ);
		int k_h = FFT_Find_Local_Max_Bin(rfft_mag, bin_target - 3, bin_target + 3);
		if (k_h < 1) {
			// 这附近根本没有像样的峰，说明该次谐波不存在或太弱。
			continue;
		}

		// 先做噪声门限判断，避免把底噪当谐波。
		if (!Task_IsAboveNoiseFloor(rfft_mag[k_h], gain_snapshot)) {
			// 峰太小就直接忽略，不再做 Rife 拟合。
			continue;
		}

		// 通过门限后，再用 Rife 算出更准确的谐波频率和幅值。
		float32_t fh = 0.0f;
		float32_t Ah = 0.0f;
		FFT_Rife_Interpolate(k_h, &fh, &Ah, gain_snapshot);
		// 同样做一次补偿，保持各次谐波口径一致（暂关）。
		// Ah = SIG_Apply_LUT_Compensation(fh, Ah);

		if (Ah * 1000.0f > 1.0f) {
			// 只有幅值足够明显的谐波才写入屏幕数据。
			fh = Task_SnapFrequencyToGrid(fh);
			g_working_screen_data.harmonic_freq[h_count] = fh / 1000.0f;
			g_working_screen_data.harmonic_amp[h_count] = Ah * 1000.0f;
			h_count++;
		}
	}

	// ===== 第六段：计算时域指标 =====
	// 谐波幅值已经有了，接下来把它们合成总 RMS。
	// Vpp 则直接从原始 DMA 数据里统计最大最小值得到。
	g_working_screen_data.harmonic_count = h_count;
	g_working_screen_data.Vrms_val = SIG_Calculate_TrueRMS(g_working_screen_data.harmonic_amp, h_count);
	g_working_screen_data.Vpp_val = SIG_Calculate_Vpp(adc_dma_buf, gain_snapshot) * 1000.0f;

	// ===== 第七段：生成波形显示数据 =====
	// 不再寻找零点。静态帧可从任意相位开始，只需保证横向跨度精确为 1/3 周期。
	float32_t points_per_period = FFT_SAMPLE_RATE_HZ / f1;
	float32_t period_count = (requested_wave_mode == 3U) ? 3.0f : 1.0f;
	float32_t wave_span = period_count * points_per_period;
	float32_t max_span = (float32_t)(FFT_LENGTH - 1U);
	if (wave_span > max_span) {
		wave_span = max_span;
	}
	float32_t wave_start = (max_span - wave_span) * 0.5f;

	// 浮点起点 + 浮点跨度直接生成 120 点，避免整数截断导致周期不足。
	SIG_CubicSpline_ResampleWindow(rfft_time_domain,
	                               FFT_LENGTH,
	                               wave_start,
	                               wave_span,
	                               g_working_screen_data.wave_pixels,
	                               120U);
	// 柱状图则由谐波幅值直接生成。
	Task_UpdateSpectrumBars(&g_working_screen_data, h_count);

	// ===== 第八段：交给显示层 =====
	// 只有完整成功的一帧才覆盖当前显示；不按键时该缓存保持不变。
	g_screen_data = g_working_screen_data;
	SerialScreen_Render_Task(&g_screen_data);

	// ===== FireWater 协议输出 =====
	// 格式：每行逗号分隔，换行触发 FireWater 绘图。
	// 计数测试：:frame, … , f1_kHz, Vpp_V, Vrms_V
	printf(":cnt,%lu,:f1,%.1f,:Vpp,%.3f,:Vrms,%.3f\n",
	       (unsigned long)frame_cnt,
	       g_screen_data.f1_base_freq / 1000.0f,
	       g_screen_data.Vpp_val / 1000.0f,
	       g_screen_data.Vrms_val / 1000.0f);
	frame_cnt++;
	ADC_Measure_ReleaseFrame();
}

void SerialScreen_Render_Task(const SerialScreenData_t *data)
{
	// 当前版本先预留接口，后续由串口显示协议实现真正发送。
	(void)data;
}

void SerialScreen_Rx_Handler(uint8_t rx_byte)
{
	// 串口命令是“边收边拼”的，方便识别类似 W1 / W3 这种简单控制指令。
	if (rx_byte == '\r' || rx_byte == '\n' || rx_byte == 0xFFU) {
		// 一行命令结束，清空缓冲区，准备接收下一条。
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
		return;
	}

	// 缓冲区没满就直接追加。
	if (g_serial_cmd_length < (sizeof(g_serial_cmd_buffer) - 1U)) {
		g_serial_cmd_buffer[g_serial_cmd_length++] = (char)rx_byte;
		g_serial_cmd_buffer[g_serial_cmd_length] = '\0';
	} else {
		// 满了就左移一位，保留最近收到的命令内容。
		memmove(g_serial_cmd_buffer, &g_serial_cmd_buffer[1], sizeof(g_serial_cmd_buffer) - 2U);
		g_serial_cmd_buffer[sizeof(g_serial_cmd_buffer) - 2U] = (char)rx_byte;
		g_serial_cmd_buffer[sizeof(g_serial_cmd_buffer) - 1U] = '\0';
		g_serial_cmd_length = (uint8_t)(sizeof(g_serial_cmd_buffer) - 1U);
	}

	// 收到 1 或 W1 / T1，就切到单周期显示。
	if (rx_byte == '1' || strstr(g_serial_cmd_buffer, "W1") != NULL || strstr(g_serial_cmd_buffer, "T1") != NULL) {
		Task_RequestWaveRefresh(1U);
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
		return;
	}

	// 收到 3 或 W3 / T3，就切到三周期显示。
	if (rx_byte == '3' || strstr(g_serial_cmd_buffer, "W3") != NULL || strstr(g_serial_cmd_buffer, "T3") != NULL) {
		Task_RequestWaveRefresh(3U);
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
	}
}

/*
 * printf 串口重定向（前提：CubeMX 中配好 USART1，hal_conf.h 中启用 HAL_UART_MODULE_ENABLED）
 * 配置完成后，printf / SerialScreen_Render_Task 均可直接发数据到串口助手。
 */
extern UART_HandleTypeDef huart1;

int fputc(int ch, FILE *f)
{
	(void)f;
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

// ==================== 陶晶驰串口屏 ====================

static void UART_SendByte(uint8_t byte)
{
	HAL_UART_Transmit(&huart1, &byte, 1, 0xFFFF);
}

// 发送指令到陶晶驰串口屏（ASCII 串 + 3 字节 0xFF 帧尾）
void HMISendString(char *str)
{
	while (*str) {
		UART_SendByte((uint8_t)*str++);
	}
	UART_SendByte(0xFF);
	UART_SendByte(0xFF);
	UART_SendByte(0xFF);
}

