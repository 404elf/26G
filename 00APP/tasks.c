#include "tasks.h"

#include <string.h>

#include "FFT.h"
#include "VCA821.h"
#include "adc_measure.h"
#include "sig_util.h"

// 屏幕显示数据放在 IRAM1，减少和 DMA/FFT 缓冲区的冲突。
SerialScreenData_t g_screen_data __attribute__((section(".iram1")));

// 保命参数：采样等待超时和谐波噪声门限。
static const uint32_t TASK_SAMPLE_TIMEOUT_MS = 10U;
static const float32_t TASK_HARMONIC_NOISE_FLOOR_V = 0.002f;

// 串口命令缓冲：用于识别简单控制指令，例如波形显示模式切换。
static char g_serial_cmd_buffer[12];
static uint8_t g_serial_cmd_length = 0;

static void Task_ResetScreenData(void)
{
	// 每一帧开始前先清掉上一帧的结果，避免旧数据残留到屏幕上。
	// 这里只保留波形显示模式，因为这是用户手动切换的状态。
	uint8_t wave_mode = g_screen_data.wave_disp_mode;
	memset(&g_screen_data, 0, sizeof(g_screen_data));
	g_screen_data.wave_disp_mode = (wave_mode == 3U) ? 3U : 1U;
}

static void Task_UpdateSpectrumBars(uint8_t harmonic_count)
{
	// 这里把“第几个谐波 + 它有多强”转成屏幕上要画的柱子位置和高度。
	// X 坐标表示第几个谐波，Y 坐标表示柱子的高度。
	for (uint8_t i = 0; i < 4U; i++) {
		if (i < harmonic_count) {
			g_screen_data.spec_bar_x[i] = (uint16_t)(24U + (uint16_t)i * 28U);
			float amp = g_screen_data.harmonic_amp[i];
			if (amp < 0.0f) {
				amp = 0.0f;
			}
			if (amp > 240.0f) {
				amp = 240.0f;
			}
			g_screen_data.spec_bar_y[i] = (uint16_t)(amp + 0.5f);
		} else {
			g_screen_data.spec_bar_x[i] = 0U;
			g_screen_data.spec_bar_y[i] = 0U;
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

void Task_Signal_Processing(void)
{
	// ===== 第一段：准备采样 =====
	float gain_snapshot = VCA821_GetLinearGain();

	// 单次触发 DMA，等它跑完再处理。
	ADC_Measure_Start();

	// ===== 第二段：等待 DMA 完成 =====
	uint32_t wait_start = HAL_GetTick();
	while ((adc_buffer_ready == 0U) && ((HAL_GetTick() - wait_start) < TASK_SAMPLE_TIMEOUT_MS)) {
	}
	if (adc_buffer_ready == 0U) {
		ADC_Measure_Stop();
		ADC_Measure_Init();
		Task_ResetScreenData();
		return;
	}
	adc_buffer_ready = 0U;

	// ===== 第三段：做频域分析 =====
	// DMA 已完成，直接处理 adc_dma_buf。
	FFT_Process_Unpack(adc_dma_buf, gain_snapshot);

	// 先在基波可能出现的范围里找最大峰。
	// 这里得到的是“最像基波”的频点，不是最终精确值。
	int k_base = FFT_Find_Max_Bin(rfft_mag, 5, 285);
	if (k_base < 1) {
		// 连基波都没找到，说明这一帧数据不可靠或者信号过弱，直接清屏并返回。
		Task_ResetScreenData();
		return;
	}

	// 用 Rife 插值把基波频率和幅值再细化一下。
	// 这样比直接拿整 bin 的峰值更准确。
	float32_t f1 = 0.0f;
	float32_t A1 = 0.0f;
	FFT_Rife_Interpolate(k_base, &f1, &A1, gain_snapshot);
	// 幅值补偿暂关：先用未经 LUT 修正的结果做验证。
	// A1 = SIG_Apply_LUT_Compensation(f1, A1);

	// ===== 第四段：组织显示用的基础结果 =====
	// 先清空上一帧结果，再把基波写进屏幕数据结构。
	Task_ResetScreenData();
	g_screen_data.f1_base_freq = f1;
	g_screen_data.harmonic_count = 1U;
	g_screen_data.harmonic_freq[0] = f1 / 1000.0f;
	g_screen_data.harmonic_amp[0] = A1 * 1000.0f;

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

		if (Ah * 1000.0f > 5.0f) {
			// 只有幅值足够明显的谐波才写入屏幕数据。
			g_screen_data.harmonic_freq[h_count] = fh / 1000.0f;
			g_screen_data.harmonic_amp[h_count] = Ah * 1000.0f;
			h_count++;
		}
	}

	// ===== 第六段：计算时域指标 =====
	// 谐波幅值已经有了，接下来把它们合成总 RMS。
	// Vpp 则直接从原始 DMA 数据里统计最大最小值得到。
	g_screen_data.harmonic_count = h_count;
	g_screen_data.Vrms_val = SIG_Calculate_TrueRMS(g_screen_data.harmonic_amp, h_count);
	g_screen_data.Vpp_val = SIG_Calculate_Vpp(adc_dma_buf, gain_snapshot) * 1000.0f;

	// ===== 第七段：生成波形显示数据 =====
	// 根据基波周期决定屏幕上要画多少个点。
	// 普通模式画 1 个周期，某些模式下画 3 个周期。
	float32_t points_per_period = FFT_SAMPLE_RATE_HZ / f1;
	if (points_per_period < 2.0f) {
		points_per_period = 2.0f;
	}
	uint16_t wave_points = (g_screen_data.wave_disp_mode == 3U) ? (uint16_t)(3.0f * points_per_period) : (uint16_t)points_per_period;
	// 波形点数太少会看不清，太多则没必要。
	if (wave_points < 2U) {
		wave_points = 2U;
	}
	if (wave_points > FFT_LENGTH) {
		wave_points = FFT_LENGTH;
	}

	// 从一个比较稳定的零交点开始截取波形，方便屏幕上显示对齐。
	// 时域信号已去直流偏置，零均值 → 用 0.0V 作为过零阈值。
	int n_start = SIG_Find_Hysteresis_ZeroCrossing(rfft_time_domain, 0.0f);
	if (n_start < 0) {
		// 找不到合适零交点时，就从头开始。
		n_start = 0;
	}
	if ((uint16_t)n_start + wave_points > FFT_LENGTH) {
		// 保证截取长度不会越界。
		n_start = (int)(FFT_LENGTH - wave_points);
	}

	// 把原始波形重采样成 120 个屏幕点。
	// 这样屏幕上看到的波形更平滑，也更适合显示。
	SIG_CubicSpline_Resample(&rfft_time_domain[n_start], wave_points, g_screen_data.wave_pixels, 120U);
	// 柱状图则由谐波幅值直接生成。
	Task_UpdateSpectrumBars(h_count);

	// ===== 第八段：交给显示层 =====
	// 这一层只负责把整理好的结果交出去，不关心具体协议怎么发。
	SerialScreen_Render_Task(&g_screen_data);
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
		g_screen_data.wave_disp_mode = 1U;
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
		return;
	}

	// 收到 3 或 W3 / T3，就切到三周期显示。
	if (rx_byte == '3' || strstr(g_serial_cmd_buffer, "W3") != NULL || strstr(g_serial_cmd_buffer, "T3") != NULL) {
		g_screen_data.wave_disp_mode = 3U;
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
	}
}




