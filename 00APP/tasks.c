#include "tasks.h"

#include <string.h>

#include "FFT.h"
#include "VCA821.h"
#include "adc_measure.h"
#include "sig_util.h"

// 屏幕显示数据放在 IRAM1，减少和 DMA/FFT 缓冲区的冲突。
SerialScreenData_t g_screen_data __attribute__((section(".iram1")));

// 串口命令缓冲：用于识别简单控制指令，例如波形显示模式切换。
static char g_serial_cmd_buffer[12];
static uint8_t g_serial_cmd_length = 0;

static void Task_ResetScreenData(void)
{
	// 清空本帧显示数据，但保留当前波形显示模式。
	uint8_t wave_mode = g_screen_data.wave_disp_mode;
	memset(&g_screen_data, 0, sizeof(g_screen_data));
	g_screen_data.wave_disp_mode = (wave_mode == 3U) ? 3U : 1U;
}

static void Task_UpdateSpectrumBars(uint8_t harmonic_count)
{
	// 把谐波幅值换算成屏幕柱状图高度。
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

void Task_Signal_Processing(void)
{
	// 1. 触发一次新的采样。
	ADC_Measure_Start();

	// 2. 等待 DMA 数据就绪，超时后直接进入下一轮。
	uint32_t wait_start = HAL_GetTick();
	while (adc_buffer_ready != 2U) {
		if ((HAL_GetTick() - wait_start) > 10U) {
			break;
		}
	}
	adc_buffer_ready = 0U;

	// 3. 先做 FFT，再找基波和谐波。
	float gain = VCA821_GetLinearGain();

	FFT_Process_Unpack(adc_dma_buf, gain);

	int k_base = FFT_Find_Max_Bin(rfft_mag, 5, 285);
	if (k_base < 1) {
		Task_ResetScreenData();
		return;
	}

	// 4. 估计基波频率和幅值，并做前端补偿。
	float32_t f1 = 0.0f;
	float32_t A1 = 0.0f;
	FFT_Rife_Interpolate(k_base, &f1, &A1);
	A1 = SIG_Apply_LUT_Compensation(f1, A1);

	Task_ResetScreenData();
	g_screen_data.f1_base_freq = f1;
	g_screen_data.harmonic_count = 1U;
	g_screen_data.harmonic_freq[0] = f1 / 1000.0f;
	g_screen_data.harmonic_amp[0] = A1 * 1000.0f;

	// 5. 继续搜索 2~4 次谐波。
	uint8_t h_count = 1U;
	for (uint8_t harmonic = 2U; harmonic <= 4U; harmonic++) {
		float32_t f_target = f1 * (float32_t)harmonic;
		if (f_target > 500000.0f) {
			break;
		}

		int bin_target = (int)(f_target / FFT_BIN_WIDTH_HZ);
		int k_h = FFT_Find_Local_Max_Bin(rfft_mag, bin_target - 3, bin_target + 3);
		if (k_h < 1) {
			continue;
		}

		float32_t fh = 0.0f;
		float32_t Ah = 0.0f;
		FFT_Rife_Interpolate(k_h, &fh, &Ah);
		Ah = SIG_Apply_LUT_Compensation(fh, Ah);

		if (Ah * 1000.0f > 3.0f && h_count < 4U) {
			g_screen_data.harmonic_freq[h_count] = fh / 1000.0f;
			g_screen_data.harmonic_amp[h_count] = Ah * 1000.0f;
			h_count++;
		}
	}

	// 6. 计算总 RMS / 峰峰值，并生成波形显示数据。
	g_screen_data.harmonic_count = h_count;
	g_screen_data.Vrms_val = SIG_Calculate_TrueRMS(g_screen_data.harmonic_amp, h_count);
	g_screen_data.Vpp_val = SIG_Calculate_Vpp(adc_dma_buf, gain) * 1000.0f;

	float32_t points_per_period = 7200000.0f / f1;
	if (points_per_period < 2.0f) {
		points_per_period = 2.0f;
	}
	uint16_t wave_points = (g_screen_data.wave_disp_mode == 3U) ? (uint16_t)(3.0f * points_per_period) : (uint16_t)points_per_period;
	if (wave_points < 2U) {
		wave_points = 2U;
	}
	if (wave_points > FFT_LENGTH) {
		wave_points = FFT_LENGTH;
	}

	int n_start = SIG_Find_Hysteresis_ZeroCrossing(rfft_input, 1.65f);
	if (n_start < 0) {
		n_start = 0;
	}
	if ((uint16_t)n_start + wave_points > FFT_LENGTH) {
		n_start = (int)(FFT_LENGTH - wave_points);
	}

	SIG_CubicSpline_Resample(&rfft_input[n_start], wave_points, g_screen_data.wave_pixels, 120U);
	Task_UpdateSpectrumBars(h_count);

	// 7. 把本帧结果交给屏幕发送层。
	SerialScreen_Render_Task(&g_screen_data);
}

void SerialScreen_Render_Task(const SerialScreenData_t *data)
{
	// 当前版本先预留接口，后续由串口显示协议实现真正发送。
	(void)data;
}

void SerialScreen_Rx_Handler(uint8_t rx_byte)
{
	// 接收一小段命令字符串，用于识别简单控制指令。
	if (rx_byte == '\r' || rx_byte == '\n' || rx_byte == 0xFFU) {
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
		return;
	}

	if (g_serial_cmd_length < (sizeof(g_serial_cmd_buffer) - 1U)) {
		g_serial_cmd_buffer[g_serial_cmd_length++] = (char)rx_byte;
		g_serial_cmd_buffer[g_serial_cmd_length] = '\0';
	} else {
		memmove(g_serial_cmd_buffer, &g_serial_cmd_buffer[1], sizeof(g_serial_cmd_buffer) - 2U);
		g_serial_cmd_buffer[sizeof(g_serial_cmd_buffer) - 2U] = (char)rx_byte;
		g_serial_cmd_buffer[sizeof(g_serial_cmd_buffer) - 1U] = '\0';
		g_serial_cmd_length = (uint8_t)(sizeof(g_serial_cmd_buffer) - 1U);
	}

	if (rx_byte == '1' || strstr(g_serial_cmd_buffer, "W1") != NULL || strstr(g_serial_cmd_buffer, "T1") != NULL) {
		g_screen_data.wave_disp_mode = 1U;
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
		return;
	}

	if (rx_byte == '3' || strstr(g_serial_cmd_buffer, "W3") != NULL || strstr(g_serial_cmd_buffer, "T3") != NULL) {
		g_screen_data.wave_disp_mode = 3U;
		g_serial_cmd_length = 0U;
		g_serial_cmd_buffer[0] = '\0';
	}
}




