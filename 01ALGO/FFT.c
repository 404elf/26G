#include "FFT.h"

#include <math.h>

#include "adc_measure.h"
#include "VCA821.h"

// FFT 运行实例，初始化后供后续每帧复用。
arm_rfft_fast_instance_f32 g_rfft_instance;
// 加窗后的时域输入放在 CCMRAM 中，供 FFT 使用。
float32_t rfft_input[FFT_LENGTH] __attribute__((section(".ccmram")));
// 去直流、未加窗的纯交流时域信号（供过零触发和波形显示使用）。
float32_t rfft_time_domain[FFT_LENGTH] __attribute__((section(".ccmram")));
// 复数域临时输出缓冲（IRAM1，FFT 时 CPU 独占，无需 CCMRAM）。
float32_t rfft_output_complex[FFT_LENGTH];
// 幅值谱，后续基波/谐波搜索都直接读这个数组。
float32_t rfft_mag[FFT_BIN_COUNT] __attribute__((section(".ccmram")));

// 增益保护：避免前端增益异常时把幅值算坏。
static float32_t FFT_ClampGain(float32_t gain)
{
  return (gain > 0.0f) ? gain : 1.0f;
}

void FFT_Init(void)
{
  // 只初始化一次 FFT 实例，后续帧循环复用。
  (void)arm_rfft_fast_init_f32(&g_rfft_instance, FFT_LENGTH);
}

// 搜索 bin 时先把下标夹紧到合法范围，避免越界。
static int FFT_ClampBinRange(int bin)
{
  if (bin < 0) {
    return 0;
  }
  if (bin > (int)FFT_BIN_COUNT - 1) {
    return (int)FFT_BIN_COUNT - 1;
  }
  return bin;
}

int FFT_Find_Max_Bin(const float32_t *mag, int start_bin, int end_bin)
{
  if (mag == NULL) {
    return -1;
  }

  start_bin = FFT_ClampBinRange(start_bin);
  end_bin = FFT_ClampBinRange(end_bin);
  if (end_bin < start_bin) {
    return -1;
  }

  int best_bin = start_bin;
  float32_t best_mag = mag[start_bin];
  for (int bin = start_bin + 1; bin <= end_bin; bin++) {
    if (mag[bin] > best_mag) {
      best_mag = mag[bin];
      best_bin = bin;
    }
  }
  return best_bin;
}

int FFT_Find_Local_Max_Bin(const float32_t *mag, int start_bin, int end_bin)
{
  return FFT_Find_Max_Bin(mag, start_bin, end_bin);
}

void FFT_Process_Unpack(const uint32_t *dma_buf, float linear_gain)
{
  if (dma_buf == NULL) {
    return;
  }

  // ===== 第一遍：解包并累加，求本帧真实直流偏置 =====
  float32_t v_sum = 0.0f;
  for (uint32_t word_index = 0; word_index < ADC_DMA_WORDS; word_index++) {
    uint32_t packed = dma_buf[word_index];
    uint16_t sample_early = (uint16_t)(packed & 0xFFFFU) & 0x0FFFU;
    uint16_t sample_late = (uint16_t)((packed >> 16U) & 0xFFFFU) & 0x0FFFU;

    uint32_t sample_index = word_index * 2U;
    float32_t v_early = (float32_t)sample_early * 3.3f / 4096.0f;
    float32_t v_late  = (float32_t)sample_late  * 3.3f / 4096.0f;

    // 暂存原始电压值，用于第二遍去直流。
    rfft_input[sample_index]     = v_early;
    rfft_input[sample_index + 1U] = v_late;
    v_sum += v_early + v_late;
  }

  // 本帧真实平均直流偏置（适应硬件容差和温漂）。
  float32_t V_avg = v_sum / (float32_t)FFT_LENGTH;
  (void)linear_gain;

  // ===== 第二遍：去直流归零 → 存 time_domain；加 Hann 窗 → 存 rfft_input =====
  for (uint32_t i = 0; i < FFT_LENGTH; i++) {
    // 减去直流偏置，得到零均值纯交流信号。
    float32_t ac_v = rfft_input[i] - V_avg;

    // 未加窗版本：留给过零触发和波形显示。
    rfft_time_domain[i] = ac_v;

    // 加 Hann 窗版本：留给 FFT 频域分析。
    float32_t window = 0.5f * (1.0f - arm_cos_f32((2.0f * 3.14159265358979323846f * (float32_t)i) / (float32_t)FFT_LENGTH));
    rfft_input[i] = ac_v * window;
  }

  arm_rfft_fast_f32(&g_rfft_instance, rfft_input, rfft_output_complex, 0);

  // 实数 FFT 的特殊输出格式：第 0 项和 Nyquist 点单独处理，其余为复数对。
  rfft_mag[0] = fabsf(rfft_output_complex[0]);
  rfft_mag[FFT_BIN_COUNT - 1U] = fabsf(rfft_output_complex[1]);
  arm_cmplx_mag_f32(&rfft_output_complex[2], &rfft_mag[1], FFT_BIN_COUNT - 1U);
}

void FFT_Rife_Interpolate(int bin_center, float32_t *out_freq, float32_t *out_amp)
{
  if (out_freq == NULL || out_amp == NULL) {
    return;
  }

  if (bin_center < 1) {
    bin_center = 1;
  }
  if (bin_center > (int)FFT_BIN_COUNT - 2) {
    bin_center = (int)FFT_BIN_COUNT - 2;
  }

  int search_left = bin_center - 3;
  int search_right = bin_center + 3;
  if (search_left < 1) {
    search_left = 1;
  }
  if (search_right > (int)FFT_BIN_COUNT - 2) {
    search_right = (int)FFT_BIN_COUNT - 2;
  }

  int k_max = FFT_Find_Max_Bin(rfft_mag, search_left, search_right);
  if (k_max < 1) {
    *out_freq = 0.0f;
    *out_amp = 0.0f;
    return;
  }

  float32_t a_max = rfft_mag[k_max];
  float32_t a_left = rfft_mag[k_max - 1];
  float32_t a_right = rfft_mag[k_max + 1];

  // 先判断峰值偏向左还是右，再做 Rife 细化。
  float32_t a_sub = a_right;
  int direction = 1;
  if (a_left >= a_right) {
    a_sub = a_left;
    direction = -1;
  }

  if (a_max <= 0.0f) {
    *out_freq = (float32_t)k_max * FFT_BIN_WIDTH_HZ;
    *out_amp = 0.0f;
    return;
  }

  float32_t r = a_sub / a_max;
  if (r < 0.0f) {
    r = 0.0f;
  }

  float32_t delta = (2.0f * r - 1.0f) / (r + 1.0f);
  if (delta < 0.0f) {
    delta = 0.0f;
  }
  if (delta > 0.5f) {
    delta = 0.5f;
  }

  float32_t corr;
  if (delta < 0.001f) {
    corr = 1.0f;
  } else {
    corr = (3.14159265358979323846f * delta) / arm_sin_f32(3.14159265358979323846f * delta);
    corr *= (1.0f - delta * delta);
  }

  float32_t gain = FFT_ClampGain(g_vca821_linear_gain);
  *out_freq = ((float32_t)k_max + ((float32_t)direction * delta)) * FFT_BIN_WIDTH_HZ;
  *out_amp = ((a_max * corr) / (float32_t)FFT_BIN_COUNT) * 2.0f / gain;
}


