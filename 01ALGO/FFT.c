#include "FFT.h"

#include <math.h>

#include "adc_measure.h"


// FFT 实例与各阶段缓冲区。
arm_rfft_fast_instance_f32 g_rfft_instance;
// CCMRAM 留给访问频繁的时域数据和幅度谱。
float32_t rfft_input[FFT_LENGTH] __attribute__((section(".ccmram")));
float32_t rfft_time_domain[FFT_LENGTH] __attribute__((section(".ccmram")));
// 复数输出只在 FFT 阶段使用，放普通 IRAM。
float32_t rfft_output_complex[FFT_LENGTH];
float32_t rfft_mag[FFT_BIN_COUNT] __attribute__((section(".ccmram")));

// Hann 窗预计算一次，别放进逐帧处理里。
static float32_t hann_window[FFT_LENGTH] __attribute__((section(".iram1")));

// 增益无效时按 1 处理，避免除零。
static float32_t FFT_ClampGain(float32_t gain)
{
  return (gain > 0.0f) ? gain : 1.0f;
}

void FFT_Init(void)
{
  (void)arm_rfft_fast_init_f32(&g_rfft_instance, FFT_LENGTH);

  // 记得：FFT_LENGTH 改动后要重新初始化这张表。
  for (uint32_t i = 0; i < FFT_LENGTH; i++) {
    hann_window[i] = 0.5f * (1.0f - arm_cos_f32((2.0f * 3.14159265358979323846f * (float32_t)i) / (float32_t)FFT_LENGTH));
  }
}

// 所有频谱搜索先过这里，防止边界 bin 越界。
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

  // DMA 每字的低/高 16 位按时间先后组成两个样本。
  uint32_t code_sum = 0;
  for (uint32_t word_index = 0; word_index < ADC_DMA_WORDS; word_index++) {
    uint32_t packed = dma_buf[word_index];
    uint16_t sample_early = (uint16_t)(packed & 0xFFFFU) & 0x0FFFU;
    uint16_t sample_late = (uint16_t)((packed >> 16U) & 0xFFFFU) & 0x0FFFU;

    code_sum += (uint32_t)sample_early + (uint32_t)sample_late;

    uint32_t sample_index = word_index * 2U;
    rfft_input[sample_index]     = (float32_t)sample_early * 3.3f / 4096.0f;
    rfft_input[sample_index + 1U] = (float32_t)sample_late  * 3.3f / 4096.0f;
  }

  // 码值先整数累加，最后一次换算直流偏置。
  float32_t V_avg = (float32_t)code_sum * 3.3f / 4096.0f / (float32_t)FFT_LENGTH;
  (void)linear_gain;

  for (uint32_t i = 0; i < FFT_LENGTH; i++) {
    float32_t ac_v = rfft_input[i] - V_avg;

    // 相位估计必须用未加窗帧；FFT 输入另行加窗。
    rfft_time_domain[i] = ac_v;

    rfft_input[i] = ac_v * hann_window[i];
  }

  arm_rfft_fast_f32(&g_rfft_instance, rfft_input, rfft_output_complex, 0);

  // CMSIS 实数 FFT 把 DC 与 Nyquist 打包在前两个元素。
  rfft_mag[0] = fabsf(rfft_output_complex[0]);
  rfft_mag[FFT_BIN_COUNT - 1U] = fabsf(rfft_output_complex[1]);
  arm_cmplx_mag_f32(&rfft_output_complex[2], &rfft_mag[1], FFT_BIN_COUNT - 1U);
}

void FFT_Rife_Interpolate(int bin_center, float32_t *out_freq, float32_t *out_amp, float linear_gain)
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

  // 取较大的相邻 bin 决定插值方向。
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

  float32_t gain = FFT_ClampGain(linear_gain);
  *out_freq = ((float32_t)k_max + ((float32_t)direction * delta)) * FFT_BIN_WIDTH_HZ;
  *out_amp = ((a_max * corr) / (float32_t)FFT_BIN_COUNT) * 2.0f / gain;
}


