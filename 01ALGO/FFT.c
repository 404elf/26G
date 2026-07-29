#include "FFT.h"

#include <math.h>

#include "adc_measure.h"
#include "VCA821.h"

// FFT 运行实例，初始化后供后续每帧复用。
arm_rfft_fast_instance_f32 g_rfft_instance;
// 时域输入放在 CCMRAM 中，减少和 DMA 区域的访问冲突。
float32_t rfft_input[FFT_LENGTH] __attribute__((section(".ccmram")));
// 复数域临时输出缓冲。
float32_t rfft_output_complex[FFT_LENGTH] __attribute__((section(".ccmram")));
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

  // 将双路打包采样拆成时域样本，并叠加 Hann 窗，减少频谱泄漏。
  (void)linear_gain;

  for (uint32_t word_index = 0; word_index < ADC_DMA_WORDS; word_index++) {
    uint32_t packed = dma_buf[word_index];
    uint16_t sample_early = (uint16_t)(packed & 0xFFFFU) & 0x0FFFU;
    uint16_t sample_late = (uint16_t)((packed >> 16U) & 0xFFFFU) & 0x0FFFU;

    uint32_t sample_index = word_index * 2U;
    float32_t voltage_early = (float32_t)sample_early * 3.3f / 4096.0f;
    float32_t voltage_late = (float32_t)sample_late * 3.3f / 4096.0f;

    float32_t window_early = 0.5f * (1.0f - arm_cos_f32((2.0f * 3.14159265358979323846f * (float32_t)sample_index) / (float32_t)FFT_LENGTH));
    float32_t window_late = 0.5f * (1.0f - arm_cos_f32((2.0f * 3.14159265358979323846f * (float32_t)(sample_index + 1U)) / (float32_t)FFT_LENGTH));

    rfft_input[sample_index] = voltage_early * window_early;
    rfft_input[sample_index + 1U] = voltage_late * window_late;
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


