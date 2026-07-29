#include "VCA821.h"

#include <math.h>

#include "adc_measure.h"

// 参考电压、控制电压和增益范围都写成常量，方便统一调试。
#define VCA821_REF_VOLTAGE      3.3f
#define VCA821_CONTROL_MAX_V    2.0f
#define VCA821_GAIN_MIN_DB      0.0f
#define VCA821_GAIN_MAX_DB      20.0f

float g_vca821_linear_gain = 1.0f;

// 记录 DAC 输出值和 AGC 调度状态。
static uint16_t g_vca821_dac_val = 0;
static uint32_t g_last_agc_ms = 0;
static uint8_t g_vca821_started = 0;

// 限幅函数，避免控制电压超出硬件允许范围。
static float VCA821_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void VCA821_ApplyVoltage(float voltage)
{
  // 把控制电压换算成 DAC 码值，再写到硬件输出。
  voltage = VCA821_ClampFloat(voltage, 0.0f, VCA821_CONTROL_MAX_V);
  g_vca821_dac_val = (uint16_t)((voltage / VCA821_REF_VOLTAGE) * 4095.0f + 0.5f);
  (void)HAL_DAC_SetValue(&hdac, VCA_DAC_CHANNEL, DAC_ALIGN_12B_R, g_vca821_dac_val);
  (void)HAL_DAC_Start(&hdac, VCA_DAC_CHANNEL);
}

void VCA821_Init(void)
{
  // 防止重复初始化。
  if (g_vca821_started) {
    return;
  }

  g_vca821_started = 1;
  VCA821_SetGain(6.0f);
}

void VCA821_SetGain(float gain_dB)
{
  // dB 和线性增益同时更新，便于幅值补偿直接使用。
  gain_dB = VCA821_ClampFloat(gain_dB, VCA821_GAIN_MIN_DB, VCA821_GAIN_MAX_DB);
  g_vca821_linear_gain = powf(10.0f, gain_dB / 20.0f);

  float control_voltage = (gain_dB / VCA821_GAIN_MAX_DB) * VCA821_CONTROL_MAX_V;
  VCA821_ApplyVoltage(control_voltage);
}

float VCA821_GetLinearGain(void)
{
  return (g_vca821_linear_gain > 0.0f) ? g_vca821_linear_gain : 1.0f;
}

void Run_Slow_AGC(void)
{
  // 通过当前 Vpp 粗略判断增益是偏低还是偏高。
  float current_vpp = Get_Vpp();
  if (current_vpp < 1.46f) {
    VGA_SetVoltage(1.8f);
  } else if (current_vpp > 1.54f) {
    VGA_SetVoltage(1.0f);
  }
}

void VGA_SetVoltage(float voltage)
{
  // 外部如果直接给控制电压，这里也同步更新线性增益。
  float clamped_voltage = VCA821_ClampFloat(voltage, 0.0f, VCA821_CONTROL_MAX_V);
  float gain_dB = (clamped_voltage / VCA821_CONTROL_MAX_V) * VCA821_GAIN_MAX_DB;
  g_vca821_linear_gain = powf(10.0f, gain_dB / 20.0f);
  VCA821_ApplyVoltage(clamped_voltage);
}

void Run_Slow_AGC_200ms(void)
{
  // 限制 AGC 调整频率，避免太快抖动。
  uint32_t now = HAL_GetTick();
  if (now - g_last_agc_ms < 200U) {
    return;
  }
  g_last_agc_ms = now;
  Run_Slow_AGC();
}
