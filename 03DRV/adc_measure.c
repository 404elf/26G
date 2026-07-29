#include "adc_measure.h"

#include <string.h>

#include "adc.h"
#include "tim.h"
#include "VCA821.h"
#include "sig_util.h"

// 原始 DMA 缓冲区放在 IRAM2，便于和主内存分离。
uint32_t adc_dma_buf[ADC_DMA_TOTAL] __attribute__((section(".iram2")));

// 采样完成标志：由 ADC 回调更新，任务层轮询读取。
volatile uint8_t adc_buffer_ready = 0;
static uint8_t g_adc_measure_started = 0;

// 统一清理采样状态，避免旧帧标志残留。
static void ADC_Measure_ClearState(void)
{
    adc_buffer_ready = 0;
}

void ADC_Measure_Init(void)
{
    // 启动前先清空 DMA 缓冲和状态位。
    memset(adc_dma_buf, 0, sizeof(adc_dma_buf));
    ADC_Measure_ClearState();
    g_adc_measure_started = 0;
}

void ADC_Measure_Start(void)
{
    // 已经启动时不重复开链路。
    if (g_adc_measure_started) {
        return;
    }

    ADC_Measure_ClearState();
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_DMA_TOTAL) == HAL_OK) {
        (void)HAL_TIM_Base_Start(&htim8);
        g_adc_measure_started = 1;
    }
}

void ADC_Measure_Stop(void)
{
    // 停止时同时关闭 TIM8 和 ADC DMA。
    if (!g_adc_measure_started) {
        return;
    }

    (void)HAL_TIM_Base_Stop(&htim8);
    (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    g_adc_measure_started = 0;
    ADC_Measure_ClearState();
}

void ADC_Cal_Vpp(uint16_t *pBuffer, uint16_t length, uint8_t ping_pong_index)
{
    // 旧接口保留，当前主要用于兼容老调用点。
    static uint16_t global_max = 0;
    static uint16_t global_min = 4095;

    for (uint16_t i = 0; i < length; i++) {
        uint16_t sample = pBuffer[i];
        if (sample > global_max) {
            global_max = sample;
        }
        if (sample < global_min) {
            global_min = sample;
        }
        (void)ping_pong_index;
    }

    global_max = 0;
    global_min = 4095;
}

float Get_Vpp(void)
{
    // 使用统一的 DMA 缓冲和增益信息来计算峰峰值。
    return SIG_Calculate_Vpp(adc_dma_buf, VCA821_GetLinearGain());
}

void Measure_ADC_HalfCpltCallback(void)
{
    // 半传输完成时先标记为 1，告诉任务层数据开始可读。
    adc_buffer_ready = 1;
}

void Measure_ADC_FullCpltCallback(void)
{
    // 全传输完成时标记为 2，表示整帧数据已经准备好。
    adc_buffer_ready = 2;
}
