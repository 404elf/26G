#include "adc_measure.h"

#include <string.h>

#include "adc.h"
#include "tim.h"
#include "VCA821.h"
#include "sig_util.h"

// 原始 DMA 缓冲区放在 IRAM2，便于和主内存分离。
uint32_t adc_dma_buf[ADC_DMA_WORDS] __attribute__((section(".iram2")));
// FullCplt 时从 DMA 区拷贝到此处（IRAM1），任务层安全读取。
uint32_t adc_safe_buf[ADC_DMA_WORDS] __attribute__((section(".iram1")));

// 采样完成标志：由 ADC FullCplt 回调置 1，任务层轮询读取。
volatile uint8_t adc_buffer_ready = 0;

// 统一清理采样状态。
static void ADC_Measure_ClearState(void)
{
    adc_buffer_ready = 0;
}

void ADC_Measure_Init(void)
{
    memset(adc_dma_buf, 0, sizeof(adc_dma_buf));
    ADC_Measure_ClearState();
}

void ADC_Measure_Start(void)
{
    volatile HAL_StatusTypeDef st;

    ADC_Measure_ClearState();
    st = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_DMA_WORDS);
    if (st == HAL_OK) {
        (void)HAL_TIM_Base_Start(&htim8);
    }
    // 断点设这里看 st：HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3
    (void)st;
}

void ADC_Measure_Stop(void)
{
    (void)HAL_TIM_Base_Stop(&htim8);
    (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
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
    // 单次触发模式下不使用半回调，保留接口兼容。
}

void Measure_ADC_FullCpltCallback(void)
{
    // DMA FullCplt：先快照到安全区，再置标志，避免任务层读到 DMA 正在覆盖的数据。
    memcpy(adc_safe_buf, adc_dma_buf, sizeof(adc_safe_buf));
    adc_buffer_ready = 1;
}
