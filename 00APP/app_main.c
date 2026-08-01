#include "app_main.h"

#include "FFT.h"
#include "adc_measure.h"
#include "tasks.h"


void app_init(void)
{
    // 这里只初始化；采样启停统一由信号处理任务管理。
    ADC_Measure_Init();
    FFT_Init();

    SerialScreen_Init();
}

void app_loop(void)
{
    // 10 ms 调度一次，别在主循环里空转占满 CPU。
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_tick) < 10U) {
        return;
    }

    last_tick = now;
    Task_Signal_Processing();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{   
    // 外部按键或控制信号可从这里接入。
    (void)GPIO_Pin;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 三重 ADC 由 ADC1 作为主 ADC，只接收它的 DMA 回调。
    if (hadc->Instance == ADC1) {
        Measure_ADC_HalfCpltCallback();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // ADC1 全完成回调表示整帧采样结束。
    if (hadc->Instance == ADC1) {
        Measure_ADC_FullCpltCallback();
    }
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
    // DAC 暂未使用，保留 HAL 回调接口。
    (void)hdac;
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
    // DAC 暂未使用，保留 HAL 回调接口。
    (void)hdac;
}
// 覆盖 HAL 的弱错误回调，便于现场断点定位。
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        // 调试提示：停在这里说明 ADC 或 DMA 报错。
        __NOP(); 
    }
}
