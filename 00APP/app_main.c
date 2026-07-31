#include "app_main.h"
#include "tasks.h"

// 由各功能模块提供，入口层只负责调用。
void ADC_Measure_Init(void);
void Measure_ADC_HalfCpltCallback(void);
void Measure_ADC_FullCpltCallback(void);
void FFT_Init(void);
void VCA821_Init(void);

void app_init(void)
{
    // 只初始化，不启动采样——由 Task_Signal_Processing 统一管理 Start → Wait → Stop 周期。
    ADC_Measure_Init();
    FFT_Init();
    VCA821_Init();
    SerialScreen_Init();
}

void app_loop(void)
{
    // 主循环只做节流调度，避免一直占用 CPU。
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
    // 预留外部中断入口，后续按键或控制信号可以接到这里。
    (void)GPIO_Pin;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 只接收 ADC1 的回调，表示一组 DMA 数据已经准备好一半。
    if (hadc->Instance == ADC1) {
        Measure_ADC_HalfCpltCallback();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 只接收 ADC1 的回调，表示这一帧采样数据已经完整。
    if (hadc->Instance == ADC1) {
        Measure_ADC_FullCpltCallback();
    }
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
    // 当前未使用 DAC 中断回调，保留接口避免后续移植出错。
    (void)hdac;
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
    // 当前未使用 DAC 中断回调，保留接口避免后续移植出错。
    (void)hdac;
}
/* 重写错误回调函数 (取消 __weak) */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        // 在这里打断点！如果停到了这里，说明 DMA 或 ADC 硬件报了 Error
        __NOP(); 
    }
}
