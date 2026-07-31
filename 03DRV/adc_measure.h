#ifndef __ADC_MEASURE_H
#define __ADC_MEASURE_H

#include "main.h"

#include "FFT.h"

// 每个 DMA 字含两个 12 位样本，解包后正好是 FFT_LENGTH 点。
#define ADC_DMA_WORDS  2048U

typedef enum {
    ADC_MEASURE_STATE_IDLE = 0U,
    ADC_MEASURE_STATE_ARMING,
    ADC_MEASURE_STATE_SAMPLING,
    ADC_MEASURE_STATE_FRAME_READY,
    ADC_MEASURE_STATE_PROCESSING,
    ADC_MEASURE_STATE_ERROR
} ADC_MeasureState_t;

// ISR 置 1，任务取帧后清零。
extern volatile uint8_t adc_buffer_ready;
extern volatile ADC_MeasureState_t adc_measure_state;
// 启动诊断：0 成功；1 状态；2 DMA；3/4 从 ADC；5 主 ADC；6 多 ADC；7 TIM8。
extern volatile uint8_t adc_last_start_stage;
extern volatile HAL_StatusTypeDef adc_last_hal_status;
extern volatile uint32_t adc_start_count;
extern volatile uint32_t adc_full_callback_count;
extern volatile uint32_t adc_process_count;
extern volatile uint32_t adc_timeout_count;
extern volatile uint32_t adc_last_dma_ndtr;
extern volatile uint32_t adc_last_dma_cr;
extern volatile uint32_t adc_last_dma_lisr;
extern volatile uint32_t adc_last_adc_sr;
extern volatile uint32_t adc_last_adc_cr2;
extern volatile uint32_t adc_last_adc_ccr;
extern volatile uint32_t adc_last_tim8_cr1;
// DMA 原始区放在 IRAM2。
extern uint32_t adc_dma_buf[ADC_DMA_WORDS];
// 预留的安全帧缓冲区；当前 Normal DMA 流程未使用。
extern uint32_t adc_safe_buf[ADC_DMA_WORDS];

// 初始化采样状态并复位硬件。
void ADC_Measure_Init(void);
// 启动一帧三重 ADC + Normal DMA。
HAL_StatusTypeDef ADC_Measure_Start(void);
// 取得完成帧的处理权。
HAL_StatusTypeDef ADC_Measure_TakeFrame(void);
// 处理结束后回到空闲态。
void ADC_Measure_ReleaseFrame(void);
// 异常或超时时强制恢复为空闲态。
void ADC_Measure_Stop(void);

// 旧版 Vpp 接口，仅为兼容保留。
void ADC_Cal_Vpp(uint16_t *pBuffer, uint16_t length, uint8_t ping_pong_index);
// 返回当前 DMA 帧的 Vpp。
float Get_Vpp(void);

// Normal 模式不处理半帧回调。
void Measure_ADC_HalfCpltCallback(void);
// 冻结采样并交付完整帧。
void Measure_ADC_FullCpltCallback(void);

#endif
