#ifndef __ADC_MEASURE_H
#define __ADC_MEASURE_H

#include "main.h"

#include "FFT.h"

// 单帧 DMA 采样长度，和 FFT 输入长度保持一致。
#define ADC_DMA_WORDS  2048U

typedef enum {
    ADC_MEASURE_STATE_IDLE = 0U,
    ADC_MEASURE_STATE_ARMING,
    ADC_MEASURE_STATE_SAMPLING,
    ADC_MEASURE_STATE_FRAME_READY,
    ADC_MEASURE_STATE_PROCESSING,
    ADC_MEASURE_STATE_ERROR
} ADC_MeasureState_t;

// 采样完成标志：0 未就绪，1 DMA 完成。
extern volatile uint8_t adc_buffer_ready;
extern volatile ADC_MeasureState_t adc_measure_state;
// 启动诊断变量：stage=0 成功；1 状态错误，2 DMA 未就绪，3/4 从 ADC，
// 5 主 ADC+DMA，6 多ADC序列器复位，7 TIM8。
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
// ADC triple mode 的 DMA 原始缓冲区，放到 IRAM2。
extern uint32_t adc_dma_buf[ADC_DMA_WORDS];
// FullCplt 时拷贝到此缓冲区（IRAM1），任务层只读这里，避免 Circular DMA 竞态。
extern uint32_t adc_safe_buf[ADC_DMA_WORDS];

// 初始化采样缓冲和状态。
void ADC_Measure_Init(void);
// 启动一次采样链路。
HAL_StatusTypeDef ADC_Measure_Start(void);
// 接收一帧并把 HAL 句柄同步到处理状态。
HAL_StatusTypeDef ADC_Measure_TakeFrame(void);
// 当前帧处理结束，允许启动下一帧。
void ADC_Measure_ReleaseFrame(void);
// 异常或超时时停止采样链路并恢复到空闲状态。
void ADC_Measure_Stop(void);

// 早期保留的 Vpp 计算接口，当前主要用于兼容旧代码。
void ADC_Cal_Vpp(uint16_t *pBuffer, uint16_t length, uint8_t ping_pong_index);
// 当前帧 Vpp 的统一计算入口。
float Get_Vpp(void);

// DMA 半传输完成回调。
void Measure_ADC_HalfCpltCallback(void);
// DMA 全传输完成回调。
void Measure_ADC_FullCpltCallback(void);

#endif
