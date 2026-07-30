#ifndef __ADC_MEASURE_H
#define __ADC_MEASURE_H

#include "main.h"

#include "FFT.h"

// 单帧 DMA 采样长度，和 FFT 输入长度保持一致。
#define ADC_DMA_WORDS  2048U

// 采样完成标志：0 未就绪，1 DMA 完成。
extern volatile uint8_t adc_buffer_ready;
// ADC triple mode 的 DMA 原始缓冲区，放到 IRAM2。
extern uint32_t adc_dma_buf[ADC_DMA_WORDS];

// 初始化采样缓冲和状态。
void ADC_Measure_Init(void);
// 启动一次采样链路。
void ADC_Measure_Start(void);
// 停止采样链路并清理状态。
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
