#ifndef __IQ_H
#define __IQ_H	 

#include "main.h"
#include "FFT.h"

// 解调后滑动平均窗口宽度
#define AM_SMOOTH_WINDOW  16U
#define FM_SMOOTH_WINDOW  16U

extern float AM_envelope_buffer[FFT_LENGTH];
extern float FM_deviation_buffer[FFT_LENGTH];
extern float I_buffer[FFT_LENGTH];           // 滤波后 I 分量 (供 PSK 解调)
extern float Q_buffer[FFT_LENGTH];           // 滤波后 Q 分量 (供 PSK 解调)

void Run_IQ_Demodulation_800k(float* pRawData);

#endif


