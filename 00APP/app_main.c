#include "app_main.h"
#include "tasks.h"
#include "tim.h"

//函数实现  
void app_init(void) {

}


void app_loop(void) {
  
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == MEASURE_ADC_INSTANCE) {
        Measure_ADC_HalfCpltCallback();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == MEASURE_ADC_INSTANCE) {
        Measure_ADC_FullCpltCallback();
    }
}

// 通道 2 半传输完成中断回调
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    if (hdac->Instance == DAC) { // 部分芯片为 DAC1

    }
}

// 通道 2 传输完成中断回调
void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    if (hdac->Instance == DAC) {

    }
}

