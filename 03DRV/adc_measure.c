#include "adc_measure.h"

#include <string.h>

#include "adc.h"
#include "tim.h"

#include "sig_util.h"

// DMA 写 IRAM2；任务只在完整帧交付后读取。
uint32_t adc_dma_buf[ADC_DMA_WORDS] __attribute__((section(".iram2")));
// 兼容旧方案的安全帧缓冲区，当前流程未使用。
uint32_t adc_safe_buf[ADC_DMA_WORDS] __attribute__((section(".iram1")));

// ISR 与任务共享，修改顺序要配合内存屏障。
volatile uint8_t adc_buffer_ready = 0U;
volatile ADC_MeasureState_t adc_measure_state = ADC_MEASURE_STATE_IDLE;

// 现场诊断计数与寄存器快照。
volatile uint8_t adc_last_start_stage = 0U;
volatile HAL_StatusTypeDef adc_last_hal_status = HAL_OK;
volatile uint32_t adc_start_count = 0U;
volatile uint32_t adc_full_callback_count = 0U;
volatile uint32_t adc_process_count = 0U;
volatile uint32_t adc_timeout_count = 0U;
volatile uint32_t adc_last_dma_ndtr = 0U;
volatile uint32_t adc_last_dma_cr = 0U;
volatile uint32_t adc_last_dma_lisr = 0U;
volatile uint32_t adc_last_adc_sr = 0U;
volatile uint32_t adc_last_adc_cr2 = 0U;
volatile uint32_t adc_last_adc_ccr = 0U;
volatile uint32_t adc_last_tim8_cr1 = 0U;

static void ADC_Measure_RecordStartFailure(uint8_t stage, HAL_StatusTypeDef status)
{
    adc_last_start_stage = stage;
    adc_last_hal_status = status;
    adc_measure_state = ADC_MEASURE_STATE_ERROR;
}

static void ADC_Measure_CaptureRegisters(void)
{
    if (hadc1.DMA_Handle != NULL) {
        adc_last_dma_ndtr = hadc1.DMA_Handle->Instance->NDTR;
        adc_last_dma_cr = hadc1.DMA_Handle->Instance->CR;
    }
    adc_last_dma_lisr = DMA2->LISR;
    adc_last_adc_sr = hadc1.Instance->SR;
    adc_last_adc_cr2 = hadc1.Instance->CR2;
    adc_last_adc_ccr = ADC_COMMON_REGISTER(&hadc1)->CCR;
    adc_last_tim8_cr1 = htim8.Instance->CR1;
}

// 回调里只冻结寄存器，别调用会改 HAL Lock/State 的停止函数。
static void ADC_Measure_FreezeHardware(void)
{
    CLEAR_BIT(htim8.Instance->CR1, TIM_CR1_CEN);
    CLEAR_BIT(hadc1.Instance->CR2, ADC_CR2_ADON);
    CLEAR_BIT(hadc2.Instance->CR2, ADC_CR2_ADON);
    CLEAR_BIT(hadc3.Instance->CR2, ADC_CR2_ADON);
    CLEAR_BIT(ADC_COMMON_REGISTER(&hadc1)->CCR, ADC_CCR_DDS);
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_OVR);
}

// 只在任务上下文同步 HAL 句柄状态。
static void ADC_Measure_SyncStoppedHandles(void)
{
    (void)HAL_TIM_Base_Stop(&htim8);
    (void)HAL_ADC_Stop(&hadc1);
    (void)HAL_ADC_Stop(&hadc2);
    (void)HAL_ADC_Stop(&hadc3);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC | ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOC | ADC_FLAG_OVR);
}

static HAL_StatusTypeDef ADC_Measure_ResetMultimodeSequencer(void)
{
    ADC_Common_TypeDef *common = ADC_COMMON_REGISTER(&hadc1);
    ADC_MultiModeTypeDef multimode;

    // 关键：Normal DMA 结束后先退回独立模式，再重设三重交错。
    CLEAR_BIT(common->CCR, ADC_CCR_MULTI | ADC_CCR_DMA);
    __DSB();

    multimode.Mode = ADC_TRIPLEMODE_INTERL;
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
    return HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);
}

void ADC_Measure_Init(void)
{
    ADC_Measure_FreezeHardware();
    ADC_Measure_SyncStoppedHandles();
    memset(adc_dma_buf, 0, sizeof(adc_dma_buf));

    adc_buffer_ready = 0U;
    adc_measure_state = ADC_MEASURE_STATE_IDLE;
    adc_last_start_stage = 0U;
    adc_last_hal_status = HAL_OK;
    adc_start_count = 0U;
    adc_full_callback_count = 0U;
    adc_process_count = 0U;
    adc_timeout_count = 0U;
    ADC_Measure_CaptureRegisters();
}

HAL_StatusTypeDef ADC_Measure_Start(void)
{
    HAL_StatusTypeDef status;
    DMA_HandleTypeDef *hdma = hadc1.DMA_Handle;

    adc_last_start_stage = 0U;
    adc_last_hal_status = HAL_OK;

    if (adc_measure_state != ADC_MEASURE_STATE_IDLE) {
        ADC_Measure_RecordStartFailure(1U, HAL_BUSY);
        return HAL_BUSY;
    }
    adc_measure_state = ADC_MEASURE_STATE_ARMING;
    adc_buffer_ready = 0U;
    adc_start_count++;

    if (hdma == NULL) {
        ADC_Measure_RecordStartFailure(2U, HAL_ERROR);
        return HAL_ERROR;
    }

    // 正常路径应为 READY；这里只清理上一帧的异常残留。
    if (((hadc1.Instance->CR2 & ADC_CR2_ADON) != RESET) ||
        ((hadc2.Instance->CR2 & ADC_CR2_ADON) != RESET) ||
        ((hadc3.Instance->CR2 & ADC_CR2_ADON) != RESET)) {
        ADC_Measure_FreezeHardware();
        ADC_Measure_SyncStoppedHandles();
    }
    if ((hdma->State != HAL_DMA_STATE_READY) ||
        (hdma->Lock != HAL_UNLOCKED) ||
        ((hdma->Instance->CR & DMA_SxCR_EN) != RESET)) {
        ADC_Measure_FreezeHardware();
        if (hdma->State == HAL_DMA_STATE_BUSY) {
            (void)HAL_DMA_Abort(hdma);
        }
        status = HAL_DMA_Init(hdma);
        if (status != HAL_OK) {
            ADC_Measure_CaptureRegisters();
            ADC_Measure_RecordStartFailure(2U, status);
            return status;
        }
    }

    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC | ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOC | ADC_FLAG_OVR);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);

    status = ADC_Measure_ResetMultimodeSequencer();
    if (status != HAL_OK) {
        ADC_Measure_CaptureRegisters();
        ADC_Measure_RecordStartFailure(6U, status);
        return status;
    }

    // 顺序不能换：先 ADC2/ADC3，再启动 ADC1 的 Normal DMA。
    status = HAL_ADC_Start(&hadc2);
    if (status != HAL_OK) {
        ADC_Measure_RecordStartFailure(3U, status);
        ADC_Measure_FreezeHardware();
        return status;
    }
    status = HAL_ADC_Start(&hadc3);
    if (status != HAL_OK) {
        ADC_Measure_RecordStartFailure(4U, status);
        ADC_Measure_FreezeHardware();
        return status;
    }
    status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_dma_buf, ADC_DMA_WORDS);
    if (status != HAL_OK) {
        ADC_Measure_RecordStartFailure(5U, status);
        ADC_Measure_FreezeHardware();
        return status;
    }

    // 快照仅供调试，别用瞬时 EN/NDTR 判断本帧成败。
    ADC_Measure_CaptureRegisters();
    adc_measure_state = ADC_MEASURE_STATE_SAMPLING;
    status = HAL_TIM_Base_Start(&htim8);
    if (status != HAL_OK) {
        ADC_Measure_RecordStartFailure(7U, status);
        ADC_Measure_FreezeHardware();
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ADC_Measure_TakeFrame(void)
{
    if ((adc_measure_state != ADC_MEASURE_STATE_FRAME_READY) ||
        (adc_buffer_ready == 0U) ||
        (hadc1.DMA_Handle == NULL) ||
        (hadc1.DMA_Handle->State != HAL_DMA_STATE_READY)) {
        ADC_Measure_CaptureRegisters();
        return HAL_ERROR;
    }

    ADC_Measure_SyncStoppedHandles();
    adc_buffer_ready = 0U;
    adc_measure_state = ADC_MEASURE_STATE_PROCESSING;
    adc_process_count++;
    return HAL_OK;
}

void ADC_Measure_ReleaseFrame(void)
{
    if (adc_measure_state == ADC_MEASURE_STATE_PROCESSING) {
        adc_measure_state = ADC_MEASURE_STATE_IDLE;
    }
}

void ADC_Measure_Stop(void)
{
    ADC_Measure_CaptureRegisters();
    if (adc_measure_state == ADC_MEASURE_STATE_SAMPLING) {
        adc_timeout_count++;
    }

    ADC_Measure_FreezeHardware();
    if ((hadc1.DMA_Handle != NULL) &&
        (hadc1.DMA_Handle->State == HAL_DMA_STATE_BUSY)) {
        (void)HAL_DMA_Abort(hadc1.DMA_Handle);
    }
    ADC_Measure_SyncStoppedHandles();

    adc_buffer_ready = 0U;
    adc_measure_state = ADC_MEASURE_STATE_IDLE;
}

void ADC_Cal_Vpp(uint16_t *pBuffer, uint16_t length, uint8_t ping_pong_index)
{
    // 旧接口只为兼容；结果不参与当前测量链路。
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
    // 当前工程固定为单位增益。
    return SIG_Calculate_Vpp(adc_dma_buf, 1.0f);
}

void Measure_ADC_HalfCpltCallback(void)
{
    // Normal 单帧模式忽略半完成事件。
}

void Measure_ADC_FullCpltCallback(void)
{
    // DMA 已回到 READY；冻结硬件后再置帧就绪标志。
    ADC_Measure_FreezeHardware();
    adc_last_dma_ndtr = hadc1.DMA_Handle->Instance->NDTR;
    adc_full_callback_count++;
    adc_measure_state = ADC_MEASURE_STATE_FRAME_READY;
    __DMB();
    adc_buffer_ready = 1U;
}
