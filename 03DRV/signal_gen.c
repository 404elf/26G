#include "signal_gen.h"
#include "math.h"
#include "dac.h"
#include "tim.h"
#include "OLED.h"

/* ============================================================================
 *                             内存与数据结构定义
 * ============================================================================ */

// 唯一的一段环形 DMA 缓冲（保留原有的 SRAM2 内存属性映射）
static uint16_t DAC_Buffer[DAC_BUF_SIZE] __attribute__((section(".sram2_data"))); 

#define REF_POINTS 1024

// 优化1: 将 float 数组重构为 uint16_t 定点正弦表 (12位 DAC 对应 0 ~ 4095)
static uint16_t SineRef[REF_POINTS]; 

// 优化2: 相位累加器与步长定点化 (32位无符号整数, Q10.22 格式)
// 高 10 位 (Bit 31~22) 代表 1024 点索引，中间 12 位 (Bit 21~10) 代表 Q12 格式小数插值量
static volatile uint32_t dds_phase = 0;
static volatile uint32_t dds_phase_step = 0; 

// 幅度与偏置参数 (原 float 变量保留，新增预计算的定点数参数)
static volatile float current_vpp_target = 0.0f;
static volatile uint32_t amp_scale_q12 = 0;   // 幅度缩放系数 (Q12定点数)
static volatile int32_t  offset_dac_val = 0;  // DAC 偏置值 (整型)


/* ============================================================================
 *                             内部辅助优化函数
 * ============================================================================ */

/**
 * @brief 内部函数：根据 Vpp 预计算定点数幅度系数与偏置
 * @note  在更新 Vpp 时调用，将浮点除法移出 DMA 核心中断，极大地减轻中断负担
 */
static void Helper_Update_Vpp_Params(float vpp) {
    current_vpp_target = vpp;

    // 计算实际 DAC 幅值范围 (12位 DAC 全量程为 4095)
    float amp_dac = (vpp / 3.3f) * 4095.0f;
    if (amp_dac > 4095.0f) amp_dac = 4095.0f;
    if (amp_dac < 0.0f)    amp_dac = 0.0f;

    // 预计算定点缩放比例 (1.0 全幅放大对应 Q12 的 4096)
    amp_scale_q12 = (uint32_t)((vpp / 3.3f) * 4096.0f);
    
    // 预计算 DAC 偏置中心点
    offset_dac_val = (int32_t)(2048.0f - (amp_dac / 2.0f));
}

/* ============================================================================
 *                             对外接口与核心逻辑
 * ============================================================================ */

/**
 * @brief 初始化基准 sin 表 (12 位量化: 0 ~ 4095)
 */
void Init_SineRef(void) {
    for(int i = 0; i < REF_POINTS; i++) {
        float norm_sine = (sinf(2.0f * PI * i / REF_POINTS) + 1.0f) / 2.0f; // 0.0 ~ 1.0
        SineRef[i] = (uint16_t)(norm_sine * 4095.0f + 0.5f);                 // 映射至 0 ~ 4095
    }
}

/**
 * @brief DDS 流式数据块填充（定点数 + 线性插值极速版）
 * @note  此函数在 DMA 回调中断中频繁调用，已实现全整型化，无任何浮点开销
 */
static void DDS_Generate_Block(uint16_t* pBuffer, uint16_t length) {
    // 局部变量寄存器化 (避免循环内频繁读取 volatile 内存)
    uint32_t phase_reg = dds_phase;
    uint32_t step_reg  = dds_phase_step;
    uint32_t scale_reg = amp_scale_q12;
    int32_t  offset_reg = offset_dac_val;

    for (int i = 0; i < length; i++) {
        // 1. 提取整数索引 (高 10 位: Bit 31 ~ 22)
        uint32_t index0 = phase_reg >> 22;
        // 位与运算 (& 1023) 替代 conditional if 快速实现环形回绕
        uint32_t index1 = (index0 + 1) & (REF_POINTS - 1); 

        // 2. 提取小数部分并转化为 Q12 定点数 (Bit 21 ~ 10，范围 0 ~ 4095)
        uint32_t frac_q12 = (phase_reg >> 10) & 0xFFF;

        // 3. 查表获取相邻两点 (12位整型幅值)
        int32_t s0 = (int32_t)SineRef[index0];
        int32_t s1 = (int32_t)SineRef[index1];

        // 4. 定点数线性插值 (仅 1 次整型乘法 + 1 次位移，替代浮点插值)
        int32_t s_interpolated = s0 + (((s1 - s0) * (int32_t)frac_q12) >> 12);

        // 5. 幅度缩放与偏置叠加 (纯整型位移计算)
        int32_t val = ((s_interpolated * (int32_t)scale_reg) >> 12) + offset_reg;

        // 6. 防御性边界裁剪 (硬件 12位 DAC 边界限制)
        if (val > 4095) val = 4095;
        if (val < 0)    val = 0;

        pBuffer[i] = (uint16_t)val;

        // 7. 相位累加 (32 位无符号整数溢出自动完成 2*PI 模运算)
        phase_reg += step_reg;
    }

    // 写回全局相位状态
    dds_phase = phase_reg;
}

/**
 * @brief 系统启动波形输出(默认直流)
 */
void SignalGen_Start(float init_vpp) {
    HAL_DAC_Stop_DMA(&hdac, SIGGEN_DAC_CH);
    
    // 初始化基准表（若外部未提前调用）
    Init_SineRef();

    // 更新定点数幅度参数
    Helper_Update_Vpp_Params(init_vpp);
    
    Set_DDS_Freq(0);
    dds_phase = 0; // 重置初始相位

    // 预先填满整个 DMA 缓冲区
    DDS_Generate_Block(DAC_Buffer, DAC_BUF_SIZE);

    __HAL_TIM_SET_COUNTER(&SIGGEN_TIM_HANDLE, 0);
    
    // 先填充，后开 DMA，避免启动时的电压毛刺
    HAL_DAC_Start_DMA(&hdac, SIGGEN_DAC_CH, (uint32_t*)DAC_Buffer, DAC_BUF_SIZE, DAC_ALIGN_12B_R);
    __HAL_TIM_SET_COUNTER(&SIGGEN_TIM_HANDLE, 0);
}

/**
 * @brief 重新接管 DAC DMA 控制权
 */
void SignalGen_Resume(void){
    HAL_DAC_Stop_DMA(&hdac, SIGGEN_DAC_CH);
    HAL_DAC_Start_DMA(&hdac, SIGGEN_DAC_CH, (uint32_t*)DAC_Buffer, DAC_BUF_SIZE, DAC_ALIGN_12B_R);
}

/**
 * @brief 重启波形输出
 */
void SignalGEN_Restart(void) {
    dds_phase = 0; // 清零 32 位累加器
    __HAL_TIM_SET_COUNTER(&SIGGEN_TIM_HANDLE, 0); 
}

/**
 * @brief 修改 Vpp。在运行时动态更新 Vpp
 */
void SignalGen_UpdateVpp(float new_vpp) {
    // 预先算好整型参数，下一半 DMA 中断触发时直接读取生效，完美平滑过渡
    Helper_Update_Vpp_Params(new_vpp);
}

/**
 * @brief 修改频率 (定点 DDS 步长计算)
 * @param freq 输出波形的目标频率 (Hz)
 */
void Set_DDS_Freq(float freq) {
    // DDS 核心公式定点化: step = (freq * 2^32) / DAC_SAMPLE_RATE
    // 使用 uint64_t 避免 freq * 2^32 导致的 32 位乘法溢出
    dds_phase_step = (uint32_t)(((uint64_t)freq * 4294967296ULL) / (uint32_t)DAC_SAMPLE_RATE);
}

/* ============================================================================
 *                             DMA 双缓冲回调中断
 * ============================================================================ */

// 当前半个缓冲被 DAC 发送出去时，触发回调
void SignalGen_DAC_HalfCpltCallback(void) {
    // DMA 正在发后半段，CPU 极速算出接下来的 DDS 点填入前半段
    DDS_Generate_Block(&DAC_Buffer[0], DAC_BUF_SIZE / 2);
}

// 当后半个缓冲也被 DAC 发送出去时，触发回调
void SignalGen_DAC_FullCpltCallback(void) {
    // DMA 扭头去发前半段，CPU 极速算出接下来的 DDS 点填入后半段
    DDS_Generate_Block(&DAC_Buffer[DAC_BUF_SIZE / 2], DAC_BUF_SIZE / 2);
}
