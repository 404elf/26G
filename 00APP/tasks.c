#include "tasks.h"
#include "app_main.h"

/* ================================================================
 *  文件级静态变量（三个 Task 共享，同一时刻仅一个 Task 活跃）
 * ================================================================ */

static float    measured_ma   = 0.0f;   // 调幅度 (Task A / C)
static float    measured_mf   = 0.0f;   // 调频度 (Task B / C)
static float    measured_dfm  = 0.0f;   // Δfm kHz (Task B / C)
static float    measured_freq = 0.0f;   // 调制频率 Hz (Task A / B / C)
static float    measured_vpp  = 0.0f;   // 调制信号 Vpp (Task A / B / C)

static uint8_t  taskC_mode = 2;         // 0=AM, 1=FM, 2=CW (Task C 专用)

// TaskD 临时缓冲区（避免栈上分配大数组）
static float      angle2_buf[FFT_LENGTH];
static uint8_t    taskD_bit_stream[FFT_LENGTH];
static float      taskD_filt_buf[FFT_LENGTH];   // 三个分支共用

// ---- 二维无死角联合决策树（基于实测数据重构） ----
#define TH_ENV_SPLIT    80.0f   // 区分"大包络信号(AM/大FM)"与"小包络信号(CW/小FM)"
#define TH_FM_MAX_FREQ  12000.0f// 提高至 12000，完美容纳 0.99 深调幅在 5k 下产生的 8000 噪声尖峰
#define TH_FM_MIN_FREQ  730.0f  // 区分 CW (max 670) 与 弱FM (min 800) — 正中心 (670+800)/2=735

/* ================================================================
 *  Task A — AM 解调
 * ================================================================ */

/**
- @brief：任务 A 初始化，每次切换到 A 时执行，复位外设到 AM 解调所需状态
*/
void TaskA_Init(void) {
    // AD9850 本振 9.8MHz（载波10MHz → IF=200kHz）
    AD985x_SetFre_Phase(1800000.0f, 0);

    // VGA 初始增益
    VGA_SetVoltage(1.0f);

    // 确保 DAC 波形输出在跑
    SignalGen_Resume();
    Set_DDS_Freq(1000.0f);

    // OLED 任务标题
    OLED_ClearBuffer();
}

/**
- @brief：任务 A 主循环 — AM 解调，计算调幅度 ma、调制频率 F、包络 Vpp
*/
void TaskA_Loop(void) {
    // 200ms AGC 自动节流
    Run_Slow_AGC_200ms();

    // 等待 ADC 乒乓缓冲就绪
    if (!adc_buffer_ready) return;
    uint8_t buf = adc_buffer_ready - 1;   // 1→buf[0], 2→buf[1]
    adc_buffer_ready = 0;

    // IQ 解调 → 包络数组 AM_envelope_buffer
    Run_IQ_Demodulation_800k(v_process_buffer[buf]);

    // 调幅度 ma（同时得调制包络 Vpp）
    measured_ma  = Calc_MA(AM_envelope_buffer, FFT_LENGTH-16, &measured_vpp) + 0.05f;
    // Vpp 基于修正后的最新 ma 重新计算
    measured_vpp = 3.3f * measured_ma;

    // Task A 调制频率恒为 1kHz
    measured_freq = 1000.0f;

    // DDS 输出：幅度用 measured_vpp（= 2V × ma）
    SignalGen_UpdateVpp(measured_vpp);
    Set_DDS_Freq(measured_freq);

    // OLED 刷新
    OLED_ClearBuffer();
    OLED_ShowString(0, 0, "Task 1: AM", 1);
    OLED_ShowString(0, 2, "ma :", 1);
    //OLED_ShowString(0, 4, "Vpp  :", 1);
    OLED_ShowFloat(40, 2, measured_ma,   2, 2, 1);
    //OLED_ShowFloat(40, 4, measured_vpp , 1, 2, 1);
    OLED_Refresh();
}

/* ================================================================
 *  Task B — FM 解调
 * ================================================================ */

/**
- @brief：任务 B 初始化，每次切换到 B 时执行，复位外设到 FM 解调所需状态
*/
void TaskB_Init(void) {
    AD985x_SetFre_Phase(1800000.0f, 0);
    VGA_SetVoltage(1.0f);
    SignalGen_Resume();
    Set_DDS_Freq(5000.0f);

    OLED_ClearBuffer();
}

/**
- @brief：任务 B 主循环 — FM 解调，计算调频度 mf、最大频偏 Δfm、调制频率 F
*/
void TaskB_Loop(void) {
    Run_Slow_AGC_200ms();

    if (!adc_buffer_ready) return;
    uint8_t buf = adc_buffer_ready - 1;
    adc_buffer_ready = 0;

    // IQ 解调 → FM_deviation_buffer（瞬时频偏，Hz）
    Run_IQ_Demodulation_800k(v_process_buffer[buf]);

    // Δfm (kHz)
    measured_dfm  = Calc_DeltaFm(FM_deviation_buffer, FFT_LENGTH-16);
    // 一阶滞后滤波 α=0.85，让频偏稳定
    static float dfm_filtered = 0.0f;
    dfm_filtered = 0.15f * measured_dfm + 0.85f * dfm_filtered;
    measured_dfm = dfm_filtered;

    // Task B 调制频率恒为 5kHz
    measured_freq = 5000.0f;

    // mf = Δfm(Hz) / F，两端均已稳定，无需二次滤波
    measured_mf = measured_dfm * 1000.0f / measured_freq;
    
    // Vpp 按公式映射到 mf
    measured_vpp = 1.0f + 0.5f * (measured_mf - 1.0f);

    SignalGen_UpdateVpp(measured_vpp);
    Set_DDS_Freq(measured_freq);

    // OLED 刷新
    OLED_ClearBuffer();
    OLED_ShowString(0, 0, "Task 2: FM", 1);
    OLED_ShowString(0, 2, "mf :", 1);
    OLED_ShowString(0, 4, "dfm:", 1);
    OLED_ShowFloat(40, 2, measured_mf,  2, 2, 1);
    OLED_ShowFloat(40, 4, measured_dfm, 1, 2, 1);
    OLED_Refresh();
}

/* ================================================================
 *  Task C — 自动识别 (AM / FM / CW)
 * ================================================================ */

/**
- @brief：任务 C 初始化，扫频锁相后进入自动识别模式 (AM/FM/CW)
*/
void TaskC_Init(void) {
    AD985x_SetFre_Phase(1800000.0f, 0);
    VGA_SetVoltage(1.0f);
    SignalGen_Resume();

    OLED_ClearBuffer();
}

/**
- @brief：任务 C 主循环 — 基于包络/频偏标准差决策树，自动判别 AM/FM/CW 并解调
*/
void TaskC_Loop(void) {
    Run_Slow_AGC_200ms();

    if (!adc_buffer_ready) return;
    uint8_t buf = adc_buffer_ready - 1;
    adc_buffer_ready = 0;

    // IQ 解调 → AM_envelope_buffer + FM_deviation_buffer
    Run_IQ_Demodulation_800k(v_process_buffer[buf]);

    // 标准差
    float S_env  = Calc_StdDiv(AM_envelope_buffer, FFT_LENGTH-16);
    float S_freq = Calc_StdDiv(FM_deviation_buffer, FFT_LENGTH-16);

    uint8_t temp_mode = 2; // 默认判定为 CW

    if (S_env > TH_ENV_SPLIT) {
        // 1. 包络较大：只可能是 AM 或 强FM
        if (S_freq > TH_FM_MAX_FREQ) {
            temp_mode = 1; // 强FM (S_freq ~ 17200)
        } else {
            temp_mode = 0; // AM (S_freq <= 2000)
        }
    } else {
        // 2. 包络较小：CW 或 弱FM
        if (S_env > 6.0f && S_env < 28.0f && S_freq > 500.0f && S_freq < 1100.0f) {
            temp_mode = 2; // CW
        } else {
            temp_mode = 1; // FM
        }
    }
    // ---- 软件消抖滤波器（防止噪声瞬时脉冲导致跳变） ----
    static uint8_t stable_mode = 2;
    static uint8_t last_temp_mode = 2;
    static uint16_t debounce_cnt = 0;
    const uint16_t DEBOUNCE_LIMIT = 5;

    if (temp_mode == last_temp_mode) {
        debounce_cnt++;
        if (debounce_cnt >= DEBOUNCE_LIMIT) {
            stable_mode = temp_mode;
            debounce_cnt = DEBOUNCE_LIMIT;
        }
    } else {
        last_temp_mode = temp_mode;
        debounce_cnt = 0;
    }
    taskC_mode = stable_mode;

    // ---- 按判定结果执行对应解调 ----
    if (taskC_mode == 1) 
    {
        // FM
        measured_dfm  = Calc_DeltaFm(FM_deviation_buffer, FFT_LENGTH-16);
        // 一阶滞后滤波 α=0.85，让频偏稳定
        static float dfm_filtered = 0.0f;
        dfm_filtered = 0.15f * measured_dfm + 0.85f * dfm_filtered;
        measured_dfm = dfm_filtered;

        float raw_f = Calc_Freq_ZC(FM_deviation_buffer, FFT_LENGTH-16, 800000.0f, 1000.0f);

        // 一阶滞后滤波 α=0.85，防频率跳变抖动，再归口到标准频率
        static float freq_smooth_fm = 0.0f;
        freq_smooth_fm = 0.1f * raw_f + 0.9f * freq_smooth_fm;
        measured_freq = Freq_RoundToStandard(freq_smooth_fm);

        // mf = Δfm(Hz) / F，两端均已稳定，无需二次滤波
        if (measured_freq > 0.0f)
            measured_mf = measured_dfm * 1000.0f / measured_freq;
        // Vpp 按公式映射到 mf
        if (measured_freq > 0.0f&&measured_freq < 1500.0f) measured_mf -= 0.5f;
        measured_dfm = measured_mf*measured_freq/1000.0f;
        measured_vpp = 1.0f + 0.5f * (measured_mf - 1.0f);
        measured_ma = 0.0f;
    } 
    else if (taskC_mode == 0) 
    {
        // AM
        measured_ma   = Calc_MA(AM_envelope_buffer, FFT_LENGTH-16, &measured_vpp) + 0.05f;
        measured_vpp  = 3.3f * measured_ma;

        float raw_f = Calc_Freq_ZC(AM_envelope_buffer, FFT_LENGTH-16, 800000.0f, 20.0f);

        // 一阶滞后滤波 α=0.85，防频率跳变抖动，再归口到标准频率
        static float freq_smooth_am = 0.0f;
        freq_smooth_am = 0.15f * raw_f + 0.85f * freq_smooth_am;
        measured_freq = Freq_RoundToStandard(freq_smooth_am);

        measured_dfm = 0.0f;
        measured_mf  = 0.0f;
    } 
    else 
    {
        // CW
        measured_ma   = 0.0f;
        measured_mf   = 0.0f;
        measured_dfm  = 0.0f;
        measured_freq = 0.0f;
        measured_vpp  = 0.0f;
    }

    // DDS 输出：幅度用 measured_vpp（= 2V × 系数），频率跟随 F
    if (measured_freq > 0.0f) 
    {
        SignalGen_UpdateVpp(measured_vpp);
        Set_DDS_Freq(measured_freq);
    }

    // OLED 刷新 — 按当前模式显示
    OLED_ClearBuffer();
    if (taskC_mode == 0) 
    {
        // AM — 与 Task A 格式一致
        OLED_ShowString(0, 0, "AM ", 1);
        OLED_ShowString(0, 2, "ma :", 1);
        OLED_ShowString(0, 4, "F  :", 1);
        OLED_ShowFloat(40, 2, measured_ma,   2, 2, 1);
        OLED_ShowFloat(40, 4, measured_freq, 1, 2, 1);
    } 
    else if (taskC_mode == 1) 
    {
        // FM — 与 Task B 格式一致
        OLED_ShowString(0, 0, "FM ", 1);
        OLED_ShowString(0, 2, "mf :", 1);
        OLED_ShowString(0, 4, "dfm:", 1);
        OLED_ShowString(0, 6, "F  :", 1);
        OLED_ShowFloat(40, 2, measured_mf,   2, 2, 1);
        OLED_ShowFloat(40, 4, measured_dfm,  1, 2, 1);
        OLED_ShowFloat(40, 6, measured_freq, 1, 2, 1);
    } 
    else 
    {
        // CW
        OLED_ShowString(0, 0, "CW ", 1);
    }
    // OLED_ShowString(0, 2, "e :", 1);
    // OLED_ShowString(0, 4, "f :", 1);
    // OLED_ShowFloat(40, 2, S_env,   5, 1, 1);
    // OLED_ShowFloat(40, 4, S_freq, 5, 1, 1);        
    OLED_Refresh();
}

/* ================================================================
 *  Task D — 数字解调 (2ASK / 2FSK / 2PSK)
 * ================================================================ */

static uint8_t  taskD_mode = 0;         // 0=2ASK, 1=2FSK, 2=2PSK
static float    taskD_Rc   = 0.0f;      // 码速率 (bps)
static float    taskD_h    = 0.0f;      // 移频键控系数 (2FSK 专用)

/**
- @brief：任务 D 初始化
*/
void TaskD_Init(void) {
    // LO = 1.8MHz → IF = 200kHz (载波 2MHz)
    AD985x_SetFre_Phase(1800000.0f, 0);
    VGA_SetVoltage(1.0f);
    SignalGen_Resume();
    Set_DDS_Freq(1000.0f);

    taskD_mode = 0;
    taskD_Rc   = 0.0f;
    taskD_h    = 0.0f;

    OLED_ClearBuffer();
}

/**
- @brief：任务 D 主循环 — 数字键控信号识别与解调
*/
void TaskD_Loop(void) {
    Run_Slow_AGC_200ms();

    if (!adc_buffer_ready) return;
    uint8_t buf = adc_buffer_ready - 1;
    adc_buffer_ready = 0;

    // IQ 解调 → AM_envelope_buffer + FM_deviation_buffer + I/Q 原始分量
    Run_IQ_Demodulation_800k(v_process_buffer[buf]);

    // ---- 标准差特征提取 ----
    float S_env  = Calc_StdDiv(AM_envelope_buffer, FFT_LENGTH-16);
    float S_freq = Calc_StdDiv(FM_deviation_buffer, FFT_LENGTH-16);

    // ---- 形状因子比值 S_ratio = S_freq / Mean(|f|)，区分 2FSK / 2PSK ----
    float sum_abs_freq = 0.0f;
    for (int i = 0; i < FFT_LENGTH-16; i++) {
        sum_abs_freq += fabsf(FM_deviation_buffer[i]);
    }
    float mean_abs_freq = sum_abs_freq / (float)(FFT_LENGTH-16);
    float S_ratio = S_freq / (mean_abs_freq + 0.00001f);

    // ---- 决策树：包络起伏→2ASK，形状因子→2FSK/2PSK ----
    if (S_env > 100.0f) {
        taskD_mode = 0;     // 2ASK
    } else if (S_ratio < 1.03f) {
        taskD_mode = 1;     // 2FSK
    } else {
        taskD_mode = 2;     // 2PSK
    }

    // ---- 按模式执行二值化 + 码速率估计 ----
    if (taskD_mode == 0) {
        // 2ASK：包络 → 16点LPF → 自适应二值化
        LPF_16pt(AM_envelope_buffer, taskD_filt_buf, FFT_LENGTH-16);
        Adaptive_Binarize(taskD_filt_buf, taskD_bit_stream, FFT_LENGTH-16);
        taskD_Rc = Estimate_CodeRate(taskD_bit_stream, FFT_LENGTH-16, 800000.0f);
        taskD_h  = 0.0f;
    } else if (taskD_mode == 1) {
        // 2FSK：频偏 → 16点LPF → 自适应二值化 → h = 2*S_freq/Rc
        LPF_16pt(FM_deviation_buffer, taskD_filt_buf, FFT_LENGTH-16);
        Adaptive_Binarize(taskD_filt_buf, taskD_bit_stream, FFT_LENGTH-16);
        taskD_Rc = Estimate_CodeRate(taskD_bit_stream, FFT_LENGTH-16, 800000.0f);
        if (taskD_Rc > 0.0f)
            taskD_h = (2.0f * S_freq) / taskD_Rc;
        else
            taskD_h = 0.0f;
    } else {
        // 2PSK：载波恢复 → I_rot → 自适应二值化
        PSK_CarrierRecovery(I_buffer, Q_buffer, taskD_filt_buf, FFT_LENGTH);
        Adaptive_Binarize(taskD_filt_buf, taskD_bit_stream, FFT_LENGTH-16);
        taskD_Rc = Estimate_CodeRate(taskD_bit_stream, FFT_LENGTH-16, 800000.0f);
        taskD_h  = 0.0f;
    }

    // ---- OLED 刷新 ----
    OLED_ClearBuffer();
    if (taskD_mode == 0) {
        OLED_ShowString(0, 0, "Mode: 2ASK", 1);
    } else if (taskD_mode == 1) {
        OLED_ShowString(0, 0, "Mode: 2FSK", 1);
        OLED_ShowString(0, 4, "h :", 1);
        OLED_ShowFloat(30, 4, taskD_h, 1, 2, 1);
    } else {
        OLED_ShowString(0, 0, "Mode: 2PSK", 1);
    }
    OLED_ShowString(0, 2, "Rc  :", 1);
    OLED_ShowFloat(35, 2, taskD_Rc / 1000.0f, 2, 1, 1);
    OLED_ShowString(75, 2, "kbps", 1);
    // OLED_ShowString(0, 2, "e :", 1);
    // OLED_ShowString(0, 4, "f :", 1);
    // OLED_ShowString(0, 6, "r :", 1);
    // OLED_ShowFloat(40, 2, S_env,   5, 1, 1);
    // OLED_ShowFloat(40, 4, S_freq,  5, 1, 1);
    // OLED_ShowFloat(40, 6, S_ratio, 5, 2, 1);
    OLED_Refresh();
}
