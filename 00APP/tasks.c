#include "tasks.h"
#include "app_main.h"


/**
- @brief：任务 A 初始化，每次切换到 A 时执行，复位外设到 AM 解调所需状态
*/
void TaskA_Init(void) {

}

/**
- @brief：任务 A 主循环 — AM 解调，计算调幅度 ma、调制频率 F、包络 Vpp
*/
void TaskA_Loop(void) {
   
}

/**
- @brief：任务 B 初始化，每次切换到 B 时执行，复位外设到 FM 解调所需状态
*/
void TaskB_Init(void) {

}

/**
- @brief：任务 B 主循环 — FM 解调，计算调频度 mf、最大频偏 Δfm、调制频率 F
*/
void TaskB_Loop(void) {
    
}


/**
- @brief：任务 C 初始化，扫频锁相后进入自动识别模式 (AM/FM/CW)
*/
void TaskC_Init(void) {

}

/**
- @brief：任务 C 主循环 — 基于包络/频偏标准差决策树，自动判别 AM/FM/CW 并解调
*/
void TaskC_Loop(void) {
   
}

