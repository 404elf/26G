#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#include "main.h"

// 入口层只暴露最少的两个接口：初始化和主循环调度。
void app_init(void);
void app_loop(void);

#endif
