#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#include "main.h"

// 应用层只暴露初始化和主循环调度接口。
void app_init(void);
void app_loop(void);

#endif
