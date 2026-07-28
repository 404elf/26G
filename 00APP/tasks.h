#ifndef __TASKS_H
#define __TASKS_H

#include "main.h"

/* Task A — AM 解调 */
void TaskA_Init(void);
void TaskA_Loop(void);

/* Task B — FM 解调 */
void TaskB_Init(void);
void TaskB_Loop(void);

/* Task C — 自动识别 (AM/FM/CW) */
void TaskC_Init(void);
void TaskC_Loop(void);

/* Task D — 数字解调 (2ASK/2FSK/2PSK) */
void TaskD_Init(void);
void TaskD_Loop(void);

#endif /* __TASKS_H */
