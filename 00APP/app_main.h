#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#include "main.h"
#include "arm_math.h"

#include "AD985x.h"
#include "adc_measure.h"
#include "FFT.h"
#include "IQ.h"
#include "OLED.h"
#include "signal_gen.h"
#include "VCA821.h"
#include "sig_util.h"

void app_init(void);
void app_loop(void);

#endif
