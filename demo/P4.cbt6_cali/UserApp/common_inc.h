#ifndef LOOP_H
#define LOOP_H

#ifdef __cplusplus
extern "C"
{
#endif
/*---------------------------- C Scope ---------------------------*/
#include "stdint.h"
#include <tim.h>
#include "spi.h"

    void Main();
 void mt6816_get_raw_data(void);

void TIM1_Callback_10ms();
void TIM4_Callback_50us();

#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/
#include <cstdio>
#include "encoder_calibrator_stm32.h"
#include "tb67h450_stm32.h"
#include "motor.h"
#include "mt6816_stm32.h"

extern TB67H450Base tb67h450base;

#endif
#endif
