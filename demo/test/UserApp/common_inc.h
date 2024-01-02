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
#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/
#include <cstdio>
#include "encoder_calibrator_base.h"
#include "tb67h450_stm32.h"
#include "motor.h"

extern TB67H450Base tb67h450base;

#endif
#endif
