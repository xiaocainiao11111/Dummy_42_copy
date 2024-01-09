#include "common_inc.h"

// extern "C"  void tim4callback(void);

uint16_t _b = 0, _g = 0, _p = 0;

Motor motor;
MT6816Base mt6816_base((uint16_t *)(0x08017C00));
TB67H450Base tb67h450_base;
EncoderCalibratorBase encoder_calibrator_base;


extern uint8_t _a;
extern uint16_t _v;

extern "C" void Main()
{

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    // motor.AttachEncoder(&mt6816_base);
    // _g = 100;
    for (;;)
    {
        // test1();
        // _g = mt6816_base.test1();
        // motor.encoder->test();
        // HAL_Delay(1000);
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == &htim4)
    {

        // mt6816_base.UpdateAngle();
        encoder_calibrator_base.Tick20kHz();
        tim4callback();
    }
}

extern "C" void tim4callback()
{
    // mt6816_base.UpdateAngle();
    _b++;
    if (_b == 1000)
    {
        _b = 0;
    }
    if (_b == 1)
    {
        _p++;
        if (_p == 1000)
        {
            _p = 0;
        }
    }
}

void test2()
{
    _g++;
    if (_g == 100)
    {
        _g = 0;
    }
}