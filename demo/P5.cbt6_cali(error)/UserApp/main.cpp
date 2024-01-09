#include "common_inc.h"

// extern "C"  void tim4callback(void);

uint8_t _b = 0;
uint16_t _p = 0;
// Motor motor;
MT6816Base mt6816_base;

extern uint8_t _a;

void test();

void Main()
{

    HAL_TIM_Base_Start_IT(&htim4);
    // motor.AttachEncoder(&mt6816_base);
    while (1)
    {
        // test();
        mt6816_base.test();
        // motor.encoder->UpdateAngle();
    }
}

// extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim == &htim4)
//     {

//         tim4callback();
//     }
// }

void tim4callback()
{
    // mt6816_base.UpdateAngle();
    _b++;
    if (_b == 100)
    {
        _b = 0;
    }
}


void test()
{
        _p++;
        if (_p == 1000)
        {
            _p = 0;
        }    
}