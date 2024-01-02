#include "common_inc.h"

void tim4callback(void);

uint8_t _a = 0;

Test test;

void Test::tick(void)
{
}

void Main()
{
    test.num = 0;
    test.tick();
    while (1)
    {
        /* code */
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim4)
    {
        tim4callback();
    }
}

void tim4callback()
{
    _a++;
    if (_a == 100)
    {
        _a = 0;
    }
}