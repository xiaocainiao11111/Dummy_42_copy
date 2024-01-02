#include "common_inc.h"




void tim4callback(void);





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
