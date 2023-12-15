#include "common_inc.h"


void TIM1_Callback_10ms(void);
void TIM4_Callback_50ms(void);

/*
测试两种模式的区别
fabs测试
*/

TB67H450 tb67h450;
TB67H450Base tb67h450base;

uint32_t goPosition = 0;

void Main()
{

    tb67h450.InitGpio();
    tb67h450.InitPwm();

    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    HAL_Delay(10);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    while (1)
    {

        /* code */
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == (&htim1))
    {
        TIM1_Callback_10ms();
    }

    if (htim == (&htim4))
    {
        TIM4_Callback_50ms();
    }
}

void TIM1_Callback_10ms(void)
{

    if (goPosition <= 51200)
    {
        // 测试：以2000mA电流跑一圈
        tb67h450.SetFocCurrentVector1(goPosition, 2000);
        goPosition += 2;
    }
}
void TIM4_Callback_50ms(void)
{
}