#include "tb67h450_stm32.h"
#include "tim.h"
#include "sin_map.h"


/// @brief 主驱动函数
/// @param _directionInCount 位置细分数
/// @param _current_mA 最大电流
void TB67H450::SetFocCurrentVector1(uint32_t _directionInCount, int32_t _current_mA)
{
    phaseB.sinMapPtr = (_directionInCount) & (0x000003FF);// 限制在1023内
    phaseA.sinMapPtr = (phaseB.sinMapPtr + (256)) & (0x000003FF);// B相延后四分之一周期

    phaseA.sinMapData = sin_pi_m2[phaseA.sinMapPtr];// 取对应sin值
    phaseB.sinMapData = sin_pi_m2[phaseB.sinMapPtr];

    uint32_t dac_reg = fabs(_current_mA);// 绝对值
    dac_reg = (uint32_t)(dac_reg * 5083) >> 12; // 3300转4095
    dac_reg = dac_reg & (0x00000FFF);
    phaseA.dacValue12Bits =
        (uint32_t)(dac_reg * fabs(phaseA.sinMapData)) >> sin_pi_m2_dpiybit;// 由实际细分数得到实际电流（对标4095）
    phaseB.dacValue12Bits =
        (uint32_t)(dac_reg * fabs(phaseB.sinMapData)) >> sin_pi_m2_dpiybit;

    // SetTwoCoilsCurrent(phaseA.dacValue12Bits, phaseB.dacValue12Bits);
    DacOutputVoltage(phaseA.dacValue12Bits, phaseB.dacValue12Bits);
    
    if (phaseA.sinMapData > 0)
        SetInputA(true, false);
    else if (phaseA.sinMapData < 0)
        SetInputA(false, true);
    else
        SetInputA(true, true);

    if (phaseB.sinMapData > 0)
        SetInputB(true, false);
    else if (phaseB.sinMapData < 0)
        SetInputB(false, true);
    else
        SetInputB(true, true);
}

void TB67H450::InitGpio()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Signal Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure Signal pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
                      GPIO_PIN_RESET);

    /*Configure Signal pins : PAPin PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void TB67H450::InitPwm()
{
    MX_TIM2_Init();
}

void TB67H450::DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, _voltageA_3300mVIn12bits >> 2);// 4095转1023得到实际占空比
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, _voltageB_3300mVIn12bits >> 2);
}

void TB67H450::SetInputA(bool _statusAp, bool _statusAm)
{
    _statusAp ? (GPIOA->BSRR = GPIO_PIN_5) : (GPIOA->BRR = GPIO_PIN_5);
    _statusAm ? (GPIOA->BSRR = GPIO_PIN_4) : (GPIOA->BRR = GPIO_PIN_4);
}

void TB67H450::SetInputB(bool _statusBp, bool _statusBm)
{
    _statusBp ? (GPIOA->BSRR = GPIO_PIN_3) : (GPIOA->BRR = GPIO_PIN_3);
    _statusBm ? (GPIOA->BSRR = GPIO_PIN_2) : (GPIOA->BRR = GPIO_PIN_2);
}
