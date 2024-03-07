#include "tb67h450_base.h"
#include "sin_map.h"
#include "tim.h"

// void TB67H450Base::InitGpio()
// {
//     GPIO_InitTypeDef GPIO_InitStruct = {0};

//     /* Signal Ports Clock Enable */
//     __HAL_RCC_GPIOA_CLK_ENABLE();
//     __HAL_RCC_GPIOB_CLK_ENABLE();

//     /*Configure Signal pin Output Level */
//     HAL_GPIO_WritePin(GPIOA, HW_ELEC_BM_Pin | HW_ELEC_BP_Pin | HW_ELEC_AM_Pin | HW_ELEC_AP_Pin,
//                       GPIO_PIN_RESET);

//     /*Configure Signal pins : PAPin PAPin PAPin PAPin */
//     GPIO_InitStruct.Pin = HW_ELEC_BM_Pin | HW_ELEC_BP_Pin | HW_ELEC_AM_Pin | HW_ELEC_AP_Pin;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
// }

// void TB67H450Base::InitPwm()
// {
//     MX_TIM2_Init();
// }

void TB67H450Base::DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, _voltageA_3300mVIn12bits >> 2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, _voltageB_3300mVIn12bits >> 2);
}

void TB67H450Base::SetInputA(bool _statusAp, bool _statusAm)
{
    _statusAp ? (GPIOA->BSRR = GPIO_PIN_5) : (GPIOA->BRR = GPIO_PIN_5);
    _statusAm ? (GPIOA->BSRR = GPIO_PIN_4) : (GPIOA->BRR = GPIO_PIN_4);
}

void TB67H450Base::SetInputB(bool _statusBp, bool _statusBm)
{
    _statusBp ? (GPIOA->BSRR = GPIO_PIN_3) : (GPIOA->BRR = GPIO_PIN_3);
    _statusBm ? (GPIOA->BSRR = GPIO_PIN_2) : (GPIOA->BRR = GPIO_PIN_2);
}

// void TB67H450Base::Init()
// {
//     InitGpio();
//     InitPwm();
// }

// 输入细分数和最大电流，相当于算0到3300的数占4095的百分比，再用百分比算在1024的数，输出为对应占空比和对应电平
void TB67H450Base::SetFocCurrentVector(uint32_t _directionInCount, int32_t _current_mA)
{
    phaseB.sinMapPtr = (_directionInCount) & (0x000003FF);
    phaseA.sinMapPtr = (phaseB.sinMapPtr + (256)) & (0x000003FF);

    phaseA.sinMapData = sin_pi_m2[phaseA.sinMapPtr];
    phaseB.sinMapData = sin_pi_m2[phaseB.sinMapPtr];

    uint32_t dac_reg = fabs(_current_mA);
    dac_reg = (uint32_t)(dac_reg * 5083) >> 12; // 将0到3300映射至0到4095
    dac_reg = dac_reg & (0x00000FFF);
    phaseA.dacValue12Bits =
        (uint32_t)(dac_reg * fabs(phaseA.sinMapData)) >> sin_pi_m2_dpiybit;
    phaseB.dacValue12Bits =
        (uint32_t)(dac_reg * fabs(phaseB.sinMapData)) >> sin_pi_m2_dpiybit;

    SetTwoCoilsCurrent(phaseA.dacValue12Bits, phaseB.dacValue12Bits);

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

void TB67H450Base::SetTwoCoilsCurrent(uint16_t _currentA_3300mAIn12Bits, uint16_t _currentB_3300mAIn12Bits)
{
    /*
     * After SetFocCurrentVector calculation a 12bits value was mapped to 0~3300mA.
     * And due to used 0.1Ohm shank resistor, 0~3300mV V-ref means 0~3300mA CurrentSetPoint,
     * For more details, see TB67H450 Datasheet page.10 .
     */

    DacOutputVoltage(_currentA_3300mAIn12Bits, _currentB_3300mAIn12Bits);
}

void TB67H450Base::Sleep()
{
    phaseA.dacValue12Bits = 0;
    phaseB.dacValue12Bits = 0;

    SetTwoCoilsCurrent(phaseA.dacValue12Bits, phaseB.dacValue12Bits);

    SetInputA(false, false);
    SetInputB(false, false);
}

void TB67H450Base::Brake()
{
    phaseA.dacValue12Bits = 0;
    phaseB.dacValue12Bits = 0;

    SetTwoCoilsCurrent(phaseA.dacValue12Bits, phaseB.dacValue12Bits);

    SetInputA(true, true);
    SetInputB(true, true);
}
