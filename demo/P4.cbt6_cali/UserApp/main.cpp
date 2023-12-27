#include "common_inc.h"


/*
fabs测试
*/

TB67H450 tb67h450;
TB67H450Base tb67h450base;

// EncoderCalibratorBase encoderCalibratorBase;

bool flag = 1;
extern uint32_t goPosition;

void TIM1_Callback_10ms();
void TIM4_Callback_50us();

void motor_run(uint32_t _position);

uint16_t SpiTransmitAndRead16Bits(uint16_t _dataTx);
uint16_t rawData = 0;
uint8_t checksumFlag = 0;
uint16_t rawAngle = 0;
bool noMagFlag = 0;
uint16_t dataRx[2] = {0};
uint16_t dataTx[2] = {0};
uint16_t rectifiedAngle = 0;

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

    HAL_Delay(10);

    for (;;)
    {
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == (&htim1))
    {

        // TIM4_Callback_50us();
    }

    if (htim == (&htim4))
    {
        // encoderCalibratorBase.Tick20kHz();
        // TIM1_Callback_10ms();
        // 进入校准
    }
}

// 测试成功
void TIM1_Callback_10ms()
{

    // mt6816_get_raw_data();

    static uint32_t _goposition = 0;
    // 测试一圈
    if (_goposition <= 51200)
    {
        tb67h450.SetFocCurrentVector1(_goposition, 1000);
        _goposition++;
    }
}

// 编码器校准
void TIM4_Callback_50us()
{
}

uint16_t SpiTransmitAndRead16Bits(uint16_t _dataTx)
{
    uint16_t dataRx;

    GPIOA->BRR = GPIO_PIN_15; // Chip select
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&_dataTx, (uint8_t *)&dataRx, 1, HAL_MAX_DELAY);
    GPIOA->BSRR = GPIO_PIN_15;

    return dataRx;
}

// 默认2000mA每相的测试电流
void motor_run(uint32_t _position)
{

    tb67h450.SetFocCurrentVector1(_position, 2000);
}

void mt6816_get_raw_data(void)
{
    dataTx[0] = (0x80 | 0x03) << 8; // 0x8300
    dataTx[1] = (0x80 | 0x04) << 8; // 0x8400

    for (uint8_t i = 0; i < 3; i++)
    {
        dataRx[0] = SpiTransmitAndRead16Bits(dataTx[0]);
        dataRx[1] = SpiTransmitAndRead16Bits(dataTx[1]);

        rawData = ((dataRx[0] & 0x00FF) << 8) | (dataRx[1] & 0x00FF);

        // 奇偶校验
        uint8_t hCount = 0;
        for (uint8_t j = 0; j < 16; j++)
        {
            if (rawData & (0x0001 << j))
                hCount++;
        }
        if (hCount & 0x01)
        {
            checksumFlag = false;
        }
        else
        {
            checksumFlag = true;
            break;
        }
    }

    if (checksumFlag)
    {
        rawAngle = rawData >> 2;
        noMagFlag = (bool)(rawData & (0x0001 << 1));
    }
}