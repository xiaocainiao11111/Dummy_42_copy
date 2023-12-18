#include "common_inc.h"
#include <tim.h>
#include "spi.h"

void TIM1_Callback_10ms();
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
    HAL_Delay(100);
    HAL_TIM_Base_Start_IT(&htim1); // 100Hz

    for (;;)
    {
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == (&htim1))
    {
        TIM1_Callback_10ms();
    }
}

void TIM1_Callback_10ms()
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

uint16_t SpiTransmitAndRead16Bits(uint16_t _dataTx)
{
    uint16_t dataRx;

    GPIOA->BRR = GPIO_PIN_15; // Chip select
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&_dataTx, (uint8_t *)&dataRx, 1, HAL_MAX_DELAY);
    GPIOA->BSRR = GPIO_PIN_15;

    return dataRx;
}
