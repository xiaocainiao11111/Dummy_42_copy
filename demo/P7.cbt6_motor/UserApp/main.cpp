#include "common_inc.h"
#include "configurations.h"

/*
sqrtf测试，计算平方根
*/

// extern "C"  void tim4callback(void);

uint16_t _b = 0, _g = 0, _p = 0;
uint8_t rx_buffer[128] = {0}, rxLen = 0;

BoardConfig_t boardConfig;
Motor motor;
MT6816Base mt6816_base((uint16_t *)(0x08017C00));
TB67H450Base tb67h450_base;
EncoderCalibratorBase encoder_calibrator_base;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t _a;
extern uint16_t _v;


uint16_t aaa=0;

extern "C" void Main()
{
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    // motor.AttachEncoder(&mt6816_base);
    // _g = 100;
    // tb67h450_base.SetFocCurrentVector(100, 1000);

    encoder_calibrator_base.isTriggered = 1;
    encoder_calibrator_base.errorCode = EncoderCalibratorBase::CALI_NO_ERROR;
    encoder_calibrator_base.state = EncoderCalibratorBase::CALI_DISABLE;
    encoder_calibrator_base.goPosition = 0;
    encoder_calibrator_base.rcdX = 0;
    encoder_calibrator_base.rcdY = 0;
    encoder_calibrator_base.resultNum = 0;

    mt6816_base.Init();
    motor.controller->Init();

    HAL_UART_Receive_DMA(&huart1, rx_buffer, 128);

    motor.AttachEncoder(&mt6816_base);

    for (;;)
    {

        encoder_calibrator_base.TickMainLoop();

        // static uint32_t _a = 0;
        // _a++;
        // if (_a == 51200)
        // {
        //     _a = 0;
        // }
        // tb67h450_base.SetFocCurrentVector(_a, 1000);
        // HAL_Delay(100);

        // test1();
        // _g = mt6816_base.test1();
        // motor.encoder->test();
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t goPosition = 0;
    if (htim == &htim1)
    {
    }
    if (htim == &htim4)
    {
        // if (goPosition <= 51200)
        // {
        //     // 测试：以2000mA电流跑一圈
        //     tb67h450_base.SetFocCurrentVector(goPosition, 1000);
        //     goPosition += 2;
        // }
        // mt6816_base.UpdateAngle();

        if (encoder_calibrator_base.isTriggered)
        {
            encoder_calibrator_base.Tick20kHz();
        }
        else
        {
            motor.Tick20kHz();
        }

        // tim4callback();
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    rxLen = 128 - temp;

    HAL_UART_Receive_DMA(&huart1, rx_buffer, 128);
}
