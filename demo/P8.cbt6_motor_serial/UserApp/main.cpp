#include "common_inc.h"
#include "configurations.h"

/*
sqrtf测试，计算平方根
*/

// extern "C"  void tim4callback(void);

extern uint8_t TxData[8];
extern uint8_t RxData[8];

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;

uint16_t _b = 0, _g = 0, _p = 0;
uint8_t rx_buffer[128] = {0}, rxLen = 0;

BoardConfig_t boardConfig;
Motor motor;
MT6816Base mt6816_base((uint16_t *)(0x08017C00));
TB67H450Base tb67h450_base;
EncoderCalibratorBase encoder_calibrator_base;
ButtonBase button1(GPIOB, GPIO_PIN_2, 1, 1000), button2(GPIOB, GPIO_PIN_12, 2, 1000);

extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t _a;
extern uint16_t _v;

uint16_t aaa = 0;
uint8_t rec_buff[100] = {0};

extern "C" void Main()
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

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

    // HAL_UART_Receive_DMA(&huart1, rx_buffer, 128);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rec_buff, 100);

    motor.AttachEncoder(&mt6816_base);

    boardConfig = BoardConfig_t{
        .configStatus = CONFIG_OK,
        // .canNodeId = defaultNodeID,
        .encoderHomeOffset = 0,
        .defaultMode = Motor::MODE_COMMAND_POSITION,
        .currentLimit = 1 * 1000,                                     // A
        .velocityLimit = 30 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s,51200*30
        .velocityAcc = 1000000,                                       // r/s^2,5120000
        .calibrationCurrent = 2000,
        .dce_kp = 200,
        .dce_kv = 80,
        .dce_ki = 300,
        .dce_kd = 250,
        .enableMotorOnBoot = false,
        .enableStallProtect = false};

    motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 900);

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
        Upload_estposition();
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }

    uint8_t id = (RxHeader.StdId >> 7);  // 4Bits ID & 7Bits Msg
    uint8_t cmd = RxHeader.StdId & 0x7F; // 4Bits ID & 7Bits Msg
    if (id == 0 || id == 1)
    {
        OnCanCmd(cmd, RxData, RxHeader.DLC);
    }
}
