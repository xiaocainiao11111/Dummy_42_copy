#include "common_inc.h"
#include "configurations.h"
#include "Platform/Utils/st_hardware.h"
#include <tim.h>

/* Component Definitions -----------------------------------------------------*/
// BoardConfig_t boardConfig;
// Motor motor;
// TB67H450 tb67H450;
// MT6816 mt6816;
// EncoderCalibrator encoderCalibrator(&motor);
Button button1(1, 1000), button2(2, 3000);
void OnButton1Event(Button::Event _event);
void OnButton2Event(Button::Event _event);
// Led statusLed;

uint8_t button_num = 0;
uint64_t serialNum = 0;
uint16_t defaultNodeID = 0;
/* Main Entry ----------------------------------------------------------------*/
void Main()
{
    serialNum = GetSerialNumber();
    defaultNodeID = 0;
    // // Change below to fit your situation
    switch (serialNum)
    {
    case 0x6d7730845187:
        defaultNodeID = 1;
        //     case 431466563640: //J1
        //         defaultNodeID = 1;
        //         break;
        //     case 384624576568: //J2
        //         defaultNodeID = 2;
        //         break;
        //     case 384290670648: //J3
        //         defaultNodeID = 3;
        //         break;
        //     case 431531051064: //J4
        //         defaultNodeID = 4;
        //         break;
        //     case 431466760248: //J5
        //         defaultNodeID = 5;
        //         break;
        //     case 431484848184: //J6
        //         defaultNodeID = 6;
        //         break;
    default:
        break;
    }

    // /*---------- Apply EEPROM Settings ----------*/
    // // Setting priority is EEPROM > Motor.h
    EEPROM eeprom;
    // eeprom.get(0, boardConfig);
    // if (boardConfig.configStatus != CONFIG_OK) // use default settings
    // {
    //     boardConfig = BoardConfig_t{
    //         .configStatus = CONFIG_OK,
    //         .canNodeId = defaultNodeID,
    //         .encoderHomeOffset = 0,
    //         .defaultMode = Motor::MODE_COMMAND_POSITION,
    //         .currentLimit = 1 * 1000,    // A
    //         .velocityLimit = 30 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s
    //         .velocityAcc = 100 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,   // r/s^2
    //         .calibrationCurrent=2000,
    //         .dce_kp = 200,
    //         .dce_kv = 80,
    //         .dce_ki = 300,
    //         .dce_kd = 250,
    //         .enableMotorOnBoot=false,
    //         .enableStallProtect=false
    //     };
    //     eeprom.put(0, boardConfig);
    // }
    // motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset;
    // motor.config.motionParams.ratedCurrent = boardConfig.currentLimit;
    // motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
    // motor.config.motionParams.ratedVelocityAcc = boardConfig.velocityAcc;
    // motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);
    // motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
    // motor.config.motionParams.caliCurrent = boardConfig.calibrationCurrent;
    // motor.config.ctrlParams.dce.kp = boardConfig.dce_kp;
    // motor.config.ctrlParams.dce.kv = boardConfig.dce_kv;
    // motor.config.ctrlParams.dce.ki = boardConfig.dce_ki;
    // motor.config.ctrlParams.dce.kd = boardConfig.dce_kd;
    // motor.config.ctrlParams.stallProtectSwitch = boardConfig.enableStallProtect;

    // /*---------------- Init Motor ----------------*/
    // motor.AttachDriver(&tb67H450);
    // motor.AttachEncoder(&mt6816);
    // motor.controller->Init();
    // motor.driver->Init();
    // motor.encoder->Init();

    // /*------------- Init peripherals -------------*/
    button1.SetOnEventListener(OnButton1Event);
    button2.SetOnEventListener(OnButton2Event);

    // /*------- Start Close-Loop Control Tick ------*/
    HAL_Delay(100);
    HAL_TIM_Base_Start_IT(&htim1); // 100Hz
    HAL_TIM_Base_Start_IT(&htim4); // 20kHz

    // if (button1.IsPressed() && button2.IsPressed())
    //     encoderCalibrator.isTriggered = true;

    for (;;)
    {
        // encoderCalibrator.TickMainLoop();

        // if (boardConfig.configStatus == CONFIG_COMMIT)
        // {
        //     boardConfig.configStatus = CONFIG_OK;
        //     eeprom.put(0, boardConfig);
        // } else if (boardConfig.configStatus == CONFIG_RESTORE)
        // {
        //     eeprom.put(0, boardConfig);
        //     HAL_NVIC_SystemReset();
        // }
    }
}

/* Event Callbacks -----------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        button1.Tick(10);
        button2.Tick(10);
    }
    if (htim->Instance == TIM4)
    {
    }
}

void OnButton1Event(Button::Event _event)
{
    switch (_event)
    {
    case ButtonBase::UP:
        button_num = 1;
        break;
    case ButtonBase::DOWN:
        button_num = 2;
        break;
    case ButtonBase::LONG_PRESS:
        button_num = 3;
        // HAL_NVIC_SystemReset();
        break;
    case ButtonBase::CLICK:
        button_num = 4;
        // if (motor.controller->modeRunning != Motor::MODE_STOP)
        // {
        //     boardConfig.defaultMode = motor.controller->modeRunning;
        //     motor.controller->requestMode = Motor::MODE_STOP;
        // } else
        // {
        //     motor.controller->requestMode = static_cast<Motor::Mode_t>(boardConfig.defaultMode);
        // }
        break;
    }
}

void OnButton2Event(Button::Event _event)
{
    switch (_event)
    {
    case ButtonBase::UP:
        button_num = 5;
        break;
    case ButtonBase::DOWN:
        button_num = 6;
        break;
    case ButtonBase::LONG_PRESS:
        button_num = 7;
        // switch (motor.controller->modeRunning)
        // {
        // case Motor::MODE_COMMAND_CURRENT:
        // case Motor::MODE_PWM_CURRENT:
        //     motor.controller->SetCurrentSetPoint(0);
        //     break;
        // case Motor::MODE_COMMAND_VELOCITY:
        // case Motor::MODE_PWM_VELOCITY:
        //     motor.controller->SetVelocitySetPoint(0);
        //     break;
        // case Motor::MODE_COMMAND_POSITION:
        // case Motor::MODE_PWM_POSITION:
        //     motor.controller->SetPositionSetPoint(0);
        //     break;
        // case Motor::MODE_COMMAND_Trajectory:
        // case Motor::MODE_STEP_DIR:
        // case Motor::MODE_STOP:
        //     break;
        // }
        break;
    case ButtonBase::CLICK:
        button_num = 8;
        // motor.controller->ClearStallFlag();
        break;
    }
}