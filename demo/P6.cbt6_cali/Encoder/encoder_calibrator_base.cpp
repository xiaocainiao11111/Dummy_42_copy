
#include <valarray>
#include "encoder_calibrator_base.h"
#include "stockpile_f103cb.h"
#include "mt6816_base.h"
#include "tb67h450_base.h"

extern MT6816Base mt6816_base;
extern TB67H450Base tb67h450_base;

void EncoderCalibratorBase::BeginWriteFlash()
{
    Stockpile_Flash_Data_Begin(&stockpile_quick_cali);
}

void EncoderCalibratorBase::EndWriteFlash()
{
    Stockpile_Flash_Data_End(&stockpile_quick_cali);
}

void EncoderCalibratorBase::ClearFlash()
{
    Stockpile_Flash_Data_Empty(&stockpile_quick_cali);
}

// 记录编码器从0开始的对应细分数
void EncoderCalibratorBase::WriteFlash16bitsAppend(uint16_t _data)
{
    Stockpile_Flash_Data_Write_Data16(&stockpile_quick_cali, &_data, 1);
}

// 获取样本数据循环平均值
int32_t EncoderCalibratorBase::CycleDataAverage(const uint16_t *_data, uint16_t _length, int32_t _cyc)
{
    int32_t sumData = 0;
    int32_t subData;
    int32_t diffData;

    sumData += (int32_t)_data[0];
    for (uint16_t i = 1; i < _length; i++)
    {
        diffData = (int32_t)_data[i];
        subData = (int32_t)_data[i] - (int32_t)_data[0]; // mt6816样本数据总差
        if (subData > (_cyc >> 1))
            diffData = (int32_t)_data[i] - _cyc;
        if (subData < (-_cyc >> 1))
            diffData = (int32_t)_data[i] + _cyc;
        sumData += diffData;
    }

    sumData = sumData / _length;

    if (sumData < 0)
        sumData += _cyc;
    if (sumData > _cyc)
        sumData -= _cyc;

    return sumData;
}

void EncoderCalibratorBase::CalibrationDataCheck()
{
    uint32_t count;
    int32_t subData;

    int32_t calibSampleResolution = mt6816_base.RESOLUTION / MOTOR_ONE_CIRCLE_HARD_STEPS; // 81
    for (count = 0; count < MOTOR_ONE_CIRCLE_HARD_STEPS + 1; count++)                     // 由正转值和反转值再求平均值
    {
        sampleDataAverageForward[count] = (uint16_t)CycleAverage((int32_t)sampleDataAverageForward[count],
                                                                 (int32_t)sampleDataAverageBackward[count],
                                                                 mt6816_base.RESOLUTION);
    }
    subData = CycleSubtract((int32_t)sampleDataAverageForward[0],
                            (int32_t)sampleDataAverageForward[MOTOR_ONE_CIRCLE_HARD_STEPS - 1],
                            mt6816_base.RESOLUTION); // 由第一次和最后一次的数据的差值得到正负值代表方向
    if (subData == 0)                                // 第一次和最后一次重叠，返回错误
    {
        errorCode = CALI_ERROR_AVERAGE_DIR;
        return;
    }
    else
    {
        goDirection = subData > 0; // 1代表顺时针
    }

    for (count = 1; count < MOTOR_ONE_CIRCLE_HARD_STEPS; count++)
    {
        subData = CycleSubtract((int32_t)sampleDataAverageForward[count],
                                (int32_t)sampleDataAverageForward[count - 1],
                                mt6816_base.RESOLUTION);     // 取每相邻的两个数据的差值进行判断，正负百分之50内通过
        if (fabs(subData) > (calibSampleResolution * 3 / 2)) // delta-data too large，121
        {
            errorCode = CALI_ERROR_AVERAGE_CONTINUTY;
            return;
        }
        if (fabs(subData) < (calibSampleResolution * 1 / 2)) // delta-data too small，40
        {
            errorCode = CALI_ERROR_AVERAGE_CONTINUTY;
            return;
        }
        if (subData == 0)
        {
            errorCode = CALI_ERROR_AVERAGE_DIR;
            return;
        }
        if ((subData > 0) && (!goDirection)) // 方向不同
        {
            errorCode = CALI_ERROR_AVERAGE_DIR;
            return;
        }
        if ((subData < 0) && (goDirection))
        {
            errorCode = CALI_ERROR_AVERAGE_DIR;
            return;
        }
    }

    uint32_t step_num = 0;
    if (goDirection) // 顺时针
    {
        for (count = 0; count < MOTOR_ONE_CIRCLE_HARD_STEPS; count++)
        {
            subData = (int32_t)sampleDataAverageForward[CycleMod(count + 1, MOTOR_ONE_CIRCLE_HARD_STEPS)] -
                      (int32_t)sampleDataAverageForward[CycleMod(count, MOTOR_ONE_CIRCLE_HARD_STEPS)];
            if (subData < 0) // 当有一次穿过时钟零点
            {
                step_num++;
                rcdX = (int32_t)count; // 记录零点对应步数位置
                rcdY = (mt6816_base.RESOLUTION - 1) -
                       sampleDataAverageForward[CycleMod(rcdX, MOTOR_ONE_CIRCLE_HARD_STEPS)]; // 16383-零点对应的mt6816校准数据
            }
        }
        if (step_num != 1)
        {
            errorCode = CALI_ERROR_PHASE_STEP;
            return;
        }
    }
    else // 逆时针
    {
        for (count = 0; count < MOTOR_ONE_CIRCLE_HARD_STEPS; count++)
        {
            subData = (int32_t)sampleDataAverageForward[CycleMod(count + 1, MOTOR_ONE_CIRCLE_HARD_STEPS)] -
                      (int32_t)sampleDataAverageForward[CycleMod(count, MOTOR_ONE_CIRCLE_HARD_STEPS)];
            if (subData > 0)
            {
                step_num++;
                rcdX = (int32_t)count;
                rcdY = (mt6816_base.RESOLUTION - 1) -
                       sampleDataAverageForward[CycleMod(rcdX + 1, MOTOR_ONE_CIRCLE_HARD_STEPS)];
            }
        }
        if (step_num != 1)
        {
            errorCode = CALI_ERROR_PHASE_STEP;
            return;
        }
    }

    errorCode = CALI_NO_ERROR;
}

// 默认2000电流
void EncoderCalibratorBase::Tick20kHz()
{
    mt6816_base.UpdateAngle();

    switch (state)
    {
    case CALI_DISABLE:
        if (isTriggered)
        {
            tb67h450_base.SetFocCurrentVector(goPosition, 2000);
            goPosition = 51200;
            sampleCount = 0;
            state = CALI_FORWARD_PREPARE;
            errorCode = CALI_NO_ERROR;
        }
        break;
    case CALI_FORWARD_PREPARE:
        goPosition += AUTO_CALIB_SPEED;
        tb67h450_base.SetFocCurrentVector(goPosition, 2000);
        if (goPosition == 2 * 51200) // 转一圈
        {
            goPosition = 51200;
            state = CALI_FORWARD_MEASURE;
        }
        break;
    case CALI_FORWARD_MEASURE:
        if ((goPosition % 256) == 0) // 记录每1.8度的数据16次
        {
            sampleDataRaw[sampleCount++] = mt6816_base.angleData.rawAngle;
            if (sampleCount == EncoderCalibratorBase::SAMPLE_COUNTS_PER_STEP)
            {
                // 取平均值放到数组里
                sampleDataAverageForward[(goPosition - 51200) /
                                         256] = CycleDataAverage(sampleDataRaw, EncoderCalibratorBase::SAMPLE_COUNTS_PER_STEP,
                                                                 mt6816_base.RESOLUTION);

                sampleCount = 0;
                goPosition += FINE_TUNE_CALIB_SPEED;
            }
        }
        else
        {
            goPosition += FINE_TUNE_CALIB_SPEED;
        }

        tb67h450_base.SetFocCurrentVector(goPosition, 2000);

        if (goPosition > (2 * 51200)) // 跑完一圈
        {
            state = CALI_BACKWARD_RETURN;
        }
        break;
    case CALI_BACKWARD_RETURN:
        goPosition += FINE_TUNE_CALIB_SPEED;
        tb67h450_base.SetFocCurrentVector(goPosition, 2000);

        if (goPosition ==
            (2 * 51200 + 256 * 20))
        { // 继续跑20个1.8度后退出
            state = CALI_BACKWARD_GAP_DISMISS;
        }
        break;
    case CALI_BACKWARD_GAP_DISMISS:
        goPosition -= FINE_TUNE_CALIB_SPEED;
        tb67h450_base.SetFocCurrentVector(goPosition, 2000);
        if (goPosition == (2 * 51200))
        { // 回到起点
            state = CALI_BACKWARD_MEASURE;
        }
        break;
    case CALI_BACKWARD_MEASURE:
        if ((goPosition % 256) == 0)
        {
            sampleDataRaw[sampleCount++] = mt6816_base.angleData.rawAngle;
            if (sampleCount == EncoderCalibratorBase::SAMPLE_COUNTS_PER_STEP)
            {
                sampleDataAverageBackward[(goPosition - 51200) /
                                          256] = CycleDataAverage(sampleDataRaw, EncoderCalibratorBase::SAMPLE_COUNTS_PER_STEP,
                                                                  mt6816_base.RESOLUTION);

                sampleCount = 0;
                goPosition -= FINE_TUNE_CALIB_SPEED;
            }
        }
        else
        {
            goPosition -= FINE_TUNE_CALIB_SPEED;
        }
        tb67h450_base.SetFocCurrentVector(goPosition, 2000);
        if (goPosition < 51200)
        {
            state = CALI_CALCULATING;
        }
        break;
    case CALI_CALCULATING: // 完成基础校准停机等待处理
        tb67h450_base.SetFocCurrentVector(0, 0);
        break;
    default:
        break;
    }
}

void EncoderCalibratorBase::TickMainLoop()
{
    int32_t dataI32;  // 每步数据差
    uint16_t dataU16; // 对应细分数

    if (state != CALI_CALCULATING) // 完成校准后进入
        return;

    tb67h450_base.Sleep();

    CalibrationDataCheck(); // 得到零点步数及零点数据

    if (errorCode == CALI_NO_ERROR) // 正反转数据处理和零点处理成功
    {
        int32_t stepX, stepY;
        resultNum = 0;

        ClearFlash();
        BeginWriteFlash();

        if (goDirection)
        {
            for (stepX = rcdX; stepX < rcdX + 200 + 1; stepX++) // 算出编码器每个数对应的细分数
            {
                dataI32 = CycleSubtract(
                    sampleDataAverageForward[CycleMod(stepX + 1, 200)],
                    sampleDataAverageForward[CycleMod(stepX, 200)],
                    mt6816_base.RESOLUTION);
                if (stepX == rcdX)
                {
                    for (stepY = rcdY; stepY < dataI32; stepY++)
                    {
                        dataU16 = CycleMod(256 * stepX +
                                               256 * stepY / dataI32,
                                           51200);
                        WriteFlash16bitsAppend(dataU16); // 编码器数据0对应细分数
                        resultNum++;
                    }
                }
                else if (stepX == rcdX + 200) // 回到rcdX
                {
                    for (stepY = 0; stepY < rcdY; stepY++)
                    {
                        dataU16 = CycleMod(256 * stepX +
                                               256 * stepY / dataI32,
                                           51200);
                        WriteFlash16bitsAppend(dataU16);
                        resultNum++;
                    }
                }
                else
                {
                    for (stepY = 0; stepY < dataI32; stepY++)
                    {
                        dataU16 = CycleMod(256 * stepX +
                                               256 * stepY / dataI32,
                                           51200);
                        WriteFlash16bitsAppend(dataU16);
                        resultNum++;
                    }
                }
            }
        }
        else
        {
            for (stepX = rcdX + 200; stepX > rcdX - 1; stepX--)
            {
                dataI32 = CycleSubtract(
                    sampleDataAverageForward[CycleMod(stepX, 200)],
                    sampleDataAverageForward[CycleMod(stepX + 1, 200)],
                    mt6816_base.RESOLUTION);
                if (stepX == rcdX + 200)
                {
                    for (stepY = rcdY; stepY < dataI32; stepY++)
                    {
                        dataU16 = CycleMod(
                            256 * (stepX + 1) -
                                256 * stepY / dataI32,
                            51200);
                        WriteFlash16bitsAppend(dataU16);
                        resultNum++;
                    }
                }
                else if (stepX == rcdX)
                {
                    for (stepY = 0; stepY < rcdY; stepY++)
                    {
                        dataU16 = CycleMod(
                            256 * (stepX + 1) -
                                256 * stepY / dataI32,
                            51200);
                        WriteFlash16bitsAppend(dataU16);
                        resultNum++;
                    }
                }
                else
                {
                    for (stepY = 0; stepY < dataI32; stepY++)
                    {
                        dataU16 = CycleMod(
                            256 * (stepX + 1) -
                                256 * stepY / dataI32,
                            51200);
                        WriteFlash16bitsAppend(dataU16);
                        resultNum++;
                    }
                }
            }
        }

        EndWriteFlash();

        if (resultNum != mt6816_base.RESOLUTION)
            errorCode = CALI_ERROR_ANALYSIS_QUANTITY;
    }

    if (errorCode == CALI_NO_ERROR)
    {
        mt6816_base.angleData.rectifyValid = true;
    }
    else
    {
        mt6816_base.angleData.rectifyValid = false;
        ClearFlash();
    }

    // motor->controller->isStalled = true; // 开启堵转保护，实际上在校准成功后只进一次

    state = CALI_DISABLE; // 返回校准流程起点
    isTriggered = false;

    if (errorCode == CALI_NO_ERROR)
        HAL_NVIC_SystemReset(); // 复位芯片
}

// 取余，限制在200内
uint32_t EncoderCalibratorBase::CycleMod(uint32_t _a, uint32_t _b)
{
    return (_a + _b) % _b;
}

// 取循环距离差，返回两者在圆上的距离，指在圆上距离小的那段值，有正负，a大b小
int32_t EncoderCalibratorBase::CycleSubtract(int32_t _a, int32_t _b, int32_t _cyc)
{
    int32_t sub_data;

    sub_data = _a - _b;
    if (sub_data > (_cyc >> 1))
        sub_data -= _cyc;
    if (sub_data < (-_cyc >> 1))
        sub_data += _cyc;
    return sub_data;
}

// 返回a,b中值，取两者距离小的那段的中值
int32_t EncoderCalibratorBase::CycleAverage(int32_t _a, int32_t _b, int32_t _cyc)
{
    int32_t sub_data;
    int32_t ave_data;

    sub_data = _a - _b;
    ave_data = (_a + _b) >> 1;

    if (fabs(sub_data) > (_cyc >> 1)) //
    {
        if (ave_data >= (_cyc >> 1))
            ave_data -= (_cyc >> 1);
        else
            ave_data += (_cyc >> 1);
    }
    return ave_data;
}