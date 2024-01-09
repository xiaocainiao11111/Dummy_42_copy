#include "encoder_calibrator_stm32.h"
#include "stockpile_f103cb.h"

void EncoderCalibrator::BeginWriteFlash()
{
    Stockpile_Flash_Data_Begin(&stockpile_quick_cali);
}


void EncoderCalibrator::EndWriteFlash()
{
    Stockpile_Flash_Data_End(&stockpile_quick_cali);
}


void EncoderCalibrator::ClearFlash()
{
    Stockpile_Flash_Data_Empty(&stockpile_quick_cali);
}

//记录编码器从0开始的对应细分数
void EncoderCalibrator::WriteFlash16bitsAppend(uint16_t _data)
{
    Stockpile_Flash_Data_Write_Data16(&stockpile_quick_cali, &_data, 1);
}
