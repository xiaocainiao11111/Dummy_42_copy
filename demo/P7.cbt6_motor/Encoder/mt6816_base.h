#ifndef CTRL_STEP_FW_MT6816_H
#define CTRL_STEP_FW_MT6816_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "encoder_base.h"
#include <cstdint>

    class MT6816Base
    {
    public:
        explicit MT6816Base(uint16_t *_quickCaliDataPtr) : quickCaliDataPtr(_quickCaliDataPtr)
        {
        }

        const int32_t RESOLUTION = ((int32_t)((0x00000001U) << 14));

        // bool Init() override;
        typedef struct
        {
            uint16_t rawAngle;       // raw data，转成mt6816的14位数据
            uint16_t rectifiedAngle; // calibrated rawAngle data，校准的数据，实际上存放的是对应细分数
            bool rectifyValid;       // 校准成功并写入flash为true
        } AngleData_t;
        AngleData_t angleData{0};

        void Init();

        uint8_t _g = 0;
        void test();
        uint8_t test1();
        uint16_t UpdateAngle(); // Get current rawAngle (rad)
        bool IsCalibrated();

    private:
        typedef struct
        {
            uint16_t rawData;  // SPI raw 16bits data，spi16位数据
            uint16_t rawAngle; // 14bits rawAngle in rawData，转成mt6816的14位数据
            bool noMagFlag;
            bool checksumFlag;
        } SpiRawData_t;

        SpiRawData_t spiRawData;
        uint16_t *quickCaliDataPtr;
        uint16_t dataTx[2];
        uint16_t dataRx[2];
        uint8_t hCount;

        /***** Port Specified Implements *****/
        void SpiInit();

        uint16_t SpiTransmitAndRead16Bits(uint16_t _dataTx);
    };

    extern MT6816Base mt6816_base;

#ifdef __cplusplus
}
#endif

#endif
