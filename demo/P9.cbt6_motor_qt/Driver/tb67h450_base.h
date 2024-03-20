#ifndef CTRL_STEP_FW_TB67H450_BASE_H
#define CTRL_STEP_FW_TB67H450_BASE_H

#ifdef __cplusplus
extern "C"
{
#endif

// #include "driver_base.h"
#include <cstdint>
    class TB67H450Base
    {
    public:
        explicit TB67H450Base() = default;

        // void Init();

        void SetFocCurrentVector(uint32_t _directionInCount, int32_t _current_mA);

        void Sleep();

        void Brake();

    protected:
        void SetTwoCoilsCurrent(uint16_t _currentA_3300mAIn12Bits, uint16_t _currentB_3300mAIn12Bits);

        /***** Port Specified Implements *****/
        // void InitGpio();

        // void InitPwm();

        void DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits);

        void SetInputA(bool _statusAp, bool _statusAm);

        void SetInputB(bool _statusBp, bool _statusBm);

        typedef struct
        {
            uint16_t sinMapPtr;
            int16_t sinMapData;
            uint16_t dacValue12Bits;
        } FastSinToDac_t;

        FastSinToDac_t phaseB{};
        FastSinToDac_t phaseA{};
    };
    extern TB67H450Base tb67h450_base;

#ifdef __cplusplus
}
#endif
#endif
