#include "button_base.h"

void ButtonBase::Tick(uint32_t _timeElapseMillis)
{
    timer += _timeElapseMillis;
    bool pinIO = ReadButtonPinIO(id);

    //根据每次发生电平变化时，按下为低电平，感觉应该加滤波
    if (lastPinIO != pinIO)
    {
        if (pinIO)
        {
            OnEventFunc(UP);
            if (timer - pressTime > longPressTime)
                OnEventFunc(LONG_PRESS);
            else
                OnEventFunc(CLICK);
        } else
        {
            OnEventFunc(DOWN);
            pressTime = timer;
        }

        lastPinIO = pinIO;
    }
}

void ButtonBase::SetOnEventListener(void (* _callback)(Event))
{
    lastPinIO =  ReadButtonPinIO(id);

    OnEventFunc = _callback;
}
