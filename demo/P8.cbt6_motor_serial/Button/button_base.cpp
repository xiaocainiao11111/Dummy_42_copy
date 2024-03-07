#include "button_base.h"

void ButtonBase::Tick(uint32_t _timeElapseMillis)
{
    timer += _timeElapseMillis;
    bool pinIO = ReadButtonPinIO(&GPIO_x, GPIO_PIN, id);

    // 根据每次发生电平变化时，按下为低电平，感觉应该加滤波
    if (lastPinIO != pinIO)
    {
        if (pinIO)
        {
            OnEventFunc(UP);
            if (timer - pressTime > longPressTime)
                OnEventFunc(LONG_PRESS);
            else
                OnEventFunc(CLICK);
        }
        else
        {
            OnEventFunc(DOWN);
            pressTime = timer;
        }

        lastPinIO = pinIO;
    }
}

void ButtonBase::SetOnEventListener(void (*_callback)(Event))
{
    lastPinIO = ReadButtonPinIO(&GPIO_x, GPIO_PIN, id);

    OnEventFunc = _callback;
}

bool ButtonBase::ReadButtonPinIO(GPIO_TypeDef *_GPIO_x, uint16_t _GPIO_PIN, uint8_t _id)
{
    switch (_id)
    {
    case 1:
        return HAL_GPIO_ReadPin(_GPIO_x, _GPIO_PIN) == GPIO_PIN_SET;
    case 2:
        return HAL_GPIO_ReadPin(_GPIO_x, _GPIO_PIN) == GPIO_PIN_SET;
    default:
        return false;
    }
}
bool ButtonBase::IsPressed()
{
    return !ReadButtonPinIO(id);
}
