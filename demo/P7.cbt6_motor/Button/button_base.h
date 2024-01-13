#ifndef CTRL_STEP_FW_BUTTON_BASE_H
#define CTRL_STEP_FW_BUTTON_BASE_H

#include <cstdint>
#include <gpio.h>

class ButtonBase
{
public:
    enum Event
    {
        UP,         // 抬起
        DOWN,       // 按下
        LONG_PRESS, // 抬起超时为长按
        CLICK       // 抬起不超时为点击
    };

    explicit ButtonBase(uint8_t _id) : id(_id)
    {
    }

    ButtonBase(GPIO_TypeDef *_GPIO_x, uint16_t _GPIO_PIN, uint8_t _id, uint32_t _longPressTime) : GPIO_x(_GPIO_x), GPIO_PIN(_GPIO_PIN), id(_id), longPressTime(_longPressTime)
    {
    }

    void Tick(uint32_t _timeElapseMillis);
    void SetOnEventListener(void (*_callback)(Event)); // 函数SetOnEventListener的参数是 参数为Event的函数，即函数作为参数传递给函数

    bool IsPressed();

protected:
    uint8_t id;
    bool lastPinIO{};
    uint32_t timer = 0;
    uint32_t pressTime{};
    uint32_t longPressTime = 2000;

    GPIO_TypeDef *GPIO_x;
    uint16_t GPIO_PIN;

    void (*OnEventFunc)(Event){};
    // 读取id的电平
    bool ReadButtonPinIO(GPIO_TypeDef *GPIO_x, uint16_t GPIO_PIN, uint8_t _id);
};

#endif
