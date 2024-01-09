#include "common_inc.h"

uint16_t _a = 0;

void test()
{
    _a++;
    if (_a == 1000)
    {
        _a = 0;
    }
}