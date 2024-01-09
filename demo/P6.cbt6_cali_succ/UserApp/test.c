#include "common_inc.h"

uint16_t _v = 0;

void test1()
{
    _v++;
    if (_v == 1000)
    {
        _v = 0;
    }
}