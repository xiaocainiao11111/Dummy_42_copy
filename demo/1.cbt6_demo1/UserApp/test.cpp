#include "test.h"

void Test::test1()
{
    _b++;
    if (_b == 1000)
    {
        _b = 0;
    }
}