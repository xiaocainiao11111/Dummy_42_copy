#include "common_inc.h"

extern uint16_t _a;


uint16_t _h=0;

Test test1;

void Main()
{

    while (1)
    {
        test();
        test1.test1();

        if(test1._b==1)
        {
            _h++;
            if(_h==1000)
            {
                _h=0;
            }
            // _h=1;
        }



    }
}


