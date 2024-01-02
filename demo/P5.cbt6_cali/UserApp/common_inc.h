#ifndef __LOOP_H
#define __LOOP_H

#ifdef __cplusplus
extern "C"
{
#endif
/*------------- C Scope -------------*/
#include "stdint.h"
#include "main.h"
#include "tim.h"
    void Main(void);



#ifdef __cplusplus
}
/*-------------- C++ Scope --------*/
/*带<cstdint>的放下面*/
#include <cstdio>
    // C++测试
    class Test
    {
    public:
        Test()
        {
        }
        explicit Test(uint8_t _id) : id(_id)
        {
        }
        enum Event
        {
        };
        void tick(void);
        uint16_t num;

    protected:
        uint8_t id;

    private:
        bool flag;
    };
#endif
#endif
