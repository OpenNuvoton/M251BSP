/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    XOM library  --  Add function
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"


int32_t XOM_Add(int32_t i32Val1, int32_t i32Val2)
{
    uint32_t u32Result;

    u32Result =  i32Val1 + i32Val2;

    return u32Result;
}


int32_t XOM_Sub(int32_t i32Val1, int32_t i32Val2)
{
    uint32_t u32Result;

    u32Result =  i32Val1 - i32Val2;

    return u32Result;
}


int32_t XOM_Mul(int32_t i32Val1, int32_t i32Val2)
{
    uint32_t u32Result;

    u32Result =  i32Val1 * i32Val2;

    return u32Result;
}


int32_t XOM_Div(int32_t i32Val1, int32_t i32Val2)
{
    uint32_t u32Result;

    u32Result =  i32Val1 / i32Val2;

    return u32Result;
}


int32_t XOM_Sum(int32_t *pi32buf, int32_t i32n)
{
    int32_t i32LoopCnt;
    int32_t i32Sum;

    i32Sum = 0;

    for (i32LoopCnt = 0; i32LoopCnt < i32n; i32LoopCnt++)
    {
        i32Sum += pi32buf[i32LoopCnt];
    }

    return i32Sum;
}
