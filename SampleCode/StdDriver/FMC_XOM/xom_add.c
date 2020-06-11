/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show how to use XOM Lirbary
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

int32_t Lib_XOM_ADD(uint32_t u32Val1, uint32_t u32Val2)
{
    uint32_t u32Result;
    u32Result =  u32Val1 + u32Val2;
    return u32Result;
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
