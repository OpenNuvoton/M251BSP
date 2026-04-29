/******************************************************************************
 * @file     xomlib.c
 * @version  V3.00
 * @brief    Function pointer for XOM APIs.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

int32_t (*XOM_Add)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x3001);
int32_t (*XOM_Div)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x300d);
int32_t (*XOM_Mul)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x3009);
int32_t (*XOM_Sub)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x3005);
int32_t (*XOM_Sum)(int32_t *pbuf, int32_t n) = (int32_t (*)(int32_t *pbuf, int32_t n))(0x3013);
