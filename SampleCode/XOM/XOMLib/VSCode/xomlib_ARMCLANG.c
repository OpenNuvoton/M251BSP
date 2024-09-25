/**************************************************************************//**
 * @file    xomlib.c
 * @version V3.00
 * @brief   Function pointer for XOM APIs.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/*------------------------------*/
/*  XOMLib address definitions  */
/*------------------------------*/
#define XOM_ADD_ADDR    0x00003001
#define XOM_SUB_ADDR    0x0000303F
#define XOM_MUL_ADDR    0x0000302B
#define XOM_DIV_ADDR    0x00003015
#define XOM_SUM_ADDR    0x00003053

int32_t (*XOM_Add)(int32_t a,     int32_t b) = (int32_t (*)(int32_t a,     int32_t b))(XOM_ADD_ADDR);
int32_t (*XOM_Div)(int32_t a,     int32_t b) = (int32_t (*)(int32_t a,     int32_t b))(XOM_DIV_ADDR);
int32_t (*XOM_Mul)(int32_t a,     int32_t b) = (int32_t (*)(int32_t a,     int32_t b))(XOM_MUL_ADDR);
int32_t (*XOM_Sub)(int32_t a,     int32_t b) = (int32_t (*)(int32_t a,     int32_t b))(XOM_SUB_ADDR);
int32_t (*XOM_Sum)(int32_t *pbuf, int32_t n) = (int32_t (*)(int32_t *pbuf, int32_t n))(XOM_SUM_ADDR);
