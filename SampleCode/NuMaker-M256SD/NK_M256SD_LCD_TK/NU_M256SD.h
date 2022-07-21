/**************************************************************************//**
 * @file     NU_M256SD.h
 * @version  V1.00
 * @brief    For NuMaker-M256SD header file.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef _NU_M256_SD_H_
#define _NU_M256_SD_H_

typedef enum
{
    eMAIN_APP_IDLE_STATE,
    eMAIN_APP_TK_STATE,
    eMAIN_APP_IDLE_HOLD_STATE,
    eMAIN_APP_TK_HOLD_STATE,
} E_MAIN_APP_STATE;

extern unsigned char tkct;
unsigned char internal_Temperature();

void LCD_Init_Setting();
void LCD_frame1();
void LCD_frame2();

#endif
