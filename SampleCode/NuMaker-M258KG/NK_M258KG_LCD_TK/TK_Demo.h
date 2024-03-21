/**************************************************************************//**
 * @file     TK_Demo.h
 * @version  V1.00
 * @brief    Touch key header file for demo
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __TK_DEMO_H__
#define __TK_DEMO_H__


#define OPT_SLIDER
#define OPT_WHEEL

#define CLK_SOURCE                       CLK_HIRC

#define DEBUG_PORT                       UART1
#define TK_UART_PORT                     UART0
#define DBG_PRINTF(...)                 //printf

#define UART1_DBG                                     /* Trace code with UART1 message */

#define FIRMWARE_MAJOR_VERSION            (0x01)
#define FIRMWARE_MINOR_VERSION            (0x11)

#define USER_ID                           (('N'<<24) | ('U'<<16) | ('V'<<8) | 'O')

#define TK_SCAN_EVENT_FREQ                (50)
#define TK_SE_NUM                         (2)

//TK_Param.c
int8_t TK_LoadPara(uint32_t *pu32ChanelMsk);
int8_t TK_SavePara(void);

//TK_Led.c
void InitLEDIO(void);
void TK_lightLED(uint8_t onOff, int chanN);

//TK_MultPinFunctions.c
void SetTkMultiFun(uint32_t u32TkMsk);

void _TK_FMC_Open(void);
uint32_t _TK_FMC_Erase(uint32_t u32Addr);
uint32_t _TK_FMC_Write(uint32_t u32Addr, uint32_t u32Data);

void UART0_Init(void);
void UART1_Init(void);

void Init_SysTick(void);
char TickSetTickEvent(unsigned long uTimeTick, void *pvFun);
void TickClearTickEvent(uint8_t u8TimeEventID);

/* TK_UartCmd.c */
void UART_SetCalibrationDone(void);

#endif /* __TK_DEMO_H__ */


