/**************************************************************************//**
 * @file        WS1812B_driver_LED.h
 * @version     V3.00
 * @brief       Worldsemi WS2812B LED Driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __WS2812B_DEIVER_LED_H__
#define __WS2812B_DEIVER_LED_H__

#define WS2812B_GREEN   0xFF0000
#define WS2812B_RED     0x00FF00
#define WS2812B_BLUE    0x0000FF
#define WS2812B_WHITE   0xFFFFFF

enum
{
    eERROR_PIN_NUMBER = -1,
    eERROR_MEMORY_ADDR = -2,
};

typedef uint8_t WS2812B_LED_Pin_CFG[8];

typedef struct PSIO_WS2812B_LED_CFG
{
    uint8_t             u8SlotCtrl;
    uint8_t             u8PDMAChannel;
    uint8_t             *pu8PinCFG;
    uint8_t             u8PinNumber;
    uint32_t            *pu32DataAddr;
    uint32_t            u32DataLength;
    uint32_t            *pu32InternalMemory;
} S_PSIO_WS2812B_LED_CFG;


int PSIO_WS2812B_Send_Pattern(S_PSIO_WS2812B_LED_CFG *psConfig);
int PSIO_WS2812B_Open(S_PSIO_WS2812B_LED_CFG *psConfig);
void PSIO_WS2812B_Close(S_PSIO_WS2812B_LED_CFG *psConfig);

#endif  //__WS2812B_DEIVER_LED_H__

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
