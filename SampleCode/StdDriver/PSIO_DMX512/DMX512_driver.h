/***************************************************************************//**
 * @file        DMX512_driver.c
 * @version     V3.00
 * @brief       DMX512 Driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __DMX512_DRIVER_H__
#define __DMX512_DRIVER_H__

#include "NuMicro.h"

#define BITRATE_DMX512 250000UL     /* 250K bps */

/* DMX512 11Bit Raw Frame for packet recognition */
#define DMX512_PATTERN_BREAK 0x000UL
#define DMX512_PATTERN_START 0x600UL
#define DMX512_PATTERN_DATA  0x601UL

/* DMX512 11Bit Data Mask */
#define DMX512_PATTERN_DATA_MASK 0x1FEUL

#define DMX512_GET_DATA(x) (( x & DMX512_PATTERN_DATA_MASK) >> 1 )

typedef enum
{
    eDMX512_BREAK_START = 0,
    eDMX512_BREAK,
    eDMX512_START,
    eDMX512_DATA,
    eDMX512_UNDEFINE
} E_DMX512_FRAME_TYPE;

typedef struct
{
    uint8_t     u8TxSlotCounter;
    uint8_t     u8RxSlotCounter;
    uint8_t     u8TxPin;
    uint8_t     u8RxPin;
    volatile    uint8_t *pu8RcvDone;        /* Receive status */
    uint32_t    u32DataLength;              /* In unit of single PDMA transfer width: 16 Bit */
    uint32_t    u32SlotCnt;                 /* Internal control use do not assign value to it */
    volatile    E_DMX512_FRAME_TYPE eState; /* Internal control use do not assign value to it */
} S_PSIO_DMX512_CFG;

typedef enum
{
    eDMX512_READY = 8,
    eDMX512_FILTER_BREAK0,
    eDMX512_FILTER_BREAK1,
    eDMX512_FILTER_BREAK1_START,
    eDMX512_FILTER_DATA,
    eDMX512_FILTER_RECEIVE_DONE,
} E_DMX512_FILTER_STATE;

typedef void (*tPSIO_IRQHandler)(void);

uint32_t PSIO_DMX512_Open(S_PSIO_DMX512_CFG *psConfig);
int PSIO_DMX512_Tx(S_PSIO_DMX512_CFG *psConfig, uint32_t u32OutData, E_DMX512_FRAME_TYPE eFrameType);
//int PSIO_DMX512_Rx(S_PSIO_DMX512_CFG *psConfig, uint16_t *pu16RxData, uint32_t u32Len);
int PSIO_DMX512_getChannelData(S_PSIO_DMX512_CFG *psConfig, uint16_t u16TargetChannel, uint16_t *pu16data);
typedef void (*FN_SET_MODE)(S_PSIO_DMX512_CFG *psConfig, E_DMX512_FRAME_TYPE eFrameType);

#endif  //__DMX512_DRIVER_H__

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
