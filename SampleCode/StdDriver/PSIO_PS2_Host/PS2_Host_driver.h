/**************************************************************************//**
 * @file        PS2_Host_driver.h
 * @version     V3.00
 * @brief       PS2 host device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __PS2_HOST_DRIVER_H__
#define __PS2_HOST_DRIVER_H__

typedef volatile uint32_t PS2_HOST_STATUS;

typedef struct
{
    uint8_t     u8ClockSC;
    uint8_t     u8DataSC;
    uint8_t     u8ClockPin;
    uint8_t     u8DataPin;
    volatile uint32_t    *p32ClockMFP;
    volatile uint32_t    *p32DataMFP;
} S_PSIO_PS2;

enum
{
    eHOST_IDLE,
    eHOST_READY_TO_READ,
    eHOST_READ,
    eHOST_WRITE,
};

extern PS2_HOST_STATUS g_Status;


#define PSIO_PS2_GET_STATUS() (g_Status)
#define PSIO_PS2_SET_STATUS(status) ((PS2_HOST_STATUS)(g_Status=(status)))

uint32_t PSIO_Encode_TxData(uint32_t *u32TxData);
void PSIO_PS2_HostSend(S_PSIO_PS2 *psConfig);
void PSIO_PS2_HostRead(S_PSIO_PS2 *psConfig);
void PSIO_PS2_Open(S_PSIO_PS2 *psConfig);
void PSIO_PS2_Close(S_PSIO_PS2 *psConfig);

#endif  //__PS2_HOST_DRIVER_H__

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
