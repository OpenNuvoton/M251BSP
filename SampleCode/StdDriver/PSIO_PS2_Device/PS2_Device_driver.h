/***************************************************************************//**
 * @file    PS2_Device_driver.h
 * @version V3.00
 * @brief   PS/2 Slave device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __PS2_SLAVE_DRIVER_H__
#define __PS2_SLAVE_DRIVER_H__

typedef volatile uint32_t PS2_DEVICE_STATUS;

#define PSIO_PS2_DEVICE_TRANSFER_TIMEOUT    (SystemCoreClock)
#define PSIO_PS2_DEVICE_READ_IDLE           (0UL)
#define PSIO_PS2_DEVICE_READ_TRIGGERED      (1UL)
#define PSIO_PS2_DEVICE_READ_TIMEOUT        (2UL)

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
    eDEVICE_IDLE,
    eDEVICE_READ,
    eDEVICE_WRITE,
};

extern PS2_DEVICE_STATUS g_eStatus;
extern uint8_t *g_pu8RxData, *g_pu8Parity;


#define PSIO_PS2_GET_STATUS() (g_eStatus)
#define PSIO_PS2_SET_STATUS(status) ((PS2_DEVICE_STATUS)(g_eStatus = (status)))

void PSIO_PS2_DeviceSend(S_PSIO_PS2 *pConfig, uint8_t *pu8TxData);
uint32_t PSIO_PS2_DeviceRead(S_PSIO_PS2 *pConfig, uint8_t *pu8RxData, uint8_t *pu8Parity);
void PSIO_PS2_Open(S_PSIO_PS2 *pConfig);
void PSIO_PS2_Close(S_PSIO_PS2 *pConfig);

#endif  //__PS2_SLAVE_DRIVER_H__

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
