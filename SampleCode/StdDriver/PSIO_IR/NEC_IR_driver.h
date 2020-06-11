/**************************************************************************//**
 * @file        NEC_IR_driver.h
 * @version     V3.00
 * @brief       NEC IR device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __NEC_IR_DRIVER_H__
#define __NEC_IR_DRIVER_H__

enum
{
    eERROR_SEND_MODE = -1,
    eERROR_MEMORY_ADDR = -2,
};

typedef struct S_PSIO_NEC_CFG
{
    uint8_t             u8SlotCtrl;
    uint8_t             u8TxPin;
} S_PSIO_NEC_CFG;


__STATIC_INLINE int PSIO_NEC_TransferDone(S_PSIO_NEC_CFG *pConfig)
{
    return PSIO_GET_BUSY_FLAG(PSIO, pConfig->u8SlotCtrl);
}

int PSIO_NEC_Send(S_PSIO_NEC_CFG *pConfig, uint8_t u8Address0, uint8_t u8Address1, uint8_t u8Command0, uint8_t u8Command1);
int PSIO_NEC_Repeat(S_PSIO_NEC_CFG *pConfig);
void PSIO_NEC_Open(S_PSIO_NEC_CFG *pConfig);
void PSIO_NEC_Close(S_PSIO_NEC_CFG *pConfig);

#endif  //__NEC_IR_DRIVER_H__

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
