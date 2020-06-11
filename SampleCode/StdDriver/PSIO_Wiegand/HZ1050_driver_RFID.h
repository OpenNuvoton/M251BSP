/**************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       HZ1050 device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __HZ1050_DRIVER_RFID_H__
#define __HZ1050_DRIVER_RFID_H__

#define WEIGAND_LENGTH      26
#define EVEN_PARITY_POS     25
#define EVEN_PARITY_MSK     (0x1<<EVEN_PARITY_POS)
#define ODD_PARITY_POS      0
#define ODD_PARITY_MSK      (0x1<<ODD_PARITY_POS)
#define FACILITY_CODE_POS   17
#define FACILITY_CODE_MSK   (0xFF<<FACILITY_CODE_POS)
#define CARD_CODE_POS       1
#define CARD_CODE_MSK       (0xFFFF<<CARD_CODE_POS)

typedef struct
{
    uint8_t     u8SlotCtrl;
    uint8_t     u8Data0Pin;
    uint8_t     u8Data1Pin;
} S_PSIO_HZ1050;


void PSIO_HZ1050_Init(S_PSIO_HZ1050 *psConfig);
void PSIO_HZ1050_Read(S_PSIO_HZ1050 *psConfig, uint32_t *pu32InData);

#endif  //__HZ1050_DRIVER_RFID_H__
