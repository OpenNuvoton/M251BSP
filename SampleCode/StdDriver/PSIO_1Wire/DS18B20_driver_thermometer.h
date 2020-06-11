/**************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       DS18B20 device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __DS18B20_DRIVER_THERMOMETER_H__
#define __DS18B20_DRIVER_THERMOMETER_H__

//DS18B20
#define ONEWIRE_CONVT               0x44
#define ONEWIRE_RDSCRATCH_PAD       0xBE
#define ONEWIRE_WRSCRATCH_PAD       0x4E
#define ONEWIRE_CPYSCRATCH_PAD      0x48
#define ONEWIRE_RECALL              0xB8
#define ONEWIRE_RDPWR               0xB4
#define ONEWIRE_RDROM               0x33
#define ONEWIRE_MATCH_ROM           0x55
#define ONEWIRE_SKIP_ROM            0xCC
#define ONEWIRE_ALARM_SRCH          0xEC

typedef struct
{
    uint8_t     u8SlotCtrl;
    uint8_t     u8DataPin;
    uint8_t     u8PDMAChannel;
} S_PSIO_DS18B20_CFG;

void PSIO_DS18B20_Open(S_PSIO_DS18B20_CFG *psConfig);
void PSIO_DS18B20_Read_Data(S_PSIO_DS18B20_CFG *psConfig, uint8_t *pu8InData);
void PSIO_DS18B20_Reset(S_PSIO_DS18B20_CFG *psConfig);
void PSIO_DS18B20_Write_Command(S_PSIO_DS18B20_CFG *psConfig, uint8_t u8CMD);

#endif  //__DS18B20_DRIVER_THERMOMETER_H__

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/