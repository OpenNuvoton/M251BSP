/**************************************************************************//**
 * @file        AT93C46D_driver_EEPROM.h
 * @version     V3.00
 * @brief       AT93C46D device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __AT93C46D_DRIVER_EEPROM_H__
#define __AT93C46D_DRIVER_EEPROM_H__

#define CMD_ERASE       0x7<<7
#define CMD_EWEN        0x13<<7
#define CMD_READ(addr)  (0x6<<15|(addr&0x7F)<<8)
#define CMD_WRITE(addr) (0x5<<15|(addr&0x7F)<<8)

#define EEPROM_SIZE 1024
#define DATA_WIDTH  8

extern volatile uint32_t *pu32ChipSelectPin, *pu32InputPin;

typedef struct
{
    uint8_t     u8SlotCtrl;
    uint8_t     u8ChipSelectPin;
    uint8_t     u8ClockPin;
    uint8_t     u8DO;
    uint8_t     u8DI;
} S_PSIO_AT93C46D;

extern void SetCSPinToPSIO(void);
extern void SetCSPinToGPIO(void);

void PSIO_AT93C46D_Read(S_PSIO_AT93C46D *psConfig, uint8_t u8Address, uint8_t *pu8Data);
void PSIO_AT93C46D_Write(S_PSIO_AT93C46D *psConfig, uint8_t u8Address, uint8_t *pu8Data);
void PSIO_AT93C46D_Erase(S_PSIO_AT93C46D *psConfig, uint8_t u8Address);
void PSIO_AT93C46D_EraseWrite_Enable(S_PSIO_AT93C46D *psConfig);
void PSIO_AT93C46D_Init(S_PSIO_AT93C46D *psConfig);

#endif  //__AT93C46D_DRIVER_EEPROM_H__

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
