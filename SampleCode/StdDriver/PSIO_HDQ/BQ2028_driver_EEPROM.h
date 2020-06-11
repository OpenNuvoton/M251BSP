/**************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       BQ2028 device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __BQ2028_DRIVER_EEPROM_H__
#define __BQ2028_DRIVER_EEPROM_H__
#include <stdbool.h>

//TI BQ2028
#define HDQ_R               0x00
#define HDQ_W               0x80
#define HDQ_DAT_WIDTH       0x8
#define HDQ_UNMAP           0x0
#define HDQ_MAP             0x40

#define HDQ_CMD_READID      0x0F
#define HDQ_CMD_READREV     0x0E
#define HDQ_CMD_PAGE        0x07
#define HDQ_CMD_ROW         0x0D
#define HDQ_CMD_STATUS      0x04
#define HDQ_CMD_BUF0        0x00
#define HDQ_CMD_BUF1        0x01
#define HDQ_CMD_BUF2        0x02
#define HDQ_CMD_BUF3        0x03
#define HDQ_CMD_PAGE_EN     0x31
#define HDQ_CMD_CTL2        0x25
#define HDQ_CMD_CTL0        0x05
#define HDQ_CMD_CTL1        0x08
#define HDQ_CMD_CRCT        0x21
#define HDQ_CMD_CRCR        0x20

#define STATUS_BUSY_MSK     0x80
#define STATUS_DRDY_MSK     0x40
#define STATUS_PAGEER_MSK   0x20
#define STATUS_MEMWR_MSK    0x10
#define STATUS_RST_MSK      0x04
#define STATUS_MEMER_MSK    0x02
#define STATUS_CRCER_MSK    0x01


typedef struct
{
    uint8_t     u8SlotCtrl;
    uint8_t     u8Data0Pin;
} PSIO_BQ2028_CFG_T;

void PSIO_BQ2028_Open(PSIO_BQ2028_CFG_T *pConfig);
void PSIO_BQ2028_Break(PSIO_BQ2028_CFG_T *pConfig);
void PSIO_BQ2028_Write(PSIO_BQ2028_CFG_T *pConfig, uint8_t u8CMD);
void PSIO_BQ2028_Read(PSIO_BQ2028_CFG_T *pConfig);
void PSIO_BQ2028_Write_OneByte(PSIO_BQ2028_CFG_T *pConfig, uint8_t u8CMD, uint8_t u8Data);
void PSIO_BQ2028_Read_OneByte(PSIO_BQ2028_CFG_T *pConfig, uint8_t u8CMD, uint8_t *pu8Data);
uint8_t PSIO_BQ2028_CRC8(uint8_t u8Data);
bool PSIO_BQ2028_BUSY(void);

#endif  //__BQ2028_DRIVER_EEPROM_H__

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
