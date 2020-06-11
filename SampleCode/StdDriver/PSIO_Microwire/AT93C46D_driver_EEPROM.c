/**************************************************************************//**
 * @file        AT93C46D_driver_EEPROM.c
 * @version     V3.00
 * @brief       AT93C46D device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"
#include "AT93C46D_driver_EEPROM.h"

void PSIO_AT93C46D_CheckStatus(S_PSIO_AT93C46D *psConfig)
{
    /* Set chip select pin to GPIO function */
    SetCSPinToGPIO();    
    
    /* Check device status is READY */
    outpw(pu32ChipSelectPin, 1);
    while(inpw(pu32InputPin) == 1);
    while(inpw(pu32InputPin) == 0);
    outpw(pu32ChipSelectPin, 0);
    
    /* Set chip select pin to PSIO function */
    SetCSPinToPSIO();    
}

void PSIO_AT93C46D_Read(S_PSIO_AT93C46D *psConfig, uint8_t u8Address, uint8_t *pu8Data)
{
    uint32_t u32Data;

    /* Enable DO pin */
    PSIO_ENABLE_PIN(PSIO, psConfig->u8DO);

    /* Loop slot1~slot2 17 times */
    PSIO_SET_SCCTL(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT1, PSIO_SLOT2, 17, PSIO_REPEAT_DISABLE);

    /* Set output/input data width as 18 */
    PSIO_SET_WIDTH(PSIO, psConfig->u8DI, 0, 18);
    PSIO_SET_WIDTH(PSIO, psConfig->u8DO, 18, 0);

    /* Set output data */
    PSIO_SET_OUTPUT_DATA(PSIO, psConfig->u8DI, CMD_READ(u8Address));

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8SlotCtrl);

    /* Wait for data buffer is full */
    while (!PSIO_GET_TRANSFER_STATUS(PSIO, PSIO_TRANSTS_INFULL0_Msk << (psConfig->u8DO * 4)));

    /* Read data from device */
    u32Data = PSIO_GET_INPUT_DATA(PSIO, psConfig->u8DO);
    *pu8Data = u32Data & 0xFF;
    
    /* Disable DO pin */
    PSIO_DISABLE_PIN(PSIO, psConfig->u8DO);   
}

void PSIO_AT93C46D_Write(S_PSIO_AT93C46D *psConfig, uint8_t u8Address, uint8_t *pu8Data)
{
    /* Loop slot1~slot2 17 times */
    PSIO_SET_SCCTL(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT1, PSIO_SLOT2, 17, PSIO_REPEAT_DISABLE);

    /* Set output data width as 18 */
    PSIO_SET_WIDTH(PSIO, psConfig->u8DI, 0, 18);

    /* Set output data */
    PSIO_SET_OUTPUT_DATA(PSIO, psConfig->u8DI, CMD_WRITE(u8Address) | (*pu8Data & 0xFF));

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8SlotCtrl);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8SlotCtrl));

    /* Check device status is READY */
    PSIO_AT93C46D_CheckStatus(psConfig);
}


void PSIO_AT93C46D_Erase(S_PSIO_AT93C46D *psConfig, uint8_t u8Address)
{  
    /* Loop slot1~slot2 9 times */
    PSIO_SET_SCCTL(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT1, PSIO_SLOT2, 9, PSIO_REPEAT_DISABLE);

    /* Set output data width as 10 */
    PSIO_SET_WIDTH(PSIO, psConfig->u8DI, 0, 10);   

    /* Set output data */
    PSIO_SET_OUTPUT_DATA(PSIO, psConfig->u8DI, CMD_ERASE | (u8Address & 0x7F));

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8SlotCtrl);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8SlotCtrl));
    
    /* Check device status is READY */
    PSIO_AT93C46D_CheckStatus(psConfig);
}


void PSIO_AT93C46D_EraseWrite_Enable(S_PSIO_AT93C46D *psConfig)
{   
    /* Loop slot1~slot2 11 times */
    PSIO_SET_SCCTL(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT1, PSIO_SLOT2, 11, PSIO_REPEAT_DISABLE);

    /* Set output data width as 12 */
    PSIO_SET_WIDTH(PSIO, psConfig->u8DI, 0, 12);

    /* Set output data */
    PSIO_SET_OUTPUT_DATA(PSIO, psConfig->u8DI, CMD_EWEN);

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8SlotCtrl);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8SlotCtrl));
}


void PSIO_AT93C46D_Init(S_PSIO_AT93C46D *psConfig)
{    
    const S_PSIO_CP_CONFIG sCSConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_HIGH,      PSIO_OUT_HIGH,      PSIO_OUT_HIGH,      PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};   
    const S_PSIO_CP_CONFIG sSKConfig 
                     = {/* Check Point0     Check Point1        Check Point2    Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,     PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_HIGH,  PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};
    const S_PSIO_CP_CONFIG sDIConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT1,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_BUFFER,    PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};
    const S_PSIO_CP_CONFIG sDOConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT2,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_IN_BUFFER,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};   
                     
    /* Enable CS, SK, DI, DO pin and setting general configuration */
    PSIO_SET_GENCTL(PSIO, psConfig->u8ChipSelectPin, PSIO_PIN_ENABLE, psConfig->u8SlotCtrl
                    , PSIO_OUTPUT_MODE, PSIO_LOW_LEVEL, PSIO_LOW_LEVEL);
    PSIO_SET_GENCTL(PSIO, psConfig->u8ClockPin, PSIO_PIN_ENABLE, psConfig->u8SlotCtrl
                    , PSIO_OUTPUT_MODE, PSIO_LOW_LEVEL, PSIO_LOW_LEVEL);
    PSIO_SET_GENCTL(PSIO, psConfig->u8DI, PSIO_PIN_ENABLE, psConfig->u8SlotCtrl
                    , PSIO_OUTPUT_MODE, PSIO_LOW_LEVEL, PSIO_LOW_LEVEL);
    PSIO_SET_GENCTL(PSIO, psConfig->u8DO, PSIO_PIN_DISABLE, psConfig->u8SlotCtrl
                    , PSIO_INPUT_MODE, PSIO_LOW_LEVEL, PSIO_LOW_LEVEL);
      
    /* Set data order and MSB */
    PSIO_SET_ORDER(PSIO, psConfig->u8ChipSelectPin, PSIO_MSB);
    PSIO_SET_ORDER(PSIO, psConfig->u8ClockPin, PSIO_MSB);
    PSIO_SET_ORDER(PSIO, psConfig->u8DO, PSIO_MSB);
    PSIO_SET_ORDER(PSIO, psConfig->u8DI, PSIO_MSB);

    /* Set slot counter number */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, 1);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT1, 8);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT2, 8); 
     
    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8ChipSelectPin, &sCSConfig); 
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8ClockPin, &sSKConfig); 
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DI, &sDIConfig); 
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DO, &sDOConfig);                                                 
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
