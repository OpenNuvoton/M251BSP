/**************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       DS18B20 device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"
#include "DS18B20_driver_thermometer.h"

void PSIO_DS18B20_Write_Command(S_PSIO_DS18B20_CFG *psConfig, uint8_t u8CMD)
{
    const S_PSIO_CP_CONFIG sDataConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_LOW,       PSIO_OUT_BUFFER,    PSIO_OUT_HIGH,      PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};  
 
    /* Set slot count */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, 2);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT1, 12);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT2, 1);
      
    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DataPin, &sDataConfig);
      
    /* Enable repeat slot0 ~ slot2 7 times */
    PSIO_SET_SCCTL(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, PSIO_SLOT2, 7, PSIO_REPEAT_DISABLE);

    /* Set out data width as 8 bit */
    PSIO_SET_WIDTH(PSIO, psConfig->u8DataPin, 0, 8);

    /* Set out data depth as 1 */
    PSIO_SET_OUTPUT_DEPTH(PSIO, psConfig->u8DataPin, PSIO_DEPTH1);

    /* Set output data to buffer */ 
    PSIO_SET_OUTPUT_DATA(PSIO, psConfig->u8DataPin, u8CMD);
    
    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8SlotCtrl);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8SlotCtrl));  
    
    /* Disable repeat slot0 ~ slot2 */
    PSIO_SET_SCCTL(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT_DISABLE, PSIO_SLOT_DISABLE, 0, PSIO_REPEAT_DISABLE);    
}


void PSIO_DS18B20_Reset(S_PSIO_DS18B20_CFG *psConfig)
{
    const S_PSIO_CP_CONFIG sDataConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_LOW,       PSIO_OUT_HIGH,      PSIO_OUT_HIGH,      PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};  
                     
    /* Set slot count */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, 15);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT1, 15);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT2, 15);

    /* Repeat slot0 8 times */
    PSIO_SET_SCCTL(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, PSIO_SLOT0, 8, PSIO_REPEAT_DISABLE);

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DataPin, &sDataConfig);
      
    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8SlotCtrl);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8SlotCtrl));

    /* Disable repeat slot0 setting */
    PSIO_SET_SCCTL(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, PSIO_SLOT0, 0, PSIO_REPEAT_DISABLE);
}


void PSIO_DS18B20_Read_Data(S_PSIO_DS18B20_CFG *psConfig, uint8_t *pu8InData)
{
    uint8_t u8Cnt = 0;
    const S_PSIO_CP_CONFIG sDataConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_LOW,       PSIO_IN_BUFFER,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};                        
   
    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DataPin, &sDataConfig); 
      
    /* Set slot0 tick count as 1, slot0 tick count as 6, and disable slot2 */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, 1);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT1, 6);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT2, 0);

    /* Set in data width as 1 bit */
    PSIO_SET_WIDTH(PSIO, psConfig->u8DataPin, 1, 0);

    /* Set in data depth as 1 */
    PSIO_SET_INPUT_DEPTH(PSIO, psConfig->u8DataPin, PSIO_DEPTH1);

    for (u8Cnt = 0; u8Cnt < 8 * 2 ; u8Cnt++)
    {
        /* Wait for slot controller is not busy */
        while (PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8SlotCtrl));

        /* Trigger slot controller */
        PSIO_START_SC(PSIO, psConfig->u8SlotCtrl);

        /* Wait for data buffer is full */
        while (!PSIO_GET_TRANSFER_STATUS(PSIO, PSIO_TRANSTS_INFULL0_Msk));

        /* Read data from device */
        pu8InData[u8Cnt >> 3] |= (PSIO_GET_INPUT_DATA(PSIO, psConfig->u8DataPin) << (u8Cnt % 8));

    }
}


void PSIO_DS18B20_Open(S_PSIO_DS18B20_CFG *psConfig)
{
    /* PSIO pin general setting */
    PSIO_SET_GENCTL(PSIO, psConfig->u8DataPin, PSIO_PIN_ENABLE, psConfig->u8SlotCtrl
                    , PSIO_OPENDRAIN_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);

    /* Set slot controller trigger source as software trigger */
    PSIO_SET_TRIGSRC(PSIO, psConfig->u8SlotCtrl, PSIO_SW_TRIGGER);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
