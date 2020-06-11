/**************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       HZ1050 device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"
#include "HZ1050_driver_RFID.h"


void PSIO_HZ1050_Init(S_PSIO_HZ1050 *psConfig)
{
    const S_PSIO_CP_CONFIG sData0Config 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_IN_BUFFER,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};   
    const S_PSIO_CP_CONFIG sData1Config 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_IN_BUFFER,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};
          
    /* PSIO pin general setting */
    PSIO_SET_GENCTL(PSIO, psConfig->u8Data0Pin, PSIO_PIN_ENABLE, psConfig->u8SlotCtrl
                    , PSIO_INPUT_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);
    PSIO_SET_GENCTL(PSIO, psConfig->u8Data1Pin, PSIO_PIN_ENABLE, psConfig->u8SlotCtrl
                    , PSIO_INPUT_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);

    /* Set data order ad MSB */
    PSIO_SET_ORDER(PSIO, psConfig->u8Data0Pin, PSIO_MSB);
    PSIO_SET_ORDER(PSIO, psConfig->u8Data1Pin, PSIO_MSB);

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8Data0Pin, &sData0Config); 
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8Data1Pin, &sData1Config); 

    /* Set slot0 tick count as 15 */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, 15);

    /* Set Pin0/Pin1 input data width as 1 bit */
    PSIO_SET_WIDTH(PSIO, psConfig->u8Data0Pin, 1, 0);
    PSIO_SET_WIDTH(PSIO, psConfig->u8Data1Pin, 1, 0);

    /* Set Pin0/Pin1 input data depth as 1 */
    PSIO_SET_INPUT_DEPTH(PSIO, psConfig->u8Data0Pin, PSIO_DEPTH1);
    PSIO_SET_INPUT_DEPTH(PSIO, psConfig->u8Data1Pin, PSIO_DEPTH1);

    /* Set slot controller trigger source as falling trigger */
    PSIO_SET_TRIGSRC(PSIO, psConfig->u8SlotCtrl, PSIO_FALLING_TRIGGER);
}


void PSIO_HZ1050_Read(S_PSIO_HZ1050 *psConfig, uint32_t *pu32InData)
{
    uint8_t u8BitIndex = 0;
    uint32_t u32Data0 = 0, u32Data1 = 0;

    /* Receive 26 bit */
    for (u8BitIndex = 0; u8BitIndex < WEIGAND_LENGTH; u8BitIndex++)
    {
        /* Wait slot controller is not busy */
        while (PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8SlotCtrl));

        /* Wait data buffer is full */
        while (!PSIO_GET_TRANSFER_STATUS(PSIO, PSIO_TRANSTS_INFULL0_Msk));

        /* Read data from device */
        u32Data0 |= (PSIO_GET_INPUT_DATA(PSIO, psConfig->u8Data0Pin) << (WEIGAND_LENGTH - u8BitIndex - 1));
        u32Data1 |= (PSIO_GET_INPUT_DATA(PSIO, psConfig->u8Data1Pin) << (WEIGAND_LENGTH - u8BitIndex - 1));
    }

    /* Clear buffer */
    *pu32InData = 0;

    /* Decode data from device */
    for (u8BitIndex = 0; u8BitIndex < WEIGAND_LENGTH; u8BitIndex++)
    {
        if ((((u32Data0 >> u8BitIndex) & 0x1) == 1) && (((u32Data1 >> u8BitIndex) & 0x1) == 0))
        {
            *pu32InData |= 0x1 << u8BitIndex;
        }
    }
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
