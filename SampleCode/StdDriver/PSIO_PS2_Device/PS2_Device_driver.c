/**************************************************************************//**
 * @file        PS2_Slave_driver.c
 * @version     V3.00
 * @brief       PS/2 Slave device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"
#include "PS2_Device_driver.h"

PS2_DEVICE_STATUS g_eStatus = eDEVICE_IDLE;
uint8_t *g_pu8RxData, *g_pu8Parity;


static void PSIO_PS2_D2H_ReadData_Config(S_PSIO_PS2 *psConfig)
{
    const S_PSIO_CP_CONFIG sClockConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT3,         PSIO_SLOT4,         PSIO_SLOT5,         PSIO_SLOT6,         PSIO_SLOT7,
      /* Action */      PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_HIGH,      PSIO_OUT_HIGH,      PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_HIGH,      PSIO_OUT_HIGH};   
    const S_PSIO_CP_CONFIG sDataConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT2,         PSIO_SLOT5,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_IN_BUFFER,     PSIO_OUT_LOW,       PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};
        
    /* Set clock slot controller slot0~7 tick count as 8 */
    /* For more efficient, accessing register directly */
    PSIO->SCCT[psConfig->u8ClockSC].SCSLOT   = (0x8 << 28) | (0x8 << 24) | (0x8 << 20) | (0x8 << 16) | (0x8 << 12) | (0x8 << 8) | (0x8 << 4) | (0x8);
      
    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8ClockPin, &sClockConfig);
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DataPin, &sDataConfig);
      
    /* Loop slot0~slot3 9 times */
    PSIO_SET_SCCTL(PSIO, psConfig->u8ClockSC, PSIO_SLOT0, PSIO_SLOT3, 9, PSIO_REPEAT_DISABLE);

    /* Set clock slot controller as software trigger */
    PSIO_SET_TRIGSRC(PSIO, psConfig->u8ClockSC, PSIO_SW_TRIGGER);

    /* Set output data width as 0, input data width as 10 */
    PSIO_SET_WIDTH(PSIO, psConfig->u8DataPin, 10, 0);

    /* Set input data as 1 */
    PSIO_SET_INPUT_DEPTH(PSIO, psConfig->u8DataPin, PSIO_DEPTH1);

    /* Set interrupt control */
    PSIO_SET_INTCTL(PSIO, psConfig->u8ClockSC, PSIO_INT0, PSIO_SLOT7);
}


static uint32_t Encode_TxData(uint8_t *pu8TxData)
{
    uint8_t u8Parity = 1;
    uint8_t i   = 0;

    for (i = 0; i < 8; i++)
    {
        u8Parity ^= ((*pu8TxData) >> i) & 0x1;
    }

    /* START + DATA(8bit) + PARITY + STOP   (LSB) */
    return (0x1 << 10 | ((u8Parity & 0x1) << 9)) | ((*pu8TxData) << 1) | (0x0 << 10);
}


static void PSIO_PS2_D2H_SendData_Config(S_PSIO_PS2 *psConfig)
{
    const S_PSIO_CP_CONFIG sClockConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT3,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_HIGH,      PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_HIGH,      PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};   
    const S_PSIO_CP_CONFIG sDataConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_BUFFER,    PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};
                     
    /* Set clock slot controller slot0~7 tick count as 8 */
    /* For more efficient, accessing register directly */
    PSIO->SCCT[psConfig->u8ClockSC].SCSLOT   = (0x8 << 12) | (0x8 << 8) | (0x8 << 4) | (0x8);

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8ClockPin, &sClockConfig);
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DataPin, &sDataConfig);
      
    /* Loop slot0~slot3 10 times */
    PSIO_SET_SCCTL(PSIO, psConfig->u8ClockSC, PSIO_SLOT0, PSIO_SLOT3, 10, PSIO_REPEAT_DISABLE);

    /* Set output data width as 11, input data width as 0 */
    PSIO_SET_WIDTH(PSIO, psConfig->u8DataPin, 0, 11);

    /* Set output data as 1 */
    PSIO_SET_OUTPUT_DEPTH(PSIO, psConfig->u8DataPin, PSIO_DEPTH1);

    /* Set interrupt control */
    PSIO_SET_INTCTL(PSIO, psConfig->u8ClockSC, PSIO_INT0, PSIO_SLOT3);
}


void PSIO_PS2_DeviceRead(S_PSIO_PS2 *psConfig, uint8_t *pu8RxData, uint8_t *pu8Parity)
{
    if (*(psConfig->p32ClockMFP))
    {
        return;
    }

    while (*(psConfig->p32DataMFP));

    while (!(*(psConfig->p32ClockMFP)));

    /* Update status */
    PSIO_PS2_SET_STATUS(eDEVICE_READ);

    PSIO_PS2_D2H_ReadData_Config(psConfig);
    g_pu8RxData = pu8RxData;
    g_pu8Parity = pu8Parity;

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8ClockSC);
}


void PSIO_PS2_DeviceSend(S_PSIO_PS2 *psConfig, uint8_t *pu8TxData)
{
    /* Update status */
    PSIO_PS2_SET_STATUS(eDEVICE_WRITE);

    PSIO_PS2_D2H_SendData_Config(psConfig);

    /* Set output data */
    PSIO_SET_OUTPUT_DATA(PSIO, psConfig->u8DataPin, Encode_TxData(pu8TxData));

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8ClockSC);
}


void PSIO_PS2_Open(S_PSIO_PS2 *psConfig)
{
    /* Set PSIO clock divider as 29 */
    CLK->CLKDIV1 = (CLK->CLKDIV1 & ~CLK_CLKDIV1_PSIODIV_Msk) | (29 << CLK_CLKDIV1_PSIODIV_Pos);

    /* PSIO pin general setting */
    PSIO_SET_GENCTL(PSIO, psConfig->u8ClockPin, PSIO_PIN_ENABLE, psConfig->u8ClockSC
                    , PSIO_OPENDRAIN_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);
    PSIO_SET_GENCTL(PSIO, psConfig->u8DataPin, PSIO_PIN_ENABLE, psConfig->u8DataSC
                    , PSIO_OPENDRAIN_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);

    /* Enable interrupt 0 */
    PSIO_ENABLE_INT(PSIO, PSIO_INTEN_CON0IE_Msk);

    /* Set PSIO corresponding NVIC bit */
    NVIC_EnableIRQ(PSIO_IRQn);
}


void PSIO_PS2_Close(S_PSIO_PS2 *psConfig)
{
    /* Disable interrupt 0 */
    PSIO_DISABLE_INT(PSIO, PSIO_INTEN_CON0IE_Msk);

    /* Disable PSIO corresponding NVIC bit */
    NVIC_DisableIRQ(PSIO_IRQn);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
