/**************************************************************************//**
 * @file        PS2_Host_driver.c
 * @version     V3.00
 * @brief       PS/2 Host device driver
 *
 * @note
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "M251.h"
#include "PS2_Host_driver.h"

PS2_HOST_STATUS g_Status = eHOST_IDLE;

static void PSIO_PS2_H2D_ReadConfig(S_PSIO_PS2 *psConfig)
{
    const S_PSIO_CP_CONFIG sClockConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};   
    const S_PSIO_CP_CONFIG sDataConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_IN_BUFFER,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};    
    
    /* Update status */
    PSIO_PS2_SET_STATUS(eHOST_READY_TO_READ);

    /* Set clock slot controller slot0 tick count as 1 */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8ClockSC, PSIO_SLOT0, 1);

    /* Set data slot controller slot0 tick count as 6 */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8DataSC, PSIO_SLOT0, 6);

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8ClockPin, &sClockConfig);
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DataPin, &sDataConfig);
    
    /* Set data pin input data depth */
    PSIO_SET_INPUT_DEPTH(PSIO, psConfig->u8DataPin, PSIO_DEPTH1);

    /* Set interrupt control */
    PSIO_SET_INTCTL(PSIO, psConfig->u8ClockSC, PSIO_INT0, PSIO_SLOT0);

    /* Set clock slot controller as falling trigger */
    PSIO_SET_TRIGSRC(PSIO, psConfig->u8ClockSC, PSIO_FALLING_TRIGGER);
}


static void PSIO_PS2_H2D_Send_Header(S_PSIO_PS2 *psConfig)
{
    const S_PSIO_CP_CONFIG sClockConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT3,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};   
    const S_PSIO_CP_CONFIG sDataConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT3,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_HIGH,      PSIO_OUT_HIGH,      PSIO_OUT_HIGH,      PSIO_OUT_LOW,       PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};
                     
    /* Update status */
    PSIO_PS2_SET_STATUS(eHOST_WRITE);

    /* Set PSIO clock divider as 29 */
    CLK->CLKDIV1 = (CLK->CLKDIV1 & ~CLK_CLKDIV1_PSIODIV_Msk) | (29 << CLK_CLKDIV1_PSIODIV_Pos);

    /* Set clock/data slot controller slot0~3 tick count as 10 */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8ClockSC, PSIO_SLOT0, 10);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8ClockSC, PSIO_SLOT1, 10);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8ClockSC, PSIO_SLOT2, 10);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8ClockSC, PSIO_SLOT3, 10);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8DataSC, PSIO_SLOT0, 10);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8DataSC, PSIO_SLOT1, 10);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8DataSC, PSIO_SLOT2, 10);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8DataSC, PSIO_SLOT3, 10);

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8ClockPin, &sClockConfig);
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DataPin, &sDataConfig);
    
    /* Set clock slot controller as software trigger */
    PSIO_SET_TRIGSRC(PSIO, psConfig->u8ClockSC, PSIO_SW_TRIGGER);

    /* Clear INT0 interrupt setting */
    PSIO_CLEAR_INTCTL(PSIO, PSIO_INT0);

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8ClockSC);
    PSIO_START_SC(PSIO, psConfig->u8DataSC);

    /* Wait for slot controller is busy */
    while (!PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8ClockSC));

    /* Waiting for slave signal, set the data pin interval keep last output */
    PSIO->GNCT[psConfig->u8DataPin].GENCTL = (PSIO->GNCT[psConfig->u8DataPin].GENCTL & ~PSIO_GNCT_GENCTL_INTERVAL_Msk)
                                            | (PSIO_LAST_OUTPUT << PSIO_GNCT_GENCTL_INTERVAL_Pos);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8ClockSC));
}


uint32_t PSIO_Encode_TxData(uint32_t *u32TxData)
{
    uint32_t u32Data = 0;
    uint8_t u8Parity = 1;
    uint8_t i   = 0;

    /* Odd parity */
    for (i = 0; i < 8; i++)
    {
        u8Parity ^= ((*u32TxData) >> i) & 0x1;
    }

    /* Stop(1 bit) + Parity(1 bit)+ Data(8 bit) */
    u32Data = (0x1 << 9) | (u8Parity << 8) | (*u32TxData & 0xff);

    return u32Data;
}


void PSIO_PS2_H2D_Send_Data(S_PSIO_PS2 *psConfig)
{
    const S_PSIO_CP_CONFIG sClockConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};   
    const S_PSIO_CP_CONFIG sDataConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_BUFFER,    PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};
        
    /* Set clock/data slot controller slot0 tick count as 1 */
    /* For more efficient, accessing register directly */
    PSIO->SCCT[psConfig->u8ClockSC].SCSLOT   = 0x1;
    PSIO->SCCT[psConfig->u8DataSC].SCSLOT    = 0x1;

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8ClockPin, &sClockConfig);
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8DataPin, &sDataConfig);
      
    /* Set clock slot controller as falling trigger */
    PSIO_SET_TRIGSRC(PSIO, psConfig->u8ClockSC, PSIO_FALLING_TRIGGER);

    /* Set interrupt control */
    PSIO_SET_INTCTL(PSIO, psConfig->u8ClockSC, PSIO_INT0, PSIO_SLOT0);

    /* Clear PSIO divider setting */
    CLK->CLKDIV1 = (CLK->CLKDIV1 & ~CLK_CLKDIV1_PSIODIV_Msk);
}


void  PSIO_PS2_HostSend(S_PSIO_PS2 *psConfig)
{
    PSIO_PS2_H2D_Send_Header(psConfig);

    PSIO_PS2_H2D_Send_Data(psConfig);
}


void PSIO_PS2_HostRead(S_PSIO_PS2 *psConfig)
{
    /* Setting PSIO to read data */
    PSIO_PS2_H2D_ReadConfig(psConfig);
}


void PSIO_PS2_Open(S_PSIO_PS2 *psConfig)
{
    /* PSIO pin general setting */
    PSIO_SET_GENCTL(PSIO, psConfig->u8ClockPin, PSIO_PIN_ENABLE, psConfig->u8ClockSC
                    , PSIO_OPENDRAIN_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);
    PSIO_SET_GENCTL(PSIO, psConfig->u8DataPin, PSIO_PIN_ENABLE, psConfig->u8DataSC
                    , PSIO_OPENDRAIN_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);

    /* Enable interrupt 0 */
    PSIO_ENABLE_INT(PSIO, PSIO_INTEN_CON0IE_Msk);

    /* Set PSIO corresponding NVIC bit */
    NVIC_EnableIRQ(PSIO_IRQn);

    /* Setting PSIO to read data */
    PSIO_PS2_H2D_ReadConfig(psConfig);
}


void PSIO_PS2_Close(S_PSIO_PS2 *psConfig)
{
    /* Disable interrupt 0 */
    PSIO_DISABLE_INT(PSIO, PSIO_INTEN_CON0IE_Msk);

    /* Disable PSIO corresponding NVIC bit */
    NVIC_DisableIRQ(PSIO_IRQn);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
