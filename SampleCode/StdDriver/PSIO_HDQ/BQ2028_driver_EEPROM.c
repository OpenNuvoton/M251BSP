/****************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       TI HDQ BQ2028
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "BQ2028_driver_EEPROM.h"

typedef void (*PSIO_FUNC)(PSIO_BQ2028_CFG_T *pConfig);
extern PSIO_FUNC s_pfnPSIOHandler;


/*---------------------------------------------------------------------------------------------------------*/
/* PSIO HDQ event definitions                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    PSIO_HDQ_IDLE           = 0,     /* The HDQ bus is in idle state */
    PSIO_HDQ_BREAK,                  /* The HDQ bus is in break state */
    PSIO_HDQ_WRITE_CMD,              /* The HDQ bus is in write state */
    PSIO_HDQ_WRITE_DATA,             /* The HDQ bus is in write data state */
    PSIO_HDQ_READ_DATA,              /* The HDQ bus is in read state */
    PSIO_HDQ_READ_FINISH,            /* The HDQ bus is in read finish state */
} PSIO_HDQ_EVENT;


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile PSIO_HDQ_EVENT g_eEvent = PSIO_HDQ_IDLE;
static volatile uint8_t g_u8OutData;
static volatile uint8_t *g_pu8InData;
static volatile uint8_t g_u8CMD;


/*---------------------------------------------------------------------------------------------------------*/
/* PSIO interrupt callback function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void PSIO_BQ2028_Write_Handler(PSIO_BQ2028_CFG_T *pConfig)
{
    switch (g_eEvent)
    {
        case PSIO_HDQ_BREAK:
            /* Change state to HDQ write */
            g_eEvent = PSIO_HDQ_WRITE_CMD;

            /* Write command + write bit */
            PSIO_BQ2028_Write(pConfig, g_u8CMD | HDQ_W);
            break;

        case PSIO_HDQ_WRITE_CMD:
            /* Change state to HDQ write */
            g_eEvent = PSIO_HDQ_WRITE_DATA;

            /* Write 1 byte data  */
            PSIO_BQ2028_Write(pConfig, g_u8OutData);
            break;

        case PSIO_HDQ_WRITE_DATA:
            /* Change state to HDQ idle */
            g_eEvent = PSIO_HDQ_IDLE;
            break;

        default:
            break;
    }
}


void PSIO_BQ2028_Read_Handler(PSIO_BQ2028_CFG_T *pConfig)
{
    static uint8_t u8BitIndex = 0;
    static uint8_t u8Data = 0;

    switch (g_eEvent)
    {
        case PSIO_HDQ_READ_DATA:

            /* Wait input data ready */
            while (!PSIO_GET_TRANSFER_STATUS(PSIO, PSIO_TRANSTS_INFULL0_Msk));

            /* Merge 8 bit data */
            u8Data |= (PSIO_GET_INPUT_DATA(PSIO, pConfig->u8Data0Pin) << u8BitIndex);

            if (u8BitIndex == 7)
            {
                *g_pu8InData = u8Data;
                u8Data = 0;
                u8BitIndex = 0;
            }
            else
            {
                u8BitIndex++;
                return;
            }

            /* Change state to HDQ idle */
            g_eEvent = PSIO_HDQ_IDLE;
            break;

        case PSIO_HDQ_WRITE_CMD:
            /* Change state to HDQ read */
            g_eEvent = PSIO_HDQ_READ_DATA;

            /* Read 1 byte data */
            PSIO_BQ2028_Read(pConfig);
            break;

        case PSIO_HDQ_BREAK:
            /* Change state to HDQ write */
            g_eEvent = PSIO_HDQ_WRITE_CMD;

            /* Write command + read bit */
            PSIO_BQ2028_Write(pConfig, g_u8CMD | HDQ_R);
            break;

        default:
            break;
    }
}


void PSIO_BQ2028_Write_OneByte(PSIO_BQ2028_CFG_T *pConfig, uint8_t u8CMD, uint8_t u8Data)
{
    g_u8CMD = u8CMD;
    g_u8OutData = u8Data;

    /* Change state to break */
    g_eEvent = PSIO_HDQ_BREAK;

    /* Setting write handler */
    s_pfnPSIOHandler = PSIO_BQ2028_Write_Handler;

    /* Send Break signal */
    PSIO_BQ2028_Break(pConfig);
}


void PSIO_BQ2028_Read_OneByte(PSIO_BQ2028_CFG_T *pConfig, uint8_t u8CMD, uint8_t *pu8Data)
{
    g_u8CMD = u8CMD;
    *pu8Data = 0;
    g_pu8InData = pu8Data;

    /* Change state to break */
    g_eEvent = PSIO_HDQ_BREAK;

    /* Setting read handler */
    s_pfnPSIOHandler = PSIO_BQ2028_Read_Handler;

    /* Send Break signal */
    PSIO_BQ2028_Break(pConfig);
}


void PSIO_BQ2028_Break(PSIO_BQ2028_CFG_T *pConfig)
{
    const S_PSIO_CP_CONFIG sData0Config
    =  /* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
    {
        /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
        /* Action */      PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_HIGH,      PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION
    };

    /* Disable loop function */
    PSIO_SET_SCCTL(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT0, PSIO_SLOT0, 0, PSIO_REPEAT_DISABLE);

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, pConfig->u8Data0Pin, &sData0Config);

    /* Set slot 0 tick count as 15, slot 1 tick count as 4, slot 2 tick count as 4 */
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT0, 15);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT1, 4);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT2, 4);

    /* Set slot controller trigger source as software trigger */
    PSIO_SET_TRIGSRC(PSIO, pConfig->u8SlotCtrl, PSIO_SW_TRIGGER);

    /* HDQ bus in break state */
    g_eEvent = PSIO_HDQ_BREAK;

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, pConfig->u8SlotCtrl);
}


void PSIO_BQ2028_Write(PSIO_BQ2028_CFG_T *pConfig, uint8_t u8CMD)
{
    const S_PSIO_CP_CONFIG sData0Config
    =  /* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
    {
        /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
        /* Action */      PSIO_OUT_LOW,       PSIO_OUT_BUFFER,    PSIO_OUT_HIGH,      PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION
    };

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, pConfig->u8Data0Pin, &sData0Config);

    /* Set slot 0 tick count as 4, slot 1 tick count as 8, slot 2 tick count as 8 */
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT0, 4);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT1, 8);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT2, 8);

    /* Loop slot 0 ~ slot 2 7 times */
    PSIO_SET_SCCTL(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT0, PSIO_SLOT2, 7, PSIO_REPEAT_DISABLE);

    /* Set output data */
    PSIO_SET_OUTPUT_DATA(PSIO, pConfig->u8Data0Pin, u8CMD);

    /* Set slot controller trigger source as software trigger */
    PSIO_SET_TRIGSRC(PSIO, pConfig->u8SlotCtrl, PSIO_SW_TRIGGER);

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, pConfig->u8SlotCtrl);
}


void PSIO_BQ2028_Read(PSIO_BQ2028_CFG_T *pConfig)
{
    const S_PSIO_CP_CONFIG sData0Config
    =  /* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
    {
        /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
        /* Action */      PSIO_IN_BUFFER,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION
    };

    /* Disable loop function */
    PSIO_SET_SCCTL(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT0, PSIO_SLOT0, 0, PSIO_REPEAT_DISABLE);

    /* Read data from device */
    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, pConfig->u8Data0Pin, &sData0Config);

    /* Set slot 0 tick count as 12, disable slot 1 and slot 2 */
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT0, 12);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT1, 0);

    /* Set slot controller trigger source as falling trigger */
    PSIO_SET_TRIGSRC(PSIO, pConfig->u8SlotCtrl, PSIO_FALLING_TRIGGER);
}


void PSIO_BQ2028_Open(PSIO_BQ2028_CFG_T *pConfig)
{
    /* PSIO pin general setting */
    PSIO_SET_GENCTL(PSIO, pConfig->u8Data0Pin, PSIO_PIN_ENABLE, pConfig->u8SlotCtrl
                    , PSIO_OPENDRAIN_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);

    /* Set data order ad LSB */
    PSIO_SET_ORDER(PSIO, pConfig->u8Data0Pin, PSIO_LSB);

    /* Set output data buffer width as 8 bit, input data width as 1 bit*/
    PSIO_SET_WIDTH(PSIO, pConfig->u8Data0Pin, 1, 8);

    /* Set output data buffer depth as 1 */
    PSIO_SET_OUTPUT_DEPTH(PSIO, pConfig->u8Data0Pin, PSIO_DEPTH1);

    /* Set input data buffer depth as 1 */
    PSIO_SET_INPUT_DEPTH(PSIO, pConfig->u8Data0Pin, PSIO_DEPTH1);

    /* Enable slot controller done interrupt */
    PSIO_ENABLE_INT(PSIO, PSIO_INTEN_SC0IE_Msk << pConfig->u8SlotCtrl);

    /* Set PSIO corresponding NVIC bit */
    NVIC_EnableIRQ(PSIO_IRQn);
}


uint8_t PSIO_BQ2028_CRC8(uint8_t u8Data)
{
    uint8_t u8CRC = 0;
    int iBitIndex = 0;

    u8CRC = u8Data;

    u8CRC ^= 0xFF;

    for (iBitIndex = 0; iBitIndex <= 7; iBitIndex ++)
    {
        if (u8CRC & 0x80)
            u8CRC = ((u8CRC << 1 & 0xff)) ^ 0x31;
        else
            u8CRC = ((u8CRC << 1 & 0xff));
    }

    return u8CRC;
}


bool PSIO_BQ2028_BUSY(void)
{
    if (g_eEvent == PSIO_HDQ_IDLE)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
