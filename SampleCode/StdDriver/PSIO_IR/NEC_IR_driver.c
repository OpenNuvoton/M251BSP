/**************************************************************************//**
 * @file        NEC_IR_driver.c
 * @version     V3.00
 * @brief       NEC IR device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <NuMicro.h>
#include "NEC_IR_driver.h"

#define MAX_TX_CNT 5

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

enum
{
    eSEND_DATA = 0,
    eREPEAT,
};

static uint32_t s_au32InternalUse[MAX_TX_CNT];     //The maximun transfer count
static S_PSIO_NEC_CFG  *s_pPSIOConfig;
volatile static uint8_t s_u8TransferDone = 0;


/*---------------------------------------------------------------------------------------------------------*/
/*  PSIO IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void PSIO_IRQHandler(void)
{
    /* INT0 interrupt */
    if (PSIO_GET_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk))
    {
        /* Clear INT0 interrupt flag */
        PSIO_CLEAR_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk);

        /* Check output empty flag is 1 */
        if (PSIO_GET_TRANSFER_STATUS(PSIO, (PSIO_TRANSTS_OUTEPY0_Msk << (s_pPSIOConfig->u8TxPin * 4))) && (s_u8TransferDone == 0))
        {
            static uint32_t u32DataIndex = 1;

            if (u32DataIndex < MAX_TX_CNT)
            {
                PSIO_SET_OUTPUT_DATA(PSIO, s_pPSIOConfig->u8TxPin, s_au32InternalUse[u32DataIndex++]);
            }
            else
            {
                u32DataIndex = 1;
                s_u8TransferDone = 1;
            }
        }
    }
    else
    {
        printf("Unknown interrupt occur!!! \n");
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Encode Pattern Function                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
static int PSIO_EncodePattern(uint8_t u8Repeat, uint8_t u8Address0, uint8_t u8Address1, uint8_t u8Command0, uint8_t u8Command1)
{
    int i = 0, j = 0;

    memset(s_au32InternalUse, 0x00, MAX_TX_CNT * sizeof(uint32_t));

    /* Encode data for send data mode or repeat mode */
    if (u8Repeat == eSEND_DATA)
    {
        uint32_t u32Data = 0;
        uint8_t u8DataIndex = 0;

        u32Data = (u8Command1 << 24) | (u8Command0 << 16) | (u8Address1 << 8) | (u8Address0);

        for (i = 0; i < MAX_TX_CNT; i++)
        {
            while (j < 32)
            {
                /* Header + Address0 + Address1 + Command0 + Command1 */
                if (u8DataIndex < 32)
                {
                    if (i == 0 && j < 16)       //9ms   high
                    {
                        s_au32InternalUse[i] |= (0x1 << j);
                        j++;
                    }
                    else if (i == 0 && j < 24)     //4.5ms low
                    {
                        s_au32InternalUse[i] |= (0x0 << j);
                        j++;
                    }
                    else
                    {
                        if (u32Data & (0x1 << u8DataIndex))   //Logic '1'
                        {
                            s_au32InternalUse[i] |= (0x1 << j);
                            j += 4;
                            u8DataIndex++;
                        }
                        else                             //Logic '0'
                        {
                            s_au32InternalUse[i] |= (0x1 << j);
                            j += 2;
                            u8DataIndex++;
                        }
                    }
                }
                else                 //STOP bit
                {
                    s_au32InternalUse[i] |= (0x1 << j);
                    return i;
                }
            }

            j -= 32;
        }
    }
    else if (u8Repeat == eREPEAT)
    {
        j = 0;

        while (j < 32)
        {
            if (j < 16)         //9ms   high
            {
                s_au32InternalUse[0] |= (0x1 << j);
                j++;
            }
            else if (j < 20)     //2.25ms low
            {
                s_au32InternalUse[0] |= (0x0 << j);
                j++;
            }
            else                 //560us high
            {
                s_au32InternalUse[0] |= (0x1ul << j);
                return j;
            }
        }
    }
    else
    {
        return eERROR_SEND_MODE;
    }


    return i;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Send Data Function                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
int PSIO_NEC_Send(S_PSIO_NEC_CFG *pConfig, uint8_t u8Address0, uint8_t u8Address1, uint8_t u8Command0, uint8_t u8Command1)
{
    int iError = 0;

    /* Encode data */
    if ((iError = PSIO_EncodePattern(eSEND_DATA, u8Address0, u8Address1, u8Command0, u8Command1)) < 0)
    {
        return iError;
    }

    /* Set output data */
    PSIO_SET_OUTPUT_DATA(PSIO, pConfig->u8TxPin, s_au32InternalUse[0]);

    s_u8TransferDone = 0;

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, pConfig->u8SlotCtrl);

    return iError;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Send Repeat Command Function                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int PSIO_NEC_Repeat(S_PSIO_NEC_CFG *pConfig)
{
    int iError = 0;

    if ((iError = PSIO_EncodePattern(eREPEAT, 0, 0, 0, 0)) < 0)
    {
        return iError;
    }

    /* Set output data */
    PSIO_SET_OUTPUT_DATA(PSIO, pConfig->u8TxPin, s_au32InternalUse[0]);

    s_u8TransferDone = 0;

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, pConfig->u8SlotCtrl);

    return iError;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Initialize PSIO Setting Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void PSIO_NEC_Open(S_PSIO_NEC_CFG *pConfig)
{
    const S_PSIO_CP_CONFIG sCheckPointConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT3,         PSIO_SLOT4,         PSIO_SLOT5,         PSIO_SLOT6,         PSIO_SLOT7,
      /* Action */      PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER};   
                     
    s_pPSIOConfig = pConfig;

    /* PSIO pin general setting */
    PSIO_SET_GENCTL(PSIO, pConfig->u8TxPin, PSIO_PIN_ENABLE, pConfig->u8SlotCtrl
                    , PSIO_OUTPUT_MODE, PSIO_LOW_LEVEL, PSIO_LOW_LEVEL);

    /* Set data order ad LSB */
    PSIO_SET_ORDER(PSIO, pConfig->u8TxPin, PSIO_LSB);
      
    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, pConfig->u8TxPin, &sCheckPointConfig);

    /* Set slot0~7 tick count as 9 */
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT0, 9);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT1, 9);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT2, 9);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT3, 9);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT4, 9);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT5, 9);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT6, 9);
    PSIO_SCSLOT_SET_SLOT(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT7, 9);

    /* Set output data width as 8 bit */
    PSIO_SET_WIDTH(PSIO, pConfig->u8TxPin, 0, 8);

    /* Set output data depth as 4 */
    PSIO_SET_OUTPUT_DEPTH(PSIO, pConfig->u8TxPin, PSIO_DEPTH4);

    /* Set interrupt 0 on slot3 */
    PSIO_SET_INTCTL(PSIO, pConfig->u8SlotCtrl, PSIO_INT0, PSIO_SLOT3);

    /* Enable interrupt 0 */
    PSIO_ENABLE_INT(PSIO, PSIO_INTEN_CON0IE_Msk);

    /* Loop slot0~slot7 17 times */
    PSIO_SET_SCCTL(PSIO, pConfig->u8SlotCtrl, PSIO_SLOT0, PSIO_SLOT7, 17, PSIO_REPEAT_DISABLE);

    //Set PSIO corresponding NVIC bit
    NVIC_EnableIRQ(PSIO_IRQn);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Release PSIO setting Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void PSIO_NEC_Close(S_PSIO_NEC_CFG *pConfig)
{
    /* Disable PSIO corresponding NVIC bit */
    NVIC_DisableIRQ(PSIO_IRQn);

    /* Disable interrupt 0 */
    PSIO_DISABLE_INT(PSIO, PSIO_INTEN_CON0IE_Msk);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
