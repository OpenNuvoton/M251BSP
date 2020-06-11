/**************************************************************************//**
 * @file        WS1812B_driver_LED.c
 * @version     V3.00
 * @brief       Worldsemi WS2812B LED Driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <string.h>
#include "NuMicro.h"
#include "WS1812B_driver_LED.h"


/*---------------------------------------------------------------------------------------------------------*/
/*  Encode Pattern Function                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
static int PSIO_EncodePattern(uint32_t *pu32SourceAddress, uint32_t *pu32DestinationAddress, uint32_t u32DataLength, uint8_t u8PinNum)
{
    int i, j;

    if (pu32DestinationAddress == NULL)
    {
        return eERROR_MEMORY_ADDR;
    }

    /* Encode Pattern */
    for (i = 0; i < u32DataLength; i++)
    {
        for (j = 0; j < 24; j++)
        {
            if (((*(pu32SourceAddress + i)) >> j) & 0x1)
            {
                (*(pu32DestinationAddress + (((i / u8PinNum)*u8PinNum * 3) + ((((23 - j) / 8)*u8PinNum) + (i % u8PinNum))))) |= (0x6 << ((j % 8) * 3));
            }
            else
            {
                (*(pu32DestinationAddress + (((i / u8PinNum)*u8PinNum * 3) + ((((23 - j) / 8)*u8PinNum) + (i % u8PinNum))))) |= (0x4 << ((j % 8) * 3));
            }
        }
    }

    return i;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Clear Internal Memory Function                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void PSIO_WS2812B_ClearInternalMemory(S_PSIO_WS2812B_LED_CFG *psConfig)
{
    memset((void *)psConfig->pu32InternalMemory, 0, (psConfig->u32DataLength) * 3 * 4);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Send Data Function                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
int PSIO_WS2812B_Send_Pattern(S_PSIO_WS2812B_LED_CFG *psConfig)
{
    int iError = 0;

    /* Encode data */
    if ((iError = PSIO_EncodePattern(psConfig->pu32DataAddr, psConfig->pu32InternalMemory, psConfig->u32DataLength, psConfig->u8PinNumber)) < 0)
    {
        return iError;
    }

    /* Set PDMA transfer configuration */
    PDMA->DSCT[psConfig->u8PDMAChannel].CTL =
        PDMA_OP_BASIC |
        PDMA_REQ_SINGLE |
        PDMA_SAR_INC |                /* source address -> incremented */
        PDMA_DAR_FIX |                /* destination address -> fixed(PSIO) */
        PDMA_WIDTH_32 |               /* transfer width -> 32-bit */
        (((psConfig->u32DataLength * 3) - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
    PDMA->DSCT[psConfig->u8PDMAChannel].SA = (uint32_t)psConfig->pu32InternalMemory;
    PDMA->DSCT[psConfig->u8PDMAChannel].DA = (uint32_t)(&(PSIO->PODAT));

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, psConfig->u8SlotCtrl);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8SlotCtrl));

    /* Clear Internal Memory */
    PSIO_WS2812B_ClearInternalMemory(psConfig);

    return iError;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Initialize PSIO Setting Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
int PSIO_WS2812B_Open(S_PSIO_WS2812B_LED_CFG *psConfig)
{
    const S_PSIO_CP_CONFIG sCheckPointConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT3,         PSIO_SLOT4,         PSIO_SLOT5,         PSIO_SLOT6,         PSIO_SLOT7,
      /* Action */      PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER,    PSIO_OUT_BUFFER};
                     
    uint8_t u8PinNum = 0;

    /* Setting all pin configure */
    while (psConfig->u8PinNumber > u8PinNum)
    {

        if (u8PinNum > 8)
        {
            return eERROR_PIN_NUMBER;
        }

        /* PSIO pin general setting */
        PSIO_SET_GENCTL(PSIO, psConfig->pu8PinCFG[u8PinNum], PSIO_PIN_ENABLE, psConfig->u8SlotCtrl
                        , PSIO_OUTPUT_MODE, PSIO_LOW_LEVEL, PSIO_LOW_LEVEL);

        /* Set data order ad LSB */
        PSIO_SET_ORDER(PSIO, psConfig->pu8PinCFG[u8PinNum], PSIO_MSB);

        /* Set check point configuration */
        PSIO_SET_CP_CONFIG(PSIO, psConfig->pu8PinCFG[u8PinNum], &sCheckPointConfig);        

        /* Set output data width as 24 bit */
        PSIO_SET_WIDTH(PSIO, psConfig->pu8PinCFG[u8PinNum], 0, 24);

        /* Set output data depth as 1 */
        PSIO_SET_OUTPUT_DEPTH(PSIO, psConfig->pu8PinCFG[u8PinNum], PSIO_DEPTH1);

        /* Counting pin number */
        u8PinNum++;
    };  /* Loop until last pin */

    /* Set PDMA output pin enable */
    if (psConfig->u8PinNumber == 1)
    {
        PSIO_SET_PDMA_OUTPUT(PSIO, psConfig->u8SlotCtrl, PSIO_PDMACTL_OPIN0EN_Msk);
    }
    else if (psConfig->u8PinNumber == 2)
    {
        PSIO_SET_PDMA_OUTPUT(PSIO, psConfig->u8SlotCtrl, PSIO_PDMACTL_OPIN0EN_Msk | PSIO_PDMACTL_OPIN1EN_Msk);
    }
    else if (psConfig->u8PinNumber == 3)
    {
        PSIO_SET_PDMA_OUTPUT(PSIO, psConfig->u8SlotCtrl
                             , PSIO_PDMACTL_OPIN0EN_Msk | PSIO_PDMACTL_OPIN1EN_Msk | PSIO_PDMACTL_OPIN2EN_Msk);
    }
    else if (psConfig->u8PinNumber == 4)
    {
        PSIO_SET_PDMA_OUTPUT(PSIO, psConfig->u8SlotCtrl
                             , PSIO_PDMACTL_OPIN0EN_Msk | PSIO_PDMACTL_OPIN1EN_Msk | PSIO_PDMACTL_OPIN2EN_Msk | PSIO_PDMACTL_OPIN3EN_Msk);
    }
    else if (psConfig->u8PinNumber == 5)
    {
        PSIO_SET_PDMA_OUTPUT(PSIO, psConfig->u8SlotCtrl
                             , PSIO_PDMACTL_OPIN0EN_Msk | PSIO_PDMACTL_OPIN1EN_Msk | PSIO_PDMACTL_OPIN2EN_Msk | PSIO_PDMACTL_OPIN3EN_Msk
                             | PSIO_PDMACTL_OPIN4EN_Msk);
    }
    else if (psConfig->u8PinNumber == 6)
    {
        PSIO_SET_PDMA_OUTPUT(PSIO, psConfig->u8SlotCtrl
                             , PSIO_PDMACTL_OPIN0EN_Msk | PSIO_PDMACTL_OPIN1EN_Msk | PSIO_PDMACTL_OPIN2EN_Msk | PSIO_PDMACTL_OPIN3EN_Msk
                             | PSIO_PDMACTL_OPIN4EN_Msk | PSIO_PDMACTL_OPIN5EN_Msk);
    }
    else if (psConfig->u8PinNumber == 7)
    {
        PSIO_SET_PDMA_OUTPUT(PSIO, psConfig->u8SlotCtrl
                             , PSIO_PDMACTL_OPIN0EN_Msk | PSIO_PDMACTL_OPIN1EN_Msk | PSIO_PDMACTL_OPIN2EN_Msk | PSIO_PDMACTL_OPIN3EN_Msk
                             | PSIO_PDMACTL_OPIN4EN_Msk | PSIO_PDMACTL_OPIN5EN_Msk | PSIO_PDMACTL_OPIN6EN_Msk);
    }
    else if (psConfig->u8PinNumber == 8)
    {
        PSIO_SET_PDMA_OUTPUT(PSIO, psConfig->u8SlotCtrl
                             , PSIO_PDMACTL_OPIN0EN_Msk | PSIO_PDMACTL_OPIN1EN_Msk | PSIO_PDMACTL_OPIN2EN_Msk | PSIO_PDMACTL_OPIN3EN_Msk
                             | PSIO_PDMACTL_OPIN4EN_Msk | PSIO_PDMACTL_OPIN5EN_Msk | PSIO_PDMACTL_OPIN6EN_Msk | PSIO_PDMACTL_OPIN7EN_Msk);
    }

    /* Set slot0~7 tick count as 9 */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, 5);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT1, 5);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT2, 5);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT3, 5);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT4, 5);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT5, 5);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT6, 5);
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT7, 5);

    /* Enable repeat mode */
    PSIO_SET_SCCTL(PSIO, psConfig->u8SlotCtrl, PSIO_SLOT0, PSIO_SLOT7, 0, PSIO_REPEAT_ENABLE);

    /* Enable PDMA channel */
    PDMA->CHCTL |= (1 << psConfig->u8PDMAChannel);

    /* Set request source */
    PDMA_SetTransferMode(PDMA, psConfig->u8PDMAChannel, PDMA_PSIO_TX, 0, 0);

    return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Release PSIO setting Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void PSIO_WS2812B_Close(S_PSIO_WS2812B_LED_CFG *psConfig)
{
    /* Close PDMA Channel x */
    PDMA->CHCTL &= ~(1 << psConfig->u8PDMAChannel);

    /* Reset PSIO */
    SYS->IPRST2 |= SYS_IPRST2_PSIORST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PSIORST_Msk;
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
