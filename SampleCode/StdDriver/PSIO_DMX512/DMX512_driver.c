/***************************************************************************//**
 * @file        DMX512_driver.c
 * @version     V3.00
 * @brief       DMX512 Driver.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "DMX512_driver.h"
#include "M251.h"
#include <stdio.h>

static volatile uint8_t s_u8RcvDone = 0;
static volatile E_DMX512_FILTER_STATE s_eFilterState = eDMX512_READY;
static S_PSIO_DMX512_CFG *s_psRxConfig = NULL;      /* For RX handler use */
static volatile uint16_t s_u16CurChannel = 0;       /* Record current channel cnt */
static volatile uint16_t s_u16TargetChannel = 0;    /* For filter function compute */
static volatile uint16_t s_u16DataTmp;
static volatile uint16_t *s_pu16DataResult = NULL;

tPSIO_IRQHandler pfPSIO_IRQHandler = NULL;

/* Decode DMX512 frame */
E_DMX512_FRAME_TYPE  DMX512_FrameDecoder(S_PSIO_DMX512_CFG *psConfig, uint16_t *pu16Data)
{
    /* Special pattern */
    switch(*pu16Data)
    {
        case DMX512_PATTERN_START:
            return eDMX512_START;

        case DMX512_PATTERN_BREAK:
            return eDMX512_BREAK;
    }

    if((DMX512_PATTERN_DATA) != ((*pu16Data) & ~DMX512_PATTERN_DATA_MASK))
        return eDMX512_DATA;
    else
        return eDMX512_UNDEFINE;
}

/* PSIO IRQ handler */
void PSIO_IRQHandler(void)
{
    pfPSIO_IRQHandler();
}

/* Interrupt callback function */
void DMX512_ISRHandler_Receive_with_Filter(void)
{
    while(!PSIO_GET_TRANSFER_STATUS(PSIO, PSIO_TRANSTS_INFULL1_Msk)) {};

    if(PSIO_GET_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk))    /* INT0 interrupt */
    {
        PSIO_CLEAR_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk);

        s_u16DataTmp = (uint16_t) PSIO_GET_INPUT_DATA(PSIO, s_psRxConfig->u8RxPin);

        switch(DMX512_FrameDecoder(s_psRxConfig, (uint16_t *)&s_u16DataTmp))
        {
            case eDMX512_DATA:

                if(eDMX512_FILTER_DATA == s_eFilterState)
                {
                    s_u16CurChannel++;

                    if(s_u16TargetChannel == s_u16CurChannel)
                    {
                        s_u8RcvDone = 1;

                        *s_pu16DataResult = s_u16DataTmp;

                        s_eFilterState = eDMX512_FILTER_RECEIVE_DONE;
                    }
                }

                break;

            case eDMX512_START:
                if(eDMX512_FILTER_BREAK0 == s_eFilterState)
                {
                    s_eFilterState = eDMX512_FILTER_DATA;
                    s_u16CurChannel = 0;
                }
                else if(eDMX512_FILTER_DATA == s_eFilterState)
                {
                    s_u16CurChannel++;

                    if(s_u16TargetChannel == s_u16CurChannel)
                    {
                        s_u8RcvDone = 1;

                        *s_pu16DataResult = s_u16DataTmp;
                        s_eFilterState = eDMX512_FILTER_RECEIVE_DONE;
                    }
                }

                break;

            case eDMX512_BREAK:
                s_eFilterState = eDMX512_FILTER_BREAK0;
                break;

            case eDMX512_BREAK_START:
                printf("impossible.!\n");
                break;

            default:
                printf("undefined\n");
        }
    }
}

static __INLINE void setToData(S_PSIO_DMX512_CFG *psConfig)
{
    const S_PSIO_CP_CONFIG sTxConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_OUT_LOW,       PSIO_OUT_BUFFER,    PSIO_OUT_HIGH,      PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION}; 
                     
    /* Set slot count */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT0, psConfig->u32SlotCnt);      /* 1 start bit */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT1, psConfig->u32SlotCnt);      /* 8 data  bits */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT2, (psConfig->u32SlotCnt) * 2);/* 2 stop  bits */

    /*Clr slot setting*/
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT3, 0);  
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT4, 0);  
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT5, 0);  
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT6, 0);  
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT7, 0); 

    /* Loop slot1 7 times */
    PSIO_SET_SCCTL(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT1, PSIO_SLOT1, 7, PSIO_REPEAT_DISABLE);

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8TxPin, &sTxConfig); 

    /* Set data pin output width as 8 bit */
    PSIO_SET_WIDTH(PSIO, psConfig->u8TxPin, 0, 8);

    /* Set in data depth as 1 */
    PSIO_SET_INPUT_DEPTH(PSIO, psConfig->u8TxPin, PSIO_DEPTH1);
}

static __INLINE void setTo_BREAK_START(S_PSIO_DMX512_CFG *psConfig)
{
    const S_PSIO_CP_CONFIG sTxConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT1,         PSIO_SLOT2,         PSIO_SLOT3,         PSIO_SLOT4,         PSIO_SLOT5,         PSIO_SLOT6,         PSIO_SLOT7,
      /* Action */      PSIO_OUT_LOW,       PSIO_OUT_HIGH,      PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_LOW,       PSIO_OUT_HIGH}; 
                     
    /* Set slot count */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT0, psConfig->u32SlotCnt);      /* BREAK: 22 bits */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT1, psConfig->u32SlotCnt * 2);  /* MAB: 2 bits */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT2, psConfig->u32SlotCnt);      /* START bit: 1bit */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT3, psConfig->u32SlotCnt * 2);  /* DATA  bit_01: 2bit */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT4, psConfig->u32SlotCnt * 2);  /* DATA  bit_23: 2bit */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT5, psConfig->u32SlotCnt * 2);  /* DATA  bit_45: 2bit */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT6, psConfig->u32SlotCnt * 2);  /* DATA  bit_67: 2bit */
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT7, psConfig->u32SlotCnt * 2);  /* STOP  bits: 2bit */

    /* BREAK period is 22 bit time. */
    PSIO_SET_SCCTL(PSIO, psConfig->u8TxSlotCounter, PSIO_SLOT0, PSIO_SLOT0, 22 - 1, PSIO_REPEAT_DISABLE);

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8TxPin, &sTxConfig);
}

static __INLINE void setPSIO_Rx_11BIT(S_PSIO_DMX512_CFG *psConfig)
{
    const S_PSIO_CP_CONFIG sRxConfig 
                     = {/* Check Point0     Check Point1        Check Point2        Check Point3        Check Point4        Check Point5        Check Point6        Check Point7 */
      /* Slot */        PSIO_SLOT0,         PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,  PSIO_SLOT_DISABLE,
      /* Action */      PSIO_IN_BUFFER,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION,     PSIO_NO_ACTION};  
                     
    /* Set slot count */
    /* A unit of slot count is 1 bit.*/
    PSIO_SCSLOT_SET_SLOT(PSIO, psConfig->u8RxSlotCounter, PSIO_SLOT0, psConfig->u32SlotCnt);

    /* SLOT0: 11 rounds */
    PSIO_SET_SCCTL(PSIO, psConfig->u8RxSlotCounter, PSIO_SLOT0, PSIO_SLOT0, 11 - 1, PSIO_REPEAT_DISABLE);

    /* Set check point configuration */
    PSIO_SET_CP_CONFIG(PSIO, psConfig->u8RxPin, &sRxConfig);

    /* Set data pin input width as 11 bit */
    PSIO_SET_WIDTH(PSIO, psConfig->u8RxPin, 11, 0);

    /* Set input data depth as 1 */
    PSIO_SET_INPUT_DEPTH(PSIO, psConfig->u8RxPin, PSIO_DEPTH1);

    PSIO_SET_INTCTL(PSIO, psConfig->u8RxSlotCounter, PSIO_INT0, PSIO_SLOT0);
    PSIO_SET_TRIGSRC(PSIO, psConfig->u8RxSlotCounter, PSIO_FALLING_TRIGGER);
    PSIO_ENABLE_INT(PSIO, PSIO_INTEN_CON0IE_Msk);
}

uint32_t PSIO_DMX512_Open(S_PSIO_DMX512_CFG *psConfig)
{
    /* A unit of slot count is 1 bit. */
    CLK->CLKDIV1 = (CLK->CLKDIV1 & ~CLK_CLKDIV1_PSIODIV_Msk) | (31ul << CLK_CLKDIV1_PSIODIV_Pos);
    psConfig->u32SlotCnt = __HIRC / (((CLK->CLKDIV1 & CLK_CLKDIV1_PSIODIV_Msk) >> CLK_CLKDIV1_PSIODIV_Pos) + 1) / BITRATE_DMX512;

    psConfig->eState =  eDMX512_UNDEFINE;

    psConfig->pu8RcvDone = &s_u8RcvDone;
    
    /* Tx pin general setting */
    PSIO_SET_GENCTL(PSIO, psConfig->u8TxPin, PSIO_PIN_ENABLE, psConfig->u8TxSlotCounter
                    , PSIO_OUTPUT_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);
    
    /* Set data order ad LSB */
    PSIO_SET_ORDER(PSIO, psConfig->u8TxPin, PSIO_LSB);
    
    /* Rx pin general setting */
    PSIO_SET_GENCTL(PSIO, psConfig->u8RxPin, PSIO_PIN_ENABLE, psConfig->u8RxSlotCounter
                    , PSIO_INPUT_MODE, PSIO_HIGH_LEVEL, PSIO_HIGH_LEVEL);

    /* Set Rx data order as LSB */
    PSIO_SET_ORDER(PSIO, psConfig->u8RxPin, PSIO_LSB);
    
    return psConfig->u32SlotCnt;
}

int PSIO_DMX512_Tx(S_PSIO_DMX512_CFG *psConfig, uint32_t u32OutData, E_DMX512_FRAME_TYPE eFrameType)
{
    /* Wait for slot controller is not busy */
    while(PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8TxSlotCounter));

    if(psConfig->eState != eFrameType)    /* Initial state is eDMX512_UNDEFINE */
    {
        switch(eFrameType)
        {
            case eDMX512_BREAK_START:
                setTo_BREAK_START(psConfig);
                psConfig->eState = eDMX512_BREAK_START;
                break;

            case eDMX512_DATA:
                setToData(psConfig);
                psConfig->eState = eDMX512_DATA;
                break;

            default:
                printf("error: wrong state\n");
                psConfig->eState = eDMX512_UNDEFINE;
                return -1;
        }
    }

    switch(eFrameType)
    {
        case eDMX512_DATA:
            PSIO_SET_OUTPUT_DATA(PSIO, psConfig->u8TxPin, u32OutData);
            break;

        case eDMX512_BREAK_START:
            break;

        default:
            printf("wrong FrameType\n");
            psConfig->eState = eDMX512_UNDEFINE;
            return -1;
    }

    /* Set slot controller trigger source as software trigger */
    PSIO_START_SC(PSIO, psConfig->u8TxSlotCounter);
    return 0;
}

/* Wait specific channel and get the data */
int PSIO_DMX512_getChannelData(S_PSIO_DMX512_CFG *psConfig, uint16_t u16TargetChannel, uint16_t *pu16data)
{
    while(PSIO_GET_BUSY_FLAG(PSIO, psConfig->u8RxSlotCounter));

    setPSIO_Rx_11BIT(psConfig);

    /* Assign handler */
    pfPSIO_IRQHandler = DMX512_ISRHandler_Receive_with_Filter;

    /* Assign configuration */
    s_psRxConfig = psConfig;

    /* Record current DMX512 channel */
    s_u16CurChannel = 0;
    s_u16TargetChannel = u16TargetChannel;
    psConfig->pu8RcvDone = &s_u8RcvDone;
    s_pu16DataResult = pu16data;

    /* Clear RcvDone flag */
    *psConfig->pu8RcvDone = 0;

    return 0;
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/