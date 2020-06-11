/****************************************************************************//**
 * @file     usbd_audio.c
 * @version  V0.10
 * @brief    NuMicro series USBD audio sample file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include <string.h>
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"
#include "buffer_control.h"
/*--------------------------------------------------------------------------*/
/* Global variables for Audio class */
volatile uint32_t g_usbd_UsbAudioState = 0;

volatile uint8_t g_usbd_RecMute       = 0x00;     /* Record MUTE control. 0 = normal. 1 = MUTE */
volatile int16_t g_usbd_RecVolumeL    = 0x1000;   /* Record left channel volume. Range is -32768 ~ 32767 */
volatile int16_t g_usbd_RecVolumeR    = 0x1000;   /* Record right channel volume. Range is -32768 ~ 32767 */
volatile int16_t g_usbd_RecMaxVolume  = 0x7FFF;
volatile int16_t g_usbd_RecMinVolume  = 0x8000;
volatile int16_t g_usbd_RecResVolume  = 0x400;

volatile uint8_t g_usbd_PlayMute      = 0x01;     /* Play MUTE control. 0 = normal. 1 = MUTE */
volatile int16_t g_usbd_PlayVolumeL   = 0x1000;   /* Play left channel volume. Range is -32768 ~ 32767 */
volatile int16_t g_usbd_PlayVolumeR   = 0x1000;   /* Play right channel volume. Range is -32768 ~ 32767 */
volatile int16_t g_usbd_PlayMaxVolume = 0x7FFF;
volatile int16_t g_usbd_PlayMinVolume = 0x8000;
volatile int16_t g_usbd_PlayResVolume = 0x400;

volatile uint8_t g_u8RecEn       = 0;
volatile uint8_t g_u8PlayEn      = 0;    /* To indicate data is output to I2S */
volatile int32_t g_i32AdjFlag    = 0;    /* To indicate current I2S frequency adjustment status */



/*--------------------------------------------------------------------------*/


/**
 * @brief       USBD Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the USBD ISR
 */
void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();

        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();

        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }

#ifdef SUPPORT_LPM

        if (u32State & USBD_STATE_L1SUSPEND)
        {
            /*
               TODO: Implement LPM SUSPEND flag here.
                     Recommend implementing the power-saving function in main loop.
            */
        }

        if (u32State & USBD_STATE_L1RESUME)
        {
            /*
               TODO: Implement LPM RESUME flag here.
            */
        }

#endif
    }

    if (u32IntSts & USBD_INTSTS_NEVWKIF_Msk)
    {
        /*Clear no-event wake up interrupt */
        USBD_CLR_INT_FLAG(USBD_INTSTS_NEVWKIF_Msk);
        /*
           TODO: Implement the function that will be executed when device is woken by non-USB event.
        */
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);

            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);

            // Isochronous IN
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);

            // Isochronous OUT
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }

        if (u32IntSts & USBD_INTSTS_EP8)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP8);
        }

        if (u32IntSts & USBD_INTSTS_EP9)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP9);
        }

        if (u32IntSts & USBD_INTSTS_EP10)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP10);
        }

        if (u32IntSts & USBD_INTSTS_EP11)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP11);
        }
    }

}

/**
 * @brief       EP2 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP2 event for recording.
 */

void EP2_Handler(void)
{
    /* ISO IN transfer ACK */
    if (g_usbd_UsbAudioState == UAC_START_AUDIO_RECORD)
    {
        UAC_DeviceEnable(UAC_MICROPHONE);
        g_usbd_UsbAudioState = UAC_PROCESSING_AUDIO_RECORD;
    }
    else if (g_usbd_UsbAudioState == UAC_PROCESSING_AUDIO_RECORD)
        g_usbd_UsbAudioState = UAC_BUSY_AUDIO_RECORD;

    if (g_usbd_UsbAudioState == UAC_BUSY_AUDIO_RECORD)
    {
        UAC_SendRecData();
    }
    else
        USBD_SET_PAYLOAD_LEN(EP2, 0);

}

/**
 * @brief       EP3 Handler (ISO OUT interrupt handler)
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP3 event (ISO OUT transfer ACK) for play audio.
 */
void EP3_Handler(void)
{
    uint8_t *pu8Buf;
    uint8_t *pu8Src;

    /* Get the address in USB buffer */
    pu8Src = (uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));

    /* Prepare for nex OUT packet */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /* Get the temp buffer */
    pu8Buf = (uint8_t *)&g_au32PLayBuffer[g_i8PlayBuffe_idx_IN][0];

    USBD_MemCopy(pu8Buf, pu8Src, EP3_MAX_PKT_SIZE);

    g_i8PlayBuffe_idx_IN++; //update buffer index

    /*Ring buffer*/
    if (g_i8PlayBuffe_idx_IN >= PLAY_BUFFER_DEPTH)
    {
        g_i8PlayBuffe_idx_IN = 0;
    }

    if (g_u8PlayEn == 0)
    {
        /* Start play data output through I2S only when we have enough data in buffer */
        if (get_1ms_SamplesInPlayBuf() >= (PLAY_BUFFER_DEPTH / 2))
        {
            g_u8PlayEn = 1;
            SPII2S_ENABLE_TX(SPI0);
            SPII2S_ENABLE_TXDMA(SPI0);
        }
    }

}



/*--------------------------------------------------------------------------*/
/**
 * @brief       UAC Class Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure endpoints for UAC class
 */
void UAC_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Isochronous IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | ISO_IN_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /*****************************************************/
    /* EP3 ==> Isochronous OUT endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | ISO_OUT_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}


/**
 * @brief       UAC class request
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process UAC class requests
 */
void UAC_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if (buf[0] & 0x80)   /* request data transfer direction */
    {
        // Device to host
        switch (buf[1])
        {
            case UAC_GET_CUR:
            {
                switch (buf[3])
                {
                    case MUTE_CONTROL:
                    {
                        if (REC_FEATURE_UNITID == buf[5])
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMute;
                        else if (PLAY_FEATURE_UNITID == buf[5])
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMute;

                        /* Data stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 1);
                        break;
                    }

                    case VOLUME_CONTROL:
                    {
                        if (REC_FEATURE_UNITID == buf[5])
                        {
                            /* Left or right channel */
                            if (buf[2] == 1)
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecVolumeL;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecVolumeL >> 8;
                            }
                            else
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecVolumeR;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecVolumeR >> 8;
                            }

                        }
                        else if (PLAY_FEATURE_UNITID == buf[5])
                        {
                            /* Left or right channel */
                            if (buf[2] == 1)
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayVolumeL;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayVolumeL >> 8;
                            }
                            else
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayVolumeR;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayVolumeR >> 8;
                            }
                        }

                        /* Data stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 2);
                        break;
                    }

                    default:
                    {
                        /* Setup error, stall the device */
                        USBD_SetStall(0);
                    }
                }

                // Trigger next Control Out DATA1 Transaction.
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case UAC_GET_MIN:
            {
                switch (buf[3])
                {
                    case VOLUME_CONTROL:
                    {
                        if (REC_FEATURE_UNITID == buf[5])
                        {
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMinVolume;
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecMinVolume >> 8;
                        }
                        else if (PLAY_FEATURE_UNITID == buf[5])
                        {
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMinVolume;
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayMinVolume >> 8;
                        }

                        /* Data stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 2);
                        break;
                    }

                    default:
                        /* STALL control pipe */
                        USBD_SetStall(0);
                }

                // Trigger next Control Out DATA1 Transaction.
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case UAC_GET_MAX:
            {
                switch (buf[3])
                {
                    case VOLUME_CONTROL:
                    {
                        if (REC_FEATURE_UNITID == buf[5])
                        {
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMaxVolume;
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecMaxVolume >> 8;
                        }
                        else if (PLAY_FEATURE_UNITID == buf[5])
                        {
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMaxVolume;
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayMaxVolume >> 8;
                        }

                        /* Data stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 2);
                        break;
                    }

                    default:
                        /* STALL control pipe */
                        USBD_SetStall(0);
                }

                // Trigger next Control Out DATA1 Transaction.
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case UAC_GET_RES:
            {
                switch (buf[3])
                {
                    case VOLUME_CONTROL:
                    {
                        if (REC_FEATURE_UNITID == buf[5])
                        {
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecResVolume;
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecResVolume >> 8;
                        }
                        else if (PLAY_FEATURE_UNITID == buf[5])
                        {
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayResVolume;
                            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayResVolume >> 8;
                        }

                        /* Data stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 2);
                        break;
                    }

                    default:
                        /* STALL control pipe */
                        USBD_SetStall(0);
                }

                // Trigger next Control Out DATA1 Transaction.
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
            }
        }
    }
    else
    {
        // Host to device
        switch (buf[1])
        {
            case UAC_SET_CUR:
            {
                switch (buf[3])
                {
                    case MUTE_CONTROL:
                        if (REC_FEATURE_UNITID == buf[5])
                            USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecMute, buf[6]);
                        else if (PLAY_FEATURE_UNITID == buf[5])
                        {
                            USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayMute, buf[6]);
                        }

                        /* Status stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 0);
                        break;

                    case VOLUME_CONTROL:
                        if (REC_FEATURE_UNITID == buf[5])
                        {
                            if (buf[2] == 1)
                            {
                                /* Prepare the buffer for new record volume of left channel */
                                USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecVolumeL, buf[6]);
                            }
                            else
                            {
                                /* Prepare the buffer for new record volume of right channel */
                                USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecVolumeR, buf[6]);
                            }
                        }
                        else if (PLAY_FEATURE_UNITID == buf[5])
                        {
                            if (buf[2] == 1)
                            {
                                /* Prepare the buffer for new play volume of left channel */
                                USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayVolumeL, buf[6]);
                            }
                            else
                            {
                                /* Prepare the buffer for new play volume of right channel */
                                USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayVolumeR, buf[6]);
                            }
                        }

                        /* Status stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 0);
                        break;

                    default:
                        /* STALL control pipe */
                        USBD_SetStall(0);
                        break;
                }

                break;
            }

            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}

/**
 * @brief       Set Interface standard request
 *
 * @param[in]   u32AltInterface Interface
 *
 * @return      None
 *
 * @details     This function is used to set UAC Class relative setting
 */
void UAC_SetInterface(void)
{
    uint8_t buf[8];
    uint32_t u32AltInterface;

    USBD_GetSetupPacket(buf);

    u32AltInterface = buf[2];

    if (buf[4] == 1)
    {
        /* Audio Iso IN interface */
        if (u32AltInterface == 1)
        {
            g_usbd_UsbAudioState = UAC_START_AUDIO_RECORD;
            USBD_SET_DATA1(EP2);
            USBD_SET_PAYLOAD_LEN(EP2, 0);
            UAC_DeviceEnable(UAC_MICROPHONE);

        }
        else if (u32AltInterface == 0)
        {
            UAC_DeviceDisable(UAC_MICROPHONE);
            USBD_SET_DATA1(EP2);
            USBD_SET_PAYLOAD_LEN(EP2, 0);
            g_usbd_UsbAudioState = UAC_STOP_AUDIO_RECORD;
        }
    }
    else if (buf[4] == 2)
    {
        /* Audio Iso OUT interface */
        if (u32AltInterface == 1)
        {
            USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
            UAC_DeviceEnable(UAC_SPEAKER);
        }
        else
            UAC_DeviceDisable(UAC_SPEAKER);
    }
}

/**
  * @brief  SendRecData, prepare the record data for next ISO transfer.
  * @param  None.
  * @retval None.
  */
__INLINE void UAC_SendRecData(void)
{
    uint8_t *pu8Dst;
    uint8_t *pu8Src;

    if (get_1ms_SamplesInRecordBuf() > 0)
    {
        /* Get the address in USB buffer */
        pu8Dst = (uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));
        pu8Src = (uint8_t *)(&g_au32RecordBuffer[g_i8RecordBuffer_idx_OUT][0]);

        g_i8RecordBuffer_idx_OUT++;

        /*//wrap around*/
        if (g_i8RecordBuffer_idx_OUT >= RECORD_BUFFER_DEPTH)
            g_i8RecordBuffer_idx_OUT = 0;

        /* Prepare the data to USB IN buffer */
        USBD_MemCopy(pu8Dst, pu8Src, EP2_MAX_PKT_SIZE);

        USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);
    }
    else
    {
        USBD_SET_PAYLOAD_LEN(EP2, 0);
    }
}


/**
  * @brief  UAC_DeviceEnable. To enable the device to play or record audio data.
  * @param  u8Object: To select the device, UAC_MICROPHONE or UAC_SPEAKER.
  * @retval None.
  */
void UAC_DeviceEnable(uint8_t u8Object)
{
    if (u8Object == UAC_MICROPHONE)
    {
        /* Enable record hardware */
        if (g_u8RecEn == 0)
        {
            /* Reset record buffer */
            memset(g_au32RecordBuffer, 0, RECORD_BUFFER_DEPTH * RECORD_BUFFER_LENHTH_IN_BYTES);
            g_i8RecordBuffer_idx_OUT = 0;
            g_i8RecordBuffer_idx_IN = 0;
            //printf("Start Record\n");
        }

        g_u8RecEn = 1;

    }
    else
    {
        /* Eanble play hardware */

        /* Reset Play buffer */
        if (g_u8PlayEn == 0)
        {

            /* Fill 0x0 to buffer before playing for buffer operation smooth */
            memset(g_au32PLayBuffer, 0, PLAY_BUFFER_DEPTH * PLAY_BUFFER_LENHTH_IN_BYTES);

            g_i8PlayBuffe_idx_IN  = 0;
            g_i8PlayBuffe_idx_OUT = 0;

            //printf("Start Play\n");

        }

    }
}


/**
  * @brief  UAC_DeviceDisable. To disable the device to play or record audio data.
  * @param  u8Object: To select the device, UAC_MICROPHONE or UAC_SPEAKER.
  * @retval None.
  */
void UAC_DeviceDisable(uint8_t u8Object)
{
    if (u8Object ==  UAC_MICROPHONE)
    {
        /* Disable record hardware/stop record */
        g_u8RecEn = 0;

        //printf("Stop Record\n");
    }
    else
    {
        /* Disable play hardware/stop play */
        g_u8PlayEn = 0;

        SPII2S_DISABLE_TX(SPI0);
        SPII2S_DISABLE_TXDMA(SPI0);

        //printf("Stop Play\n");
    }
}












