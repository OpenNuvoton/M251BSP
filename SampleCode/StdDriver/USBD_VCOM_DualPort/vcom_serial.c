/****************************************************************************//**
 * @file     vcom_serial.c
 * @version  V0.10
 * @brief    M251 series USBD VCOM sample file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "vcom_serial.h"

uint32_t volatile g_u32OutToggle0 = 0, g_u32OutToggle1 = 0;

uint8_t volatile g_u8Suspend = 0;

uint8_t g_u8Idle = 0, g_u8Protocol = 0;

/*--------------------------------------------------------------------------*/
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
            g_u32OutToggle0 = 0;
            g_u32OutToggle1 = 0;
            g_u8Suspend = 0;
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enter power down to wait USB attached */
            g_u8Suspend = 1;

            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
            g_u8Suspend = 0;
        }

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
        extern uint8_t g_USBD_au8SetupPacket[];

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

            /* UART setting */
            if (g_USBD_au8SetupPacket[4] == 0) /* VCOM-1 */
                VCOM_LineCoding(0);

            if (g_USBD_au8SetupPacket[4] == 2) /* VCOM-2 */
                VCOM_LineCoding(1);
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Bulk IN
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk Out
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
            // Bulk Out
            EP6_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
            // Bulk IN
            EP7_Handler();

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

void EP2_Handler(void)
{
    g_u32TxSize0 = 0;
}


void EP3_Handler(void)
{
    /* Bulk OUT */
    if (g_u32OutToggle0 == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
    else
    {
        g_u32RxSize0 = USBD_GET_PAYLOAD_LEN(EP3);
        g_pu8RxBuf0 = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));

        g_u32OutToggle0 = USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk;
        /* Set a flag to indicate bulk out ready */
        g_i8BulkOutReady0 = 1;
    }
}

void EP6_Handler(void)
{
    /* Bulk OUT */
    if (g_u32OutToggle1 == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS6_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }
    else
    {
        g_u32RxSize1 = USBD_GET_PAYLOAD_LEN(EP6);
        g_pu8RxBuf1 = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP6));

        g_u32OutToggle1 = USBD->EPSTS0 & USBD_EPSTS0_EPSTS6_Msk;
        /* Set a flag to indicate bulk out ready */
        g_i8BulkOutReady1 = 1;
    }
}

void EP7_Handler(void)
{
    g_u32TxSize1 = 0;
}



/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void VCOM_Init(void)
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
    /* EP2 ==> Bulk IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /* EP4 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer offset for EP4 ->  */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /*****************************************************/
    /* EP5 ==> Interrupt IN endpoint, address 6 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM_1);
    /* Buffer offset for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);

    /* EP6 ==> Bulk Out endpoint, address 5 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_1);
    /* Buffer offset for EP6 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);

    /* EP7 ==> Bulk IN endpoint, address 4 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_1);
    /* Buffer offset for EP7 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);
}


void VCOM_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if (buf[0] & 0x80)   /* request data transfer direction */
    {
        // Device to host
        switch (buf[1])
        {
            case GET_LINE_CODE:
            {
                if (buf[4] == 0)   /* VCOM-1 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_sLineCoding0, 7);
                }

                if (buf[4] == 2)   /* VCOM-2 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_sLineCoding1, 7);
                }

                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 7);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
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
    else
    {
        // Host to device
        switch (buf[1])
        {
            case SET_CONTROL_LINE_STATE:
            {
                if (buf[4] == 0)   /* VCOM-1 */
                {
                    g_u16CtrlSignal0 = buf[3];
                    g_u16CtrlSignal0 = (g_u16CtrlSignal0 << 8) | buf[2];
                    //printf("RTS=%d  DTR=%d\n", (g_u16CtrlSignal0 >> 1) & 1, g_u16CtrlSignal0 & 1);
                }

                if (buf[4] == 2)   /* VCOM-2 */
                {
                    g_u16CtrlSignal1 = buf[3];
                    g_u16CtrlSignal1 = (g_u16CtrlSignal1 << 8) | buf[2];
                    //printf("RTS=%d  DTR=%d\n", (g_u16CtrlSignal0 >> 1) & 1, g_u16CtrlSignal0 & 1);
                }

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_LINE_CODE:
            {
                if (buf[4] == 0) /* VCOM-1 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_sLineCoding0, 7);

                if (buf[4] == 2) /* VCOM-2 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_sLineCoding1, 7);

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);

                break;
            }

            default:
            {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}

void VCOM_LineCoding(uint8_t port)
{
    uint32_t u32DataWidth;
    uint32_t u32Parity;
    uint32_t u32StopBits;

    if (port == 0)
    {
        NVIC_DisableIRQ(UART0_IRQn);
        // Reset software FIFO
        g_u16ComRbytes0 = 0;
        g_u16ComRhead0 = 0;
        g_u16ComRtail0 = 0;

        g_u16ComTbytes0 = 0;
        g_u16ComThead0 = 0;
        g_u16ComTtail0 = 0;

        // Reset hardware FIFO
        UART0->FIFO = UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk;

        // Set parity
        switch (g_sLineCoding0.u8ParityType)
        {
            case 0:
                u32Parity = UART_PARITY_NONE; // none parity
                break;

            case 1:
                u32Parity = UART_PARITY_ODD; // odd parity
                break;

            case 2:
                u32Parity = UART_PARITY_EVEN; // even parity
                break;

            default:
                u32Parity = UART_PARITY_NONE; // none parity

        }

        // bit width
        switch (g_sLineCoding0.u8DataBits)
        {
            case 5:
                u32DataWidth = UART_WORD_LEN_5;
                break;

            case 6:
                u32DataWidth = UART_WORD_LEN_6;
                break;

            case 7:
                u32DataWidth = UART_WORD_LEN_7;
                break;

            case 8:
                u32DataWidth = UART_WORD_LEN_8;
                break;

            default:
                u32DataWidth = UART_WORD_LEN_8;
                break;
        }

        // stop bit
        if (g_sLineCoding0.u8CharFormat > 0)
            u32StopBits = UART_STOP_BIT_2; // 2 or 1.5 bits
        else
            u32StopBits = UART_STOP_BIT_1;

        UART_SetLine_Config(UART0, g_sLineCoding0.u32DTERate, u32DataWidth, u32Parity, u32StopBits);


        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART0_IRQn);
    }
    else
    {
        NVIC_DisableIRQ(UART1_IRQn);
        // Reset software FIFO
        g_u16ComRbytes1 = 0;
        g_u16ComRhead1 = 0;
        g_u16ComRtail1 = 0;

        g_u16ComTbytes1 = 0;
        g_u16ComThead1 = 0;
        g_u16ComTtail1 = 0;

        // Reset hardware FIFO
        UART1->FIFO = UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk;

        // Set baudrate
        // Set parity
        switch (g_sLineCoding1.u8ParityType)
        {
            case 0:
                u32Parity = UART_PARITY_NONE; // none parity
                break;

            case 1:
                u32Parity = UART_PARITY_ODD; // odd parity
                break;

            case 2:
                u32Parity = UART_PARITY_EVEN; // even parity
                break;

            default:
                u32Parity = UART_PARITY_NONE; // none parity

        }

        // bit width
        switch (g_sLineCoding1.u8DataBits)
        {
            case 5:
                u32DataWidth = UART_WORD_LEN_5;
                break;

            case 6:
                u32DataWidth = UART_WORD_LEN_6;
                break;

            case 7:
                u32DataWidth = UART_WORD_LEN_7;
                break;

            case 8:
                u32DataWidth = UART_WORD_LEN_8;
                break;

            default:
                u32DataWidth = UART_WORD_LEN_8;
                break;
        }

        // stop bit
        if (g_sLineCoding1.u8CharFormat > 0)
            u32StopBits = UART_STOP_BIT_2; // 2 or 1.5 bits
        else
            u32StopBits = UART_STOP_BIT_1;

        UART_SetLine_Config(UART1, g_sLineCoding1.u32DTERate, u32DataWidth, u32Parity, u32StopBits);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART1_IRQn);
    }
}







