/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Demonstrate how to implement a composite device (USB micro printer device and HID Transfer).
 *           Transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with a USB device.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "micro_printer_and_hid_transfer.h"

#define CRYSTAL_LESS    1 /* CRYSTAL_LESS must be 1 if USB clock source is HIRC */
#define TRIM_INIT           (SYS_BASE+0x118)

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal HIRC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

#if (CRYSTAL_LESS)
    /* Switch HCLK clock source to Internal HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select module clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));
#else
    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for External XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_48MHZ);

    /* Select module clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBDSEL_PLL, CLK_CLKDIV0_USB(2));

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, BIT2 | BIT3);
#endif


    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

}

void PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if (CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    uint8_t au8Str[9];

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB Composite Device Sample Code     |\n");
    printf("|          USB Micro Printer + HID Transfer             |\n");
    printf("+-------------------------------------------------------+\n");

    /* Open USB controller */
    USBD_Open(&gsInfo, PTR_ClassRequest, NULL);

    /* Endpoint configuration */
    PTR_Init();
    /* Start USB device */
    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

    PB->MODE = 0x500;   // PB.4, PB.5 output mode


#if CRYSTAL_LESS
    /* Backup default trim value */
    u32TrimInit = M32(TRIM_INIT);

    /* Clear SOF */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
#endif

    while (1)
    {

#if CRYSTAL_LESS

        /* Start USB trim function if it is not enabled. */
        if ((SYS->HIRCTRIMCTL & SYS_HIRCTRIMCTL_FREQSEL_Msk) != 0x1)
        {
            /* Start USB trim only when USB signal arrived */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /*
                    USB clock trim function:
                    HIRC Trimming with boundary function enhances robustility
                    and keeps HIRC in right frequency while receiving unstable USB signal
                */
                SYS->HIRCTRIMCTL = (0x1 << SYS_HIRCTRIMCTL_REFCKSEL_Pos)
                                   | (0x1 << SYS_HIRCTRIMCTL_FREQSEL_Pos)
                                   | (0x0 << SYS_HIRCTRIMCTL_LOOPSEL_Pos)
                                   | (0x1 << SYS_HIRCTRIMCTL_BOUNDEN_Pos)
                                   | (10  << SYS_HIRCTRIMCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when any error found */
        if (SYS->HIRCTRIMSTS & (SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable USB clock trim function */
            SYS->HIRCTRIMCTL = 0;

            /* Clear trim error flags */
            SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
        }

#endif

        /* Enter power down when USB suspend */
        if (g_u8Suspend)
            PowerDown();


        CLK_SysTickDelay(2000);   // delay

        if (++au8Str[1] > 0x39)
            au8Str[1] = 0x30;      // increase 1 to 10 than reset to 0

        PB->DOUT ^= 0x10; // PB.4
    }
}



/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

