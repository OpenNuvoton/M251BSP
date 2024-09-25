/*************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement a USB audio class device.
 *           Codec is used in this sample code to play the audio data from Host.
 *           It also supports to record data from codec to Host.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x118)

void SYS_Init(void)
{
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Debug UART clock setting */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_HIRC, 0);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable I2C0 peripheral clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    Uart0DefaultMPF();

    /*Set PB.4, PB.5 as I2C0_SDA and I2C0_SCL function pins*/
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk) | SYS_GPB_MFPL_PB4MFP_I2C0_SDA;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk) | SYS_GPB_MFPL_PB5MFP_I2C0_SCL;

    /* Configure SPI0 related multi-function pins. */
    /* GPD[3:0] : SPI0_CLK (I2S_BCLK), SPI0_MISO (I2S_DI), SPI0_MOSI (I2S_DO), SPI0_SS (I2S_LRCLK). */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD0MFP_SPI0_MOSI | SYS_GPD_MFPL_PD1MFP_SPI0_MISO | SYS_GPD_MFPL_PD2MFP_SPI0_CLK | SYS_GPD_MFPL_PD3MFP_SPI0_SS;

    /* GPA[4] : SPI0_I2S_MCLK */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA4MFP_Msk)) | SYS_GPA_MFPL_PA4MFP_SPI0_I2SMCLK;

    PB->SMTEN |= GPIO_SMTEN_SMTEN4_Msk;
    PB->SMTEN |= GPIO_SMTEN_SMTEN5_Msk;
}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/* Init I2C interface */
void I2C0_Init(void)
{
    /* Open I2C2 and set clock to 400k */
    I2C_Open(I2C0, 400000);

    /* Get I2C2 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART for message printing */
    UART0_Init();

#ifdef __HID__
    GPIO_Init();
#endif

    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB Audio CODEC Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C2 to access codec */
    I2C0_Init();

    /* Open SPI0 as slave mode */
    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, AUDIO_RATE, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);

    /* Set PD4 low to enable phone jack on NuMaker board. */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD4MFP_Msk);
    GPIO_SetMode(PD, BIT4, GPIO_MODE_OUTPUT);
    PD4 = 0;

    /* Set MCLK and enable MCLK */
    SPII2S_EnableMCLK(SPI0, 12000000);

#if NAU8822
    //    NAU8822_Setup();
#else
    NAU88L25_Reset();
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(10000);
    SPI0->I2SCTL |= SPI_I2SCTL_ORDER_Msk;
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(20000);
    NAU88L25_Setup();
#endif

    /* Configure PDMA */
    PDMA_Init();

#ifndef __FEEDBACK__
    /* Configure TIMER0 for adjusting codec's PLL */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 500);
    TIMER_EnableInt(TIMER0);
    NVIC_SetPriority(TMR0_IRQn, 3);
    NVIC_EnableIRQ(TMR0_IRQn);
#endif

    USBD_Open(&gsInfo, UAC_ClassRequest, UAC_SetInterface);

    /* Endpoint configuration */
    UAC_Init();
    NVIC_SetPriority(USBD_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while (1)
    {
#ifdef __HID__
        HID_UpdateHidData();
#endif

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
    }
}
