/******************************************************************************//**
 * @file     main.c
 * @version  V1.01
 * @brief    Use embedded data flash as storage to implement a USB Mass-Storage device.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "M251_User.h"
#include "massstorage.h"

#define DetectPin           PA0
#define TRIM_INIT           (SYS_BASE + 0x118)
#define PLL_CLOCK           48000000

/* There are dummy or customized function for code size limitation. */
void ISP_CLK_SysTickDelay(uint32_t u32USec)
{
    SysTick->LOAD = u32USec * CyclesPerUs;
    SysTick->VAL  = (0x0u);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0u)
    {
    }

    /* Disable SysTick counter */
    SysTick->CTRL = 0u;
}

uint32_t CLK_GetPLLClockFreq(void)
{
    return PLL_CLOCK;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal HIRC 48 MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to Internal HIRC and HCLK source divide 1 , USB clock source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;

    SystemCoreClock = __HIRC;             // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()

    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
    CLK->AHBCLK  |= (CLK_AHBCLK_ISPCKEN_Msk | CLK_AHBCLK_GPBCKEN_Msk);

    /* Set detect pin to input mode */
    PA->MODE &= ~GPIO_MODE_MODE0_Msk;
    SYS->GPA_MFPL &= ~SYS_GPA_MFPL_PA0MFP_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TrimInit;

    /* The code should boot from LDROM: check the boot setting */
    SYS_UnlockReg();

    SYS_Init();

    FMC->ISPCTL = FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk;

    USBD_Open(&gsInfo);

    /* Endpoint configuration */
    MSC_Init();

    /* Start of USBD_Start() */
    ISP_CLK_SysTickDelay(100000);

    /* Disable software-disconnect function */
    USBD->SE0 = 0;

    /* Clear USB-related interrupts before enable interrupt */
    USBD->INTSTS = (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);

    /* Enable USB-related interrupts. */
    USBD->INTEN = (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);
    /* End of USBD_Start() */

    NVIC_EnableIRQ(USBD_IRQn);

    /* Backup default trim value */
    u32TrimInit = M32(TRIM_INIT);

    /* Clear SOF */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

    /* Check if detect pin is low */
    while (DetectPin == 0)
    {
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

        MSC_ProcessCmd();
    }

    /* Reset system and boot from APROM */
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    /* FMC_ISPCTL_BS_Msk only works with Boot from APROM/LDROM without IAP mode. */
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    /* Wait system reset */
    NVIC_SystemReset();
}
