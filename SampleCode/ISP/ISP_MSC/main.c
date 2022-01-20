/******************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use embedded data flash as storage to implement a USB Mass-Storage device.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "M251_User.h"
#include "massstorage.h"

#define PLL_CLOCK               48000000

/* This is a dummy implementation to replace the same function in clk.c for size limitation. */
uint32_t CLK_GetPLLClockFreq(void)
{
    return PLL_CLOCK;
}

/*--------------------------------------------------------------------------*/

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
    CLK->AHBCLK  |= CLK_AHBCLK_ISPCKEN_Msk;
    CLK->AHBCLK  |= CLK_AHBCLK_GPBCKEN_Msk;

    /* Set PB.15 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE15_Msk);

    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk) | SYS_GPB_MFPH_PB15MFP_GPIO;
}

void gotoAPROM(void)
{
    /* Boot from AP */
    FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk;
    NVIC_SystemReset();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* The code should boot from LDROM: check the boot setting */

    /* Check if GPB.15 is low */
    if (PB15 != 0)
    {
        /* Boot from AP */
        gotoAPROM();
    }

    SYS_UnlockReg();

    SYS_Init();

    FMC->ISPCTL = FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk;


    USBD_Open(&gsInfo);

    /* Endpoint configuration */
    MSC_Init();

    /* Start of USBD_Start() */
    CLK_SysTickDelay(100000);


    /* Disable software-disconnect function */
    USBD->SE0 = 0;

    /* Clear USB-related interrupts before enable interrupt */
    USBD->INTSTS = (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);

    /* Enable USB-related interrupts. */
    USBD->INTEN = (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);
    /* End of USBD_Start() */

    NVIC_EnableIRQ(USBD_IRQn);

    while (1)
    {
        MSC_ProcessCmd();

        if (PB15)
        {
            /* Reset */
            gotoAPROM();
        }
    }
}
