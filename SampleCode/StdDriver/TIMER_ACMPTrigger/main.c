/****************************************************************************//**
 * @file     main.c
 * @version  V1.01
 * @brief    Use ACMP to trigger Timer0 counter reset mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CLK_HIRC    1
#define CLK_HXT     0
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

void TMR0_IRQHandler(void)
{
    printf("ACMP triggered timer reset while counter is at %d\n", TIMER_GetCaptureData(TIMER0));
    // Clear timer capture interrupt flag.
    TIMER_ClearCaptureIntFlag(TIMER0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

#if (CLK_SOURCE == CLK_HIRC )
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL frequency */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC_DIV4, PLL_CLOCK);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

#else

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Select HCLK clock source as HXT and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Set PLL frequency */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, PLL_CLOCK);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, BIT2 | BIT3);
#endif

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(ACMP01_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PB4 for ACMP1 positive input pin, PB5 for negative input pin */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB4MFP_Msk;
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_ACMP1_P1;
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB5MFP_Msk;
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB5MFP_ACMP1_N;

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);


    PB->DOUT |= 1 << GPIO_DOUT_DOUT3_Pos;    // Set PB.3 init state to high
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE3_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE3_Pos);  // Set PB.3 to output mode

    printf("\nThis sample code demonstrate ACMP trigger timer counter reset mode.\n");
    printf("Please connect ACMP1_P (PB.4) to PB.3 and ACMP1_N (PB.5)to GND,  press any key to continue\n");
    getchar();

    // Give a dummy target frequency here. Will over write capture resolution with macro
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);

    // Update prescale to set proper resolution.
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);

    // Set compare value as large as possible, so don't need to worry about counter overrun too frequently.
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFF);

    // Configure Timer 0 free counting mode
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_COUNTER_RESET_MODE, TIMER_CAPTURE_EVENT_RISING);
    // Set capture source from ACMP
    TIMER0->CTL |= TIMER_CTL_CAPSRC_Msk;
    // Set capture source from ACMP1
    TIMER0->EXTCTL &= ~TIMER_EXTCTL_INTERCAPSEL_Msk;
    TIMER0->EXTCTL |= TIMER_INTER_CAPTURE_FROM_ACMP1;

    // Start Timer 0
    TIMER_Start(TIMER0);

    /* Configure ACMP1. Enable ACMP1 and select band-gap voltage as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);

    /* Configure ACMP1. Enable ACMP1 and select ACMP1_N pin as the source of ACMP negative input. */
    //ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_PIN, ACMP_CTL_HYSTERESIS_DISABLE);

    /* Select P1 as ACMP1 positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    // Enable timer interrupt
    TIMER_EnableCaptureInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    while (1)
    {
        PB3 = 0; // low
        CLK_SysTickDelay(10000);
        PB3 = 1;  // high
        CLK_SysTickDelay(10000);
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
