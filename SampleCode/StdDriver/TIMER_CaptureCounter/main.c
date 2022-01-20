/**************************************************************************//**
 * @file     main.c
 * @version  V1.01
 * @brief    Show how to use the timer2 capture function to capture timer2 counter value.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CLK_HIRC    1
#define CLK_HXT     0
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

volatile uint32_t g_au32TMRINTCount[4] = {0};

void TMR2_IRQHandler(void)
{
    if (TIMER_GetCaptureIntFlag(TIMER2))
    {
        /* Clear Timer2 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER2);

        g_au32TMRINTCount[2]++;
    }
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

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));

    /* Enable TIMER peripheral clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HIRC, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set multi-function pins for UART0 RXD(PA.0) and TXD(PA.1) */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_UART0_RXD | SYS_GPA_MFPL_PA1MFP_UART0_TXD);

    /* Set multi-function pins for Timer0/Timer3 toggle-output pin and Timer2 event counter pin (PB.5)(PB.4)(PB.3)(PB.2)*/
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk
                       | SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB5MFP_TM0 | SYS_GPB_MFPL_PB4MFP_TM1
                      | SYS_GPB_MFPL_PB3MFP_TM2 | SYS_GPB_MFPL_PB2MFP_TM3);

    /* Set multi-function pin for Timer2 external capture pin (PB.13)*/
    SYS->GPB_MFPH = SYS_GPB_MFPH_PB13MFP_TM2_EXT;

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32InitCount;
    uint32_t au32CAPValue[10], u32CAPDiff;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART for printf */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------+\n");
    printf("|    Timer2 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 1000 Hz\n");
    printf("    - Toggle-output mode and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Toggle-output mode and frequency is 1 Hz\n");
    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HIRC              \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Interrupt enable                  \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - Event counter mode enable         \n");
    printf("    - External capture mode enable      \n");
    printf("    - Capture trigger interrupt enable  \n");
    printf("# Connect TM0(PB.5) toggle-output pin to TM2(PB.3) event counter pin.\n");
    printf("# Connect TM3(PB.2) toggle-output pin to TM2_EXT(PB.13) external capture pin.\n\n");

    /* Enable Timer2 NVIC */
    NVIC_EnableIRQ(TMR2_IRQn);

    /* Open Timer0 in toggle-output mode and toggle-output frequency is 500 Hz*/
    if (TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1000) != 1000)
    {
        printf("Set the frequency different from the user\n");
    }

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 Hz */
    if (TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, 2) != 2)
    {
        printf("Set the frequency different from the user\n");
    }

    /* Enable Timer2 event counter input and external capture function */
    if (TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1) != 1)
    {
        printf("Set the frequency different from the user\n");
    }

    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);
    TIMER_EnableCapture(TIMER2, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_FALLING);
    TIMER_EnableInt(TIMER2);
    TIMER_EnableCaptureInt(TIMER2);

    /* case 1. */
    printf("# Period between two falling edge captured event should be 500 counts.\n");

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[2] = 0;

    /* Start Timer0, Timer3 and Timer2 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER3);
    TIMER_Start(TIMER2);

    /* Check Timer2 capture trigger interrupt counts */
    while ((g_au32TMRINTCount[2] <= 10) && (u32InitCount < 10))
    {
        if (g_au32TMRINTCount[2] != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER2);

            if (u32InitCount ==  0)
            {
                printf("    [%2u]: %4u. (1st captured value)\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);

                if (au32CAPValue[u32InitCount] != 0)   // First capture event will reset counter value
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }
            else if (u32InitCount ==  1)
            {
                printf("    [%2u]: %4u. (2nd captured value) \n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);

                if (au32CAPValue[u32InitCount] != 500)   // Second event gets two capture event duration counts directly
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2u]: %4u. Diff: %u.\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount], u32CAPDiff);

                if (u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }

            u32InitCount = g_au32TMRINTCount[2];
        }
    }

    printf("*** PASS ***\n\n");

    /* case 2. */
    TIMER_StopCapture(TIMER2);
    TIMER_Stop(TIMER2);

    while (TIMER_IS_ACTIVE(TIMER2));

    TIMER_ClearIntFlag(TIMER2);
    TIMER_ClearCaptureIntFlag(TIMER2);

    /* Enable Timer2 event counter input and external capture function */
    if (TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1) != 1)
    {
        printf("Set the frequency different from the user\n");
    }

    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);
    TIMER_EnableCapture(TIMER2, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_GET_LOW_PERIOD);
    TIMER_EnableInt(TIMER2);
    TIMER_EnableCaptureInt(TIMER2);
    TIMER_Start(TIMER2);

    printf("# Get first low duration should be 250 counts.\n");
    printf("# And follows duration between two rising edge captured event should be 500 counts.\n");

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[2] = 0;

    /* Enable Timer2 event counter input and external capture function */
    TIMER2->CMP = 0xFFFFFF;
    TIMER2->CTL = TIMER_CTL_CNTEN_Msk | TIMER_CTL_INTEN_Msk | TIMER_CTL_EXTCNTEN_Msk | TIMER_CONTINUOUS_MODE;
    TIMER2->EXTCTL = TIMER_EXTCTL_CAPEN_Msk | TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_EVENT_GET_LOW_PERIOD | TIMER_EXTCTL_CAPIEN_Msk;

    /* Check Timer2 capture trigger interrupt counts */
    while ((g_au32TMRINTCount[2] <= 10) && (u32InitCount < 10))
    {
        if (g_au32TMRINTCount[2] != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER2);

            if (u32InitCount ==  0)
            {
                printf("    [%2u]: %4u. (1st captured value)\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);

                if (au32CAPValue[u32InitCount] != 0)   // First capture event will reset counter value
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }
            else if (u32InitCount ==  1)
            {
                printf("    [%2u]: %4u. (2nd captured value)\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);

                if (au32CAPValue[u32InitCount] != 250)   // Get low duration counts directly
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2u]: %4u. Diff: %u.\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount], u32CAPDiff);

                if (u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }

            u32InitCount = g_au32TMRINTCount[2];
        }
    }

    /* Stop Timer0, Timer2 and Timer3 counting */
    TIMER_Stop(TIMER0);
    TIMER_Stop(TIMER2);
    TIMER_Stop(TIMER3);

    printf("*** PASS ***\n");

    while (1)
    {
        ;
    };
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
