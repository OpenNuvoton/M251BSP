/**************************************************************************//**
 * @file     main.c
 * @version  V1.01
 * @brief    Change duty cycle and period of output waveform in PWM down count type.
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

volatile uint32_t g_u32Period;

void TMR0_IRQHandler(void)
{
    if (TPWM_GET_PERIOD_INT_FLAG(TIMER0))
    {
        static uint32_t s_u32Toggle = 0;

        if (s_u32Toggle == 0)
        {
            /* Set PWM period to generate output frequency 36000 Hz */
            TPWM_SET_PERIOD(TIMER0, ((g_u32Period / 2) - 1));

            /* Set PWM duty, 40% */
            TPWM_SET_CMPDAT(TIMER0, (((g_u32Period / 2) * 4) / 10));
        }
        else
        {
            /* Set PWM period to generate output frequency 18000 Hz */
            TPWM_SET_PERIOD(TIMER0, (g_u32Period - 1));

            /* Set PWM duty, 50% */
            TPWM_SET_CMPDAT(TIMER0, (g_u32Period / 2));
        }

        s_u32Toggle ^= 1;
        TPWM_CLEAR_PERIOD_INT_FLAG(TIMER0);
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

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set Timer0 PWM CH0(T0) pin (PB.5)*/
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB5MFP_TM0;

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

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    printf("+-----------------------------------------------+\n");
    printf("|    Timer PWM Change Duty Cycle Sample Code    |\n");
    printf("+-----------------------------------------------+\n\n");

    printf("# Timer0 PWM_CH0 frequency of first period is 18000 Hz and duty is 50%%.\n");
    printf("# Timer0 PWM_CH0 frequency of second period is 36000 Hz and duty is 40%%.\n");
    printf("# I/O configuration:\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    if (TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50) != 18000)
    {
        printf("Set the frequency different from the user\n");
    }

    /* Get initial period and comparator value */
    g_u32Period = TPWM_GET_PERIOD(TIMER0) + 1;

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Enable period event interrupt */
    TPWM_ENABLE_PERIOD_INT(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER0);

    printf("*** Check Timer0 PWM_CH0 output waveform by oscilloscope ***\n");

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
