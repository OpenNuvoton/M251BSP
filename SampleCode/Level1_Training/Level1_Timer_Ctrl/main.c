/**************************************************************************//**
 * @file     main.c
 * @version  V1.10
 * @brief    TIMER function for level1 training course.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#define LED_R   PC4
#define LED_B   PC3
#define LED_G   PC5
#define LEDR1   PB14

#define LED_ON      0
#define LED_OFF     1

void TMR0_IRQHandler(void)
{
    /* Check if the interrupt occurred */
    if (TIMER_GetIntFlag(TIMER0))
    {
        /* Toggle LED */
        LED_B ^= 1;
        /* Clear TIMER0 interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
    }
}

void LED_Init(void)
{
    /* Set PC.3 ~ PC.5 to GPIO */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk)) |
                    (SYS_GPC_MFPL_PC3MFP_GPIO | SYS_GPC_MFPL_PC4MFP_GPIO | SYS_GPC_MFPL_PC5MFP_GPIO);

    /* Set PC.3 ~ PC.5 to GPIO output */
    GPIO_SetMode(PC, (BIT3 | BIT4 | BIT5), GPIO_MODE_OUTPUT);

    /* Let LED off after initialize */
    LED_R = LED_OFF;
    LED_G = LED_OFF;
    LED_B = LED_OFF;
}

void Timer_Init(void)
{
    /**************  TIMER0 ***************/
    /* Set TIMER0 in periodic mode ,frequency 1Hz and enable its interrupt */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    /**************  TIMER1 ***************/
    /* Set PB.14 as TM1_EXT pin */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_TM1_EXT);
    /* Set TIMER1 in periodic mode ,frequency 2Hz and enable its interrupt */
    TIMER_Open(TIMER1, TIMER_TOGGLE_MODE, 2);
    /* Set TM1_EXT as TOUT pin */
    TIMER_SELECT_TOUT_PIN(TIMER1, TIMER_TOUT_PIN_FROM_TX_EXT);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Select module clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+----------------------------------------+\n");
    printf("|    Level1 TIMER control Sample Code    |\n");
    printf("+----------------------------------------+\n\n");

    /* Init LED */
    LED_Init();

    /* Init TIMER */
    Timer_Init();

    /* Let TIMER0 & TIMER1 start to count */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER1);

    /* Got no where to go, just loop forever */
    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
