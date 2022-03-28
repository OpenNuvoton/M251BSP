/**************************************************************************//**
 * @file     main.c
 * @version  V1.10
 * @brief    Change duty cycle of output waveform to show different brightness of Red LED.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"


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

    /* Enable PWM1 module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PLL clock as 96 MHz from HIRC/4 */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC_DIV4, 96000000);

    /* Waiting for PLL clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    //    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK0, 0);

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PLL, 0);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Reset PWM1 module */
    SYS_ResetModule(PWM1_RST);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PC.4 multi-function pins for PWM1 Channel 1 for Red of LED */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) |
                    SYS_GPC_MFPL_PC4MFP_PWM1_CH1;

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    SYS_Init();

    /* Set frequency and duty of PWM1 Channel 1 to control Red LED */
    PWM_ConfigOutputChannel(PWM1, 1, 1, 50);
    /* Enable PWM1 Output path for channel 1 */
    PWM_EnableOutput(PWM1, BIT1);
    /* Start PWM1 Counter */
    PWM_Start(PWM1, BIT1);

    /* Got no where to go, just loop forever */
    while (1);
}
