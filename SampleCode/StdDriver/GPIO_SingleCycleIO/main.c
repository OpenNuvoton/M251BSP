/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show GPIO single cycle IO bus performance.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void);
void UART0_Init(void);
extern void GPIO_SingleCycleIO_Test(void);

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;
    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, BIT2 | BIT3);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART, TIMER0 and TIMER2 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);

    /* Select module clock source and clock divider */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set Timer 2 event counter pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SYS_GPB_MFPL_PB3MFP_TM2;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|    GPIO Single Cycle IO Bus Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");
    printf("GPIO supports Single Cycle IO Bus. User can access GPIO in one clock cycle.\n");
    printf("This sample code toggles PA.0 state 50 times and measures the toggle speed.\n");
    printf("Please connect PA.0 and PB.3(TIMER2 event counter pin).\n");
    printf("Enter any key to continue.\n\n");
    getchar();
    printf("Testing ...\n\n");

    /* Configure PA.0 as output mode */
    PA0 = 0;
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);

    /* Toggle PA.0 state 50 times and measure the toggle speed. */
    GPIO_SingleCycleIO_Test();

    while (1);
}
