/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to use PWM brake function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       PWM0 Brake0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM0 Brake0 interrupt event
 */
void BRAKE0_IRQHandler(void)
{
    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (PWM0 channel 0 output will toggle again)\n");
    getchar();

    /* Unlock protected registers */
    SYS_UnlockReg();
    // Clear brake interrupt flag
    PWM_ClearFaultBrakeIntFlag(PWM0, PWM_FB_EDGE);
    /* Lock protected registers */
    SYS_LockReg();
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable IP module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency can be set equal to HCLK by choosing case 1 */
    /* case 1.PWM clock frequency is set equal to PCLK */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Reset PWM0 module */
    SYS_ResetModule(PWM0_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set UART0 Default MPF */
    Uart0DefaultMPF() ;

    /* Set PB multi-function pins for PWM0 Channel 0*/
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk) | SYS_GPB_MFPL_PB5MFP_PWM0_CH0;

    /* Set PB1 multi-function pin for PWM0 brake pin 0 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB1MFP_Msk) | SYS_GPB_MFPL_PB1MFP_PWM0_BRAKE0;

    /* Set PA3 multi-function pin for GPIO */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA3MFP_Msk) | SYS_GPA_MFPL_PA3MFP_GPIO;
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PA3 to generate brake event to PWM0 brake pin 0(PB.1).\n");
    printf("  It will generate brake interrupt and PWM0 channel 0 output stop toggling.\n");
    printf("  I/O configuration:\n");
    printf("    PWM0 brake pin 0(PB.1) <--> PA3\n\n");

    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
    PA3 = 0;

    // PWM0 frequency is 100Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM0, 0, 100, 30);

    // Enable PWM0 Channel0 output
    PWM_EnableOutput(PWM0, 0x1);

    /* Unlock protected registers */
    SYS_UnlockReg();

    // Enable brake and interrupt
    PWM_EnableFaultBrake(PWM0, PWM_CH_0_MASK, 1, PWM_FB_EDGE_BKP0);
    PWM_EnableFaultBrakeInt(PWM0, 0);

    // Enable brake noise filter : brake pin 0, filter count=7, filter clock=HCLK/128
    PWM_EnableBrakeNoiseFilter(PWM0, 0, 7, PWM_NF_CLK_DIV_128);

    // Clear brake interrupt flag
    PWM_ClearFaultBrakeIntFlag(PWM0, PWM_FB_EDGE);

    /* Lock protected registers */
    SYS_LockReg();

    NVIC_ClearPendingIRQ(BRAKE0_IRQn);
    NVIC_EnableIRQ(BRAKE0_IRQn);

    // Start
    PWM_Start(PWM0, 1);

    printf("\nPress any key to generate a brake event.\n");
    getchar();
    PA3 = 1;

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
