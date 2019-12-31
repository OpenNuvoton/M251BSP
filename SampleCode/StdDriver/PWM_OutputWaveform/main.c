/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to use PWM counter output waveform.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PWM0 and PWM1 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);
    CLK_EnableModuleClock(PWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency can be set equal to HCLK by choosing case 1 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Reset PWM0 and PWM1 module */
    SYS_ResetModule(PWM0_RST);
    SYS_ResetModule(PWM1_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set UART0 Default MPF */
    //    Uart0DefaultMPF() ;

    /* Set PB multi-function pins for PWM0 Channel0~5 */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk |
                       SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB0MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB5MFP_PWM0_CH0 | SYS_GPB_MFPL_PB4MFP_PWM0_CH1 | SYS_GPB_MFPL_PB3MFP_PWM0_CH2 |
                      SYS_GPB_MFPL_PB2MFP_PWM0_CH3 | SYS_GPB_MFPL_PB1MFP_PWM0_CH4 | SYS_GPB_MFPL_PB0MFP_PWM0_CH5);

    /* Set PB and PC multi-function pins for PWM1 Channel0~5 */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB12MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_PWM1_CH0 | SYS_GPB_MFPH_PB14MFP_PWM1_CH1 | SYS_GPB_MFPH_PB13MFP_PWM1_CH2 | SYS_GPB_MFPH_PB12MFP_PWM1_CH3);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC1MFP_Msk  | SYS_GPC_MFPL_PC0MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC1MFP_PWM1_CH4 | SYS_GPC_MFPL_PC0MFP_PWM1_CH5);
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
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM0SEL_Msk) ? "CPU" : "PLL");
    printf("PWM1 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM1SEL_Msk) ? "CPU" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with PWM0 and PWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PB.5), PWM0_CH1(PB.4), PWM0_CH2(PB.3), PWM0_CH3(PB.2), PWM0_CH4(PB.1), PWM0_CH5(PB.0)\n");
    printf("                         PWM1_CH0(PB.15),  PWM1_CH1(PB.14),  PWM1_CH2(PB.13),  PWM1_CH3(PB.12),  PWM1_CH4(PC.1), PWM1_CH5(PC.0)\n");


    /* PWM0 and PWM1 channel 0~5 frequency and duty configuration are as follows */
    PWM_ConfigOutputChannel(PWM0, 0, 5000, 25);
    PWM_ConfigOutputChannel(PWM0, 1, 5000, 50);
    PWM_ConfigOutputChannel(PWM0, 2, 15000, 25);
    PWM_ConfigOutputChannel(PWM0, 3, 15000, 50);
    PWM_ConfigOutputChannel(PWM0, 4, 25000, 25);
    PWM_ConfigOutputChannel(PWM0, 5, 25000, 50);
    PWM_ConfigOutputChannel(PWM1, 0, 35000, 25);
    PWM_ConfigOutputChannel(PWM1, 1, 35000, 50);
    PWM_ConfigOutputChannel(PWM1, 2, 45000, 25);
    PWM_ConfigOutputChannel(PWM1, 3, 45000, 50);
    PWM_ConfigOutputChannel(PWM1, 4, 55000, 25);
    PWM_ConfigOutputChannel(PWM1, 5, 55000, 50);

    /* Enable output of PWM0 and PWM1 channel 0~5 */
    PWM_EnableOutput(PWM0, 0x3F);
    PWM_EnableOutput(PWM1, 0x3F);

    /* Run in semihost, the DBGTRIOFF need to be 1 and pwm would output waveform*/
    SYS_UnlockReg();
    PWM0->CTL0 |= (0x1U << PWM_CTL0_DBGTRIOFF_Pos) ;
    PWM1->CTL0 |= (0x1U << PWM_CTL0_DBGTRIOFF_Pos) ;
    SYS_LockReg();

    /* Start PWM0 counter */
    PWM_Start(PWM0, 0x3F);
    /* Start PWM1 counter */
    PWM_Start(PWM1, 0x3F);

    printf("Press any key to stop.\n");
    getchar();

    /* Start PWM0 counter */
    PWM_ForceStop(PWM0, 0x3F);
    /* Start PWM1 counter */
    PWM_ForceStop(PWM1, 0x3F);

    printf("Done.");

    while (1);

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

