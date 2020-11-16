/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show how to print and get character with IDE console window.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


/**
 *  @brief  Init system clock and I/O multi function .
 *  @param  None
 *  @return None
 */
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

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

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


/**
 *  @brief  main function.
 *  @param  None
 *  @return None
 */
int32_t main()
{
    /*
    To enable semihost, user must define "DEBUG_ENABLE_SEMIHOST" constant when buildind sample code.
    If defined DEBUG_ENABLE_SEMIHOST = 1 or 2 and ICE connected, the message will output to ICE.
    if defined DEBUG_ENABLE_SEMIHOST = 1 and ICE off line, the message will re-direct to UART debug port.
    if defined DEBUG_ENABLE_SEMIHOST = 2 and ICE off line, no any debug message output.

    This sample code is used to show how to print message/getchar on IDE debug environment.
    It will echo all input character back on UART #1 of KEIL IDE.

    In KEIL MDK, user need to open "View->Serial Window->UART #0" windows in debug mode.
    In IAR Workbench, user need to open "View->Terminal I/O" in debug mode.

    NOTE1: Hardfault_Handler is used for semihost. User cannot overwrite it when using semihost.
           If it is necessary to process hardfault, user can append code to ProcessHardfault of retarget.c
    NOTE2: Semihost only works with Nuvoton NuLink ICE Dongle in debug mode.
    NOTE3: The message will output to debug port if Nuvoton NuLink ICE Dongle is not connected.


    Semihost On/Off | NuLink Connected | Output Path
    ==============================================================
          1         |         1        |  ICE
          1         |         0        |  UART Debug Port / NULL when DEBUG_ENABLE_SEMIHOST=2
          0         |         1        |  UART Debug Port
          0         |         0        |  UART Debug Port
    --------------------------------------------------------------
    */

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);


    printf("\n Start SEMIHOST test: \n");

    while (1)
    {
        int8_t i8item;
        i8item = getchar();
        printf("%c\n", i8item);
    }

}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
