/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to config/erase XOM region.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "M251.h"
#include "xomapi.h"



void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set UART MFP */
    Uart0DefaultMPF();
}


int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();


    /* Configure UART: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code is used to show how to build an XOM libary.

        The location of XOM region is defined by scatter file: xom_scatter.scf
        The API header file is xomapi.h
        The XOM functions are implemented in xom.c

        This project is only used to build code for XOM region and test its funcitons.
        To enable XOM region, please use "NuMicro ICP Programming Tool".

        example flow:
        1. Build XOMlib
        2. Download XOMlib code by press key "F8" in Keil MDK.
        3. Open "NuMicro ICP Programming Tool" to enable XOM region and according to xom_scatter.scf settings.
        4. The library (xomlib.o) and header (xomlib.h) is located at lib directory.
        5. Pass xomlib.o & xomlib.h to the people who will call the funcitons in XOM.

    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|      FMC XOM Libary Build Example      |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active*/
    FMC_Open();
    FMC_EnableAPUpdate();

    /* Read User Configuration */
    printf("\b");
    printf("XOM Status = 0x%X\n", FMC->XOMSTS);

    /* Run XOM function */
    printf("\n");
    printf(" 100 + 200 = %d\n", XOM_Add(100, 200));
    printf(" 500 - 100 = %d\n", XOM_Sub(500, 100));
    printf(" 200 * 100 = %d\n", XOM_Mul(200, 100));
    printf("1000 / 250 = %d\n", XOM_Div(1000, 250));
    XOM_Sum(0, 3);


    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
