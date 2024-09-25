/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show how to config/erase XOM region.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define XOMR0   0
#define XOMR1   1
#define XOMR2   2
#define XOMR3   3

#define XOMR0_BASE    0x7000

extern int32_t Lib_XOM_ADD(uint32_t u32Val1, uint32_t u32Val2);

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

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();
}

int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code is used to show how to use StdDriver API to enable/erase XOM.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|   FMC XOM config & erase Sample Code   |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active*/
    FMC_Open();
    FMC_EnableAPUpdate();

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG0_ADDR));

    printf("XOM Status = 0x%X\n", FMC->XOMSTS);
    printf("Any key to continue...\n");
    getchar();

    if ((FMC->XOMSTS & 0x1) != 0x1)
    {
        /* Config XOMR0 */
        if (FMC_Is_XOM_Actived(XOMR0) == 0)
        {
            uint32_t u32Status;
            u32Status = FMC_Config_XOM(XOMR0, XOMR0_BASE, 0x1);

            if (u32Status)
                printf("XOMR0 Config fail...\n");
            else
                printf("XOMR0 Config OK...\n");
        }

        /* Config XOMR0 */
        printf("\nAny key to reset chip to eanble XOM regions...\n");
        getchar();

        /* Reset chip to enable XOM region. */
        SYS_ResetChip();

        while (1) {};
    }

    /* Run XOM function */
    printf("\n");
    printf(" 100 + 200 = %d\n", Lib_XOM_ADD(100, 200));

    printf("\nXOMR0 active success....\n");
    printf("\nAny key to Erase XOM...\n");
    getchar();

    if ((FMC->XOMSTS & 0x1) == 0x1)
    {
        /* Erase XOMR0 region */
        if (FMC_Erase_XOM(XOMR0) == 0)
            printf("Erase XOMR0....OK\n");
        else
            printf("Erase XOMR0....Fail\n");



        printf("\nAny key to reset chip...\n");
        getchar();

        /* Reset chip to finish erase XOM region. */
        SYS_ResetChip();
    }

    printf("LOOP\n");

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
