/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a code and execute it in SRAM to program embedded Flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


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
    uint32_t u32Addr;
    uint32_t u32Cnt;

    /* Unlock protected registers to operate SYS_Init and FMC ISP function */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      FMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Check FMC function execution address\n");
    printf("FMC_Erase: 0x%X, FMC_Write: 0x%X, FMC_Read: 0x%X\n",
           (uint32_t)FMC_Erase, (uint32_t)FMC_Write, (uint32_t)FMC_Read);

    /*
       This sample code is used to demonstrate how to implement a code to execute in SRAM.
       By setting KEIL's scatter file: scatter.scf,
                  IAR's linker configuration file: FMC_ExeInSRAM.icf,
                  GCC's linker script file: FMC_ExeInSRAM.ld,
       RO code is placed to 0x20000000 ~ 0x20000fff with RW is placed to 0x20001000 ~ 0x20001fff.
    */

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Enable FMC ISP functions */
    FMC_Open();

    /* Update APROM enabled */
    FMC_ENABLE_AP_UPDATE();

    /* The ROM address for erase/write/read demo */
    u32Addr = 0x4000;
    FMC_Erase(u32Addr); /* Erase page */

    for (u32Cnt = 0; u32Cnt < 0x100; u32Cnt += 4)
    {
        uint32_t u32Data, u32RData;

        /* Write Demo */
        u32Data = u32Cnt + 0x12345678;
        FMC_Write(u32Addr + u32Cnt, u32Data);

        if ((u32Cnt & 0xf) == 0)
            printf(".");

        /* Read Demo */
        u32RData = FMC_Read(u32Addr + u32Cnt);

        if (u32Data != u32RData)
        {
            printf("[Read/Write FAIL]\n");

            while (1);
        }
    }

    printf("\nISP function run at SRAM finished\n");

    /* Disable FMC ISP function */
    FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
