/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a code and execute in SRAM to program embedded Flash.
 *           (Support KEIL MDK Only)
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define MULTI_WORD_PROG_LEN         128          /* The maximum length is 128. */
#define APROM_TEST_BASE             0x7000
#define APROM_TEST_END              (APROM_TEST_BASE + ((FMC_FLASH_PAGE_SIZE)*1))
#define TEST_PATTERN                0x5A5A5A5A


uint32_t    g_au32page_buff[FMC_FLASH_PAGE_SIZE / 4];


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

    /* Set core clock as HCLK from HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, 0);

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
    uint32_t  u32LoopCnt, u32addr, u32maddr;          /* temporary variables */

    /* Unlock protected registers to operate SYS_Init and FMC ISP function */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\n");
    printf("+---------------------------------------------+\n");
    printf("|       Multi-word Program Sample             |\n");
    printf("+---------------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    FMC_ENABLE_AP_UPDATE();            /* Enable APROM erase/program */

    for (u32addr = APROM_TEST_BASE; u32addr < APROM_TEST_END; u32addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("Multi-word program APROM page 0x%x =>\n", u32addr);
        printf("  Erase...");

        if (FMC_Erase(u32addr) < 0)
        {
            printf("    Erase failed!!\n");
            goto err_out;
        }

        printf("[OK]\n");
        printf("  Program...");

        for (u32maddr = u32addr; u32maddr < u32addr + FMC_FLASH_PAGE_SIZE; u32maddr += MULTI_WORD_PROG_LEN)
        {
            /* Prepare test pattern */
            for (u32LoopCnt = 0; u32LoopCnt < MULTI_WORD_PROG_LEN; u32LoopCnt += 4)
                g_au32page_buff[u32LoopCnt / 4] = u32maddr + u32LoopCnt;

            /* execute multi-word program */
            if (FMC_Write128(u32maddr, g_au32page_buff) != 0)
            {
                printf("Failed !\n");
                goto err_out;
            }
        }

        printf("[OK]\n");
        printf("  Verify...");

        for (u32LoopCnt = 0; u32LoopCnt < FMC_FLASH_PAGE_SIZE; u32LoopCnt += 4)
            g_au32page_buff[u32LoopCnt / 4] = u32addr + u32LoopCnt;

        for (u32LoopCnt = 0; u32LoopCnt < FMC_FLASH_PAGE_SIZE; u32LoopCnt += 4)
        {
            if (FMC_Read(u32addr + u32LoopCnt) != g_au32page_buff[u32LoopCnt / 4])
            {
                printf("Failed !\n");
                printf("\n[FAILED] Data mismatch at address 0x%x, expect: 0x%x, read: 0x%x!\n", u32addr + u32LoopCnt, g_au32page_buff[u32LoopCnt / 4], FMC_Read(u32addr + u32LoopCnt));
                goto err_out;
            }
        }

        printf("[OK]\n");
    }

    printf("\n\nMulti-word program demo done.\n");

    /* Lock protected registers */
    SYS_LockReg();

    while (1);

err_out:
    printf("\n\nERROR!\n");

    while (1);

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
