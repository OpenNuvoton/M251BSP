/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to use FMC CRC32 ISP command to calculate the CRC32 checksum of APROM and LDROM.
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
    uint32_t    u32Data, u32ChkSum;    /* temporary data */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("\n");
    printf("+------------------------------------+\n");
    printf("|      FMC CRC32 Sample Demo         |\n");
    printf("+------------------------------------+\n");
    printf("* Start address and size of CRC32 calculation region must be page alignment.\n");

    SYS_UnlockReg();                   /* Unlock protected registers to operate FMC ISP function */

    FMC_Open();                        /* Enable FMC ISP function */

    u32Data = FMC_ReadCID();           /* Read company ID. Should be 0xDA. */
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* Read product ID. */
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 4));

    printf("\nLDROM (0x100000 ~ 0x101000) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 caculation on LDROM.
     */
    u32ChkSum = FMC_GetChkSum(FMC_LDROM_BASE, FMC_LDROM_SIZE);

    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating LDROM CRC32 checksum!\n");
        goto lexit;                    /* failed */
    }

    printf("0x%x\n", u32ChkSum);       /* print out LDROM CRC32 check sum value */

    printf("\nAPROM (0x0 ~ 0x4000) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 caculation on APROM.
     */
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE, 0x4000);

    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM CRC32 checksum!\n");
        goto lexit;
    }

    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    /*
     *  Request FMC hardware to run CRC32 caculation on APROM.
     */
    printf("\nAPROM (0x0 ~ 0x8000) CRC32 checksum =>  ");
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE, 0x8000);

    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM CRC32 checksum!\n");
        goto lexit;
    }

    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    printf("\nFMC CRC32 checksum test done.\n");

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
