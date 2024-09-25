/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a multi-boot system to boot from different applications in APROM.
 *           A LDROM code and 4 APROM code are implemented in this sample code.
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
    uint8_t u8Ch;

    /* Unlock protected registers to operate SYS_Init and FMC ISP function */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            FMC_MultiBoot, RO=0x0
            FMC_Boot0, RO=0x4000
            FMC_Boot1, RO=0x8000
            FMC_Boot2, RO=0xC000
            FMC_Boot3, RO=0x10000
        2. Reset MCU to execute FMC_MultiBoot.

    */

    printf("\nPlease check boot from APROM with IAP first.\n\n");
    printf("\tMulti-Boot Sample Code(0x%X)\n\n", FMC_GetVECMAP());


    /* Enable FMC ISP function */
    FMC_Open();

#if defined(__BASE__)
    printf("Boot from 0x0\n");
#endif
#if defined(__BOOT0__)
    printf("Boot from 0x4000\n");
#endif
#if defined(__BOOT1__)
    printf("Boot from 0x8000\n");
#endif
#if defined(__BOOT2__)
    printf("Boot from 0xC000\n");
#endif
#if defined(__BOOT3__)
    printf("Boot from 0x10000\n");
#endif
#if defined(__LDROM__)
    printf("Boot from LDROM\n");
#endif

    printf("VECMAP = 0x%X\n", FMC_GetVECMAP());


    printf("Select one boot image: \n");
#if defined (__ARMCC_VERSION) || defined (__ICCARM__)
    printf("[0] Boot 0, base = 0x4000\n");
#endif
    printf("[1] Boot 1, base = 0x8000\n");
#if defined (__ARMCC_VERSION) || defined (__ICCARM__)
    printf("[2] Boot 2, base = 0xC000\n");
#endif
    printf("[3] Boot 3, base = 0x10000\n");
#if defined (__ARMCC_VERSION) || defined (__ICCARM__)
    printf("[4] Boot 4, base = 0x100000\n");
#endif
    printf("[Others] Boot, base = 0x0\n");

    u8Ch = getchar();

    switch (u8Ch)
    {
#if defined (__ARMCC_VERSION) || defined (__ICCARM__)

        case '0':
            FMC_SetVectorPageAddr(0x4000);
            break;
#endif

        case '1':
            FMC_SetVectorPageAddr(0x8000);
            break;
#if defined (__ARMCC_VERSION) || defined (__ICCARM__)

        case '2':
            FMC_SetVectorPageAddr(0xC000);
            break;
#endif

        case '3':
            FMC_SetVectorPageAddr(0x10000);
            break;
#if defined (__ARMCC_VERSION) || defined (__ICCARM__)

        case '4':
            FMC_SetVectorPageAddr(0x100000);
            break;
#endif

        default:
            FMC_SetVectorPageAddr(0x0);
            break;
    }

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Disable ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nDone\n");

    while (SYS->PDID) __WFI();

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
