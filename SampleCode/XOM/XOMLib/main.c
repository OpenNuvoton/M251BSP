/**************************************************************************//**
 * @file     main.c
 * @version  V3.01
 * @brief    Show how to config/test XOM region.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "xomapi.h"

#define XOM_START       0x00003000
#define XOM_SIZE        0x00000400

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
    char cUserSel;
    int32_t i, j, i32Result;
    int32_t ai32NumArray[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    uint32_t u32XOMBaseAddr;

#if defined (__ICCARM__)
    extern uint32_t __region_XOM_start__;
    u32XOMBaseAddr = (uint32_t)&__region_XOM_start__;
#elif defined(__ARMCC_VERSION)
    extern uint32_t Image$$XOM0_ROM$$Base;

    u32XOMBaseAddr = (uint32_t)&Image$$XOM0_ROM$$Base;
#else
    extern uint32_t __region_XOM_start__;

    u32XOMBaseAddr = (uint32_t)&__region_XOM_start__;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code demonstrates how to build an XOM (eXecute-Only Memory) library.

        The location of the XOM region is defined by the linker file:
            - xom_scatter.scf (Keil)
            - xom.icf (IAR)
            - XOMLib_Code.ld (GCC/VSCode)
        The exported XOM library header file is located at: .\lib\xomlib.h.
        The exported XOM functions are implemented in xom.c.

        This project is solely for building code for the XOM region and testing its functions.
        To enable the XOM region, please use the "NuMicro ICP Programming Tool" or the "[0] Set XOM" option.

        ### Example Workflow:
        1. Build XOMLib_Code and test the XOM functions.
        2. There are two methods to set and enable the XOM region:
            * Execute "[0] Set XOM" in XOMLib_Code.
            * Open the "NuMicro ICP Programming Tool" and check XOM_START and XOM_SIZE to
              set and enable the XOM region.
        3. Test the XOM functions with the XOM enabled again.
        4. Review xomlib.c and .\lib\xomlib.h to ensure all XOM function pointers are
            included correctly (Check the function addresses in the "[1] Test XOM" output).
            Manually copy the XOMLib address definitions to xomlib.c.
        5. Build the XOMLib project to generate the XOM library files:
            - xomlib.lib (Keil)
            - xomlib.a (IAR)
            - libXOMLib.a (GCC/VSCode)
            These include the function pointers for the XOM.
            The XOM library files (xomlib.lib, xomlib.a, and libXOMLib.a) and header (xomlib.h)
            are located in the lib directory.
        6. Distribute xomlib.lib (Keil) / xomlib.a (IAR) / libXOMLib.a (GCC/VSCode) and xomlib.h to users
            who will call these functions in the XOM region.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|      FMC XOM Library Build Example     |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active*/
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    FMC_ENABLE_CFG_UPDATE();

    /* Read XOM Status */
    printf("XOM Status = 0x%X\n", FMC->XOMSTS);
    printf("[0] Set XOM [1] Test XOM\n");

    cUserSel = (char)getchar();

    if (cUserSel == '0')
    {
        printf("Config XOM0\n");
        printf("  Base address: 0x%08X, page count: %d\n", (uint32_t)XOM_START, (uint32_t)(XOM_SIZE / FMC_FLASH_PAGE_SIZE));
        UART_WAIT_TX_EMPTY(UART0);

        FMC_Config_XOM(0, XOM_START, (XOM_SIZE / FMC_FLASH_PAGE_SIZE));
        SYS_ResetChip();

        while (1) ;
    }

    printf("\nCopy below XOM libraray address definitions to xomlib.c to build XOMLib.\n");
    printf("/*------------------------------*/\n");
    printf("/*  XOMLib address definitions  */\n");
    printf("/*------------------------------*/\n");
    printf("#define XOM_ADD_ADDR    0x%08X\n", (uint32_t)XOM_Add);
    printf("#define XOM_SUB_ADDR    0x%08X\n", (uint32_t)XOM_Sub);
    printf("#define XOM_MUL_ADDR    0x%08X\n", (uint32_t)XOM_Mul);
    printf("#define XOM_DIV_ADDR    0x%08X\n", (uint32_t)XOM_Div);
    printf("#define XOM_SUM_ADDR    0x%08X\n", (uint32_t)XOM_Sum);

    /* Run XOM function */
    printf("\nCheck XOM execution\n");
    printf("  [XOM_Add]  100 + 200 = %d\n", XOM_Add(100, 200));
    printf("  [XOM_Sub]  500 - 100 = %d\n", XOM_Sub(500, 100));
    printf("  [XOM_Mul]  200 * 100 = %d\n", XOM_Mul(200, 100));
    printf("  [XOM_Div] 1000 / 250 = %d\n", XOM_Div(1000, 250));
    printf("  [XOM_Sum] Sum of ai32NumArray = %d\n", XOM_Sum(ai32NumArray, sizeof(ai32NumArray) / sizeof(ai32NumArray[0])));

    for (i = 0; i < 1000; i++)
    {
        i32Result = XOM_Add(500, 700 + i);

        if (i32Result != (1200 + i))
        {
            printf("XOM ADD fail. It should be %d but %d\n", (1200 + i), i32Result);
            goto lexit;
        }
    }

    printf("\n");

    if (FMC->XOMSTS == 0x1)
        printf("Check CPU access XOM region all 0xFFFFFFFF.\n");
    else
        printf("Check CPU access XOM region not 0xFFFFFFFF.\n");

    for (i = 0; i < 0x40; i += 16)
    {
        printf("  [0x%08X] ", u32XOMBaseAddr + i);

        for (j = 0; j < 16; j += 4)
            printf("0x%08X ", M32(u32XOMBaseAddr + i + j));

        printf("\n");
    }

lexit:
    printf("Done.\n");

    while (1);
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/

