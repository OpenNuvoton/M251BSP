/****************************************************************************//**
 * @file     LDROM_iap.c
 * @version  V1.00
 * @brief    FMC LDROM IAP sample program run on LDROM.
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

typedef void (FN_FUNC_PTR)(void);


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART0->LINE = 0x3;
    UART0->BAUD = 0x3000019E;
}


/*
 *  Set stack base address to SP register.
 */
#ifdef __ARMCC_VERSION                 /* for Keil compiler */
void __set_SP(uint32_t _sp)
{
    __ASM(
        "MSR MSP, r0 \n"
        "BX lr			 \n"
    );
}
#endif


/**
 * @brief       Routine to send a char
 * @param[in]   ch Character to send to debug port.
 * @returns     Send value from UART debug port
 * @details     Send a target char to UART debug port .
 */
static void SendChar_ToUART(int i16Ch)
{
    while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    UART0->DAT = i16Ch;

    if (i16Ch == '\n')
    {
        while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

        UART0->DAT = '\r';
    }
}

/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while (1)
    {
        if ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            return (UART0->DAT);
        }
    }
}

static void PutString(char *pi8Str)
{
    while (*pi8Str != '\0')
    {
        SendChar_ToUART(*pi8Str++);
    }
}

#ifdef __GNUC__                        /* for GNU C compiler */
/**
 * @brief       Hard fault handler
 * @param[in]   stack pointer points to the dumped registers in SRAM
 * @return      None
 * @details     Replace while(1) at the end of this function with chip reset if WDT is not enabled for end product
 */
void Hard_Fault_Handler(uint32_t stack[])
{
    PutString("In Hard Fault Handler\n");

    while (1);
}
#endif

int main()
{
#if defined(__GNUC__) && !defined (__ARMCC_VERSION)    /* for GNU C compiler */
    uint32_t    u32Data;
#endif
    FN_FUNC_PTR    *pfnfunc;           /* function pointer */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    PutString("\n\n");
    PutString("+-------------------------------------+\n");
    PutString("|        FMC IAP Sample Code          |\n");
    PutString("|          [LDROM code]               |\n");
    PutString("+-------------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers to operate FMC ISP function*/

    FMC_Open();                        /* Enable FMC ISP function */

    PutString("\n\nPress any key to branch to APROM...\n");
    GetChar();                         /* block on waiting for any one character input from UART0 */

    PutString("\n\nChange VECMAP and branch to APROM...\n");

    while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk));       /* wait until UART3 TX FIFO is empty */

    /*  NOTE!
     *     Before change VECMAP, user MUST disable all interrupts.
     */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);        /* Vector remap APROM page 0 to address 0. */
    SYS_LockReg();                                /* Lock protected registers */

    /*
     *  The reset handler address of an executable image is located at offset 0x4.
     *  Thus, this sample get reset handler address of APROM code from FMC_APROM_BASE + 0x4.
     */
    pfnfunc = (FN_FUNC_PTR *) * (uint32_t *)(FMC_APROM_BASE + 4);

    /*
     *  The stack base address of an executable image is located at offset 0x0.
     *  Thus, this sample get stack base address of APROM code from FMC_APROM_BASE + 0x0.
     */
#if defined( __GNUC__ ) && !defined (__ARMCC_VERSION)                 /* for GNU C compiler */
    u32Data = *(uint32_t *)FMC_LDROM_BASE;
    asm("msr msp, %0" : : "r"(u32Data));
#else
    __set_SP((*(volatile uint32_t *)(FMC_APROM_BASE)));
#endif

    /*
     *  Branch to the LDROM code's reset handler in way of function call.
     */
    pfnfunc();

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
