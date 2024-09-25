/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show hard fault information when hard fault happened.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

/*
 The ARM M23 Generic User Guide lists the following sources for a hard fault:

 All faults result in the HardFault exception being taken or cause lockup if
 they occur in the NMI or HardFault handler. The faults are:
  - execution of an SVC instruction at a priority equal or higher than SVCall
  - execution of a BKPT instruction without a debugger attached
  - a system-generated bus error on a load or store
  - execution of an instruction from an XN memory address
  - execution of an instruction from a location for which the system generates a bus fault
  - a system-generated bus error on a vector fetch execution of an Undefined instruction
  - execution of an instruction when not in Thumb-State as a result of the T-bit being previously cleared to 0
  - an attempted load or store to an unaligned address.

 In this sample code, we will show you some information to debug with hardfault exception.
 if HardFault_Handler is not be implemented, hardfault exception will jump to ProcessHardFault.

  - Default Process Hard Fault Handler

    The default Process Hard Fault handler is implement in retarget.c and called ProcessHardFault
    By default, ProcessHardFault will print out the information of r0, r1, r2, r3, r12, lr, pc and psr.

  - Overwrite the default Process Hard Fault Handler

    The default ProcessHardFault is a weak function.
    User can over write it by implementing their own ProcessHardFault.

 NOTE:
    Because hard fault exception will cause data stacking.
    User must make sure SP is pointing to an valid memory location.
    Otherwise, it may cause system lockup and reset when hard fault.
*/

#define USE_MY_HARDFAULT    1   /* Select to use default process hard fault handler or not. 0: Default  1: User define */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

#if defined ( __GNUC__ ) && !defined (__ARMCC_VERSION)
void HardFault_Handler(void)
{
    __ASM(
        "MOV     R0, LR  \n"
        "MRS     R1, MSP \n"
        "MRS     R2, PSP \n"
        "LDR     R3, =ProcessHardFault \n"
        "BLX     R3 \n"
        "BX      R0 \n"
    );
}

#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}


void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    UART_Open(UART0, 115200);
}


#if USE_MY_HARDFAULT
/**
  * @brief      User defined Process HardFault
  * @param      stack   A pointer to current stack
  * @return     None
  * @details    This function is an example to show how to implement user's process hard fault handler
  *
  */
uint32_t ProcessHardFault(uint32_t u32_lr, uint32_t u32msp, uint32_t u32psp)
{
    uint32_t u32exception_num;
    uint32_t u32r0, u32r1, u32r2, u32r3, u32r12, u32lr, u32pc, u32psr, *pu32sp;

    if (u32_lr & 4)
        pu32sp = (uint32_t *)u32psp;
    else
        pu32sp = (uint32_t *)u32msp;

    /* Get information from stack */
    u32r0  = pu32sp[0];
    u32r1  = pu32sp[1];
    u32r2  = pu32sp[2];
    u32r3  = pu32sp[3];
    u32r12 = pu32sp[4];
    u32lr  = pu32sp[5];
    u32pc  = pu32sp[6];
    u32psr = pu32sp[7];


    /* Check T bit of psr */
    if ((u32psr & (1 << 24)) == 0)
    {
        printf("PSR T bit is 0.\nHard fault caused by changing to ARM mode!\n");

        while (1);
    }

    /* Check hard fault caused by ISR */
    u32exception_num = u32psr & xPSR_ISR_Msk;

    if (u32exception_num > 0)
    {
        /*
        Exception number
            0 = Thread mode
            1 = Reserved
            2 = NMI
            3 = HardFault
            4-10 = Reserved11 = SVCall
            12, 13 = Reserved
            14 = PendSV
            15 = SysTick, if implemented[a]
            16 = IRQ0.
                .
                .
            n+15 = IRQ(n-1)[b]
            (n+16) to 63 = Reserved.
        The number of interrupts, n, is 32
        */

        printf("Hard fault is caused in IRQ #%u\n", u32exception_num - 16);

        while (1);
    }

    printf("Hard fault location is at 0x%08x\n", u32pc);
    /*
        If the hard fault location is a memory access instruction, You may debug the load/store issues.

        Memory access faults can be caused by:
            Invalid address - read/write wrong address
            Data alignment issue - Violate alignment rule of Cortex-M processor
            Memory access permission - MPU violations or unprivileged access (Cortex-M23)
            Bus components or peripheral returned an error response.
    */

    printf("r0  = 0x%x\n", u32r0);
    printf("r1  = 0x%x\n", u32r1);
    printf("r2  = 0x%x\n", u32r2);
    printf("r3  = 0x%x\n", u32r3);
    printf("r12 = 0x%x\n", u32r12);
    printf("lr  = 0x%x\n", u32lr);
    printf("pc  = 0x%x\n", u32pc);
    printf("psr = 0x%x\n", u32psr);

    while (1);
}

#endif


void TMR1_IRQHandler(void)
{
    printf("This is exception n = %d\n", TMR1_IRQn);
    M32(0) = 0;
    TIMER1->INTSTS = TIMER_INTSTS_TIF_Msk;
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    void (*func)(void) = (void (*)(void))0x1000;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();

    while (1)
    {
        printf("\n\n");
        printf("+----------------------------------------------------+\n");
        printf("|        Hard Fault Handler Sample Code              |\n");
        printf("+----------------------------------------------------+\n");
        printf("| [0] Test Load/Store Hard Fault                     |\n");
        printf("| [1] Test Thumb/ARM mode Hard Fault                 |\n");
        printf("| [2] Test Hard Fault in ISR                         |\n");
        printf("+----------------------------------------------------+\n");
        char i8ch;
        i8ch = getchar();

        switch (i8ch)
        {
            case '0':
                /* Write APROM will cause hard fault exception. (Memory access hard fault) */
                M32(0) = 0;
                break;

            case '1':
                /* Call function with bit0 = 0 will cause hard fault. (Change to ARM mode hard fault) */
                func();
                break;

            case '2':
                /* Generate Timer Interrupt to test hard fault in ISR */
                NVIC_EnableIRQ(TMR1_IRQn);
                TIMER1->CMP = 3;
                TIMER1->CTL = TIMER_CTL_INTEN_Msk | TIMER_CTL_CNTEN_Msk | TIMER_CTL_ACTSTS_Msk | TIMER_ONESHOT_MODE;
                break;

            default:
                break;
        }

    }
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
