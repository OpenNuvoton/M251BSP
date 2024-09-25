/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive UART data in UART IrDA mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Define functions prototype                                      */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void IrDA_FunctionTxTest(void);
void IrDA_FunctionRxTest(void);
void IrDA_FunctionTest(void);
void ClearRxFIFO(UART_T *uart);
int32_t main(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


/*---------------------------------------------------------------------------------------------------------*/
/*                                    Init System Clock                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

#if (CLK_SOURCE == CLK_HIRC )
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));


#else
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Select IP clock source */
    /* Select UART0 clock source is HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

#endif

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for UART1 TXD and RXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_UART1_RXD | SYS_GPA_MFPL_PA3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                               Init UART0                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{

    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                               Init UART1                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{

    UART_Open(UART1, 57600);

}

/**
 *    @brief        Clear the UART Rx FIFO buffer function
 *
 *    @param[in]    uart    The pointer of the specified UART module.
 *
 *    @return       None
 *
 *    @details      The function is used to clear UART Rx FIFO buffer data.
 */
void ClearRxFIFO(UART_T *uart)
{
    uart->FIFO |= UART_FIFO_RXRST_Msk;

    while (uart->FIFO & UART_FIFO_RXRST_Msk) {};
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif


    /* Init UART1 */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+------------------------+\n");
    printf("|   IrDA function test   |\n");
    printf("+------------------------+\n");

    IrDA_FunctionTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTest()
{
    uint8_t u8Item;

    printf("+-------------------------------------------------------------+\n");
    printf("|                     Pin Configure                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  ______                                      ______         |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |Master|---TXD1(PA3)  <====>  RXD1(PA2)  ---|Slave |        |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |______|                                    |______|        |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");

    printf("\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     IrDA Function Test                                      |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and      |\n");
    printf("|    the other is slave.  Master will send data based on      |\n");
    printf("|    terminal input and Slave will printf received data on    |\n");
    printf("|    terminal screen.                                         |\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n\n");
    u8Item = getchar();

    if (u8Item == '0')
        IrDA_FunctionTxTest();
    else
        IrDA_FunctionRxTest();

    printf("\nIrDA Sample Code End.\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                      IrDA Function Transmit Test                                        */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTxTest(void)
{
    uint8_t u8OutChar;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Tx Mode Test                            |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Input char by UART terminal.                         |\n");
    printf("| 2). UART will send a char according to step 1.           |\n");
    printf("| 3). Return step 1. (Press '0' to exit)                    |\n");
    printf("+-----------------------------------------------------------+\n\n");

    printf("\nIRDA Sample Code Start. \n");

    /* Select IrDA Tx mode and set baud rate */
    UART_SelectIrDAMode(UART1, 57600, TRUE); // TRUE is TX mode

    /* Wait Terminal input to send data to UART TX pin */
    do
    {
        u8OutChar = getchar();
        printf(" Input: %c , Send %c out\n", u8OutChar, u8OutChar);
        UART_WRITE(UART1, u8OutChar);
    } while (u8OutChar != '0');

}

/*---------------------------------------------------------------------------------------------------------*/
/*                               IrDA Function Receive Test                                            */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionRxTest(void)
{
    uint8_t u8InChar = 0xFF;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Rx Mode Test                            |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Polling RDA_Flag to check data input though UART     |\n");
    printf("| 2). If received data is '0', the program will exit.       |\n");
    printf("|     Otherwise, print received data on terminal            |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* Select IrDA Rx mode and set baud rate */
    UART_SelectIrDAMode(UART1, 57600, FALSE); // FALSE is RX mode

    printf("Waiting...\n");
    /* Clear Rx FIFO bufeer data */
    ClearRxFIFO(UART1);

    /* Use polling method to wait master data */
    do
    {
        if (UART_IS_RX_READY(UART1))
        {
            u8InChar = UART_READ(UART1);
            printf("   Input: %c \n", u8InChar);
        }
    } while (u8InChar != '0');

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
