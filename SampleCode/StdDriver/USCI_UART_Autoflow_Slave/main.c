/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Transmit and receive data with auto flow control.
 *           This sample code needs to work with USCI_UART_Autoflow_Master.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_48MHZ

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

#define RXBUFSIZE  1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8RecData[RXBUFSIZE]  = {0};
volatile int32_t g_i32Pointer = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionRxTest(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
#if !(defined(DEBUG_ENABLE_SEMIHOST))
    Uart0DefaultMPF();
#endif
    /* Set UUART0 multi-function pins */
    SYS->GPB_MFPH = SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH = SYS->GPB_MFPH | (SYS_GPB_MFPH_PB13MFP_USCI0_DAT0 | SYS_GPB_MFPH_PB14MFP_USCI0_DAT1 | SYS_GPB_MFPH_PB15MFP_USCI0_CTL1);
    SYS->GPC_MFPH = SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC14MFP_Msk;
    SYS->GPC_MFPH = SYS->GPC_MFPH | SYS_GPC_MFPH_PC14MFP_USCI0_CTL0;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
#if !(defined(DEBUG_ENABLE_SEMIHOST))
    UART0_Init();
#endif
    /* Init USCI0 for test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART auto flow sample slave function */
    USCI_AutoFlow_FunctionRxTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{

    volatile uint32_t u32ProtSts = UUART_GET_PROT_STATUS(UUART0);
    volatile uint32_t u32BufSts = UUART_GET_BUF_STATUS(UUART0);

    if (u32ProtSts & UUART_PROTSTS_RXENDIF_Msk)     /* Receive end interrupt */
    {
        uint32_t u32Temp;

        /* Handle received data */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);
        u32Temp = UUART_READ(UUART0);
        g_au8RecData[g_i32Pointer] = u32Temp;
        g_i32Pointer++;
    }
    else if (u32BufSts & UUART_BUFSTS_RXOVIF_Msk)      /* Receive buffer over-run error interrupt */
    {
        UUART_CLR_BUF_INT_FLAG(UUART0, UUART_BUFSTS_RXOVIF_Msk);
        printf("\nRx buffer is over-run.");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Slave)                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionRxTest()
{
    uint32_t u32Idx;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|                                          |Slave| |\n");
    printf("| |    TX|--USCI0_DAT1   <======> USCI0_DAT0      --|RX   | |\n");
    printf("| |  nCTS|--USCI0_CTL0   <======> USCI0_CTL1      --|nRTS | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test (Slave)                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave. Slave will check if received data is correct |\n");
    printf("|    after getting 1k bytes data.                           |\n");
    printf("|    Press any key to start...                              |\n");
    printf("+-----------------------------------------------------------+\n");
    getchar();

    /* Enable CTS and RTS auto flow control */
    UUART_EnableFlowCtrl(UUART0);
    /* Enable USCI receive end and receive buffer over-run error Interrupt */
    UUART_EnableInt(UUART0, UUART_RXEND_INT_MASK | UUART_BUF_RXOV_INT_MASK);
    NVIC_EnableIRQ(USCI0_IRQn);

    printf("\n Starting to receive data...\n");

    /* Wait for receive 1k bytes data */
    while (g_i32Pointer < RXBUFSIZE);

    /* Compare Data */
    for (u32Idx = 0; u32Idx < RXBUFSIZE; u32Idx++)
    {
        if (g_au8RecData[u32Idx] != (u32Idx & 0xFF))
        {
            printf("\n Compare Data Failed  \n");
            printf("u32Idx = %u\n", u32Idx);
            printf("g_au8RecData[%u] = %u\n", u32Idx, g_au8RecData[u32Idx]);

            while (1);
        }
    }

    printf("\n Receive OK & Check OK\n");

    /* Disable USCI interrupt */
    UUART_DisableInt(UUART0, UUART_RXEND_INT_MASK | UUART_BUF_RXOV_INT_MASK);
    NVIC_DisableIRQ(USCI0_IRQn);

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
