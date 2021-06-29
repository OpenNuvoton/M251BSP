/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Transmit and receive data from PC terminal through RS232 interface.
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

#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8RecData[RXBUFSIZE] = {0};

volatile uint32_t g_u32ComRbytes = 0;
volatile uint32_t g_u32ComRhead = 0;
volatile uint32_t g_u32ComRtail = 0;
volatile int8_t g_i8Wait = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_TEST_HANDLE(void);
void USCI_UART_FunctionTest(void);

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
/* USCI UART Test Sample                                                                                   */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

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

    /* USCI UART sample function */
    USCI_UART_FunctionTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI UART interrupt event                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    USCI_UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* USCI UART Callback function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_TEST_HANDLE()
{
    uint32_t u32IntSts = UUART0->PROTSTS;

    if (u32IntSts & UUART_PROTSTS_RXENDIF_Msk)
    {

        /* Clear RX end interrupt flag */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);

        printf("\nInput:");

        /* Get all the input characters */
        while (!UUART_IS_RX_EMPTY(UUART0))
        {

            /* Get the character from USCI UART Buffer */
            uint8_t u8InChar = UUART_READ(UUART0);

            printf("%c ", u8InChar);

            if (u8InChar == '0')
            {
                g_i8Wait = FALSE;
            }

            /* Check if buffer full */
            if (g_u32ComRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_au8RecData[g_u32ComRtail] = u8InChar;
                g_u32ComRtail = (g_u32ComRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32ComRtail + 1);
                g_u32ComRbytes++;
            }
        }

        printf("\nTransmission Test:");

    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI UART Function Test                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_FunctionTest()
{

    printf("+-----------------------------------------------------------+\n");
    printf("|  USCI UART Function Test                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect USCI-UART0 and PC.
        USCI-UART0 is set to debug port. USCI-UART0 is enable RX end interrupt.
        When inputting char to terminal screen, RX end interrupt will happen and
        USCI-UART0 will print the received char on screen.
    */

    /* Enable USCI UART interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_EnableIRQ(USCI0_IRQn);

    while (g_i8Wait)
    {
        uint32_t u32Temp = g_u32ComRtail;

        if (g_u32ComRhead != u32Temp)
        {
            uint8_t u8InChar = g_au8RecData[g_u32ComRhead];

            while (UUART_IS_TX_FULL(UUART0)); /* Wait Tx is not full to transmit data */

            UUART_WRITE(UUART0, u8InChar);
            g_u32ComRhead = (g_u32ComRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32ComRhead + 1);
            g_u32ComRbytes--;
        }
    }

    /* Disable USCI UART receive and transmit end interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk | UUART_INTEN_TXENDIEN_Msk);
    NVIC_DisableIRQ(USCI0_IRQn);
    g_i8Wait = TRUE;
    printf("\nUSCI UART Sample Demo End.\n");
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
