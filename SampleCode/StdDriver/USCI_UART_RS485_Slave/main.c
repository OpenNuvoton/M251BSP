/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Transmit and receive data in RS485 mode.
 *           This sample code needs to work with USCI_UART_RS485_Master.
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

#define RXBUFSIZE   40
#define ADDRSIZE    4

volatile uint8_t g_au8Address[ADDRSIZE];
volatile uint8_t g_au8RecData[RXBUFSIZE];
volatile uint32_t g_u32AddrIndex = 0;
volatile uint32_t g_u32DataIndex = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE(void);
void RS485_9bitModeSlave(void);
void RS485_FunctionTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    RS485_HANDLE();
}


/*---------------------------------------------------------------------------------------------------------*/
/* RS485 Callback function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE()
{
    volatile uint32_t u32ProtSts = UUART_GET_PROT_STATUS(UUART0);
    volatile uint32_t u32BufSts = UUART_GET_BUF_STATUS(UUART0);

    if (u32ProtSts & UUART_PROTSTS_RXENDIF_Msk)     /* Receive end interrupt */
    {
        uint32_t u32Data;

        /* Handle received data */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);
        u32Data = UUART_READ(UUART0);

        if (u32Data & 0x100)
        {
            g_au8Address[g_u32AddrIndex] = u32Data;
            g_u32AddrIndex++;
        }
        else
        {
            g_au8RecData[g_u32DataIndex] = u32Data;
            g_u32DataIndex++;
        }
    }
    else if (u32BufSts & UUART_BUFSTS_RXOVIF_Msk)      /* Receive buffer over-run error interrupt */
    {
        UUART_CLR_BUF_INT_FLAG(UUART0, UUART_BUFSTS_RXOVIF_Msk);
        printf("\nBuffer Error...\n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Receive Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeSlave()
{
    uint32_t u32DataIdx, u32AddrIdx;

    /* Set UART line configuration and control signal output inverse */
    UUART_SetLine_Config(UUART0, 0, UUART_WORD_LEN_9, UUART_PARITY_NONE, UUART_STOP_BIT_1);
    UUART0->LINECTL |= UUART_LINECTL_CTLOINV_Msk;

    /* Enable RTS auto direction function */
    UUART0->PROTCTL |= UUART_PROTCTL_RTSAUDIREN_Msk;

    printf("+-----------------------------------------------------------+\n");
    printf("|    RS485 Mode                                             |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("| Receive address and data byte.                            |\n");
    printf("+-----------------------------------------------------------+\n");

    /* Enable USCI receive end and receive buffer over-run error Interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    UUART_ENABLE_BUF_INT(UUART0, UUART_BUFCTL_RXOVIEN_Msk);
    NVIC_EnableIRQ(USCI0_IRQn);

    printf("Starting to receive RS485 Address and %d bytes data...\n", RXBUFSIZE);

    while (g_u32DataIndex < RXBUFSIZE);

    for (u32AddrIdx = 0; u32AddrIdx < ADDRSIZE; u32AddrIdx++)
    {
        printf("\n Receive Address: 0x%x ", g_au8Address[u32AddrIdx]);
        printf("\n Receive Data:");

        for (u32DataIdx = u32AddrIdx * 10; u32DataIdx < (u32AddrIdx + 1) * 10; u32DataIdx++)
        {
            printf(" %u,", g_au8RecData[u32DataIdx]);
        }
    }

    /* Disable USCI interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    UUART_DISABLE_BUF_INT(UUART0, UUART_BUFCTL_RXOVIEN_Msk);
    NVIC_DisableIRQ(USCI0_IRQn);

    printf("\nEnd test\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_FunctionTest()
{
    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      RS485 Function Test IO Setting                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--USCI0_DAT1   <======> USCI0_DAT0      --|Slave| |\n");
    printf("| |      |--USCI0_CTL1   <======> USCI0_CTL1      --|     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");


    /*
        The sample code is used to test RS485 9-bit mode and needs
        two Module test board to complete the test.

        Master:
            1.Set RTS auto direction enabled and HW will control RTS pin. CTLOINV is set to '1'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'.
            4.Data bytes : the parity bit should be '0'.
            5.RTS pin is low in idle state. When master is sending, RTS pin will be pull high.

        Slave:
            1.Set RTS auto direction enabled and HW will control RTS pin. CTLOINV is set to '1'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".
    */

    RS485_9bitModeSlave();

}

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

    /* UART RS485 sample slave function */
    RS485_FunctionTest();

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
