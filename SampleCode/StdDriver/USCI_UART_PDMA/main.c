/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    This is a USCI_UART PDMA demo and need to connect UACI_UART TX and RX.
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

#define PDMA_TEST_LENGTH   128

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_au8TxBuffer[PDMA_TEST_LENGTH] = {0};
uint8_t g_au8RxBuffer[PDMA_TEST_LENGTH] = {0};

volatile uint32_t g_u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PDMATest(void);

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA);

    if (u32Status & 0x1)   /* abort */
    {
        printf("target abort interrupt !!\n");

        if (PDMA_GET_ABORT_STS(PDMA) & 0x4)
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
    }
    else if (u32Status & 0x2)     /* done */
    {
        if ((PDMA_GET_TD_STS(PDMA) & (1 << 0)) && (PDMA_GET_TD_STS(PDMA) & (1 << 1)))
        {
            g_u32IsTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA, PDMA_GET_TD_STS(PDMA));
        }
    }
    else if (u32Status & 0x300)     /* channel 2 timeout */
    {
        printf("timeout interrupt !!\n");
        g_u32IsTestOver = 3;
        PDMA_CLR_TMOUT_FLAG(PDMA, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA, 1);
    }
    else
        printf("unknown interrupt !!\n");
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
    CLK_EnableModuleClock(PDMA_MODULE);

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

void PDMA_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(PDMA, 1 << 0); // Channel 0 for UART1 TX
    PDMA_Open(PDMA, 1 << 1); // Channel 1 for UART1 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA, 0, PDMA_USCI0_TX, 0, 0);
    PDMA_SetTransferMode(PDMA, 1, PDMA_USCI0_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA, 0, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA, 1, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA, 0, ((uint32_t)(&g_au8TxBuffer[0])), PDMA_SAR_INC, (uint32_t)(&(UUART0->TXDAT)), PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA, 1, (uint32_t)(&(UUART0->RXDAT)), PDMA_SAR_FIX, ((uint32_t)(&g_au8RxBuffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA, 0, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA, 1, PDMA_REQ_SINGLE, 0);

#ifdef ENABLE_PDMA_INTERRUPT
    PDMA_EnableInt(PDMA, 0, 0);
    PDMA_EnableInt(PDMA, 1, 0);
    NVIC_EnableIRQ(PDMA_IRQn);
    g_u32IsTestOver = 0;
#endif
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

    /* USCI UART sample function */
    USCI_UART_PDMATest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI UART Function Test                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PDMATest()
{
    uint32_t u32Idx;

    printf("+-----------------------------------------------------------+\n");
    printf("|  USCI UART PDMA Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo USCI_UART PDMA function      |\n");
    printf("|    Please connect USCI0_UART_TX and USCI0_UART_RX pin.    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");
    getchar();

    for (u32Idx = 0; u32Idx < PDMA_TEST_LENGTH; u32Idx++)
    {
        g_au8TxBuffer[u32Idx] = u32Idx;
        g_au8RxBuffer[u32Idx] = 0xff;
    }

    /* PDMA reset */
    UUART0->PDMACTL |= UUART_PDMACTL_PDMARST_Msk;

    while (UUART0->PDMACTL & UUART_PDMACTL_PDMARST_Msk);

    PDMA_Init();

    /* Enable UART DMA Tx and Rx */
    UUART_TRIGGER_RX_PDMA(UUART0);
    UUART_TRIGGER_TX_PDMA(UUART0);


#ifdef ENABLE_PDMA_INTERRUPT

    while (g_u32IsTestOver == 0);

    if (g_u32IsTestOver == 1)
        printf("test done...\n");
    else if (g_u32IsTestOver == 2)
        printf("target abort...\n");
    else if (g_u32IsTestOver == 3)
        printf("timeout...\n");

#else

    while ((!(PDMA_GET_TD_STS(PDMA) & (1 << 0))) || (!(PDMA_GET_TD_STS(PDMA) & (1 << 1))));

    PDMA_CLR_TD_FLAG(PDMA, PDMA_GET_TD_STS(PDMA));
#endif

    for (u32Idx = 0; u32Idx < PDMA_TEST_LENGTH; u32Idx++)
    {
        if (g_au8RxBuffer[u32Idx] != u32Idx)
        {
            printf("\n Receive Data Compare Error !!");

            while (1);
        }

    }

    printf("\nUART PDMA test Pass.\n");

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
