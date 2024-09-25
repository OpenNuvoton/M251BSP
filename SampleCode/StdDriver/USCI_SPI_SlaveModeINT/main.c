/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief   Configure USCI_SPI0 as Slave mode, demonstrate how to communicate with an off-chip SPI Master device,
 *          and transmit and receive data in the interrupt handler.
 *          This sample code needs to work with USCI_SPI_MasterMode sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT 16

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*On M251 C version no USCI_CTL0(SS) pins, you can use this definition to open alternatives*/
//#define NOT_SUPPORT_SS_PIN

/*Use GPIO PA2 instead of USCI_CTL0(SS) pin*/
#if defined (NOT_SUPPORT_SS_PIN)
    #define  USPI_SS_PIN   PA2
#endif

/*----------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                    */
/*----------------------------------------------------------------------------------------------------------*/
uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/*----------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                               */
/*----------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void USCI_SPI_Init(void);

void USCI0_IRQHandler(void)
{
    uint32_t u32RxData;

    /* Clear RX end interrupt flag */
    USPI_CLR_PROT_INT_FLAG(USPI0, USPI_PROTSTS_RXENDIF_Msk);

#if  (defined(NOT_SUPPORT_SS_PIN))

    if ((USPI0->PROTSTS) & USPI_PROTSTS_SLVUDR_Msk)
    {
        USPI0->BUFCTL |= USPI_BUFCTL_TXRST_Msk;

        while ((USPI0->BUFCTL) & USPI_BUFCTL_TXRST_Msk) ;
    }

#endif

    /* Check TX data count */
    if (g_u32TxDataCount < TEST_COUNT)
    {
        /* Write to TX Buffer */
        USPI_WRITE_TX(USPI0, g_au32SourceData[g_u32TxDataCount++]);
    }

    /* Check RX EMPTY flag */
    while (USPI_GET_RX_EMPTY_FLAG(USPI0) == 0)
    {
        /* Read RX Buffer */
        u32RxData = USPI_READ_RX(USPI0);
        g_au32DestinationData[g_u32RxDataCount++] = u32RxData;
    }
}


/*----------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                            */
/*----------------------------------------------------------------------------------------------------------*/

int main()
{
    uint32_t u32TxDataCount, u32RxDataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Lock protected registers */
    SYS_LockReg();

    /* Initiate UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                          */
    /*------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* Initiate USCI_SPI0 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|           USCI_SPI Slave Mode Sample Code           |\n");
    printf("+-----------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI0 as a slave.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI0:\n");
    printf("    USCI_SPI0_SS (PC.14)\n    USCI_SPI0_CLK (PB.12)\n");
    printf("    USCI_SPI0_MISO (PB.14)\n    USCI_SPI0_MOSI (PB.13)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for (u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32TxDataCount] = 0xAA00 + u32TxDataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32TxDataCount] = 0;
    }

    /* Enable TX end interrupt */
    USPI_EnableInt(USPI0, USPI_RXEND_INT_MASK);
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(USCI0_IRQn);

    /* Write to TX Buffer */
    USPI_WRITE_TX(USPI0, g_au32SourceData[g_u32TxDataCount++]);

    printf("Press any key if the master device configuration is ready.");
    getchar();
    printf("\n");

    /* Wait for transfer done */
    while (g_u32RxDataCount < TEST_COUNT);

    /* Disable TX end interrupt */
    USPI_DisableInt(USPI0, USPI_RXEND_INT_MASK);
    NVIC_DisableIRQ(USCI0_IRQn);

    /* Print the received data */
    printf("Received data:\n");

    for (u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, g_au32DestinationData[u32RxDataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Close USCI_SPI0 */
    USPI_Close(USPI0);

    while (1);
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
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    Uart0DefaultMPF();
#else
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB8MFP_Msk) | SYS_GPB_MFPH_PB8MFP_UART0_RXD;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB9MFP_Msk) | SYS_GPB_MFPH_PB9MFP_UART0_TXD;
#endif

    /* Set USPI0 multi-function pins */
    SYS->GPB_MFPH = SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk);
    SYS->GPB_MFPH = SYS->GPB_MFPH | (SYS_GPB_MFPH_PB12MFP_USCI0_CLK | SYS_GPB_MFPH_PB13MFP_USCI0_DAT0 | SYS_GPB_MFPH_PB14MFP_USCI0_DAT1);
#if  !(defined(NOT_SUPPORT_SS_PIN))
    SYS->GPC_MFPH = SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC14MFP_Msk;
    SYS->GPC_MFPH = SYS->GPC_MFPH | SYS_GPC_MFPH_PC14MFP_USCI0_CTL0;
#endif
}

void USCI_SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI_SPI0                                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 as a slave, USCI_SPI0 clock rate = f_PCLK1,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI_Open(USPI0, USPI_SLAVE, USPI_MODE_0, 16, 0);

    /* Configure USCI_SPI_SS pin as low-active. */
#if  !(defined(NOT_SUPPORT_SS_PIN))
    USPI0->CTLIN0 = (USPI0->CTLIN0 & ~USPI_CTLIN0_ININV_Msk) | USPI_CTLIN0_ININV_Msk;
#else
    USPI_ENABLE_3WIRE_MODE(USPI0) ;
#endif
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
