/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Implement USCI_SPI0 Master loop back transfer.
 *           This sample code needs to connect USCI_SPI0_MISO pin and USCI_SPI0_MOSI pin together.
 *           It will compare the received data with transmitted data.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_48MHZ
#define TEST_COUNT  64

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*On M251 C version no USCI_CTL0(SS) pins, you can use this definition to open alternatives*/
//#define NOT_SUPPORT_SS_PIN

/*Use GPIO PA2 instead of USCI_CTL0(SS) pin*/
#if defined (NOT_SUPPORT_SS_PIN)
    #define  USPI_SS_PIN   PA2
#endif

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

/* Function prototype declaration */
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
#else
    /*M251 C is not supoort USCI_CTL0(SS)*/
    /*Replace the USCI_CTL0(SS) pin with GPIO PA2 */
    SYS->GPA_MFPL = SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA2MFP_Msk;
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
#endif

}

void USCI_SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI_SPI0                                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 as a master, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set USCI_SPI0 clock rate = 2MHz */
    USPI_Open(USPI0, USPI_MASTER, USPI_MODE_0, 16, 2000000);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32DataCount, u32TestCount, u32Err;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init USCI_SPI0 */
    USCI_SPI_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|                   USCI_SPI Driver Sample Code                    |\n");
    printf("+------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates USCI_SPI0 self loop back data transfer.\n");
    printf(" USCI_SPI0 configuration:\n");
    printf("     Master mode; data width 16 bits.\n");
    printf(" I/O connection:\n");
    printf("     PB.13 USCI_SPI0_MOSI <--> PB.14 USCI_SPI0_MISO \n");

    printf("\nUSCI_SPI0 Loopback test ");

    /* set the source data and clear the destination buffer */
    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        g_au32SourceData[u32DataCount] = u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }

    u32Err = 0;

    for (u32TestCount = 0; u32TestCount < 0x1000; u32TestCount++)
    {
        /* set the source data and clear the destination buffer */
        for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            g_au32SourceData[u32DataCount]++;
            g_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount = 0;

        if ((u32TestCount & 0x1FF) == 0)
        {
            putchar('.');
        }

        while (1)
        {
            /* Write to TX register */
            USPI_WRITE_TX(USPI0, g_au32SourceData[u32DataCount]);

            /* Check SPI0 busy status */
            while (USPI_IS_BUSY(USPI0));

            /* Read received data */
            g_au32DestinationData[u32DataCount] = USPI_READ_RX(USPI0);
            u32DataCount++;

            if (u32DataCount == TEST_COUNT)
                break;
        }

        /*  Check the received data */
        for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            if (g_au32DestinationData[u32DataCount] != g_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if (u32Err)
            break;
    }

    if (u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    /* Close USCI_SPI0 */
    USPI_Close(USPI0);

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
