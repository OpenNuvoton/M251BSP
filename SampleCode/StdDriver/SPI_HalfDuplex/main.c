/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Demonstrate SPI half-duplex mode.
 *           QSPI0 will be configured as Master mode and SPI0 will be configured as Slave mode.
 *           Both QSPI0 and SPI0 will be configured as half-duplex mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT  4

uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select PCLK as the clock source of QSPI0 and SPI0 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk);
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);


    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable QSPI0 peripheral clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set UART0 Default MPF */
    Uart0DefaultMPF() ;

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL = SYS_GPA_MFPL_PA0MFP_QSPI0_MOSI0 | SYS_GPA_MFPL_PA1MFP_QSPI0_MISO0 | SYS_GPA_MFPL_PA2MFP_QSPI0_CLK | SYS_GPA_MFPL_PA3MFP_QSPI0_SS ;

    /* Enable QSPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Setup SPI0 multi-function pins */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_SPI0_MOSI | SYS_GPD_MFPL_PD1MFP_SPI0_MISO | SYS_GPD_MFPL_PD2MFP_SPI0_CLK | SYS_GPD_MFPL_PD3MFP_SPI0_SS ;

    /* Enable SPI0 clock pin (PD2) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    QSPI_Open(QSPI0, QSPI_MASTER, QSPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    QSPI_EnableAutoSS(QSPI0, QSPI_SS, QSPI_SS_ACTIVE_LOW);

    /* Configure SPI0 */
    /* Configure SPI0 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI0 as a low level active device. SPI peripheral clock rate = f_PCLK0 */
    SPI_Open(SPI0, SPI_SLAVE, SPI_MODE_0, 32, 0);
}

int main(void)
{
    uint32_t u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|             SPI Half-duplex Mode Sample Code                         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure QSPI0 as a master and SPI0 as a slave.\n");
    printf("Set both QSPI0 and SPI0 to half-duplex.\n");
    printf("Bit length of a transaction: 32\n");
    printf("Please connect below I/O connections for QSPI0 and SPI0:\n");
    printf("    QSPI0_SS(PA3)   <->   SPI0_SS(PD3)\n");
    printf("    QSPI0_CLK(PA2)  <->   SPI0_CLK(PD2)\n");
    printf("    QSPI0_MOSI(PA0) <->   SPI0_MOSI(PD0)\n\n");
    printf("After the transfer is done, the received data will be printed out.\n");


    /* Set slave SPI0 to half-duplex mode */
    SPI0->CTL |= SPI_CTL_HALFDPX_Msk;

    /* Enable half-duplex will produce TXFBCLR (SPIx_FIFOCTL[9]) and RXFBCLR (SPIx_FIFOCTL[8])*/
    while (SPI0->STATUS & SPI_STATUS_TXRXRST_Msk) {}

    /* Set slave SPI0 data direction to output */
    SPI0->CTL |= SPI_CTL_DATDIR_Msk;

    /* Slave SPI0 prepare data to TX FIFO */
    SPI_WRITE_TX(SPI0, 0x55AA0000);
    SPI_WRITE_TX(SPI0, 0x55AA0001);
    SPI_WRITE_TX(SPI0, 0x55AA0002);
    SPI_WRITE_TX(SPI0, 0x55AA0003);

    /* Set master QSPI0 to half-duplex mode */
    QSPI0->CTL |= QSPI_CTL_HALFDPX_Msk;

    /* Enable half-duplex will produce TXFBCLR (QSPIx_FIFOCTL[9]) and RXFBCLR (QSPIx_FIFOCTL[8])*/
    while (QSPI0->STATUS & QSPI_STATUS_TXRXRST_Msk) {}

    /* Set master QSPI0 data direction to input */
    QSPI0->CTL &= ~QSPI_CTL_DATDIR_Msk;

    /* Master QSPI0 receive four data from slave SPI0 */
    for (g_u32RxDataCount = 0; g_u32RxDataCount < 4; g_u32RxDataCount++)
    {
        /* Master write TX for generating clock */
        QSPI_WRITE_TX(QSPI0, 0);

        /* Wait for Rx FIFO not empty */
        while (QSPI_GET_RX_FIFO_EMPTY_FLAG(QSPI0)) {}

        /* Read data from RX register */
        g_au32DestinationData[g_u32RxDataCount] = QSPI_READ_RX(QSPI0);
    }

    /* Print the received data */
    printf("Received data:\n");

    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%u:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Reset QSPI0 */
    QSPI_Close(QSPI0);
    /* Reset SPI0 */
    SPI_Close(SPI0);

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
