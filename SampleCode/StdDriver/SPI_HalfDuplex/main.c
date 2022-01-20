/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Demonstrate SPI half-duplex mode.
 *           USPI0 will be configured as Master mode and SPI0 will be configured as Slave mode.
 *           Both USPI0 and SPI0 will be configured as half-duplex mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define TEST_COUNT                              (4)

/*On M251 C version no USCI_CTL0(SS) pins, you can use this definition to open alternatives*/
//#define NOT_SUPPORT_SS_PIN

/*Use GPIO PA2 instead of USCI_CTL0(SS) pin*/
#if defined (NOT_SUPPORT_SS_PIN)
    #define  USPI_SS_PIN   PA2
#endif //NOT_SUPPORT_SS_PIN

//------------------------------------------------------------------------------
/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);

//------------------------------------------------------------------------------
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

    /* Select PCLK as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable USPI0 peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set UART0 Default MPF */
    Uart0DefaultMPF() ;

    /* Set USPI0 multi-function pins */
    SYS->GPA_MFPH = SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk  |
                                      SYS_GPA_MFPH_PA10MFP_Msk |
                                      SYS_GPA_MFPH_PA11MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA11MFP_USCI0_CLK  |
                      SYS_GPA_MFPH_PA10MFP_USCI0_DAT0 |
                      SYS_GPA_MFPH_PA9MFP_USCI0_DAT1);

#if !(defined(NOT_SUPPORT_SS_PIN))
    SYS->GPC_MFPH = SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC14MFP_Msk;
    SYS->GPC_MFPH = SYS->GPC_MFPH | SYS_GPC_MFPH_PC14MFP_USCI0_CTL0;
#else
    /*M251 C is not supoort USCI_CTL0(SS)*/
    /*Replace the USCI_CTL0(SS) pin with GPIO PA2 */
    SYS->GPA_MFPL = SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA2MFP_Msk;
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
#endif //NOT_SUPPORT_SS_PIN

    /* Setup SPI0 multi-function pins */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk |
                       SYS_GPD_MFPL_PD1MFP_Msk |
                       SYS_GPD_MFPL_PD2MFP_Msk |
                       SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_SPI0_MOSI |
                    SYS_GPD_MFPL_PD1MFP_SPI0_MISO |
                    SYS_GPD_MFPL_PD2MFP_SPI0_CLK  |
                    SYS_GPD_MFPL_PD3MFP_SPI0_SS;

    /* Enable SPI0 clock pin (PD2) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USPI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 as a master, USCI_SPI0 clock rate 2 MHz,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI_Open(USPI0, USPI_MASTER, USPI_MODE_0, 16, 2000000);

    /* Enable the automatic hardware slave selection function and configure USCI_SPI_SS pin as low-active. */
#if !(defined(NOT_SUPPORT_SS_PIN))
    USPI_EnableAutoSS(USPI0, 0, USPI_SS_ACTIVE_LOW);
#else
    USPI_SS_PIN = 1 ;
#endif //NOT_SUPPORT_SS_PIN

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 */
    /* Configure SPI0 as a slave, clock idle low, 16-bit transaction,
       drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI0 as a low level active device. SPI peripheral clock rate = f_PCLK0 */
    SPI_Open(SPI0, SPI_SLAVE, SPI_MODE_0, 16, 0);
}

int main(void)
{
    uint16_t au16DestinationData[TEST_COUNT] = {0};
    uint16_t u16DataCount = 0;
    uint16_t u16RxDataCount = 0;

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
    printf("Configure USPI0 as a master and SPI0 as a slave.\n");
    printf("Set both USPI0 and SPI0 to half-duplex.\n");
    printf("Bit length of a transaction: 16\n");
    printf("Please connect below I/O connections for USPI0 and SPI0:\n");
    printf("    USPI0_SS  (PC14) <-> SPI0_SS  (PD3)\n");
    printf("    USPI0_CLK (PA11) <-> SPI0_CLK (PD2)\n");
    printf("    USPI0_MOSI(PA10) <-> SPI0_MOSI(PD0)\n\n");
    printf("After the transfer is done, the received data will be printed out.\n");


    /* Set slave SPI0 to half-duplex mode */
    SPI0->CTL |= SPI_CTL_HALFDPX_Msk;

    /* Enable half-duplex will produce TXFBCLR (SPIx_FIFOCTL[9]) and RXFBCLR (SPIx_FIFOCTL[8])*/
    while (SPI0->STATUS & SPI_STATUS_TXRXRST_Msk) {}

    /* Set slave SPI0 data direction to output */
    SPI0->CTL |= SPI_CTL_DATDIR_Msk;

    /* Set master USPI0 to half-duplex mode */
    USPI0->PROTCTL &= (~USPI_PROTCTL_TSMSEL_Msk);
    USPI0->PROTCTL |= (4 << USPI_PROTCTL_TSMSEL_Pos);

    /* Slave SPI0 prepare data to TX FIFO */
    SPI_WRITE_TX(SPI0, 0x55A0);
    SPI_WRITE_TX(SPI0, 0x55A1);
    SPI_WRITE_TX(SPI0, 0x55A2);
    SPI_WRITE_TX(SPI0, 0x55A3);

    /* Master USPI0 receive four data from slave SPI0 */
    for (u16RxDataCount = 0; u16RxDataCount < TEST_COUNT; u16RxDataCount++)
    {
        /* Master write TX for generating clock */
        USPI_WRITE_TX(USPI0, 0x00010000);

        /* Check TX end interrupt flag */
        while (USPI_GetIntFlag(USPI0, USPI_PROTSTS_TXENDIF_Msk));

        /* Clear TX start interrupt flag */
        USPI_ClearIntFlag(USPI0, USPI_PROTSTS_TXENDIF_Msk);

        /* Check BUSY flag */
        while (USPI_GetStatus(USPI0, USPI_PROTSTS_BUSY_Msk));

        /* Read data from RX register */
        au16DestinationData[u16RxDataCount] = USPI_READ_RX(USPI0);
    }

    /* Print the received data */
    printf("Received data:\n");

    for (u16DataCount = 0; u16DataCount < TEST_COUNT; u16DataCount++)
    {
        printf("%u:\t0x%X\n", u16DataCount, au16DestinationData[u16DataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Reset USPI0 */
    USPI_Close(USPI0);

    /* Reset SPI0 */
    SPI_Close(SPI0);

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
