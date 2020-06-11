/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    This is a I2S demo for recording data and demonstrate how I2S works with PDMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define I2S_TXData_DMA_CH 1
#define I2S_RX_DMA_CH 2

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN 4
#define TX_BUFF_LEN 8

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

DESC_TABLE_T g_asDescTable_DataTX[1], g_asDescTable_RX[2];

/* Function prototype declaration */
void SYS_Init(void);

/* Global variable declaration */
volatile uint8_t u8RxIdx = 0;
volatile uint8_t g_u8TransDone = 1;
uint32_t g_au32PcmRxBuff[2][BUFF_LEN] = {0};
uint32_t g_au32PcmTxDataBuff[TX_BUFF_LEN] = {0};

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

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set UART0 Default MPF */
    Uart0DefaultMPF() ;

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL = SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS ;

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

}

void PDMA_Init(void)
{
    /* Enable PDMA channels */
    PDMA_Open(PDMA, (1 << I2S_TXData_DMA_CH) | (1 << I2S_RX_DMA_CH));

    /* Tx description */
    PDMA->DSCT[I2S_TXData_DMA_CH].CTL = ((TX_BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_BASIC;
    PDMA->DSCT[I2S_TXData_DMA_CH].SA = (uint32_t)&g_au32PcmTxDataBuff[0];
    PDMA->DSCT[I2S_TXData_DMA_CH].DA = (uint32_t)&SPI0->TX;

    /* Rx(Record) description */
    g_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_TBINTDIS_DISABLE | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[0].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[0].DA = (uint32_t)&g_au32PcmRxBuff[0];
    g_asDescTable_RX[0].FIRST = (uint32_t)&g_asDescTable_RX[1] - (PDMA->SCATBA);

    g_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_BASIC;
    g_asDescTable_RX[1].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[1].DA = (uint32_t)&g_au32PcmRxBuff[1];

    PDMA_SetTransferMode(PDMA, I2S_TXData_DMA_CH, PDMA_SPI0_TX, 0, 0);
    PDMA_SetTransferMode(PDMA, I2S_RX_DMA_CH, PDMA_SPI0_RX, 1, (uint32_t)&g_asDescTable_RX[0]);

    /* Enable PDMA channel 2 interrupt */
    PDMA_EnableInt(PDMA, I2S_TXData_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, I2S_RX_DMA_CH, PDMA_INT_TRANS_DONE);

    NVIC_EnableIRQ(PDMA_IRQn);
}

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA);

    if (u32Status & 0x1)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA) & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (u32Status & 0x2)
    {
        if (PDMA_GET_TD_STS(PDMA) & 0x2)            /* channel 1 done */
        {
            PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
        }

        if (PDMA_GET_TD_STS(PDMA) & 0x4)            /* channel 2 done */
        {
            g_u8TransDone = 0;
            PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF2_Msk);
        }
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32InitValue, u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+----------------------------------------------+\n");
    printf("|        SPII2S + PDMA  Record Sample Code     |\n");
    printf("+----------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0x50005000, 0x50015001, ... \n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S_LRCLK (PA3)\n      I2S_BCLK(PA2)\n");
    printf("      I2S_DI (PA1)\n         I2S_DO (PA0)\n\n");
    printf("      This sample code will transmit and receive data with PDMA transfer.\n");
    printf("      Connect I2S_DI and I2S_DO to check if the data which stored in two receive\n buffers are the same with the transmitted values.\n");
    printf("      After PDMA transfer is finished, the received values will be printed.\n\n");

    /* Select PCLK as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /* Enable I2S TX and RX functions */
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    /* Master mode, 32-bit word width, stereo mode, I2S format. */
    SPII2S_Open(SPI0, SPII2S_MODE_MASTER, 1000000, SPII2S_DATABIT_32, SPII2S_STEREO, SPII2S_FORMAT_I2S);

    g_u8TransDone = 1 ;
    /* Data initiation */
    u32InitValue = 0x50005000;

    for (u32DataCount = 0; u32DataCount < TX_BUFF_LEN; u32DataCount++)
    {
        g_au32PcmTxDataBuff[u32DataCount] = u32InitValue;
        u32InitValue += 0x00010001;
    }

    /* Data initiation */
    for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        g_au32PcmRxBuff[0][u32DataCount] = 0xFFFFFFFF ;
        g_au32PcmRxBuff[1][u32DataCount] = 0xFFFFFFFF ;
    }

    PDMA_Init() ;

    /* Clear RX FIFO */
    SPII2S_CLR_RX_FIFO(SPI0);

    /* Enable RX function and RX PDMA function */
    SPII2S_ENABLE_TXDMA(SPI0);
    SPII2S_ENABLE_RXDMA(SPI0);


    /* Enable TX function and TX PDMA function */
    SPII2S_ENABLE_TX(SPI0);
    SPII2S_ENABLE_RX(SPI0);

    printf("RX Buffer 1\tRX Buffer 2\n");

    while (g_u8TransDone) {};

    /* Print the received data */
    for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        printf("0x%8X\t\t0x%8X\n", g_au32PcmRxBuff[0][u32DataCount], g_au32PcmRxBuff[1][u32DataCount]);
    }

    printf("\n\nExit I2S sample code.\n");

    while (1);

}



/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
