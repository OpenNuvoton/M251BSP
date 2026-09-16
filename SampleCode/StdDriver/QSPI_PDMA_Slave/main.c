/***************************************************************************//**
 * @file    main.c
 * @version V0.10
 * @brief   Demonstrate QSPI slave mode data transfer with PDMA and Quad mode.
 *          Configure QSPI as Slave mode and demonstrate how to communicate
 *          with an off-chip SPI Master device with FIFO mode. This sample
 *          code needs to work with QSPI_PDMA_Master sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define QSPI_SLAVE_RX_DMA_CH    3
#define TEST_COUNT              (64)
#define QSPI_PDMA_TIMEOUT       (2UL * SystemCoreClock)

//------------------------------------------------------------------------------
/* Function prototype declaration */
void SYS_Init(void);
void QSPI_Init(void);
void QSPI_Slave_Receive(void);

//------------------------------------------------------------------------------
/* Global variable declaration */
uint32_t g_au32SlaveRxBuffer[TEST_COUNT];

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

    /* Select PCLK0 as the clock source of QSPI0 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable QSPI0 peripheral clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

    /* Enable PDMA peripheral clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set UART0 Default MPF */
    Uart0DefaultMPF();

    /* Setup QSPI0 multi-function pins */
    SYS->GPA_MFPL = (SYS_GPA_MFPL_PA0MFP_QSPI0_MOSI0 |
                     SYS_GPA_MFPL_PA1MFP_QSPI0_MISO0 |
                     SYS_GPA_MFPL_PA2MFP_QSPI0_CLK   |
                     SYS_GPA_MFPL_PA3MFP_QSPI0_SS    |
                     SYS_GPA_MFPL_PA4MFP_QSPI0_MOSI1 |
                     SYS_GPA_MFPL_PA5MFP_QSPI0_MISO1);

    /* Enable QSPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void QSPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure QSPI0 */
    /* Enable QSPI0 Quad input mode */
    QSPI_ENABLE_QUAD_INPUT_MODE(QSPI0);
    /* Configure QSPI0 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure QSPI0 as a low level active device. SPI peripheral clock rate = f_PCLK0 */
    QSPI_Open(QSPI0, QSPI_SLAVE, QSPI_MODE_0, 32, (uint32_t)NULL);
}

void QSPI_Slave_Receive(void)
{
    uint32_t u32DataCount;
    uint32_t u32RegValue, u32Abort, u32TimeOutCount;
    int32_t i32Err;

    printf("\nQSPI0 Quad mode test with PDMA ");

    /* Enable PDMA channel */
    PDMA_Open(PDMA, (1 << QSPI_SLAVE_RX_DMA_CH));

    /*=======================================================================
      QSPI slave PDMA RX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = TEST_COUNT
        Source = QSPI0->RX
        Source Address = Fixed
        Destination = g_au32SlaveRxBuffer
        Destination Address = Increasing
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA, QSPI_SLAVE_RX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA, QSPI_SLAVE_RX_DMA_CH, (uint32_t)&QSPI0->RX, PDMA_SAR_FIX, (uint32_t)g_au32SlaveRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA, QSPI_SLAVE_RX_DMA_CH, PDMA_QSPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA, QSPI_SLAVE_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[QSPI_SLAVE_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Enable QSPI slave DMA function */
    QSPI_TRIGGER_RX_PDMA(QSPI0);

    i32Err = 0;
    u32TimeOutCount = QSPI_PDMA_TIMEOUT;

    while (1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);

        /* Check the PDMA transfer done interrupt flag */
        if (u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done flags */
            if (((PDMA_GET_TD_STS(PDMA) & (1 << QSPI_SLAVE_RX_DMA_CH)) == (1 << QSPI_SLAVE_RX_DMA_CH)))
            {

                /* Clear the PDMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA, (1 << QSPI_SLAVE_RX_DMA_CH));

                /* Disable QSPI master's PDMA transfer function */
                QSPI_DISABLE_RX_PDMA(QSPI0);

                /* Check the transfer data */
                for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                {
                    printf("Rx[%d] 0x%x\n", u32DataCount, g_au32SlaveRxBuffer[u32DataCount]);
                }

                break;
            }
        }

        /* Check the DMA transfer abort interrupt flag */
        if (u32RegValue & PDMA_INTSTS_ABTIF_Msk)
        {
            /* Get the target abort flag */
            u32Abort = PDMA_GET_ABORT_STS(PDMA);
            /* Clear the target abort flag */
            PDMA_CLR_ABORT_FLAG(PDMA, u32Abort);
            i32Err = 1;
            break;
        }

        /* Check the DMA time-out interrupt flag */
        if (u32RegValue & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))
        {
            /* Clear the time-out flag */
            PDMA->INTSTS = u32RegValue & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk);
            i32Err = 1;
            break;
        }

        if (u32TimeOutCount-- == 0UL)
        {
            i32Err = 1;
            break;
        }
    }

    /* Disable all PDMA channels */
    PDMA_Close(PDMA);

    if (i32Err)
    {
        printf(" [FAIL]\n");
    }
    else
    {
        printf(" [PASS]\n");
    }

    return;
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    QSPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|                  QSPI slave mode + PDMA Sample Code          |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure QSPI0 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("QSPI controller will enable Quad mode and receive data from a off-chip master device.\n");
    printf("The I/O connection for these two QSPI0:\n");
    printf("    QSPI0_SS  (PA3)  <--> QSPI0_SS  (PA3)\n    QSPI0_CLK(PA2)  <--> QSPI0_CLK(PA2)\n");
    printf("    QSPI0_MISO0(PA1) <--> QSPI0_MISO0(PA1)\n    QSPI0_MOSI0(PA0) <--> QSPI0_MOSI0(PA0)\n\n");
    printf("    QSPI0_MISO1(PA5) <--> QSPI0_MISO1(PA5)\n    QSPI0_MOSI1(PA4) <--> QSPI0_MOSI1(PA4)\n\n");
    printf("After the transfer is done, the received data will be printed out.\n");
    printf("Demo only: this sample requires a paired QSPI_PDMA_Master peer and lab wiring.\n");
    printf("Production code should add bounded timeout, recovery handling, and peer-loss handling.\n");
    printf("\n");

    QSPI_Slave_Receive();

    printf("\n\nExit QSPI driver sample code.\n");

    /* Close QSPI0 */
    QSPI_Close(QSPI0);

    while (1);
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
