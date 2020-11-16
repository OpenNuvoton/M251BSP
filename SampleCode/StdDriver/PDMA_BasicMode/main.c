/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use PDMA channel 2 to transfer data from memory to memory.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMA_TEST_LENGTH  64

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef __ICCARM__
    #pragma data_alignment=4
    uint8_t g_au8SrcArray[256];
    uint8_t g_au8DestArray[256];
#else
    __attribute__((aligned(4))) uint8_t g_au8SrcArray[256];
    __attribute__((aligned(4))) uint8_t g_au8DestArray[256];
#endif

uint32_t volatile g_u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M251.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA);

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        /* Check if channel 2 has abort error */
        if (PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;

        /* Clear abort flag of channel 2 */
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        /* Check transmission of channel 2 has been transfer done */
        if (PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;

        /* Clear transfer done flag of channel 2 */
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|    PDMA Memory to Memory Driver Sample Code          | \n");
    printf("+------------------------------------------------------+ \n");


    /*------------------------------------------------------------------------------------------------------

                         au8SrcArray                         au8DestArray
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      PDMA transfer configuration:

        Channel = 2
        Operation mode = basic mode
        Request source = PDMA_MEM(memory to memory)
        transfer done and table empty interrupt = enable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/

    /* Open Channel 2 */
    PDMA_Open(PDMA, 1 << 2);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA, 2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA, 2, (uint32_t)g_au8SrcArray, PDMA_SAR_INC, (uint32_t)g_au8DestArray, PDMA_DAR_INC);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA, 2, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA, 2, PDMA_REQ_BURST, PDMA_BURST_4);

    /* Enable interrupt */
    PDMA_EnableInt(PDMA, 2, PDMA_INT_TRANS_DONE);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);
    g_u32IsTestOver = 0;

    /* Generate a software request to trigger transfer with PDMA channel 2  */
    PDMA_Trigger(PDMA, 2);

    /* Waiting for transfer done */
    while (g_u32IsTestOver == 0);

    /* Check transfer result */
    if (g_u32IsTestOver == 1)
        printf("test done...\n");
    else if (g_u32IsTestOver == 2)
        printf("target abort...\n");

    /* Close channel 2 */
    PDMA_Close(PDMA);

    while (1);
}
