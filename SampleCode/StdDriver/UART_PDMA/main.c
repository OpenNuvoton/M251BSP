/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate UART transmit and receive function with PDMA
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

#define ENABLE_PDMA_INTERRUPT
#define PDMA_TEST_LENGTH 100
#define PDMA_TIME 0x5555          //PDMA Timeout count
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_au8TxBuffer[PDMA_TEST_LENGTH];
static uint8_t g_au8RxBuffer[PDMA_TEST_LENGTH];

volatile uint32_t g_u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
int32_t main(void);
void PDMA_IRQHandler(void);
void UART_PDMATest(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

#if (CLK_SOURCE == CLK_HIRC )
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

#else

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Select IP clock source */
    /* Select UART0 clock source is HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

#endif


    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
    /* Enable PDMA peripheral clock */
    CLK_EnableModuleClock(PDMA_MODULE);


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for UART1 TXD and RXD*/
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_UART1_RXD | SYS_GPA_MFPL_PA3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART0                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART1                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{
    UART_Open(UART1, 115200);
}

void PDMA_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(PDMA, 1 << 0); // Channel 0 for UART1 TX
    PDMA_Open(PDMA, 1 << 1); // Channel 1 for UART1 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA, 0, PDMA_UART1_TX, 0, 0);
    PDMA_SetTransferMode(PDMA, 1, PDMA_UART1_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA, 0, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA, 1, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA, 0, ((uint32_t)(&g_au8TxBuffer[0])), PDMA_SAR_INC, UART1_BASE, PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA, 1, UART1_BASE, PDMA_SAR_FIX, ((uint32_t)(&g_au8RxBuffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA, 0, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA, 1, PDMA_REQ_SINGLE, 0);

#ifdef ENABLE_PDMA_INTERRUPT
    //Set timeout
    PDMA_SetTimeOut(PDMA, 0, 1, PDMA_TIME);
    PDMA_SetTimeOut(PDMA, 1, 1, PDMA_TIME);

    PDMA_EnableInt(PDMA, 0, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, 1, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, 1, PDMA_INT_TIMEOUT);
    NVIC_EnableIRQ(PDMA_IRQn);
    g_u32IsTestOver = 0;
#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    UART_PDMATest();

    while (1);
}

/**
 * @brief       PDMA IRQ
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
        printf("target abort interrupt !!\n");
        g_u32IsTestOver = 0x2;
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)  /* done */
    {
        if ((PDMA_GET_TD_STS(PDMA) & (1 << 0)))
        {
            PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF0_Msk);
        }

        if ((PDMA_GET_TD_STS(PDMA) & (1 << 1)))
        {
            g_u32IsTestOver = 0x1;
            PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
        }
    }
    else if (u32Status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))    /* channel 0-1 timeout */
    {
        printf("timeout interrupt !!\n");
        g_u32IsTestOver = 0x3;
        /* Disable timeout  */
        PDMA_SetTimeOut(PDMA, 0, 0, 0);
        /* Clear timeout flag */
        PDMA_CLR_TMOUT_FLAG(PDMA, 0);
        /* Enable timeout and Set timeout */
        PDMA_SetTimeOut(PDMA, 0, 1, PDMA_TIME);

        /* Disable timeout  */
        PDMA_SetTimeOut(PDMA, 1, 0, 0);
        /* Clear timeout flag */
        PDMA_CLR_TMOUT_FLAG(PDMA, 1);
        /* Enable timeout and Set timeout */
        PDMA_SetTimeOut(PDMA, 1, 1, PDMA_TIME);
    }
    else
        printf("unknown interrupt !!\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART PDMA Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PDMATest(void)
{
    uint32_t u32LenCnt;

    printf("+-----------------------------------------------------------+\n");
    printf("|                    UART PDMA Test                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo UART1 PDMA function.         |\n");
    printf("|    Please connect UART1_TX and UART1_RX pin.              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");

    getchar();

    /*
        Using UAR1 external loop back.
        This code will send data from UART1_TX and receive data from UART1_RX.
    */

    for (u32LenCnt = 0; u32LenCnt < PDMA_TEST_LENGTH; u32LenCnt++)
    {
        g_au8TxBuffer[u32LenCnt] = u32LenCnt;
        g_au8RxBuffer[u32LenCnt] = 0xff;
    }

    while (1)
    {
        PDMA_Init();
        // Enable Receive Line interrupt
        UART_ENABLE_INT(UART1, UART_INTEN_RLSIEN_Msk);
        // Enable UART PDMA Tx and Rx
        UART_PDMA_ENABLE(UART1, UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);

        NVIC_EnableIRQ(UART1_IRQn);

#ifdef  ENABLE_PDMA_INTERRUPT

        while (g_u32IsTestOver == 0);

        if (g_u32IsTestOver == 1)
            printf("test done...\n");
        else if (g_u32IsTestOver == 2)
            printf("target abort...\n");
        else if (g_u32IsTestOver == 3)
            printf("timeout...\n");

        g_u32IsTestOver = 0;
#else

        while ((!(PDMA_GET_TD_STS(PDMA) & (1 << 0))) || (!(PDMA_GET_TD_STS(PDMA) & (1 << 1))));

        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF0_Msk | PDMA_TDSTS_TDIF1_Msk);
#endif

        // Disable UART PDMA Tx and Rx
        UART_PDMA_DISABLE(UART1, UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);

        for (u32LenCnt = 0; u32LenCnt < PDMA_TEST_LENGTH; u32LenCnt++)
        {
            if (g_au8RxBuffer[u32LenCnt] != u32LenCnt)
            {
                printf("\n Receive Data Compare Error !!");

                while (1);
            }

            g_au8RxBuffer[u32LenCnt] = 0xff;
        }

        printf("\nUART PDMA test Pass.\n");
    }

}

/**
 * @brief       UART1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The UART1 default IRQ, declared in startup_M251.s.
 */
void UART1_IRQHandler(void)
{

    uint32_t u32IntSts = UART1->INTSTS;

    if (u32IntSts & UART_INTSTS_HWRLSIF_Msk)
    {
        if (UART1->FIFOSTS & UART_FIFOSTS_BIF_Msk)
            printf("\n BIF \n");

        if (UART1->FIFOSTS & UART_FIFOSTS_FEF_Msk)
            printf("\n FEF \n");

        if (UART1->FIFOSTS & UART_FIFOSTS_PEF_Msk)
            printf("\n PEF \n");

        uint32_t u32Data = UART1->DAT; // read out data

        printf("\n Error Data is '0x%x' \n", u32Data);
        UART1->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk);
    }
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
