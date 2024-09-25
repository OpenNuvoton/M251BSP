/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive data using auto flow control.
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


#define RXBUFSIZE 1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8RecData[RXBUFSIZE]  = {0};
volatile int32_t g_i32pointer = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void UART_TEST_HANDLE(void);
void UART1_IRQHandler(void);
void AutoFlowFunctionTest(void);

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

    /* Set both PCLK0 and PCLK1 as HCLK */
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


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for UART1 TXD, RXD, CTS and RTS */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk |
                       SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_UART1_nRTS | SYS_GPA_MFPL_PA1MFP_UART1_nCTS |
                      SYS_GPA_MFPL_PA2MFP_UART1_RXD | SYS_GPA_MFPL_PA3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                                 Init UART0                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                                 Init UART1                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{
    UART_Open(UART1, 115200);
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
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------+\n");
    printf("|  Auto-Flow function test |\n");
    printf("+--------------------------+\n");

    AutoFlowFunctionTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE(void)
{
    volatile uint32_t u32IntSts = UART1->INTSTS;;

    /* Rx Ready or Time-out INT */
    if (UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk) ||  UART_GET_INT_FLAG(UART1, UART_INTSTS_RXTOINT_Msk))
    {
        /* Handle received data */
        g_au8RecData[g_i32pointer] = UART_READ(UART1);
        g_i32pointer++;
        g_au8RecData[g_i32pointer] = UART_READ(UART1);
        g_i32pointer++;
        g_au8RecData[g_i32pointer] = UART_READ(UART1);
        g_i32pointer++;
        g_au8RecData[g_i32pointer] = UART_READ(UART1);
        g_i32pointer++;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlowFunctionTest(void)
{
    uint8_t u8Item;
    uint32_t u32Loop;
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  _______                                      _______     |\n");
    printf("| |       |                                    |       |    |\n");
    printf("| |Master |---TXD1    <============>    RXD1---| Slave |    |\n");
    printf("| |       |---RTS1    <============>    CTS1---|       |    |\n");
    printf("| |_______|---CTS1    <============>    RTS1---|_______|    |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* Set Trigger Level */
    /* Set RTS Low level trigger and CTS Low level trigger*/
    /* Enable RTS and CTS auto flow control */
    UART_EnableFlowCtrl(UART1);

    //UART1->MODEM &= ~(UART_MODEM_RTSACTLV_Msk ); // Set RTS high level trigger
    //UART1->MODEMSTS &= ~UART_MODEMSTS_CTSACTLV_Msk; // Set CTS high level trigger

    UART1->FIFO = (UART1->FIFO & ~ UART_FIFO_RTSTRGLV_Msk) | UART_FIFO_RTSTRGLV_4BYTES;

    printf("+-----------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test                              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave.Slave will check if received data is correct  |\n");
    printf("|    after getting 1k bytes data.                           |\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n\n");
    u8Item = getchar();


    if (u8Item == '0')
    {
        for (u32Loop = 0; u32Loop < RXBUFSIZE; u32Loop++)
        {
            UART_WRITE(UART1, ((u32Loop) & 0xFF));

            while (!(UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));
        }

        printf("\n Transmit Done\n");
    }
    else
    {
        g_i32pointer = 0;

        /* Enable RDA\RLS\RTO Interrupt  */
        UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

        /* Set RX Trigger Level = 4 */
        UART1->FIFO = (UART1->FIFO & ~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_4BYTES;

        /* Set Timeout time 0x3E bit-time */
        UART_SetTimeoutCnt(UART1, 0x3E);

        NVIC_EnableIRQ(UART1_IRQn);

        printf("Starting to receive %d bytes data...\n", RXBUFSIZE);

        while (g_i32pointer < (RXBUFSIZE - 1))
        {
        }

        /* Compare Data */
        for (u32Loop = 0; u32Loop < RXBUFSIZE ; u32Loop++)
        {
            if (g_au8RecData[u32Loop] != ((u32Loop) & 0xFF))
            {
                printf("Compare Data Failed\n");

                while (1);
            }
        }

        printf("\n Receive OK & Check OK\n");
    }

    NVIC_DisableIRQ(UART1_IRQn);

    UART_DISABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
