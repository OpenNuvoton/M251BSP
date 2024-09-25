/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to use auto baud rate detection function.
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

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void AutoBaudRateTestItem(void);
void AutoBaudRateTxTest(void);
void AutoBaudRateRxTest(void);
void AutoBaudFunctionTest(void);
uint32_t GetUartBaudrate(UART_T *uart);

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

    /* Set PA multi-function pins for UART1 TXD and RXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_UART1_RXD | SYS_GPA_MFPL_PA3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART0                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{

    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART1                                                                                              */
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
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------+\n");
    printf("|  Auto Baud Rate function test |\n");
    printf("+-------------------------------+\n");

    AutoBaudFunctionTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test Menu(Master)                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRateTestItem(void)
{
    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|           Auto Baud Rate Function Test (Master)           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] baud rate 38400 bps                                   |\n");
    printf("| [2] baud rate 57600 bps                                   |\n");
    printf("| [3] baud rate 115200 bps                                  |\n");
    printf("|                                                           |\n");
    printf("| Select baud rate and master will send 0x1 to slave ...    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRateTxTest(void)
{
    uint8_t u8Item;


    do
    {
        AutoBaudRateTestItem();
        u8Item = getchar();
        printf("%c\n\n", u8Item);

        /* Set different baud rate */
        switch (u8Item)
        {
            case '1':
                UART_SetLine_Config(UART1, 38400, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
                break;

            case '2':
                UART_SetLine_Config(UART1, 57600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
                break;

            default:
                UART_SetLine_Config(UART1, 115200, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
                break;
        }

        /* Send input pattern 0x1 for auto baud rate detection bit length is 1-bit */
        UART_WRITE(UART1, 0x1);

    } while (u8Item != 27);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Get UART Baud Rate Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetUartBaudrate(UART_T *uart)
{
    uint8_t u8UartClkSrcSel = 0ul, u8UartClkDivNum;
    uint32_t au32ClkTbl[6] = {__HXT, 0ul, __LXT, __HIRC, 0ul, __LIRC};
    uint32_t u32Baud_Div;

    if (uart == (UART_T *)UART0)
    {
        /* Get UART clock source selection */
        u8UartClkSrcSel = ((uint32_t)(CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk)) >> CLK_CLKSEL1_UART0SEL_Pos;
        /* Get UART clock divider number */
        u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;
    }
    else if (uart == (UART_T *)UART1)
    {
        /* Get UART clock source selection */
        u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART1SEL_Msk) >> CLK_CLKSEL1_UART1SEL_Pos;
        /* Get UART clock divider number */
        u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;
    }
    else if (uart == (UART_T *)UART2)
    {
        /* Get UART clock source selection */
        u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART2SEL_Msk) >> CLK_CLKSEL3_UART2SEL_Pos;
        /* Get UART clock divider number */
        u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART2DIV_Msk) >> CLK_CLKDIV4_UART2DIV_Pos;
    }
    else if (uart == (UART_T *)UART3)
    {
        /* Get UART clock source selection */
        u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART3SEL_Msk) >> CLK_CLKSEL3_UART3SEL_Pos;
        /* Get UART clock divider number */
        u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART3DIV_Msk) >> CLK_CLKDIV4_UART3DIV_Pos;
    }


    /* Get PLL clock frequency if UART clock source selection is PLL */
    if (u8UartClkSrcSel == 1ul)
    {
        au32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLClockFreq();
    }
    /* Get PCLK clock frequency if UART clock source selection is PCLK */
    else if (u8UartClkSrcSel == 4ul)
    {
        /* UART Port as UART0 or UART2 */
        if (uart == (UART_T *)UART0 || uart == (UART_T *)UART2)
            au32ClkTbl[u8UartClkSrcSel] =  CLK_GetPCLK0Freq();
        else /* UART Port as UART1 or UART3*/
            au32ClkTbl[u8UartClkSrcSel] =  CLK_GetPCLK1Freq();
    }

    /* Get UART baud rate divider */
    u32Baud_Div = (uart->BAUD & UART_BAUD_BRD_Msk) >> UART_BAUD_BRD_Pos;

    /* Calculate UART baud rate if baud rate is set in MODE 0 */
    if ((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE0)
        return ((au32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1ul) / (u32Baud_Div + 2ul)) >> 4;

    /* Calculate UART baud rate if baud rate is set in MODE 2 */
    else if ((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE2)
        return ((au32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1ul) / (u32Baud_Div + 2ul));

    /* Calculate UART baud rate if baud rate is set in MODE 1 */
    else if ((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_BAUDM1_Msk)
        return ((au32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1ul) / (u32Baud_Div + 2ul)) / (((uart->BAUD & UART_BAUD_EDIVM1_Msk) >> UART_BAUD_EDIVM1_Pos) + 1ul);

    /* Unsupported baud rate setting */
    else
        return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Rx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRateRxTest(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|        Auto Baud Rate Function Test (Slave)               |\n");
    printf("+-----------------------------------------------------------+\n");

    /* Enable auto baud rate detect function */
    UART1->ALTCTL |= UART_ALTCTL_ABRDEN_Msk;

    printf("\nreceiving input pattern... ");

    /* Wait until auto baud rate detect finished or time-out */
    while ((UART1->ALTCTL & UART_ALTCTL_ABRIF_Msk) == 0ul);

    if (UART1->FIFOSTS & UART_FIFOSTS_ABRDIF_Msk)
    {
        /* Clear auto baud rate detect finished flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDIF_Msk;
        printf("Baud rate is %ubps.\n", GetUartBaudrate(UART1));
    }
    else if (UART0->FIFOSTS & UART_FIFOSTS_ABRDTOIF_Msk)
    {
        /* Clear auto baud rate detect time-out flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDTOIF_Msk;
        printf("Time-out!\n");
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudFunctionTest(void)
{

    uint8_t u8Item;
    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|                        Pin Configure                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PA.3)  <==>  UART1_RXD(PA.2)--|Slave| |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|               Auto Baud Rate Function Test                |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send input pattern    |\n");
    printf("|    0x1 with different baud rate. It can check if Slave    |\n");
    printf("|    calculates correct baud rate.                          |\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");

    u8Item = getchar();

    if (u8Item == '0')
    {
        AutoBaudRateTxTest();
    }
    else
    {
        AutoBaudRateRxTest();
    }

    printf("UART Sample Code End.\n");


}



/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
