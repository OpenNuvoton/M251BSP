/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show how to wake up system from Power-down mode by USCI interrupt in UART mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_48MHZ

#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_DataWakeUp(void);
void USCI_UART_CTSWakeUp(void);
void USCI_UART_PowerDown_TestItem(void);
void USCI_UART_PowerDownWakeUpTest(void);


void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    while (IsDebugFifoEmpty() == 0) {}

    /* Enter to Power-down mode */
    CLK_PowerDown();
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
    //Uart0DefaultMPF();

    /* Set GPA multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA1MFP_Msk) | SYS_GPA_MFPL_PA1MFP_UART0_TXD;

    /* Set UUART0 multi-function pins */
    SYS->GPB_MFPH = SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH = SYS->GPB_MFPH | (SYS_GPB_MFPH_PB13MFP_USCI0_DAT0 | SYS_GPB_MFPH_PB14MFP_USCI0_DAT1 | SYS_GPB_MFPH_PB15MFP_USCI0_CTL1);
    SYS->GPC_MFPH = SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC14MFP_Msk;
    SYS->GPC_MFPH = SYS->GPC_MFPH | SYS_GPC_MFPH_PC14MFP_USCI0_CTL0;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* USCI UART Test Sample                                                                                   */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init USCI0 for test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART Power-down and Wake-up sample function */
    USCI_UART_PowerDownWakeUpTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    uint32_t u32IntSts = UUART_GET_PROT_STATUS(UUART0);
    uint32_t u32WkSts = UUART_GET_WAKEUP_FLAG(UUART0);

    if (u32WkSts & UUART_WKSTS_WKF_Msk)   /* USCI UART wake-up flag */
    {
        UUART_CLR_WAKEUP_FLAG(UUART0);
        printf("USCI UART wake-up.\n");
    }

    if (u32IntSts & UUART_PROTSTS_RXENDIF_Msk)   /* USCI UART receive end interrupt flag */
    {
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);

        while (UUART_GET_RX_EMPTY(UUART0) == 0)
        {
            printf("Data: 0x%X\n", UUART_READ(UUART0));
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_CTSWakeUp(void)
{
    /* Enable UART nCTS wake-up function */
    UUART_EnableWakeup(UUART0, UUART_PROTCTL_CTSWKEN_Msk);

    printf("System enter to Power-down mode.\n");
    printf("Toggle USCI-UART0 nCTS to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_DataWakeUp(void)
{
    printf("Due to PLL clock stable too slow \n");
    printf("Before demo Data wake-up, this demo code will switch HCLK from PLL to HIRC \n");

    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set UUART0 Line config */
    UUART_SetLine_Config(UUART0, 1200, UUART_WORD_LEN_8, UUART_PARITY_NONE, UUART_STOP_BIT_1);

    /* Enable UART data wake-up function */
    UUART_EnableWakeup(UUART0, UUART_PROTCTL_DATWKEN_Msk);

    /* Set UART data wake-up counter */
    UUART0->PROTCTL = (UUART0->PROTCTL & (~UUART_PROTCTL_WAKECNT_Msk)) | (4 << UUART_PROTCTL_WAKECNT_Pos);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 1200bps to USCI-UART0 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  USCI-UART Power-down and wake-up test                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enable UART receive end interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_EnableIRQ(USCI0_IRQn);

    USCI_UART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);

    switch (u32Item)
    {
        case '1':
            USCI_UART_CTSWakeUp();
            break;

        case '2':
            USCI_UART_DataWakeUp();
            break;

        default:
            break;
    }

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    printf("Enter any key to end test.\n\n");
    getchar();

    /* Disable UART wake-up function */
    UUART_DisableWakeup(UUART0);

    /* Disable UART receive end interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_DisableIRQ(USCI0_IRQn);

    printf("USCI UART Sample Program End.\n");
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
