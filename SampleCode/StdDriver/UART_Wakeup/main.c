/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up system from Power-down mode by UART interrupt.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

#define RS485_ADDRESS   0xC0  // RS485 Address
#define PD_MODE         0     // Power-down mode
#define FWPD_MODE       1     // Fast wake up

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t  g_i32WakeUp =     FALSE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
int32_t main(void);
void UART_DataWakeUp(void);
void UART_CTSWakeUp(void);
void UART_RxThresholdWakeUp(void);
void UART_RS485WakeUp(void);
void UART_PowerDown_TestItem(void);
void UART_PowerDownWakeUpTest(void);
void PWRWU_IRQHandler(void);
void UART1_Init(void);
void UART1_IRQHandler(void);
void UART1_TEST_HANDLE(void);
void EnterToPowerDown(uint32_t u32PDMode);
extern int IsDebugFifoEmpty(void);

/**
  * @brief      Enter To Power Down
  * @param[in]   u32PDMode    The specified Power down module.
  *                               - \ref CLK_PMUCTL_PDMSEL_PD      : Power-down mode
  *                               - \ref CLK_PMUCTL_PDMSEL_FWPD    : Fast wake up
  *
  * @return     None
  *
  * @details    This API is used to get the current RTC date and time value.
  */

void EnterToPowerDown(uint32_t u32PDMode)
{
    g_i32WakeUp = FALSE;
    SYS_UnlockReg();
    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PMUCTL &= ~(CLK_PMUCTL_PDMSEL_Msk | CLK_PMUCTL_RTCWKEN_Msk);

    if (u32PDMode == PD_MODE)
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_PD;    //Power down
    else if ((u32PDMode == FWPD_MODE))
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_FWPD;  //Fast Wake Up

    CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);
    CLK->PWRCTL |=  CLK_PWRCTL_PDWKIEN_Msk;
    CLK_PowerDown();
    SYS_LockReg();

    while (g_i32WakeUp == FALSE);

}


/*---------------------------------------------------------------------------------------------------------*/
/*                                       Init System Clock                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

#if (CLK_SOURCE == CLK_HIRC )

    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk);
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Disable digital input path of analog pin X32_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 4));

    /* Disable digital input path of analog pin X32_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 5));

#else

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk);
    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Select IP clock source */
    /* Select UART0 clock source is HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

    /* Disable digital input path of analog pin X32_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 4));

    /* Disable digital input path of analog pin X32_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 5));

#endif

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                        Init I/O Multi-function                                          */
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
/*                                              Init UART                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 9600);

}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                      Main Function                                                      */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf and test */
    UART0_Init();
    /* Init UART1 for test */
    UART1_Init();
    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART Sample Program\n\n");

    /* UART Power-down and Wake-up sample function */
    UART_PowerDownWakeUpTest();

    printf("\nUART Sample Program End\n");

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART1_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle Wake-up interrupt event                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    g_i32WakeUp = TRUE;

    CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;
    printf("MCU Wake-Up.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                      UART Callback function                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE(void)
{
    uint32_t u32Data;

    if (UART_GET_INT_FLAG(UART1, UART_INTSTS_WKINT_Msk))    /* UART wake-up interrupt flag */
    {
        if (UART1->WKSTS & UART_WKSTS_CTSWKF_Msk)
            printf("CTS wake-up.\n");
        else if (UART1->WKSTS & UART_WKSTS_DATWKF_Msk)
            printf("Data wake-up.\n");
        else if (UART1->WKSTS & UART_WKSTS_RS485WKF_Msk)
            printf("RS485 address macth wake-up.\n");
        else if (UART1->WKSTS & UART_WKSTS_RFRTWKF_Msk)
            printf("Rx Threshold wake-up.\n");
        else if (UART1->WKSTS & UART_WKSTS_TOUTWKF_Msk)
            printf("Rx Threshold time out wake-up.\n");

        UART_ClearIntFlag(UART1, UART_INTSTS_WKINT_Msk);

    }
    else if (UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) /* UART receive data available flag */
    {
        while (UART_GET_RX_EMPTY(UART1) == 0)
        {
            u32Data = UART_READ(UART1);

            if (u32Data & UART_DAT_PARITY_Msk)
                printf("Address: 0x%X\n", (u32Data & 0xFF));
            else
                printf("Data: 0x%X\n", u32Data);
        }
    }
}

//*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_CTSWakeUp(void)
{

    /* Enable UART nCTS wake-up function */
    UART1->WKCTL |= UART_WKCTL_WKCTSEN_Msk;

    printf("System enter to Power-down mode.\n");
    printf("Toggle UART1 nCTS to wake-up system.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void)
{

    /* Enable UART data wake-up function */
    UART1->WKCTL |= UART_WKCTL_WKDATEN_Msk;

    /* Set UART data wake-up start bit compensation value.
       It indicates how many clock cycle selected by UART_CLK does the UART controller can get the first bit (start bit)
       when the device is wake-up from power-down mode.
       If UART_CLK is selected as HIRC(48MHz) and the HIRC stable time is about 10.67us,
       the data wake-up start bit compensation value can be set as 520. */
    UART1->DWKCOMP = 520;

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to UART1 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Rx threshold and time-out Wake-up function                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void UART_RxThresholdWakeUp(void)
{
    /* Select UART clock source as LXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_LXT, CLK_CLKDIV0_UART1(1));

    /* Set UART baud rate and baud rate compensation */
    UART_Open(UART1, 9600);
    UART1->BRCOMP = 0xA5;

    /* Enable UART Rx Threshold and Rx time-out wake-up function */
    UART1->WKCTL |= UART_WKCTL_WKRFRTEN_Msk | UART_WKCTL_WKTOUTEN_Msk;

    /* Set Rx FIFO interrupt trigger level */
    UART1->FIFO = (UART1->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES;

    /* Enable UART Rx time-out function */
    UART_SetTimeoutCnt(UART1, 40);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to UART1 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART RS485 address match (AAD mode) Wake-up function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void UART_RS485WakeUp(void)
{
    /* Select UART clock source as LXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_LXT, CLK_CLKDIV0_UART1(1));

    /* Set UART baud rate and baud rate compensation */
    UART_Open(UART1, 9600);
    UART1->BRCOMP = 0xA5;

    /* RS485 address match (AAD mode) setting */
    UART_SelectRS485Mode(UART1, (UART_ALTCTL_RS485AAD_Msk | UART_ALTCTL_ADDRDEN_Msk), RS485_ADDRESS);

    /* Enable parity source selection function */
    UART1->LINE |= (UART_LINE_PSS_Msk | UART_LINE_PBE_Msk);

    /* Enable UART RS485 address match and Rx time-out wake up function */
    UART1->WKCTL |= UART_WKCTL_WKRS485EN_Msk | UART_WKCTL_WKTOUTEN_Msk;
    /* Enable UART Rx time-out function */
    UART_SetTimeoutCnt(UART1, 40);

    printf("System enter to Power-down mode.\n");
    printf("Send RS485 address byte 0x%X with baud rate 9600bps to UART1 to wake-up system.\n\n", RS485_ADDRESS);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Power-down and wake-up test                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("| [3] Rx threshold and time-out wake-up test                |\n");
    printf("| [4] RS485 wake-up test                                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                           - [Others] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~4): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

#if (CLK_SOURCE == CLK_HXT )
    printf("Due to PLL clock stable too slow.\n");
    printf("Before demo UART wake-up, this demo code will switch HCLK from PLL to HIRC.\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Lock protected registers */
    SYS_LockReg();
#endif

    /* Enable UART wake-up and receive data available interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, UART_INTEN_WKIEN_Msk | UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(PWRWU_IRQn);

    UART_PowerDown_TestItem();
    u32Item = (uint32_t)getchar();
    printf("%c\n\n", u32Item);

    switch (u32Item)
    {
        case '1':
            UART_CTSWakeUp();
            break;

        case '2':
            UART_DataWakeUp();
            break;

        case '3':
            UART_RxThresholdWakeUp();
            break;

        case '4':
            UART_RS485WakeUp();
            break;

        default:
            return;
    }

    while (!IsDebugFifoEmpty()) {};

    /* Enter to Power-down mode */
    EnterToPowerDown(PD_MODE);

    /* Wait for all data transmission is finished */
    while ((UART1->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk) == 0);


    printf("Enter any key to end test.\n");
    getchar();

    /* Disable UART wake-up function */
    UART1->WKCTL = 0;

    /* Disable UART Interrupt */
    NVIC_DisableIRQ(UART1_IRQn);
    NVIC_DisableIRQ(PWRWU_IRQn);
    UART_DisableInt(UART1, UART_INTEN_WKIEN_Msk | UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);


#if (CLK_SOURCE == CLK_HXT )
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Lock protected registers */
    SYS_LockReg();
#endif

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

