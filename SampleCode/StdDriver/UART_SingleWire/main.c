/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Two Single-Wire Loopback data test.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ


#define BUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_au8TxData [BUFSIZE] = {0};
volatile uint8_t g_au8RecData[BUFSIZE] = {0};
volatile uint32_t g_u32RecLen  =  0;
volatile int32_t  g_i32RecOK  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void SC0UART_Init(void);
void UART0_IRQHandler(void);
void UART0_TEST_HANDLE(void);
void UART1_IRQHandler(void);
void UART1_TEST_HANDLE(void);
void UART_FunctionTest(void);
uint8_t CheckPattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length);
void BuildSrcPattern(uint32_t u32Addr, uint8_t u8Type, uint32_t u32Length);

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
    /* Select SC0_UART clock source is HIRC */
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(1));

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
    /* Select SC0_UART clock source is HXT */
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HXT, CLK_CLKDIV1_SC0(1));

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));
#endif

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
    /* Enable GPA peripheral clock */
    CLK_EnableModuleClock(GPA_MODULE);
    /* Enable SC0 peripheral clockk */
    CLK_EnableModuleClock(SC0_MODULE);


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for SC_CLK(TXD)and SC_DAT(RXD) */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB5MFP_SC0_CLK | SYS_GPB_MFPL_PB4MFP_SC0_DAT);

    /*Set PA0 IO status is Pull-up*/
    GPIO_SetPullCtl(PA, BIT0, GPIO_PUSEL_PULL_UP);

    /*Set PA2 IO status is Pull-up*/
    GPIO_SetPullCtl(PA, BIT2, GPIO_PUSEL_PULL_UP);

    /* Set PA multi-function pins for UART0 RXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_UART0_RXD);


    /* Set PA multi-function pins for UART1 RXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_UART1_RXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                     Init SC0 UART                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void SC0UART_Init(void)
{
    /* Reset USCI0 */
    SYS_ResetModule(SC0_RST);
    /* Configure SC0_UART and set SC0_UART baud rate */
    SCUART_Open(SC0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                Init Single Wire(UART0)                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{

    /* Configure Single Wire(UART0) and set Single Wire(UART0) baud rate */
    UART_Open(UART0, 115200);
    UART_SelectSingleWireMode(UART0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                               Init Single Wire(UART1)                                                   */
/*---------------------------------------------------------------------------------------------------------*/

void UART1_Init(void)
{
    /* Configure Single Wire(UART1) and set Single Wire(UART1) baud rate */
    UART_Open(UART1, 115200);
    UART_SelectSingleWireMode(UART1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* Debug port control the Single wire 0(UART0) send data to Single wire 1(UART1)  or Single wire 1(UART1)  */
/* send data to Single wire 0(UART0)                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Lock protected registers */
    SYS_LockReg();

    /* Init SC0_UART for printf */
    SC0UART_Init();

    /* Init UART0 and UART1 for Single Wire Test*/
    UART0_Init();

    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                           SAMPLE CODE                                                   */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                       ISR to handle UART Channel 1 interrupt event                                      */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART1_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                  UART1 Callback function                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE(void)
{

    uint32_t u32IntSts = UART1->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            g_au8RecData[g_u32RecLen] = UART_READ(UART1);

            if (g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    if (u32IntSts & UART_INTSTS_SWBEINT_Msk)
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART1, UART_INTSTS_SWBEINT_Msk);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 2 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    UART0_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART1 Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_TEST_HANDLE(void)
{

    uint32_t u32IntSts = UART0->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART0))
        {
            /* Get the character from UART Buffer */
            g_au8RecData[g_u32RecLen] = UART_READ(UART0);

            if (g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    if (u32IntSts & UART_INTSTS_SWBEINT_Msk)
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART0, UART_INTSTS_SWBEINT_Msk);

    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*                              Bulid Source Pattern function                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint8_t u8Type, uint32_t u32Length)
{
    uint32_t u32Index = 0, u32Pattern = 0;
    uint8_t *pu8Addr;
    pu8Addr = (uint8_t *)u32Addr;

    if (u8Type == 0)      u32Pattern = 0x1f;
    else if (u8Type == 1) u32Pattern = 0x3f;
    else if (u8Type == 2) u32Pattern = 0x7f;
    else if (u8Type == 3) u32Pattern = 0xff;
    else  u32Pattern = 0xff;

    for (u32Index = 0; u32Index < u32Length ; u32Index++)
        pu8Addr[u32Index] = (u32Index & u32Pattern);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                    Verify that the received data is correct                                             */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CheckPattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length)
{
    uint32_t u32Index = 0;
    uint8_t u8Result = 1;
    uint8_t *pu8Addr0;
    uint8_t *pu8Addr1;
    pu8Addr0 = (uint8_t *)u32Addr0;
    pu8Addr1 = (uint8_t *)u32Addr1;

    for (u32Index = 0; u32Index < u32Length ; u32Index++)
    {
        if (pu8Addr0[u32Index] != pu8Addr1[u32Index])
        {
            printf("Data Error Idex=%u,tx =%u,rx=%u\n", u32Index, pu8Addr0[u32Index], pu8Addr1[u32Index]) ;
            u8Result = 0;
        }
    }

    return u8Result;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest(void)
{
    char chCmd ;

    printf("+-----------------------------------------------------------+\n");
    printf("|            UART Single Wire Function Test                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    The user must connect the UART0 RX pin(PA0) to         |\n");
    printf("|    UART1_Rx Pin(PB2).                                     |\n");
    printf("|    Single Wire 0(PA0)send data to Single Wire 1(PA2).     |\n");
    printf("|    Single Wire 1(PA2)send data to Single Wire 0(PA0).     |\n");
    printf("|    Please enter any to start    (Press '0' to exit)       |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect SC0_UART and PC.SC0_UART is set to debug port.
        UART0 and UART1 is enable RDA and RLS interrupt.
        The user can use SC0_UART to control the transmission or reception of UART0(Single Wire mode)
        When UART0(Single Wire 0)transfers data to UART1(Single Wire 1), if data is valid,
        it will enter the interrupt and receive the data.And then check the received data.
        When UART1(Single Wire 1)transfers data to UART0(Single Wire 0), if data is valid,
        it will enter the interrupt and receive the data.And then check the received data.
    */

    /* Enable UART0 RDA/Time-out / Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART0_IRQn);
    UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    /* Enable UART1 RDA/Time-out / Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));


    do
    {
        printf("+------------------------------------------------------------+\n");
        printf("|              UART Single Wire Test Item                    |\n");
        printf("+------------------------------------------------------------+\n");
        printf("|    (1)Single Wire 0(PA0)send data to Single Wire 1(PA2).   |\n");
        printf("|    (2)Single Wire 1(PA2)send data to Single Wire 0(PA0).   |\n");
        printf("|    (E)Exit                                                 |\n");
        printf("+------------------------------------------------------------+\n");

        chCmd = getchar();

        switch (chCmd)
        {
            case '1':
            {
                printf("SW0(UART0) --> SW1(UART1)Test :");
                g_i32RecOK  = FALSE;
                BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE);

                /* Check the Rx status is Idle */
                while (!UART_RX_IDLE(UART0)) {};

                UART_Write(UART0, g_au8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {}

                CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");
                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_au8TxData, 0, BUFSIZE);
                memset((uint8_t *)g_au8RecData, 0, BUFSIZE);
            }
            break;

            case '2':
            {
                printf("SW1(UART1) --> SW0(UART0)Test :");
                g_i32RecOK  = FALSE;
                BuildSrcPattern((uint32_t)g_au8TxData, UART_WORD_LEN_8, BUFSIZE);

                /* Check the Rx status is Idle */
                while (!UART_RX_IDLE(UART1)) {};

                UART_Write(UART1, g_au8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {};

                CheckPattern((uint32_t)g_au8TxData, (uint32_t)g_au8RecData, BUFSIZE) ? printf(" Pass\n") :   printf(" Fail\n");

                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_au8TxData, 0, BUFSIZE);

                memset((uint8_t *)g_au8RecData, 0, BUFSIZE);
            }
            break;

            default:
                break;
        }

    } while ((chCmd != 'E') && (chCmd != 'e'));

    /* Disable UART0 RDA/Time-out interrupt */
    UART_DisableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    /* Disable UART1 RDA/Time-out interrupt */
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    printf("\nUART Sample Demo End.\n");

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
