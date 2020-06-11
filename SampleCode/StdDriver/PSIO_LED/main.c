/***************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       Demonstrate how to light up the WS1812B LED array.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "WS1812B_driver_LED.h"

#define LED_NUMBER  8
#define PIN_NUMBER  2


typedef enum
{
    eCASE_GREEN_BLUE = 0,
    eCASE_RED_GREEN,
    eCASE_BLUE_RED,
    eCASE_WHITE,
} E_LED_COLOR;


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

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO_MODULE);

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 4 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_HIRC, CLK_CLKDIV1_PSIO(4));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* PDMA Clock Enable */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* PSIO multi-function pin CH0(PB.15) and CH1(PC.4)*/
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_PSIO0_CH0);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC4MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC4MFP_PSIO0_CH1);
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
    /* For library internal used, this memory size should be (data length)*3 word at least, */
    /* this case is  LED_NUMBER*PIN_NUMBER*3 word.                                          */
    uint32_t   au32LedPattern[LED_NUMBER * PIN_NUMBER], au32InternalUse[LED_NUMBER * PIN_NUMBER * 3];

    S_PSIO_WS2812B_LED_CFG  sConfig;
    E_LED_COLOR             eColor = eCASE_GREEN_BLUE;
    WS2812B_LED_Pin_CFG     sPinCFG = {PSIO_PIN0, PSIO_PIN1, 0, 0, 0, 0, 0, 0}; //Enable Pin0 and Pin1

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
    printf("|         Worldsemi WS2812B LED sample code            | \n");
    printf("|          Please connected PSIO_CH0(PB.15)            | \n");
    printf("|           and PSIO_CH1(PC.4) to device               | \n");
    printf("+------------------------------------------------------+ \n");

    /* Set Led configuration */
    sConfig.u8SlotCtrl      = PSIO_SC0;
    sConfig.u8PDMAChannel   = 0;
    sConfig.pu8PinCFG       = sPinCFG;
    sConfig.u8PinNumber     = 2;
    sConfig.pu32DataAddr    = au32LedPattern;
    sConfig.u32DataLength   = LED_NUMBER * PIN_NUMBER;
    sConfig.pu32InternalMemory = au32InternalUse;           /* For library internal used, the memory size should be (data length)*3 word at least. */
    /* This case is  LED_NUMBER*PIN_NUMBER*3 word. */

    /* Initialize PSIO setting for WS2812B LED */
    PSIO_WS2812B_Open(&sConfig);

    do
    {
        switch (eColor)
        {
            case eCASE_GREEN_BLUE:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_GREEN;
                au32LedPattern[1]    = WS2812B_BLUE;    //LED0
                au32LedPattern[2]    = WS2812B_GREEN;
                au32LedPattern[3]    = WS2812B_BLUE;    //LED1
                au32LedPattern[4]    = WS2812B_GREEN;
                au32LedPattern[5]    = WS2812B_BLUE;    //LED2
                au32LedPattern[6]    = WS2812B_GREEN;
                au32LedPattern[7]    = WS2812B_BLUE;    //LED3
                au32LedPattern[8]    = WS2812B_GREEN;
                au32LedPattern[9]    = WS2812B_BLUE;    //LED4
                au32LedPattern[10]   = WS2812B_GREEN;
                au32LedPattern[11]   = WS2812B_BLUE;    //LED5
                au32LedPattern[12]   = WS2812B_GREEN;
                au32LedPattern[13]   = WS2812B_BLUE;    //LED6
                au32LedPattern[14]   = WS2812B_GREEN;
                au32LedPattern[15]   = WS2812B_BLUE;    //LED7
                eColor = eCASE_RED_GREEN;
                break;

            case eCASE_RED_GREEN:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_RED;
                au32LedPattern[1]    = WS2812B_GREEN;    //LED0
                au32LedPattern[2]    = WS2812B_RED;
                au32LedPattern[3]    = WS2812B_GREEN;    //LED1
                au32LedPattern[4]    = WS2812B_RED;
                au32LedPattern[5]    = WS2812B_GREEN;    //LED2
                au32LedPattern[6]    = WS2812B_RED;
                au32LedPattern[7]    = WS2812B_GREEN;    //LED3
                au32LedPattern[8]    = WS2812B_RED;
                au32LedPattern[9]    = WS2812B_GREEN;    //LED4
                au32LedPattern[10]   = WS2812B_RED;
                au32LedPattern[11]   = WS2812B_GREEN;    //LED5
                au32LedPattern[12]   = WS2812B_RED;
                au32LedPattern[13]   = WS2812B_GREEN;    //LED6
                au32LedPattern[14]   = WS2812B_RED;
                au32LedPattern[15]   = WS2812B_GREEN;    //LED7
                eColor = eCASE_BLUE_RED;
                break;

            case eCASE_BLUE_RED:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_BLUE;
                au32LedPattern[1]    = WS2812B_RED;    //LED0
                au32LedPattern[2]    = WS2812B_BLUE;
                au32LedPattern[3]    = WS2812B_RED;    //LED1
                au32LedPattern[4]    = WS2812B_BLUE;
                au32LedPattern[5]    = WS2812B_RED;    //LED2
                au32LedPattern[6]    = WS2812B_BLUE;
                au32LedPattern[7]    = WS2812B_RED;    //LED3
                au32LedPattern[8]    = WS2812B_BLUE;
                au32LedPattern[9]    = WS2812B_RED;    //LED4
                au32LedPattern[10]   = WS2812B_BLUE;
                au32LedPattern[11]   = WS2812B_RED;    //LED5
                au32LedPattern[12]   = WS2812B_BLUE;
                au32LedPattern[13]   = WS2812B_RED;    //LED6
                au32LedPattern[14]   = WS2812B_BLUE;
                au32LedPattern[15]   = WS2812B_RED;    //LED7
                eColor = eCASE_WHITE;
                break;

            case eCASE_WHITE:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_WHITE;
                au32LedPattern[1]    = WS2812B_WHITE;    //LED0
                au32LedPattern[2]    = WS2812B_WHITE;
                au32LedPattern[3]    = WS2812B_WHITE;    //LED1
                au32LedPattern[4]    = WS2812B_WHITE;
                au32LedPattern[5]    = WS2812B_WHITE;    //LED2
                au32LedPattern[6]    = WS2812B_WHITE;
                au32LedPattern[7]    = WS2812B_WHITE;    //LED3
                au32LedPattern[8]    = WS2812B_WHITE;
                au32LedPattern[9]    = WS2812B_WHITE;    //LED4
                au32LedPattern[10]   = WS2812B_WHITE;
                au32LedPattern[11]   = WS2812B_WHITE;    //LED5
                au32LedPattern[12]   = WS2812B_WHITE;
                au32LedPattern[13]   = WS2812B_WHITE;    //LED6
                au32LedPattern[14]   = WS2812B_WHITE;
                au32LedPattern[15]   = WS2812B_WHITE;    //LED7
                eColor = eCASE_GREEN_BLUE;
                break;
        }

        /* Send LED pattern by PSIO */
        PSIO_WS2812B_Send_Pattern(&sConfig);

        /* Delay 50000 us */
        CLK_SysTickDelay(500000);
    } while (1);

    PSIO_WS2812B_Close(&sConfig);

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
