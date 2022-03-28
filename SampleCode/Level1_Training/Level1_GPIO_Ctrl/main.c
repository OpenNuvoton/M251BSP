/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show the usage of GPIO control LED function.
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define LED_R                                   PC4
#define LED_G                                   PC5
#define LED_B                                   PC3

#define LED_ON                                  0
#define LED_OFF                                 1

//------------------------------------------------------------------------------
// Global variable
//------------------------------------------------------------------------------
static volatile uint32_t gu32Sw1IntCnt = 0;
static volatile uint32_t gu32Sw2IntCnt = 0;

//------------------------------------------------------------------------------
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

}

void UART0_Init()
{
    /* Set GPB multi-function pins to UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void LED_Init(void)
{
    /* Set PC.3 ~ PC.5 to GPIO */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC3MFP_GPIO | SYS_GPC_MFPL_PC4MFP_GPIO | SYS_GPC_MFPL_PC5MFP_GPIO);

    /* Set PC.3 ~ PC.5 to GPIO output */
    GPIO_SetMode(PC, (BIT3 | BIT4 | BIT5), GPIO_MODE_OUTPUT);

    /* Let LED off after initialize */
    LED_R = LED_OFF;
    LED_G = LED_OFF;
    LED_B = LED_OFF;
}

void BTN_Init(void)
{
    /**************  SW1 ***************/
    /* Set PB.4 to GPIO */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk)) | (SYS_GPB_MFPL_PB4MFP_GPIO);
    /* Set PB.4 to GPIO intput */
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 4, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPB_IRQn);

    /**************  SW2 ***************/
    /* Set PB.0 to GPIO */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk)) | (SYS_GPB_MFPL_PB0MFP_GPIO);
    /* Set PB.0 to GPIO intput */
    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 0, GPIO_INT_FALLING);

    /* Set de-bounce function */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_512);
    GPIO_ENABLE_DEBOUNCE(PB, BIT4);
}

int main(void)
{
    uint32_t u32Sw1Cnt = 0, u32Sw2Cnt = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+---------------------------------------+\n");
    printf("|    M251 GPIO control Sample Code    |\n");
    printf("+---------------------------------------+\n\n");

    /* Init LED */
    LED_Init();

    /* Init BTN */
    BTN_Init();

    while (1)
    {
        /* Check if the SW1 is pressed */
        if (gu32Sw1IntCnt != u32Sw1Cnt)
        {
            u32Sw1Cnt = gu32Sw1IntCnt;
            printf("SW1 interrupt count: %d\n", u32Sw1Cnt);
        }

        /* Check if the SW2 is pressed */
        if (gu32Sw2IntCnt != u32Sw2Cnt)
        {
            u32Sw2Cnt = gu32Sw2IntCnt;
            printf("SW2 interrupt count: %d\n", u32Sw2Cnt);
        }
    }
}

void GPB_IRQHandler(void)
{
    /* Check if PB.4 the interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT4))
    {
        LED_R ^= 1;
        gu32Sw1IntCnt++;
        /* Clear PB.4 interrupt flag */
        GPIO_CLR_INT_FLAG(PB, BIT4);
        /* Check if PB.0 the interrupt occurred */
    }
    else if (GPIO_GET_INT_FLAG(PB, BIT0))
    {
        LED_G ^= 1;
        gu32Sw2IntCnt++;
        /* Clear PB.0 interrupt flag */
        GPIO_CLR_INT_FLAG(PB, BIT0);
    }
    else
    {
        LED_B ^= 1;
        /* Un-expected interrupt. Just clear all PB interrupts */
        PB->INTSRC = PB->INTSRC;
        printf("Un-expected interrupts.\n");
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
