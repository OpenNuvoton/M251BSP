/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Monitor EADC conversion result by the digital compare function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32EadcInt0Flag, g_u32EadcCmp0IntFlag, g_u32EadcCmp1IntFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void EADC_FunctionTest(void);


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

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is PCLK1, set divider to 8, ADC clock is PCLK1/8 MHz */
    /* Note: The EADC_CLK speed should meet datasheet spec (<16MHz) and rules in following table.   */
    /* +--------------+------------------+                                                          */
    /* | PCLK divider | EADC_CLK divider |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 1            | 1, 2, 3, 4, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 2, 4, 8, 16  | 2, 4, 6, 8, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    Uart0DefaultMPF();

    /* Set PB.2 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE2_Msk);

    /* Configure the EADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB2MFP_Msk) | SYS_GPB_MFPL_PB2MFP_EADC0_CH2;

    /* Disable the digital input path to avoid the leakage current for EADC analog input pins. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);  /* Disable PB2 */
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    int32_t  i32ConversionData;
    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;

    u32IntNum = 0;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 1;   /* Use Sample Module 1 */
    u32ChannelNum = 2;
    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                   EADC compare function sample code                  |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC, 0);

    while (1)
    {
        printf("Select test items:\n");
        printf("  [1] Basic EADC compare function\n");
        printf("  [2] EADC compare window mode\n");
        printf("  Other keys: exit EADC test\n");

        uint8_t  u8Option;
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Configure the sample module for analog input channel and software trigger source. */
            EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

            /* Set sample module external sampling time to 10 */
            EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable EADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
            printf("Set the compare condition of comparator 0:\n  channel %u is less than 0x800; match count is 1.\n", u32ChannelNum);
            EADC_ENABLE_CMP0(EADC, u32ModuleNum, EADC_CMP_CMPCOND_LESS_THAN, 0x800, 1);

            /* Enable EADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
            printf("Set the compare condition of comparator 1:\n  channel %u is greater than or equal to 0x800; match count is 1.\n", u32ChannelNum);
            EADC_ENABLE_CMP1(EADC, u32ModuleNum, EADC_CMP_CMPCOND_GREATER_OR_EQUAL, 0x800, 1);

            /* Enable the sample module interrupt.  */
            EADC_ENABLE_INT(EADC, u32IntMask);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
            NVIC_EnableIRQ(EADC_INT0_IRQn);

            /* Config ADINT3 for EADC comparator */

            /* Clear the ADINT3 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF3_Msk);

            /* Enable the EADC interrupt 3 for all comparators */
            EADC_ENABLE_INT(EADC, BIT3);
            NVIC_EnableIRQ(EADC_INT3_IRQn);

            /* Clear the EADC comparator 0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADCMPF0_Msk);
            /* Enable EADC comparator 0 interrupt */
            EADC_ENABLE_CMP_INT(EADC, 0);

            /* Clear the EADC comparator 1 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADCMPF1_Msk);
            /* Enable EADC comparator 1 interrupt */
            EADC_ENABLE_CMP_INT(EADC, 1);

            /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
            g_u32EadcInt0Flag = 0;
            g_u32EadcCmp0IntFlag = 0;
            g_u32EadcCmp1IntFlag = 0;
            EADC_START_CONV(EADC, u32ModuleMask);    /* software trigger sample module 4 */

            /* Wait EADC interrupt (g_u32EadcInt0Flag will be set at EADC_INT0_IRQHandler() function) */
            while (g_u32EadcInt0Flag == 0);

            /* Get the conversion result of the sample module */
            i32ConversionData = EADC_GET_CONV_DATA(EADC, u32ModuleNum);
            printf("Conversion result of channel %u: 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);

            /* Wait EADC compare interrupt */
            while ((g_u32EadcCmp0IntFlag == 0) && (g_u32EadcCmp1IntFlag == 0));

            if (g_u32EadcCmp0IntFlag == 1)
            {
                printf("Comparator 0 interrupt occurs.\nThe conversion result of channel %u is less than 0x800\n\n", u32ChannelNum);
            }

            if (g_u32EadcCmp1IntFlag == 1)
            {
                printf("Comparator 1 interrupt occurs.\nThe conversion result of channel %u is greater than or equal to 0x800\n\n", u32ChannelNum);
            }

            /* Disable compare function */
            EADC_DISABLE_CMP0(EADC);
            EADC_DISABLE_CMP1(EADC);

            /* Disable the sample module interrupt.  */
            EADC_DISABLE_INT(EADC, u32IntMask);
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
            NVIC_DisableIRQ(EADC_INT0_IRQn);

            /* Disable the EADC interrupt 3 for all comparators */
            EADC_DISABLE_INT(EADC, BIT3);
            NVIC_DisableIRQ(EADC_INT3_IRQn);

            /* Disable EADC comparator interrupt */
            EADC_DISABLE_CMP_INT(EADC, 0);
            EADC_DISABLE_CMP_INT(EADC, 1);
        }
        else if (u8Option == '2')
        {
            /* Configure the sample module for analog input channel and software trigger source. */
            EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable EADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
            printf("Set the compare condition of comparator 0:\n  channel %u is less than 0x800; match count is 1.\n", u32ChannelNum);
            EADC_ENABLE_CMP0(EADC, u32ModuleNum, EADC_CMP_CMPCOND_LESS_THAN, 0x800, 1);

            /* Enable EADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
            printf("Set the compare condition of comparator 1:\n  channel %u is greater than or equal to 0x200; match count is 1.\n", u32ChannelNum);
            EADC_ENABLE_CMP1(EADC, u32ModuleNum, EADC_CMP_CMPCOND_GREATER_OR_EQUAL, 0x200, 1);

            printf("Enable Compare Window Mode.\n  Compare interrupt occurred only if both comparator 0 and 1 are match.\n");
            EADC_ENABLE_CMP_WINDOW_MODE(EADC, 0);

            /* Enable the sample module interrupt.  */
            EADC_ENABLE_INT(EADC, u32IntMask);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
            NVIC_EnableIRQ(EADC_INT0_IRQn);

            /* Config ADINT3 for EADC comparator */

            /* Clear the ADINT3 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF3_Msk);

            /* Enable the EADC interrupt 3 for all comparators */
            EADC_ENABLE_INT(EADC, BIT3);
            NVIC_EnableIRQ(EADC_INT3_IRQn);

            /* Clear the EADC comparator 0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADCMPF0_Msk);
            /* Enable EADC comparator 0 interrupt */
            EADC_ENABLE_CMP_INT(EADC, 0);

            /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
            g_u32EadcInt0Flag = 0;
            g_u32EadcCmp0IntFlag = 0;
            EADC_START_CONV(EADC, u32ModuleMask);    /* software trigger sample module 4 */

            /* Wait EADC interrupt (g_u32EadcInt0Flag will be set at EADC_INT0_IRQHandler() function) */
            while (g_u32EadcInt0Flag == 0);

            /* Get the conversion result of the sample module */
            i32ConversionData = EADC_GET_CONV_DATA(EADC, u32ModuleNum);
            printf("Conversion result of channel %u: 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);

            if (g_u32EadcCmp0IntFlag == 1)
            {
                printf("Comparator 0 interrupt occurs.\nThe conversion result of channel %u is < 0x800 and >= 0x200\n\n", u32ChannelNum);
            }
            else
            {
                printf("Comparator 0 interrupt don't occurs.\nThe conversion result of channel %u is >= 0x800 or < 0x200\n\n", u32ChannelNum);
            }

            /* Disable compare function */
            EADC_DISABLE_CMP0(EADC);
            EADC_DISABLE_CMP1(EADC);

            EADC_DISABLE_CMP_WINDOW_MODE(EADC, 0);

            /* Disable the sample module interrupt */
            EADC_DISABLE_INT(EADC, u32IntMask);
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
            NVIC_DisableIRQ(EADC_INT0_IRQn);

            /* Disable the EADC interrupt 3 for all comparators */
            EADC_DISABLE_INT(EADC, BIT3);
            NVIC_DisableIRQ(EADC_INT3_IRQn);

            /* Disable EADC comparator interrupt */
            EADC_DISABLE_CMP_INT(EADC, 0);
        }
        else
            break;
    }   /* End of while(1) */

    /* Disable the A/D converter */
    EADC_Close(EADC);
}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_INT0_IRQHandler(void)
{
    g_u32EadcInt0Flag = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
}


void EADC_INT3_IRQHandler(void)
{
    if (EADC_GET_INT_FLAG(EADC, EADC_STATUS2_ADCMPF0_Msk))
    {
        g_u32EadcCmp0IntFlag = 1;
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADCMPF0_Msk);
    }

    if (EADC_GET_INT_FLAG(EADC, EADC_STATUS2_ADCMPF1_Msk))
    {
        g_u32EadcCmp1IntFlag = 1;
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADCMPF1_Msk);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %u Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
