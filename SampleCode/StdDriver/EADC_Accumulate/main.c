/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to get accumulate conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32EadcInt0Flag;

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

    /* Set PB.0 and PB.1 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk);

    /* Configure the EADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB0MFP_Msk) | SYS_GPB_MFPL_PB0MFP_EADC0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB1MFP_Msk) | SYS_GPB_MFPL_PB1MFP_EADC0_CH1;

    /* Disable the digital input path to avoid the leakage current for EADC analog input pins. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1);  /* Disable PB0 and PB1 */
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
/* @brief Left shift Accumulate raw data back to correct conversion result                                 */
/* @param[in] accu_raw_result  Raw data of Accumulate conversion                                           */
/* @param[in] accu_count       Conversion count of Accumulate conversion.                                  */
/*                             Valid values are 1, 2, 4, 8, 16, 32, 64, 128, 256.                          */
/* @return The correct conversion result of Accumulate conversion                                          */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t EADC_Accumulate_LeftShift(uint32_t u32AccuRawResult, uint32_t u32AccuCount)
{
    switch (u32AccuCount)
    {
        case 1:
            return u32AccuRawResult;

        case 2:
            return u32AccuRawResult;

        case 4:
            return u32AccuRawResult;

        case 8:
            return u32AccuRawResult;

        case 16:
            return u32AccuRawResult;

        case 32:
            return (u32AccuRawResult << 1);

        case 64:
            return (u32AccuRawResult << 2);

        case 128:
            return (u32AccuRawResult << 3);

        case 256:
            return (u32AccuRawResult << 4);

        default:
            printf("*** Error! Wrong parameter %u for M251 Accumulate.\n\n", u32AccuCount);
            return u32AccuRawResult;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;

    u32IntNum = 0;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 1;   /* Use Sample Module 1 */

    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                   M251 EADC Accumulate sample code                   |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC, 0);

    while (1)
    {
        printf("Select test items:\n");
        printf("  [1] Basic EADC conversion (channel 0 only)\n");
        printf("  [2] Basic EADC conversion (channel 1 only)\n");
        printf("  Other keys: exit EADC test\n");

        uint8_t  u8Option;
        u8Option = getchar();

        if (u8Option == '1')
            u32ChannelNum = 0;
        else if (u8Option == '2')
            u32ChannelNum = 1;
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and software trigger source. */
        EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

        /* Set sample module external sampling time to 10 */
        EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

        /* Enable Accumulate feature */
        EADC_ENABLE_ACU(EADC, u32ModuleNum, EADC_MCTL1_ACU_8);
        /* Disable Average feature */
        EADC_DISABLE_AVG(EADC, u32ModuleNum);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC, u32IntMask);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
        NVIC_EnableIRQ(EADC_INT0_IRQn);

        /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
        g_u32EadcInt0Flag = 0;
        EADC_START_CONV(EADC, u32ModuleMask);

        /* Wait EADC interrupt (g_u32EadcInt0Flag will be set at EADC_INT0_IRQHandler() function) */
        while (g_u32EadcInt0Flag == 0) {};

        int32_t  i32ConversionData;

        /* Get the conversion result of the sample module */
        i32ConversionData = EADC_GET_CONV_DATA(EADC, u32ModuleNum);

        printf("Conversion result of channel %u: 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);

        i32ConversionData = EADC_Accumulate_LeftShift(i32ConversionData, 8);

        printf("Conversion result of channel %u after left shifted: 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);

        printf("The average that calculated by software is 0x%X (%d)\n\n", (int)(i32ConversionData / 8), (int)(i32ConversionData / 8));

        /* Disable Accumulate feature */
        EADC_DISABLE_ACU(EADC, u32ModuleNum);

        /* Disable the sample module interrupt.  */
        EADC_DISABLE_INT(EADC, u32IntMask);

        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);

        NVIC_DisableIRQ(EADC_INT0_IRQn);
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
