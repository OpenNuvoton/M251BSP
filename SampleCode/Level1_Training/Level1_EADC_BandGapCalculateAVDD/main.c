/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to calculate battery voltage (AVdd) by using band-gap.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define EADC_GET_VALID_VALUE(CONVERT_DATA)       ((CONVERT_DATA)&0xFFF)

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;

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

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is PCLK1, set divider to 3, ADC clock is PCLK1/3 MHz */
    /* Note: The EADC_CLK speed should meet datasheet spec (<16MHz) and rules in following table.   */
    /* +--------------+------------------+                                                          */
    /* | PCLK divider | EADC_CLK divider |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 1            | 1, 2, 3, 4, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 2, 4, 8, 16  | 2, 4, 6, 8, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(3));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    Uart0DefaultMPF();
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
/* EADC function test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    int32_t  i32ConversionData, i32BuiltInData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|   ADC for calculate battery voltage (AVdd) by using band-gap test    |\n");
    printf("+----------------------------------------------------------------------+\n\n");


    printf("+----------------------------------------------------------------------+\n");
    printf("|   ADC clock source -> PCLK1  = 48 MHz                                |\n");
    printf("|   ADC clock divider          = 3                                     |\n");
    printf("|   ADC clock                  = 48 MHz / 3 = 16 MHz                   |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, 0);

    /* Set sample module 16 external sampling time to 160 (M251 suggest total sample band-gap more than 10 us)*/
    EADC_SetExtendSampleTime(EADC, 16, 160);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 16 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT0);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT16);//Enable sample module 16 interrupt.
    NVIC_EnableIRQ(EADC_INT0_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 16 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC, BIT16);

    /* Wait EADC conversion done */
    while (g_u32AdcIntFlag == 0);

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, BIT0);

    /* Get the conversion result of the sample module 16 */
    i32ConversionData = EADC_GET_CONV_DATA(EADC, 16);

    /* Enable FMC ISP function to read built-in band-gap A/D conversion result*/
    SYS_UnlockReg();
    FMC_Open();
    i32BuiltInData = FMC_ReadVBGCode();


    /* Use Conversion result of Band-gap to calculating AVdd */


    printf("      AVdd           i32BuiltInData                   \n");
    printf("   ---------- = -------------------------             \n");
    printf("      %d          i32ConversionData                   \n", (i32BuiltInData & 0x01000000) ? 3072 : 3300);
    printf("                                                      \n");
    printf("AVdd =  %d * i32BuiltInData / i32ConversionData     \n\n", (i32BuiltInData & 0x01000000) ? 3072 : 3300);


    printf("Built-in band-gap A/D conversion result: 0x%X (%d) \n", EADC_GET_VALID_VALUE(i32BuiltInData), EADC_GET_VALID_VALUE(i32BuiltInData));
    printf("Conversion result of Band-gap:           0x%X (%d) \n\n", i32ConversionData, i32ConversionData);

    if (i32BuiltInData & 0x01000000)
        printf("AVdd = 3072 * %d / %d = %d mV \n\n", EADC_GET_VALID_VALUE(i32BuiltInData), i32ConversionData, 3072 * EADC_GET_VALID_VALUE(i32BuiltInData) / i32ConversionData);
    else
        printf("AVdd = 3300 * %d / %d = %d mV \n\n", EADC_GET_VALID_VALUE(i32BuiltInData), i32ConversionData, 3300 * EADC_GET_VALID_VALUE(i32BuiltInData) / i32ConversionData);

    FMC_Close();
    FMC_Close();
    SYS_LockReg();
}



/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_INT0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
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

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while (1);

}
