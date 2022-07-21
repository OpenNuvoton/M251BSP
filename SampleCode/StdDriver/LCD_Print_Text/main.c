/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to print text on RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

#include "lcdlib.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define LCD_ALPHABET_NUM    7
static S_LCD_CFG_T g_LCDCfg =
{
    __LIRC,                             /*!< LCD clock source frequency */
    LCD_COM_DUTY_1_8,                   /*!< COM duty */
    LCD_BIAS_LV_1_4,                    /*!< Bias level */
    64,                                 /*!< Operation frame rate */
    LCD_WAVEFORM_TYPE_A_NORMAL,         /*!< Waveform type */
    LCD_DISABLE_ALL_INT,                /*!< Interrupt source */
    LCD_HIGH_DRIVING_OFF_AND_BUF_ON,    /*!< Driving mode */
    LCD_VOLTAGE_SOURCE_CP,              /*!< Voltage source */
};

void LCD_Init(void);
void SYS_Init(void);
void UART_Init(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


void LCD_Init(void)
{
    uint32_t u32ActiveFPS;
    /*
        Summary of LCD pin usage:
            COM 0~3   : PB.5, PB.4, PB.3, PB.2
            COM 4~5   : PD.11, PD.10
            COM 6~7   : PE.13, PC.8
            SEG 0~1   : PB.0, PB.1
            SEG 2~3   : PC.9, PC.10
            SEG 4~13  : PB.6, PB.7, PB.8, PB.9, PB.10, PB.11, PB.12, PB.13, PB.14, PB.15
            SEG 14    : PC.14
            SEG 15~16 : PE.6, PE.7
            SEG 17~20 : PE.11, PE.10, PE.9, PE.8
            SEG 21~25 : PD.13, PD.0, PD.1, PD.2, PD.3
            SEG 26~30 : PC.0, PC.1, PC.2, PC.3, PC.4
            SEG 31    : PC.5
            SEG 32~33 : PD.8, PD.9
            SEG 34    : PE.14
            SEG 35    : PF.15
            SEG 36~37 : PA.6, PA.7
            SEG 38~39 : PC.6, PC.7
    */

    /* Configure LCD multi-function pins */

    /* COM 0~3 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk)) |
                    (SYS_GPB_MFPL_PB5MFP_LCD_COM0 | SYS_GPB_MFPL_PB4MFP_LCD_COM1 | SYS_GPB_MFPL_PB3MFP_LCD_COM2 | SYS_GPB_MFPL_PB2MFP_LCD_COM3);
    /* COM 4~5 */
    SYS->GPD_MFPH = (SYS->GPD_MFPH & ~(SYS_GPD_MFPH_PD10MFP_Msk | SYS_GPD_MFPH_PD11MFP_Msk)) |
                    (SYS_GPD_MFPH_PD11MFP_LCD_SEG43 | SYS_GPD_MFPH_PD10MFP_LCD_SEG42);
    /* COM 6 */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE13MFP_Msk)) |
                    (SYS_GPE_MFPH_PE13MFP_LCD_SEG41);
    /* COM 7 */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC8MFP_Msk)) |
                    (SYS_GPC_MFPH_PC8MFP_LCD_SEG40);
    /* SEG 0~1 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk)) |
                    (SYS_GPB_MFPL_PB0MFP_LCD_SEG0 | SYS_GPB_MFPL_PB1MFP_LCD_SEG1);
    /* SEG 2~3 */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC9MFP_Msk | SYS_GPC_MFPH_PC10MFP_Msk)) |
                    (SYS_GPC_MFPH_PC9MFP_LCD_SEG2 | SYS_GPC_MFPH_PC10MFP_LCD_SEG3);
    /* SEG 4~13 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk)) |
                    (SYS_GPB_MFPL_PB6MFP_LCD_SEG4 | SYS_GPB_MFPL_PB7MFP_LCD_SEG5);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB8MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk | SYS_GPB_MFPH_PB10MFP_Msk | SYS_GPB_MFPH_PB11MFP_Msk |
                                       SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB15MFP_Msk)) |
                    (SYS_GPB_MFPH_PB8MFP_LCD_SEG6 | SYS_GPB_MFPH_PB9MFP_LCD_SEG7 | SYS_GPB_MFPH_PB10MFP_LCD_SEG8 | SYS_GPB_MFPH_PB11MFP_LCD_SEG9 |
                     SYS_GPB_MFPH_PB12MFP_LCD_SEG10 | SYS_GPB_MFPH_PB13MFP_LCD_SEG11 | SYS_GPB_MFPH_PB14MFP_LCD_SEG12 | SYS_GPB_MFPH_PB15MFP_LCD_SEG13);
    /* SEG 14 */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC14MFP_Msk)) | (SYS_GPC_MFPH_PC14MFP_LCD_SEG14);
    /* SEG 15~16 */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE6MFP_Msk | SYS_GPE_MFPL_PE7MFP_Msk)) | (SYS_GPE_MFPL_PE6MFP_LCD_SEG15 | SYS_GPE_MFPL_PE7MFP_LCD_SEG16);
    /* SEG 17~20 */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE11MFP_Msk | SYS_GPE_MFPH_PE10MFP_Msk | SYS_GPE_MFPH_PE9MFP_Msk | SYS_GPE_MFPH_PE8MFP_Msk)) |
                    (SYS_GPE_MFPH_PE11MFP_LCD_SEG17 | SYS_GPE_MFPH_PE10MFP_LCD_SEG18 | SYS_GPE_MFPH_PE9MFP_LCD_SEG19 | SYS_GPE_MFPH_PE8MFP_LCD_SEG20);
    /* SEG 21~25 */
    SYS->GPD_MFPH = (SYS->GPD_MFPH & ~(SYS_GPD_MFPH_PD13MFP_Msk)) | (SYS_GPD_MFPH_PD13MFP_LCD_SEG21);
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk)) |
                    (SYS_GPD_MFPL_PD0MFP_LCD_SEG22 | SYS_GPD_MFPL_PD1MFP_LCD_SEG23 | SYS_GPD_MFPL_PD2MFP_LCD_SEG24 | SYS_GPD_MFPL_PD3MFP_LCD_SEG25);
    /* SEG 26~31 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk |
                                       SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_LCD_SEG26 | SYS_GPC_MFPL_PC1MFP_LCD_SEG27 | SYS_GPC_MFPL_PC2MFP_LCD_SEG28 | SYS_GPC_MFPL_PC3MFP_LCD_SEG29 |
                     SYS_GPC_MFPL_PC4MFP_LCD_SEG30 | SYS_GPC_MFPL_PC5MFP_LCD_SEG31);
    /* SEG 32~33 */
    SYS->GPD_MFPH = (SYS->GPD_MFPH & ~(SYS_GPD_MFPH_PD8MFP_Msk | SYS_GPD_MFPH_PD9MFP_Msk)) |
                    (SYS_GPD_MFPH_PD8MFP_LCD_SEG32 | SYS_GPD_MFPH_PD9MFP_LCD_SEG33);
    /* SEG 34 */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE14MFP_Msk)) | (SYS_GPE_MFPH_PE14MFP_LCD_SEG34);
    /* SEG 35 */
    SYS->GPF_MFPH = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF15MFP_Msk)) | (SYS_GPF_MFPH_PF15MFP_LCD_SEG35);
    /* SEG 36~37 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk)) |
                    (SYS_GPA_MFPL_PA6MFP_LCD_SEG36 | SYS_GPA_MFPL_PA7MFP_LCD_SEG37);
    /* SEG 38~39 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk)) |
                    (SYS_GPC_MFPL_PC6MFP_LCD_SEG38 | SYS_GPC_MFPL_PC7MFP_LCD_SEG39);

    /* Reset LCD module */
    SYS_ResetModule(LCD_RST);

    /* Output Setting Select */
    LCD_OUTPUT_SET(LCD_OUTPUT_SEL8_TO_COM4 | LCD_OUTPUT_SEL9_TO_COM5 | LCD_OUTPUT_SEL10_TO_SEG20 | LCD_OUTPUT_SEL11_TO_SEG19 |
                   LCD_OUTPUT_SEL12_TO_SEG18 | LCD_OUTPUT_SEL13_TO_SEG17 | LCD_OUTPUT_SEL14_TO_COM6 | LCD_OUTPUT_SEL15_TO_COM7 |
                   LCD_OUTPUT_SEL24_TO_SEG31 | LCD_OUTPUT_SEL25_TO_SEG30 | LCD_OUTPUT_SEL26_TO_SEG29 | LCD_OUTPUT_SEL27_TO_SEG28 |
                   LCD_OUTPUT_SEL28_TO_SEG27 | LCD_OUTPUT_SEL29_TO_SEG26 | LCD_OUTPUT_SEL41_TO_SEG14 | LCD_OUTPUT_SEL42_TO_SEG13 |
                   LCD_OUTPUT_SEL47_TO_SEG8 | LCD_OUTPUT_SEL48_TO_SEG7 | LCD_OUTPUT_SEL49_TO_SEG6);

    /* LCD Initialize and calculate real frame rate */
    u32ActiveFPS = LCD_Open(&g_LCDCfg);
    printf("Working frame rate is %uHz on Type-%c.\n\n", u32ActiveFPS, (g_LCDCfg.u32WaveformType == LCD_PSET_TYPE_Msk) ? 'B' : 'A');

    /* Select output voltage level 9 for 4.8V */
    LCD_SET_CP_VOLTAGE(LCD_CP_VOLTAGE_LV_9);
}

void SYS_Init(void)
{
    /* Enable all GPIO clock */
    CLK->AHBCLK |= (CLK_AHBCLK_GPACKEN_Msk | CLK_AHBCLK_GPBCKEN_Msk | CLK_AHBCLK_GPCCKEN_Msk | CLK_AHBCLK_GPDCKEN_Msk |
                    CLK_AHBCLK_GPECKEN_Msk | CLK_AHBCLK_GPFCKEN_Msk);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set SysTick source to HCLK/2*/
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);


#if !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING)
    /* Configure UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
#endif

    /* Configure LCD module clock */
    CLK_EnableModuleClock(LCD_MODULE);
    CLK_SetModuleClock(LCD_MODULE, CLK_CLKSEL2_LCDSEL_LIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
#if !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING)
    Uart0DefaultMPF();
#endif
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    char input, text[LCD_ALPHABET_NUM + 1];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
#if !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING)
    UART_Init();
#endif

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    printf("+----------------------------------+\n");
    printf("|    LCD Print Text Sample Code    |\n");
    printf("+----------------------------------+\n\n");

    printf("LCD configurations:\n");
    printf(" * Clock source is LIRC\n");
    printf(" * 8 COM, 40 SEG and 1/4 Bias\n");
    printf(" * Driving waveform is Type-%c\n", (g_LCDCfg.u32WaveformType == LCD_PSET_TYPE_Msk) ? 'B' : 'A');
    printf(" * Target frame rate is %uHz\n\n", g_LCDCfg.u32Framerate);

    /* Init LCD multi-function pins and settings */
    LCD_Init();

    /* Enable LCD display */
    LCD_ENABLE_DISPLAY();

    while (1)
    {
        uint32_t idx = 0;

        printf("Input text: \n");

        do
        {
            input = (char)getchar();
            putchar(input);

            if (input == 0xD) // "ENTER" key to exit current input
                break;

            text[idx++] = input;
        } while (idx < LCD_ALPHABET_NUM);

        text[idx] = 0x00;   // C string ended with 0x00
        printf("\n");

        LCDLIB_Printf(ZONE_MAIN_DIGIT, text);
    }
}
