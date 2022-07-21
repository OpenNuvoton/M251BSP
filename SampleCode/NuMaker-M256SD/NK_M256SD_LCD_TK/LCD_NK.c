/**************************************************************************//**
 * @file     LCD_NK.c
 * @version  V1.00
 * @brief    LCD Initial & Frame config.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"
#include "NU_M256SD.h"
#include "tklib.h"
#include "lcdlib.h"

static S_LCD_CFG_T g_LCDCfg =
{
    __LIRC,                     /*!< LCD clock source frequency */
    LCD_COM_DUTY_1_8,           /*!< COM duty */
    LCD_BIAS_LV_1_4,            /*!< Bias level */
    64,                         /*!< Operation frame rate */
    LCD_WAVEFORM_TYPE_A_NORMAL, /*!< Waveform type */
    LCD_DISABLE_ALL_INT,        /*!< Interrupt source */
    LCD_HIGH_DRIVING_OFF_AND_BUF_ON, /*!< Driving mode */
    LCD_VOLTAGE_SOURCE_CP,      /*!< Voltage source */
};

void LCD_Init(void)
{
    uint32_t u32ActiveFPS;

    /* Configure LCD multi-function pins */
    /*
        Summary of LCD pin usage:
            COM 0~3   : PB.5, PB.4, PB.3, PB.2
            COM 4~7   : PC.5, PC.4, PC.3, PC.2
            SEG 0~1   : PB.0, PB.1
            SEG 2~3   : PC.9, PC.10
            SEG 4~13  : PB.6, PB.7, PB.8, PB.9, PB.10, PB.11, PB.12, PB.13, PB.14, PB.15
            SEG 14    : PC.14
            SEG 17~20 : PA.15, PA.14, PA.13, PA.12
            SEG 22~25 : PD.0, PD.1, PD.2, PD.3
            SEG 26~27 : PC.0, PC.1,
            SEG 35    : PF.15
            SEG 36~37 : PA.6, PA.7
            SEG 38~39 : PC.6, PC.7
    */
    /* COM 0~3 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk)) |
                    (SYS_GPB_MFPL_PB5MFP_LCD_COM0 | SYS_GPB_MFPL_PB4MFP_LCD_COM1 | SYS_GPB_MFPL_PB3MFP_LCD_COM2 | SYS_GPB_MFPL_PB2MFP_LCD_COM3);
    /* COM 4~7 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk)) |
                    (SYS_GPC_MFPL_PC2MFP_LCD_COM7 | SYS_GPC_MFPL_PC3MFP_LCD_COM6 | SYS_GPC_MFPL_PC4MFP_LCD_COM5 | SYS_GPC_MFPL_PC5MFP_LCD_COM4);

    /* SEG 0~1 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk)) |
                    (SYS_GPB_MFPL_PB0MFP_LCD_SEG0 | SYS_GPB_MFPL_PB1MFP_LCD_SEG1);

    /* SEG 4~13 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk)) |
                    (SYS_GPB_MFPL_PB6MFP_LCD_SEG4 | SYS_GPB_MFPL_PB7MFP_LCD_SEG5);

    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB8MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk | SYS_GPB_MFPH_PB10MFP_Msk | SYS_GPB_MFPH_PB11MFP_Msk |
                                       SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB15MFP_Msk)) |
                    (SYS_GPB_MFPH_PB8MFP_LCD_SEG6 | SYS_GPB_MFPH_PB9MFP_LCD_SEG7 | SYS_GPB_MFPH_PB10MFP_LCD_SEG8 | SYS_GPB_MFPH_PB11MFP_LCD_SEG9 |
                     SYS_GPB_MFPH_PB14MFP_LCD_SEG12 | SYS_GPB_MFPH_PB15MFP_LCD_SEG13);

    /* SEG 14 */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC14MFP_Msk)) | (SYS_GPC_MFPH_PC14MFP_LCD_SEG14);

    /* SEG 17~20 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA15MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA12MFP_Msk)) |
                    (SYS_GPA_MFPH_PA15MFP_LCD_SEG17 | SYS_GPA_MFPH_PA14MFP_LCD_SEG18 | SYS_GPA_MFPH_PA13MFP_LCD_SEG19 | SYS_GPA_MFPH_PA12MFP_LCD_SEG20);

    /* SEG 22~25 */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk)) |
                    (SYS_GPD_MFPL_PD0MFP_LCD_SEG22 | SYS_GPD_MFPL_PD1MFP_LCD_SEG23 | SYS_GPD_MFPL_PD2MFP_LCD_SEG24 | SYS_GPD_MFPL_PD3MFP_LCD_SEG25);
    /* SEG 26~27 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_LCD_SEG26 | SYS_GPC_MFPL_PC1MFP_LCD_SEG27);

    /* SEG 35 is used for connnecting with the other unused SEG to avoid floating*/
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
    LCD_OUTPUT_SET(LCD_OUTPUT_SEL24_TO_COM4 | LCD_OUTPUT_SEL25_TO_COM5 | LCD_OUTPUT_SEL26_TO_COM6 | LCD_OUTPUT_SEL27_TO_COM7 |
                   LCD_OUTPUT_SEL35_TO_SEG20 | LCD_OUTPUT_SEL36_TO_SEG19 | LCD_OUTPUT_SEL37_TO_SEG18 | LCD_OUTPUT_SEL38_TO_SEG17);

    /* LCD Initialize and calculate real frame rate */
    LCD_Open(&g_LCDCfg);

    /* Select output voltage level 9 for 4.8V */
    LCD_SET_CP_VOLTAGE(LCD_CP_VOLTAGE_LV_9);
}

//----------------------------------------------------------------------------------------------//
void LCD_Init_Setting(void)
{
    /* Init LCD multi-function pins and settings */
    LCD_Init();

    /* Enable LCD display */
    LCD_ENABLE_DISPLAY();
}

void LCD_frame1(void)
{
    /* Show logo and battery level */
    LCDLIB_SetSymbol(SYMBOL_NVT_14, 0);
    LCDLIB_SetSymbol(SYMBOL_BAT_FRAME_18, 1);
    LCDLIB_SetSymbol(SYMBOL_BAT_1_18, 1);
    LCDLIB_SetSymbol(SYMBOL_BAT_2_18, 0);
    LCDLIB_SetSymbol(SYMBOL_BAT_3_18, 0);

    /*Show Char */
    LCDLIB_Printf(ZONE_MAIN_DIGIT, "*M256*");

    /* Show counter */
    LCDLIB_PrintNumber(ZONE_PPM_DIGIT, 888);

    /* Show temperature */
    LCDLIB_PrintNumber(ZONE_TEMP_DIGIT, 555);
    LCDLIB_SetSymbol(SYMBOL_TEMP_C_35, 0);

    /* Show bsp version */
    LCDLIB_PrintNumber(ZONE_VER_DIGIT, 0);
    LCDLIB_SetSymbol(SYMBOL_VERSION_37, 1);
}

void LCD_frame2(void)
{
    unsigned long temp;

    /* Show logo and battery level */
    LCDLIB_SetSymbol(SYMBOL_NVT_14, 1);
    LCDLIB_SetSymbol(SYMBOL_BAT_FRAME_18, 1);
    LCDLIB_SetSymbol(SYMBOL_BAT_1_18, 1);
    LCDLIB_SetSymbol(SYMBOL_BAT_2_18, 1);
    LCDLIB_SetSymbol(SYMBOL_BAT_3_18, 1);

    /* Show Char */
    LCDLIB_Printf(ZONE_MAIN_DIGIT, "PDSTAT");

    temp = internal_Temperature();
    LCDLIB_PrintNumber(ZONE_TEMP_DIGIT, temp);
    LCDLIB_SetSymbol(SYMBOL_TEMP_C_35, 1);

    /* Show counter */
    LCDLIB_PrintNumber(ZONE_PPM_DIGIT, tkct);

    /* Show bsp version */
    LCDLIB_PrintNumber(ZONE_VER_DIGIT, (TKLIB_MAJOR_VERSION * 100000) + (TKLIB_MINOR_VERSION * 1000) + TOUCHKEY_VERSION);
    LCDLIB_SetSymbol(SYMBOL_VERSION_37, 1);
}
