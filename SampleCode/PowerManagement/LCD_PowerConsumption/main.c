/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the LCD run in low power consumption
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "lcdlib.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

#define PD_MODE   0     // Power-down mode
#define FWPD_MODE 1     // Fast wake up
#define DPD_MODE  2     // Deep Power-down mode

#define T_60SEC   60    //60Sec
#define T_60MIN   60    //60Min

/*---------------------------------------------------------------------------------------------------------*/
/*                                       Global variables                                                  */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t  g_i32Alarm  = FALSE;
volatile int32_t  g_i32WakeUp = FALSE;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
    uint32_t Power_Down_Mode;
    uint32_t LCD_Power_Saving_Mode;
    uint32_t LCD_Power_Saving_Level;
    uint32_t AlarmTime;
} CONFIG_INFO_T;

const CONFIG_INFO_T User_Config =
{
    PD_MODE,                            /*!< Power Down Mode */
    LCD_PWR_SAVING_BUF_MODE,            /*!< LCD Power Saving Mode */
    LCD_PWR_SAVING_LEVEL3,              /*!< LCD Power Saving Level */
    10,                                 /*!< Alarm Time Setting(Sec) */
};

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

/*---------------------------------------------------------------------------------------------------------*/
/*                            Define functions prototype                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void EnterToPowerDown(uint32_t u32PDMode);
void PWRWU_IRQHandler(void);
void RTC_IRQHandler(void);
void LXT_Enable(void);
void SYS_Init(void);
void LCD_Init(void);
int32_t main(void);

/**
  * @brief      Enter To Power Down
  * @param[in]   u32PDMode    The specified Power down module.
  *                               - \ref CLK_PMUCTL_PDMSEL_PD      : Power-down mode
  *                               - \ref CLK_PMUCTL_PDMSEL_FWPD    : Fast wake up
  *                               - \ref CLK_PMUCTL_PDMSEL_DPD     : Deep Power-down mode
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
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_FWPD;  //fast up
    else if ((u32PDMode == DPD_MODE))
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_DPD | CLK_PMUCTL_RTCWKEN_Msk; // DPD(Deep power dwon) Mode and RTC WK enable

    CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);
    CLK->PWRCTL |=  CLK_PWRCTL_PDWKIEN_Msk;
    CLK_PowerDown();
    SYS_LockReg();

    while (g_i32WakeUp == FALSE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         PWRWU  Handle                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    g_i32WakeUp = TRUE;
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;
}

/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    if ((RTC->INTEN & RTC_INTEN_ALMIEN_Msk) && (RTC->INTSTS & RTC_INTSTS_ALMIF_Msk))          /* alarm interrupt occurred */
    {
        g_i32Alarm = TRUE;
        RTC_CLEAR_ALARM_INT_FLAG();
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*                           Enable the external 32768Hz XTAL Clock                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LXT_Enable(void)
{
    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk);

    /* Enable external 32768Hz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Disable digital input path of analog pin X32_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 4));

    /* Disable digital input path of analog pin XT32_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 5));
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                          Init System Clock                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
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

    /* Select HIRC as the clock source of UART */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

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

    /* Select HXT as the clock source of UART */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));
#endif

    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /*Enable external 32768Hz XTAL */
    LXT_Enable();

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Configure LCD module clock */
    CLK_EnableModuleClock(LCD_MODULE);

    /* Select LCD Clock Source */
    CLK_SetModuleClock(LCD_MODULE, CLK_CLKSEL2_LCDSEL_LIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                      Init I/O Multi-function                                            */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();

}
/*---------------------------------------------------------------------------------------------------------*/
/*                                       Init UART                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                       Init LCD                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void LCD_Init(void)
{
    uint32_t u32ActiveFPS;

    /* Configure LCD multi-function pins */
#ifdef M256D
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

#else
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
#endif
    /* LCD Initialize and calculate real frame rate */
    u32ActiveFPS = LCD_Open(&g_LCDCfg);
    printf("Working frame rate is %uHz on Type-%c.\n\n", u32ActiveFPS, (g_LCDCfg.u32WaveformType == LCD_PSET_TYPE_Msk) ? 'B' : 'A');

    /* Select output voltage level 9 for 4.8V */
    LCD_SET_CP_VOLTAGE(LCD_CP_VOLTAGE_LV_9);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                     MAIN function                                                       */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    S_RTC_TIME_DATA_T sInitTime, *sInitTime_ptr;
    S_RTC_TIME_DATA_T sCurTime;
    char text[] = "NUVOTON";

    /* Init System, peripheral clock */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);

    /* Init LCD multi-function pins and settings */
    LCD_Init();

    /* Enable LCD display */
    LCD_ENABLE_DISPLAY();

    /* Set LCD power saving mode and level */
    LCD_SetSavingMode(User_Config.LCD_Power_Saving_Mode, User_Config.LCD_Power_Saving_Level);

    /* Set specified text on LCD */
    LCDLIB_Printf(ZONE_MAIN_DIGIT, text);

    /* Time setting */
    sInitTime.u32Year       = 2022;
    sInitTime.u32Month      = 5;
    sInitTime.u32Day        = 1;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 0;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_TUESDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    /* check rtc reset status */
    sInitTime_ptr = (RTC->INIT & RTC_INIT_ACTIVE_Msk) ? NULL : &sInitTime;

    if (RTC_Open(sInitTime_ptr) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");

        while (1);
    }

    printf("\n[ RTC Alarm Interrupt Wake-up Test ](Alarm after %d seconds)\n\n", User_Config.AlarmTime);

    g_i32Alarm = FALSE;

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf(" Current Time:%u/%02u/%02u %02u:%02u:%02u\n", sCurTime.u32Year, sCurTime.u32Month,
           sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

    /* Enable RTC Alarm Interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    NVIC_EnableIRQ(RTC_IRQn);

    NVIC_EnableIRQ(PWRWU_IRQn);

    while (1)
    {
        /* Wait the UART FIFO buffer is empty */
        while (!IsDebugFifoEmpty()) {};

        /* The alarm time setting */
        sCurTime.u32Second = sCurTime.u32Second + User_Config.AlarmTime;

        /* Remainder processing of the seconds*/
        if (sCurTime.u32Second >= T_60SEC)
        {
            sCurTime.u32Minute = sCurTime.u32Minute + (sCurTime.u32Second / T_60SEC);
            sCurTime.u32Second = sCurTime.u32Second % T_60SEC;
        }

        /* Remainder processing of the mintunes*/
        if (sCurTime.u32Minute >= T_60MIN)
        {
            sCurTime.u32Hour = sCurTime.u32Hour + (sCurTime.u32Minute / T_60SEC);
            sCurTime.u32Minute = sCurTime.u32Minute % T_60MIN;
        }

        /* Set the alarm time */
        RTC_SetAlarmDateAndTime(&sCurTime);

        /* Clear interrupt status */
        RTC->INTSTS = RTC_INTSTS_ALMIF_Msk;

        EnterToPowerDown(User_Config.Power_Down_Mode);

        printf(" RTC Wake Up success\n");

        /* IOCTLSEL will automatically be set by hardware to 1 when system power is off and RTC_GPIOCTL0 had been configured. */
        /* when system wake up from PD mode, you need clear this bit if you want PF.4, 5, 6 pin I/O function is controlled by GPIO module. */
        RTC->LXTCTL &= ~(RTC_LXTCTL_IOCTLSEL_Msk);

        while (!g_i32Alarm) {};

        /* Get the current time */
        RTC_GetDateAndTime(&sCurTime);

        printf(" Current Time:%u/%02u/%02u %02u:%02u:%02u\n", sCurTime.u32Year, sCurTime.u32Month,
               sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);
    }
}

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
