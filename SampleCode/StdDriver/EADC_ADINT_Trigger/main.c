/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Trigger EADC by ADINT interrupt.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32EadcInt0Flag, g_u32EadcInt1Flag, g_u32EadcInt2Flag;

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

    /* Set PB.0 and PB.1 and PB.3 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE3_Msk);

    /* Configure the EADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB0MFP_Msk) | SYS_GPB_MFPL_PB0MFP_EADC0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB1MFP_Msk) | SYS_GPB_MFPL_PB1MFP_EADC0_CH1;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB3MFP_Msk) | SYS_GPB_MFPL_PB3MFP_EADC0_CH3;

    /* Disable the digital input path to avoid the leakage current for EADC analog input pins. */
    GPIO_DISABLE_DIGITAL_PATH(PB, (uint32_t)(BIT0 | BIT1 | BIT3));  /* Disable PB0 and PB1 and PB3 */
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

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                    EADC ADINT trigger sample code                    |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC, 0);

    while (1)
    {
        printf("Select test items:\n");
        printf("  [1] Trigger by ADINT0\n");
        printf("  [2] Trigger by ADINT0 and then ADINT1\n");
        printf("  Other keys: exit EADC test\n");
        uint8_t  u8Option;
        u8Option = getchar();

        if (u8Option == '1')
        {
            printf("Config EADC sample modules as below:\n");

            u32IntNum = 0;
            u32ModuleNum = 0;
            u32ChannelNum = 0;
            u32IntMask = (BIT0 << u32IntNum);
            u32ModuleMask = (BIT0 << u32ModuleNum);
            printf("Software trigger sample module %u, channel %u and generate interrupt ADINT%u.\n", u32ModuleNum, u32ChannelNum, u32IntNum);
            /* Configure the sample module for analog input channel and software trigger source. */
            EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

            /* Set sample module external sampling time to 10 */
            EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module interrupt.  */
            EADC_ENABLE_INT(EADC, u32IntMask);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
            NVIC_EnableIRQ(EADC_INT0_IRQn);

            u32IntNum = 1;
            u32ModuleNum = 1;
            u32ChannelNum = 1;
            u32IntMask = (BIT0 << u32IntNum);
            u32ModuleMask = (BIT0 << u32ModuleNum);
            printf("ADINT0   trigger sample module %u, channel %u and generate interrupt ADINT%u.\n", u32ModuleNum, u32ChannelNum, u32IntNum);
            /* Configure the sample module for analog input channel and software trigger source. */
            EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_ADINT0_TRIGGER, u32ChannelNum);

            /* Set sample module external sampling time to 10 */
            EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

            /* Clear the A/D ADINT1 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

            /* Enable the sample module interrupt.  */
            EADC_ENABLE_INT(EADC, u32IntMask);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
            NVIC_EnableIRQ(EADC_INT1_IRQn);

            printf("Config EADC sample modules completed !\n");

            /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
            g_u32EadcInt0Flag = 0;
            g_u32EadcInt1Flag = 0;
            printf("Software trigger sample module 0 here !\n");
            EADC_START_CONV(EADC, BIT0);    /* software trigger sample module 0 */

            /* Wait EADC interrupt (g_u32EadcInt0Flag will be set at EADC_INT0_IRQHandler() function) */
            while (g_u32EadcInt0Flag == 0);


            /* Wait EADC interrupt (g_u32EadcInt1Flag will be set at EADC_INT1_IRQHandler() function) */
            while (g_u32EadcInt1Flag == 0);


            /* Get the conversion result of the sample module */
            i32ConversionData = EADC_GET_CONV_DATA(EADC, 0);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 0, i32ConversionData, i32ConversionData);

            /* Get the conversion result of the sample module */
            i32ConversionData = EADC_GET_CONV_DATA(EADC, 1);
            printf("Conversion result of channel %d: 0x%X (%d)\n\n", 1, i32ConversionData, i32ConversionData);

            /* Disable the ADINTx interrupt */
            EADC_DISABLE_INT(EADC, BIT0 | BIT1);
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0 << 0);
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 1, BIT0 << 1);
            NVIC_DisableIRQ(EADC_INT0_IRQn);
            NVIC_DisableIRQ(EADC_INT1_IRQn);
        }
        else if (u8Option == '2')
        {
            printf("Config EADC sample modules as below:\n");

            u32IntNum = 0;
            u32ModuleNum = 0;
            u32ChannelNum = 0;
            u32IntMask = (BIT0 << u32IntNum);
            u32ModuleMask = (BIT0 << u32ModuleNum);
            printf("Software trigger sample module %u, channel %u and generate interrupt ADINT%u.\n", u32ModuleNum, u32ChannelNum, u32IntNum);
            /* Configure the sample module for analog input channel and software trigger source. */
            EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

            /* Set sample module external sampling time to 10 */
            EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module interrupt.  */
            EADC_ENABLE_INT(EADC, u32IntMask);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
            NVIC_EnableIRQ(EADC_INT0_IRQn);

            u32IntNum = 1;
            u32ModuleNum = 1;
            u32ChannelNum = 1;
            u32IntMask = (BIT0 << u32IntNum);
            u32ModuleMask = (BIT0 << u32ModuleNum);
            printf("ADINT0   trigger sample module %u, channel %u and generate interrupt ADINT%u.\n", u32ModuleNum, u32ChannelNum, u32IntNum);
            /* Configure the sample module for analog input channel and software trigger source. */
            EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_ADINT0_TRIGGER, u32ChannelNum);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

            /* Enable the sample module interrupt.  */
            EADC_ENABLE_INT(EADC, u32IntMask);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
            NVIC_EnableIRQ(EADC_INT1_IRQn);

            u32IntNum = 2;
            u32ModuleNum = 2;
            u32ChannelNum = 3;
            u32IntMask = (BIT0 << u32IntNum);
            u32ModuleMask = (BIT0 << u32ModuleNum);
            printf("ADINT1   trigger sample module %u, channel %u and generate interrupt ADINT%u.\n", u32ModuleNum, u32ChannelNum, u32IntNum);
            /* Configure the sample module for analog input channel and software trigger source. */
            EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_ADINT1_TRIGGER, u32ChannelNum);

            /* Set sample module external sampling time to 10 */
            EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

            /* Clear the A/D ADINT2 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);

            /* Enable the sample module interrupt.  */
            EADC_ENABLE_INT(EADC, u32IntMask);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
            NVIC_EnableIRQ(EADC_INT2_IRQn);

            printf("Config EADC sample modules completed !\n");

            /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
            g_u32EadcInt0Flag = 0;
            g_u32EadcInt1Flag = 0;
            g_u32EadcInt2Flag = 0;
            printf("Software trigger sample module 0 here !\n");
            EADC_START_CONV(EADC, BIT0);    /* software trigger sample module 0 */

            /* Wait EADC interrupt (g_u32EadcInt0Flag will be set at EADC_INT0_IRQHandler() function) */
            while (g_u32EadcInt0Flag == 0);

            /* Wait EADC interrupt (g_u32EadcInt1Flag will be set at EADC_INT1_IRQHandler() function) */
            while (g_u32EadcInt1Flag == 0);

            /* Wait EADC interrupt (g_u32EadcInt2Flag will be set at EADC_INT2_IRQHandler() function) */
            while (g_u32EadcInt2Flag == 0);


            /* Get the conversion result of the sample module */
            i32ConversionData = EADC_GET_CONV_DATA(EADC, 0);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 0, i32ConversionData, i32ConversionData);

            /* Get the conversion result of the sample module */
            i32ConversionData = EADC_GET_CONV_DATA(EADC, 1);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 1, i32ConversionData, i32ConversionData);

            /* Get the conversion result of the sample module */
            i32ConversionData = EADC_GET_CONV_DATA(EADC, 2);
            printf("Conversion result of channel %d: 0x%X (%d)\n\n", 3, i32ConversionData, i32ConversionData);

            /* Disable the ADINTx interrupt */
            EADC_DISABLE_INT(EADC, BIT0 | BIT1 | BIT2);
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0 << 0);
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 1, BIT0 << 1);
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 2, BIT0 << 2);
            NVIC_DisableIRQ(EADC_INT0_IRQn);
            NVIC_DisableIRQ(EADC_INT1_IRQn);
            NVIC_DisableIRQ(EADC_INT2_IRQn);
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
    printf("ADINT0 interrupt occurred !\n");
}


void EADC_INT1_IRQHandler(void)
{
    g_u32EadcInt1Flag = 1;
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);
    printf("ADINT1 interrupt occurred !\n");
}


void EADC_INT2_IRQHandler(void)
{
    g_u32EadcInt2Flag = 1;
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);
    printf("ADINT2 interrupt occurred !\n");
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
