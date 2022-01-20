/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to trigger EADC by PWM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32EadcInt0Flag, g_u32ConvNum;

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

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Select PWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

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

    /* Set PA multi-function pins for PWM Channel0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA5MFP_Msk));
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA5MFP_PWM0_CH0;
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


void PWM0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init PWM0                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 1);

    /* Center-aligned type */
    PWM_SET_ALIGNED_TYPE(PWM0, PWM_CH_0_MASK, PWM_CENTER_ALIGNED);

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 0, 108);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 0, 216);

    /* PWM period point trigger ADC enable */
    PWM_EnableADCTrigger(PWM0, 0, PWM_TRIGGER_ADC_EVEN_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, 0x1, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, 0x1);
}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    int32_t  ai32ConversionData[6] = {0};
    int32_t  i32Temp;
    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;

    u32IntNum = 0;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 1;   /* Use Sample Module 1 */

    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      EADC PWM trigger sample code                    |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

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

        /* Configure the sample module for analog input channel and enable PWM0 trigger source */
        EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_PWM0TG0_TRIGGER, u32ChannelNum);

        /* Set sample module external sampling time to 10 */
        EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC, u32IntMask);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
        NVIC_EnableIRQ(EADC_INT0_IRQn);

        /* Reset the EADC indicator and enable PWM0 channel 0 counter */
        g_u32EadcInt0Flag = 0;
        g_u32ConvNum = 0;
        PWM_Start(PWM0, BIT0);

        while (1)
        {
            /* Wait EADC interrupt (g_u32EadcInt0Flag will be set at EADC_INT0_IRQHandler() function) */
            while (g_u32EadcInt0Flag == 0);

            /* Reset the EADC interrupt indicator */
            g_u32EadcInt0Flag = 0;

            /* Get the conversion result of the sample module 0 */
            i32Temp = EADC_GET_CONV_DATA(EADC, u32ModuleNum);
            ai32ConversionData[g_u32ConvNum - 1] = i32Temp;

            if (g_u32ConvNum >= 6)
                break;
        }

        /* Disable PWM0 channel 0 counter */
        PWM_ForceStop(PWM0, BIT0);  /* PWM0 counter stop running. */

        printf("Conversion result of channel %u:\n", u32ChannelNum);

        for (g_u32ConvNum = 0; g_u32ConvNum < 6; g_u32ConvNum++)
        {
            i32Temp = ai32ConversionData[g_u32ConvNum];
            printf("                                0x%X (%d)\n", i32Temp, i32Temp);
        }

        printf("\n");

        /* Disable the ADINTx interrupt */
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
    g_u32ConvNum++;
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

    /* Init PWM0 for EADC */
    PWM0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %u Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset PWM0 module */
    SYS_ResetModule(PWM0_RST);

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable PWM0 IP clock */
    CLK_DisableModuleClock(PWM0_MODULE);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
