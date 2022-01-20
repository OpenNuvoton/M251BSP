/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to trigger multiple sample modules and got conversion results in order of priority.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32EadcInt0Flag, g_u32EadcInt1Flag, g_u32EadcInt2Flag, g_u32EadcInt3Flag;

uint32_t g_au32IntModule[4];    /* save the sample module number for ADINT0~3 */
volatile uint32_t g_au32IntSequence[4];  /* save the interrupt sequence for ADINT0~3 */
volatile uint32_t g_u32IntSequenceIndex;

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

    /* Set PB.0 ~ PB.3 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Configure the EADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB0MFP_Msk) | SYS_GPB_MFPL_PB0MFP_EADC0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB1MFP_Msk) | SYS_GPB_MFPL_PB1MFP_EADC0_CH1;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB2MFP_Msk) | SYS_GPB_MFPL_PB2MFP_EADC0_CH2;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB3MFP_Msk) | SYS_GPB_MFPL_PB3MFP_EADC0_CH3;

    /* Disable the digital input path to avoid the leakage current for EADC analog input pins. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1 | BIT2 | BIT3);   /* Disable PB0 ~PB3 */
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
    int32_t  i32ConversionData, i;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                   EADC Pending Priority sample code                  |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC, 0);

    while (1)
    {
        printf("Select test items:\n");
        printf("  [1] Assign interrupt ADINT0~3 to Sample Module 0~3\n");
        printf("  [2] Assign interrupt ADINT3~0 to Sample Module 0~3\n");
        printf("  Other keys: exit EADC test\n");

        uint8_t  u8Option;
        u8Option = getchar();

        if (u8Option == '1')
        {
            g_au32IntModule[0] = 0;  /* Assign ADINT0 to Sample module 0 */
            g_au32IntModule[1] = 1;  /* Assign ADINT1 to Sample module 1 */
            g_au32IntModule[2] = 2;  /* Assign ADINT2 to Sample module 2 */
            g_au32IntModule[3] = 3;  /* Assign ADINT3 to Sample module 3 */
        }
        else if (u8Option == '2')
        {
            g_au32IntModule[0] = 3;  /* Assign ADINT0 to Sample module 3 */
            g_au32IntModule[1] = 2;  /* Assign ADINT1 to Sample module 2 */
            g_au32IntModule[2] = 1;  /* Assign ADINT2 to Sample module 1 */
            g_au32IntModule[3] = 0;  /* Assign ADINT3 to Sample module 0 */
        }
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and software trigger source. */
        EADC_ConfigSampleModule(EADC, 0, EADC_SOFTWARE_TRIGGER, 0);
        EADC_ConfigSampleModule(EADC, 1, EADC_SOFTWARE_TRIGGER, 1);
        EADC_ConfigSampleModule(EADC, 2, EADC_SOFTWARE_TRIGGER, 2);
        EADC_ConfigSampleModule(EADC, 3, EADC_SOFTWARE_TRIGGER, 3);

        /* Set sample module external sampling time to 10 */
        EADC_SetExtendSampleTime(EADC, 0, 10);
        EADC_SetExtendSampleTime(EADC, 1, 10);
        EADC_SetExtendSampleTime(EADC, 2, 10);
        EADC_SetExtendSampleTime(EADC, 3, 10);

        /* Clear the A/D ADINTx interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk | EADC_STATUS2_ADIF1_Msk | EADC_STATUS2_ADIF2_Msk | EADC_STATUS2_ADIF3_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0 << g_au32IntModule[0]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, BIT0 << g_au32IntModule[1]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, BIT0 << g_au32IntModule[2]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 3, BIT0 << g_au32IntModule[3]);

        NVIC_EnableIRQ(EADC_INT0_IRQn);
        NVIC_EnableIRQ(EADC_INT1_IRQn);
        NVIC_EnableIRQ(EADC_INT2_IRQn);
        NVIC_EnableIRQ(EADC_INT3_IRQn);

        /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
        g_u32IntSequenceIndex = 0;
        g_u32EadcInt0Flag = 0;
        g_u32EadcInt1Flag = 0;
        g_u32EadcInt2Flag = 0;
        g_u32EadcInt3Flag = 0;

        /* Start EADC conversion for sample module 0 ~ 3 at the same time */
        EADC_START_CONV(EADC, BIT0 | BIT1 | BIT2 | BIT3);

        /* Wait all EADC interrupt (g_u32EadcIntxFlag will be set at EADC_INTx_IRQHandler() function) */
        while ((g_u32EadcInt0Flag == 0) || (g_u32EadcInt1Flag == 0) ||
                (g_u32EadcInt2Flag == 0) || (g_u32EadcInt3Flag == 0));

        //        for (i=0; i<80000; i++) __NOP();

        /* Get the conversion result of the sample module */
        printf("The ADINTx interrupt sequence is:\n");

        for (i = 0; i < 4; i++)
        {
            i32ConversionData = EADC_GET_CONV_DATA(EADC, g_au32IntModule[i]);
            printf("ADINT%d: #%d, Module %d, Conversion result: 0x%X (%d)\n", i, g_au32IntSequence[i], g_au32IntModule[i], i32ConversionData, i32ConversionData);
        }

        printf("\n");

        /* Disable the ADINTx interrupt */
        EADC_DISABLE_INT(EADC, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0 << g_au32IntModule[0]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 1, BIT0 << g_au32IntModule[1]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 2, BIT0 << g_au32IntModule[2]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 3, BIT0 << g_au32IntModule[3]);

        NVIC_DisableIRQ(EADC_INT0_IRQn);
        NVIC_DisableIRQ(EADC_INT1_IRQn);
        NVIC_DisableIRQ(EADC_INT2_IRQn);
        NVIC_DisableIRQ(EADC_INT3_IRQn);
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

    /* Save the interrupt sequence about ADINT0 */
    g_au32IntSequence[0] = g_u32IntSequenceIndex++;
}


void EADC_INT1_IRQHandler(void)
{
    g_u32EadcInt1Flag = 1;
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

    /* Save the interrupt sequence about ADINT1 */
    g_au32IntSequence[1] = g_u32IntSequenceIndex++;
}


void EADC_INT2_IRQHandler(void)
{
    g_u32EadcInt2Flag = 1;
    /* Clear the A/D ADINT2 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);

    /* Save the interrupt sequence about ADINT2 */
    g_au32IntSequence[2] = g_u32IntSequenceIndex++;
}


void EADC_INT3_IRQHandler(void)
{
    g_u32EadcInt3Flag = 1;
    /* Clear the A/D ADINT3 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF3_Msk);

    /* Save the interrupt sequence about ADINT3 */
    g_au32IntSequence[3] = g_u32IntSequenceIndex++;
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
