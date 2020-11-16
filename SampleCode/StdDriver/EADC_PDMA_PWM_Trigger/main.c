/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to trigger EADC by PWM and transfer conversion data by PDMA.
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
volatile uint32_t g_u32IsTestOver = 0;
volatile int16_t  g_ai16ConversionData[6] = {0};
volatile uint32_t g_u32SampleModuleNum = 0;

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
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Select PWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

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


void PDMA_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init PDMA                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    /* Configure PDMA peripheral mode form EADC to memory */
    /* Open Channel 2 */
    PDMA_Open(PDMA, BIT2);

    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA_SetTransferCnt(PDMA, 2, PDMA_WIDTH_16, 6);

    /* Set source address as EADC data register(no increment) and destination address as g_ai16ConversionData array(increment) */
    PDMA_SetTransferAddr(PDMA, 2, (uint32_t) & (EADC->CURDAT), PDMA_SAR_FIX, (uint32_t)g_ai16ConversionData, PDMA_DAR_INC);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, 2, PDMA_EADC_RX, FALSE, 0);

    /* Set PDMA as single request type for EADC */
    PDMA_SetBurstType(PDMA, 2, PDMA_REQ_SINGLE, PDMA_BURST_4);

    PDMA_EnableInt(PDMA, 2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
}


void ReloadPDMA(void)
{
    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA_SetTransferCnt(PDMA, 2, PDMA_WIDTH_16, 6);

    /* Select PDMA request source as EADC RX */
    PDMA_SetTransferMode(PDMA, 2, PDMA_EADC_RX, FALSE, 0);
}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    int32_t  i32ConversionData;
    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;

    u32IntNum = g_u32SampleModuleNum;
    u32ModuleNum = 1;   /* Use Sample Module 1 */

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|   EADC PWM trigger and transfer conversion data by PDMA sample code  |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC, 0);

    while (1)
    {
        /* reload PDMA configuration for next transmission */
        ReloadPDMA();

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

        EADC_ENABLE_PDMA(EADC, u32ModuleNum);

        PWM_Start(PWM0, BIT0);

        while (1)
        {
            /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
            while (g_u32IsTestOver == 0);

            break;
        }

        g_u32IsTestOver = 0;

        /* Disable PWM0 channel 0 counter */
        PWM_ForceStop(PWM0, BIT0);  /* PWM0 counter stop running. */

        printf("Conversion result of channel %u:\n", u32ChannelNum);

        for (g_u32ConvNum = 0; g_u32ConvNum < 6; g_u32ConvNum++)
        {
            i32ConversionData = g_ai16ConversionData[g_u32ConvNum];
            printf("                                0x%X (%d)\n", i32ConversionData, i32ConversionData);
        }

        EADC_DISABLE_PDMA(EADC, u32IntNum);
    }   /* End of while(1) */

    /* Disable the A/D converter */
    EADC_Close(EADC);
}


/*---------------------------------------------------------------------------------------------------------*/
/* PDMA interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & 0x1)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA) & BIT2)
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (status & 0x2)     /* done */
    {
        if (PDMA_GET_TD_STS(PDMA) & BIT2)
            g_u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else
        printf("PDMA_IRQHandler(): unknown interrupt !!\n");
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

    /* Init PDMA for EADC */
    PDMA_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

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

    /* Disable PDMA clock source */
    CLK_DisableModuleClock(PDMA_MODULE);

    /* Disable PDMA Interrupt */
    NVIC_DisableIRQ(PDMA_IRQn);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
