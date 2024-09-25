/****************************************************************************
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to PDMA and trigger DAC by Timer.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

const uint16_t g_au16Sine[] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                               4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                               3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
                               639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
                               238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876
                              };
const uint32_t g_u32ArraySize = sizeof(g_au16Sine) / sizeof(uint16_t);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable DAC module clock */
    CLK_EnableModuleClock(DAC_MODULE);

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Set MFPs for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA4MFP_Msk) | SYS_GPA_MFPL_PA4MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA5MFP_Msk) | SYS_GPA_MFPL_PA5MFP_UART0_TXD;

    if ((SYS->PDID & 0x01920000) == 0x01920000)
    {
        /* Enable GPA peripheral clock */
        CLK_EnableModuleClock(GPA_MODULE);
        /* Set PA multi-function pin for DAC voltage output */
        SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk) | SYS_GPA_MFPH_PA8MFP_DAC0_OUT;
        /* Set PA.8 to input mode */
        PA->MODE &= ~(GPIO_MODE_MODE8_Msk) ;
        /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PA, (1ul << 8));
    }
    else
    {
        /* Enable GPB peripheral clock */
        CLK_EnableModuleClock(GPB_MODULE);
        /* Set PB multi-function pin for DAC voltage output */
        SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_DAC0_OUT;
        /* Set PB.12 to input mode */
        PB->MODE &= ~(GPIO_MODE_MODE12_Msk) ;
        /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 12));
    }

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

    printf("+----------------------------------------------------------+\n");
    printf("|            DAC Driver Sample Code                        |\n");
    printf("+----------------------------------------------------------+\n");
    printf("This sample code use PDMA and trigger DAC output sine wave by Timer 0.\n");

    /* Open Channel 0 */
    PDMA_Open(PDMA, 0x1);

    /* Set transfer data width, and transfer count */
    PDMA_SetTransferCnt(PDMA, 0, PDMA_WIDTH_16, g_u32ArraySize);

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferAddr(PDMA, 0, (uint32_t)&g_au16Sine[0], PDMA_SAR_INC, (uint32_t)&DAC0->DAT, PDMA_DAR_FIX);

    /* Select channel 0 request source from DAC */
    PDMA_SetTransferMode(PDMA, 0, PDMA_DAC0_TX, FALSE, 0);

    /* Set transfer type and burst size */
    PDMA_SetBurstType(PDMA, 0, PDMA_REQ_SINGLE, PDMA_BURST_128);

    /* Set the timer 0 trigger DAC and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_TIMER0_TRIGGER);

    /* The DAC conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the PDMA Mode */
    DAC_ENABLE_PDMA(DAC0);

    /* Enable Timer0 counting to start D/A conversion */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_DAC);
    TIMER_Start(TIMER0);

    while (1)
    {
        if (PDMA_GET_TD_STS(PDMA) == 0x1)
        {
            /* Re-Set transfer count and basic operation mode */
            PDMA_SetTransferCnt(PDMA, 0, PDMA_WIDTH_16, g_u32ArraySize);
            PDMA_SetTransferMode(PDMA, 0, PDMA_DAC0_TX, FALSE, 0);

            /* Clear PDMA channel 0 transfer done flag */
            PDMA_CLR_TD_FLAG(PDMA, 0x1);
        }
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
