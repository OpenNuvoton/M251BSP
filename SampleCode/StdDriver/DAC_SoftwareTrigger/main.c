/****************************************************************************
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to trigger DAC conversion by software.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

const uint16_t g_au16Sine[] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                               4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                               3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
                               639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
                               238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876
                              };

volatile uint32_t g_u32Index = 0;
const uint32_t g_u32ArraySize = sizeof(g_au16Sine) / sizeof(uint16_t);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

void DAC_IRQHandler(void)
{
    if (DAC_GET_INT_FLAG(DAC0, 0))
    {

        if (g_u32Index == g_u32ArraySize)
            g_u32Index = 0;
        else
        {
            DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index++]);
            DAC_START_CONV(DAC0);
            /* Clear the DAC conversion complete finish flag */
            DAC_CLR_INT_FLAG(DAC0, 0);
        }
    }

    return;
}


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

    /* Set UART0 Default MPF */
    //    Uart0DefaultMPF() ;

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
    printf("Please hit any key to start trigger DAC by software\n");
    getchar();

    /* Set the software trigger DAC and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_SOFTWARE_TRIGGER);

    /* The DAC conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC0, 0, 0x400);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the DAC interrupt */
    DAC_ENABLE_INT(DAC0, 0);
    NVIC_EnableIRQ(DAC_IRQn);

    /* Start A/D conversion */
    DAC_START_CONV(DAC0);

    while (1) {};

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
