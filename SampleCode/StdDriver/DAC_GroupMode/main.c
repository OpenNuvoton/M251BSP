/*************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate DAC0 and DAC1 work in group mode
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

const uint16_t g_au16Sine[] =
{
    2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
    4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
    3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
    639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
    238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876
};

const uint32_t g_u32Array_Size = sizeof(g_au16Sine) / sizeof(uint16_t);
volatile uint32_t g_u32DAC0Index = 0, g_u32DAC1Index = 31;
volatile uint32_t g_u32Dac0_Done = 0, g_u32Dac1_Done = 0;

void DAC_IRQHandler(void)
{
    if (DAC_GET_INT_FLAG(DAC0, 0))
    {

        /* Clear the DAC0 conversion complete finish flag */
        DAC_CLR_INT_FLAG(DAC0, 0);
        DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32DAC0Index]);
        g_u32Dac0_Done = 1;
        g_u32DAC0Index++;

    }

    if (DAC_GET_INT_FLAG(DAC1, 0))
    {

        /* Clear the DAC1 conversion complete finish flag */
        DAC_CLR_INT_FLAG(DAC1, 0);
        DAC_WRITE_DATA(DAC1, 0, g_au16Sine[g_u32DAC1Index]);
        g_u32Dac1_Done = 1;
        g_u32DAC1Index++;

    }

    if (g_u32Dac0_Done == 1 && g_u32Dac1_Done == 1)
    {
        DAC_START_CONV(DAC0);
        g_u32Dac0_Done = g_u32Dac1_Done = 0;

        if (g_u32DAC0Index == g_u32Array_Size)
            g_u32DAC0Index = 0;

        if (g_u32DAC1Index == g_u32Array_Size)
            g_u32DAC1Index = 0;
    }

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

    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Set MFPs for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA4MFP_Msk) | SYS_GPA_MFPL_PA4MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA5MFP_Msk) | SYS_GPA_MFPL_PA5MFP_UART0_TXD;

    if ((SYS->PDID & 0x01920000) == 0x01920000)
    {
        /* Set PA.8 and PA.9 to input mode */
        GPIO_SetMode(PA, BIT8, GPIO_MODE_INPUT);
        GPIO_SetMode(PA, BIT9, GPIO_MODE_INPUT);
        /* Set PA multi-function pin for DAC0 voltage output */
        SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk) | SYS_GPA_MFPH_PA8MFP_DAC0_OUT;
        /* Set PA multi-function pin for DAC1 voltage output */
        SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA9MFP_Msk) | SYS_GPA_MFPH_PA9MFP_DAC1_OUT;

        /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PA, (1ul << 8));
        /* Disable digital input path of analog pin DAC1_OUT to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PA, (1ul << 9));
    }
    else
    {
        /* Set PB.12 and PB.13 to input mode */
        GPIO_SetMode(PB, BIT12, GPIO_MODE_INPUT);
        GPIO_SetMode(PB, BIT13, GPIO_MODE_INPUT);

        /* Set PB multi-function pin for DAC0 voltage output */
        SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_DAC0_OUT;
        /* Set PB multi-function pin for DAC1 voltage output */
        SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB13MFP_Msk) | SYS_GPB_MFPH_PB13MFP_DAC1_OUT;

        /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 12));
        /* Disable digital input path of analog pin DAC1_OUT to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 13));
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
    printf("|            DAC Group Sample Code                         |\n");
    printf("+----------------------------------------------------------+\n");
    printf("DAC0 and DAC1 are configured in group mode and update simultaneously\n");
    printf("* Only chip with 2 DACs supports group mode function.\n");
    /* Single Mode test */
    /* Set the software trigger DAC and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_SOFTWARE_TRIGGER);
    DAC_Open(DAC1, 0, DAC_SOFTWARE_TRIGGER);

    /* Enable DAC to work in group mode, once group mode enabled, DAC1 is configured by DAC0 registers */
    DAC_ENABLE_GROUP_MODE(DAC0);

    /* The DAC0 conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC0, 0, g_au16Sine[0]);
    DAC_WRITE_DATA(DAC1, 0, g_au16Sine[g_u32DAC1Index]);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the DAC interrupt */
    DAC_ENABLE_INT(DAC0, 0);
    DAC_ENABLE_INT(DAC1, 0);
    NVIC_EnableIRQ(DAC_IRQn);

    /* Start A/D conversion */
    DAC_START_CONV(DAC0);

    while (1) {};

}
