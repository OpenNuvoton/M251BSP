/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of ACMP window compare function
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

/*---------------------------------------------------------------------------------------------------------*/
/*                                 Define functions prototype                                              */
/*---------------------------------------------------------------------------------------------------------*/
void ACMP01_IRQHandler(void);
void SYS_Init(void);
int32_t main(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

void ACMP01_IRQHandler(void)
{
    /* Clear interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    if (ACMP01->STATUS & ACMP_STATUS_ACMPWO_Msk)
    {
        printf("The input voltage is inside the window\n");
    }
    else
    {
        printf("The input voltage is outside the window\n");
    }
}


void SYS_Init(void)
{

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
    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);
    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB.2 and PB.4 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE4_Msk);

    /* Set PB2 multi-function pin for ACMP0 positive input pin */
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB2MFP_ACMP0_P1;

    /* Set PB4 multi-function pin for ACMP1 positive input pin */
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB4MFP_ACMP1_P1;

    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Disable digital input path of analog pin ACMP0_P0 and ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 2));
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));
}


int32_t main(void)
{
    uint32_t volatile u32DelayCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\nThis sample code demonstrates ACMP window compare function.\n");
    printf("  ACMP0_P1: Using PB2 as ACMP0 positive input.\n");
    printf("  ACMP1_P1: Using PB4 as ACMP1 positive input.\n");
    printf("Connect the analog voltage source to the positive inputs of both comparators.\n");
    printf("\nACMP will monitor if the input voltage is between\n");
    printf("  (VDDA * 14 / 24) and internal band-gap voltage.\n");
    printf("Press any key to continue ...\n");
    getchar();


    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV_SRC(ACMP01, ACMP_VREF_CRVSSEL_AVDD);
    /* Select CRV level: VDDA * 14 / 24 */
    ACMP_CRV_SEL(ACMP01, 14);
    /* Configure ACMP0. Enable ACMP0 and select VBG as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 0, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Configure ACMP1. Enable ACMP1 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_CRV, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select P1 as ACMP0 positive input channel */
    ACMP_SELECT_P(ACMP01, 0, ACMP_CTL_POSSEL_P1);
    /* Select P1 as ACMP1 positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    /* Enable window compare mode */
    ACMP_ENABLE_WINDOW_COMPARE(ACMP01, 0);
    ACMP_ENABLE_WINDOW_COMPARE(ACMP01, 1);

    /* Clear ACMP 0 and 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    // Give ACMP some time to settle
    for (u32DelayCount = 0; u32DelayCount < 1000; u32DelayCount++);

    if (ACMP01->STATUS & ACMP_STATUS_ACMPWO_Msk)
    {
        printf("The input voltage is inside the window\n");
    }
    else
    {
        printf("The input voltage is outside the window\n");
    }

    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 0);
    ACMP_ENABLE_INT(ACMP01, 1);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while (1);

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/


