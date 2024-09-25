/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of ACMP window latch function.
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
    static uint32_t u32Cnt = 0;

    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    /* Check Comparator 1 Output Status */
    if (ACMP_GET_OUTPUT(ACMP01, 1))
        printf("ACMP1_P voltage > Band-gap voltage (%u) ACMP1_O(%d)\n", u32Cnt, PC0);
    else
        printf("ACMP1_P voltage <= Band-gap voltage (%u) ACMP1_O(%d)\n", u32Cnt, PC0);

    u32Cnt++;
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
    /* Enable GPC peripheral clock */
    CLK_EnableModuleClock(GPC_MODULE);


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB.4 and PC.0 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE4_Msk);
    PC->MODE &= ~(GPIO_MODE_MODE0_Msk);

    /* Set PA6 multi-function pin for ACMP1 window latch pin */
    SYS->GPA_MFPL = SYS_GPA_MFPL_PA6MFP_ACMP1_WLAT;

    /* Set PB4 multi-function pin for ACMP1 positive input pin and PC0 multi-function pin for ACMP1 output pin*/
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB4MFP_ACMP1_P1;
    SYS->GPC_MFPL = SYS_GPC_MFPL_PC0MFP_ACMP1_O;

    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));
}

/*
 * In window latch mode and window latch pin is at high level, the voltage of the positive
 * input is greater than the  voltage of the negative input, the analog comparator outputs
 * logical one; otherwise, it outputs logical zero. When window latch pin is at low level,
 * the output level does not change no matter how positive  input voltage changes.
 */

int32_t main(void)
{
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

    printf("\nThis sample code demonstrates [ACMP1] window latch function.\n");
    printf("  ACMP1_N:    Using internal band-gap voltage as negative input.\n");
    printf("  ACMP1_P1:   Using PB4 as positive input.\n");
    printf("  ACMP1_O:    Using PC0 as compare output.\n");
    printf("  ACMP1_WLAT: Using PA6 as window latch pin.\n");
    printf("\nWhen ACMP1_WLAT is low,  ACMP1_O does not change with ACMP1_P1.\n");
    printf("When ACMP1_WLAT is high, ACMP1_O works as usual.\n");
    printf("Press any key to start ...\n");
    getchar();
    /* Configure ACMP1. Enable ACMP1 and select band-gap voltage as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    //    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_PIN, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select P1 as ACMP positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 1);
    /* Enable window latch mode */
    ACMP_ENABLE_WINDOW_LATCH(ACMP01, 1);

    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while (1);
}
