/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up MCU from Power-down mode by ACMP wake-up function.
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

#define PD_MODE   0  // Power-down mode
#define FWPD_MODE 2  // Fast wake up

/*---------------------------------------------------------------------------------------------------------*/
/*                                       Global variables                                                  */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t  g_i32WakeUp = FALSE;

/*---------------------------------------------------------------------------------------------------------*/
/*                                 Define functions prototype                                              */
/*---------------------------------------------------------------------------------------------------------*/
void ACMP01_IRQHandler(void);
void PWRWU_IRQHandler(void);
void EnterToPowerDown(uint32_t u32PDMode);
void SYS_Init(void);
extern int IsDebugFifoEmpty(void);
int32_t main(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*                                         ACMP01  Handle                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void ACMP01_IRQHandler(void)
{
    printf("\nACMP1 interrupt!\n");

    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);
    /* Clear wake-up interrupt flag */
    ACMP_CLR_WAKEUP_INT_FLAG(ACMP01, 1);
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
  * @brief      Enter To Power Down
  * @param[in]   u32PDMode    The specified Power down module.
  *                               - \ref CLK_PMUCTL_PDMSEL_PD      : Power-down mode
  *                               - \ref CLK_PMUCTL_PDMSEL_FWPD    : Fast wake up
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
    else if (u32PDMode == FWPD_MODE)
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_FWPD;  //fast up


    CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);
    CLK->PWRCTL |=  CLK_PWRCTL_PDWKIEN_Msk;
    CLK_PowerDown();
    SYS_LockReg();

    while (g_i32WakeUp == FALSE);

}



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

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPB_MODULE);
    /* Enable GPC peripheral clock */
    CLK_EnableModuleClock(GPC_MODULE);

    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB.4 and PC.0 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE4_Msk);
    PC->MODE &= ~(GPIO_MODE_MODE0_Msk);

    /* Set PB4 multi-function pin for ACMP1 positive input pin and PC0 multi-function pin for ACMP1 output pin*/
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB4MFP_ACMP1_P1;
    SYS->GPC_MFPL = SYS_GPC_MFPL_PC0MFP_ACMP1_O;

    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));

}

/*
 * When the voltage of the positive input is greater than the voltage of the negative input,
 * the analog comparator outputs logical one; otherwise, it outputs logical zero.
 * This chip will be waked up from power down mode when detecting a transition of analog comparator's output.
 */

int32_t main(void)
{
    uint32_t u32DelayCount;

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

    printf("\nThis sample code demonstrates [ACMP1] wakeup function.\n");
    printf("  ACMP1_N:  Using internal CRV as negative input.\n");
    printf("  ACMP1_P1: Using PB4 as positive input.\n");
    printf("  ACMP1_O:  Using PC0 as compare output.\n");
    printf("\nThe compare result reflects on ACMP1_O (PC0).\n");
    printf("Press any key to enter power down mode ...\n");
    getchar();

    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV_SRC(ACMP01, ACMP_VREF_CRVSSEL_AVDD);
    /* Select CRV level: VDDA * 5 / 24 */
    ACMP_CRV_SEL(ACMP01, 5);
    /* Configure ACMP1. Enable ACMP1 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_CRV, ACMP_CTL_HYSTERESIS_DISABLE);
    //    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_PIN, ACMP_CTL_HYSTERESIS_DISABLE);

    /* Select P1 as ACMP positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    __NOP();

    for (u32DelayCount = 0; u32DelayCount < 100; u32DelayCount++); /* For ACMP setup time */

    __NOP();
    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    /* Enable wake-up function */
    ACMP_ENABLE_WAKEUP(ACMP01, 1);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 1);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);
    /* Enable PWRWU interrupt */
    NVIC_EnableIRQ(PWRWU_IRQn);
    printf("\nSystem enter power-down mode ... \n");

    /* To check if all the debug messages are finished */
    while (IsDebugFifoEmpty() == 0) {};

    EnterToPowerDown(PD_MODE);

    printf("Wake up by ACMP1!\n");

    ACMP_DISABLE_WAKEUP(ACMP01, 1);

    NVIC_DisableIRQ(ACMP01_IRQn);

    NVIC_DisableIRQ(PWRWU_IRQn);

    while (1);

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/


