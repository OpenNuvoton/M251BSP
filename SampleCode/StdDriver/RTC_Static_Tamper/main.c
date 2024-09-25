/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC static tamper function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

/*---------------------------------------------------------------------------------------------------------*/
/*                                     Global variables                                                    */
/*---------------------------------------------------------------------------------------------------------*/

volatile int32_t   g_i32Tamper  = FALSE;

/*---------------------------------------------------------------------------------------------------------*/
/*                            Define functions prototype                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void TAMPER_IRQHandler(void);
void LXT_Enable(void);
void SYS_Init(void);
void UART0_Init(void);
int32_t main(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/**
  * @brief  TAMPER ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void TAMPER_IRQHandler(void)
{
    uint32_t u32TmpStatus;


    u32TmpStatus = RTC->INTSTS & (0x3F << RTC_INTEN_TAMP0IEN_Pos);

    if (u32TmpStatus)          /* tamper interrupt occurred */
    {


        if (u32TmpStatus & (0x1 << (RTC_INTEN_TAMP0IEN_Pos)))
            printf(" Tamper0 Detected!!\n");


        printf(" Tamper detected date: 20%d%d / %d%d / %d%d \n",
               (int)((RTC->TAMPCAL & RTC_TAMPCAL_TENYEAR_Msk) >> RTC_TAMPCAL_TENYEAR_Pos),
               (int)((RTC->TAMPCAL & RTC_TAMPCAL_YEAR_Msk) >> RTC_TAMPCAL_YEAR_Pos),
               (int)((RTC->TAMPCAL & RTC_TAMPCAL_TENMON_Msk) >> RTC_TAMPCAL_TENMON_Pos),
               (int)((RTC->TAMPCAL & RTC_TAMPCAL_MON_Msk) >> RTC_TAMPCAL_MON_Pos),
               (int)((RTC->TAMPCAL & RTC_TAMPCAL_TENDAY_Msk) >> RTC_TAMPCAL_TENDAY_Pos),
               (int)((RTC->TAMPCAL & RTC_TAMPCAL_DAY_Msk) >> RTC_TAMPCAL_DAY_Pos));
        printf(" Tamper detected Time: %d%d: %d%d: %d%d \n",
               (int)((RTC->TAMPTIME & RTC_TAMPTIME_TENHR_Msk) >> RTC_TAMPTIME_TENHR_Pos),
               (int)((RTC->TAMPTIME & RTC_TAMPTIME_HR_Msk) >> RTC_TAMPTIME_HR_Pos),
               (int)((RTC->TAMPTIME & RTC_TAMPTIME_TENMIN_Msk) >> RTC_TAMPTIME_TENMIN_Pos),
               (int)((RTC->TAMPTIME & RTC_TAMPTIME_MIN_Msk) >> RTC_TAMPTIME_MIN_Pos),
               (int)((RTC->TAMPTIME & RTC_TAMPTIME_TENSEC_Msk) >> RTC_TAMPTIME_TENSEC_Pos),
               (int)((RTC->TAMPTIME & RTC_TAMPTIME_SEC_Msk) >> RTC_TAMPTIME_SEC_Pos));
        RTC->INTSTS = u32TmpStatus;
        g_i32Tamper = TRUE;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*                           Enable the external 32768Hz XTAL Clock                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LXT_Enable(void)
{
    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk);

    /* Enable external 32768Hz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Disable digital input path of analog pin X32_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 4));

    /* Disable digital input path of analog pin XT32_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 5));
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                       Init System Clock                                                 */
/*---------------------------------------------------------------------------------------------------------*/

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
    /* Enable external 32768Hz XTAL */
    LXT_Enable();

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set PF   multi-function pins for Tamper0 */
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF6MFP_Msk);
    SYS->GPF_MFPL |= SYS_GPF_MFPL_PF6MFP_TAMPER0 ;

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                              Init UART                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{

    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                          MAIN function                                                  */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    S_RTC_TIME_DATA_T sInitTime, *sInitTime_ptr;

    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    UART0_Init();
    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    printf("\n RTC Tamper Test Start:");
    printf("\n Please connect tamper pin to High ");
    printf("\n Press any key to start test.\n");

    getchar();

    /* Time Setting */
    sInitTime.u32Year       = 2018;
    sInitTime.u32Month      = 12;
    sInitTime.u32Day        = 11;
    sInitTime.u32Hour       = 13;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_TUESDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    /* check rtc reset status */
    sInitTime_ptr = (RTC->INIT & RTC_INIT_ACTIVE_Msk) ? NULL : &sInitTime;

    if (RTC_Open(sInitTime_ptr) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");

        while (1);
    }

    RTC_CLEAR_TAMPER_INT_FLAG(RTC_INTSTS_TAMP0IF_Msk);

    RTC_StaticTamperEnable(RTC_TAMPER0_SELECT, RTC_TAMPER_LOW_LEVEL_DETECT, RTC_TAMPER_DEBOUNCE_ENABLE);

    g_i32Tamper = FALSE;
    /* Enable RTC Tamper Interrupt */
    RTC_EnableInt(RTC_INTEN_TAMP0IEN_Msk);
    NVIC_EnableIRQ(TAMPER_IRQn);

    printf("\n Please connect tamper pin to Low \n\n");

    while (!g_i32Tamper);

    RTC_StaticTamperDisable(RTC_TAMPER0_SELECT);

    /* Disable RTC Alarm Interrupt */
    RTC_DisableInt(RTC_INTEN_TAMP0IEN_Msk);
    NVIC_DisableIRQ(TAMPER_IRQn);

    printf("\n RTC static Tamper Test End !!\n");

    while (1);

}



/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/



