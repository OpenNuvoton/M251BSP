/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC spare_register read/write function and displays test result to the
 *           UART console
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

#define SPARE_REG_SIZE_MAX   5

/*---------------------------------------------------------------------------------------------------------*/
/*                            Define functions prototype                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void LXT_Enable(void);
void SYS_Init(void);
void UART0_Init(void);
void RTC_AccessEnable(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

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
/*                                      Init System Clock                                                  */
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

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                          Init UART                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{

    UART_Open(UART0, 115200);
}



/*---------------------------------------------------------------------------------------------------------*/
/*                                           MAIN function                                                 */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    S_RTC_TIME_DATA_T sInitTime, *sInitTime_ptr;
    uint32_t u32SpareRegData, u32Len;

    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    UART0_Init();
    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    /* Time Setting */
    sInitTime.u32Year       = 2018;
    sInitTime.u32Month      = 12;
    sInitTime.u32Day        = 11;
    sInitTime.u32Hour       = 13;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_TUESDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    sInitTime_ptr = (RTC->INIT & RTC_INIT_ACTIVE_Msk) ? NULL : &sInitTime;

    if (RTC_Open(sInitTime_ptr) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");

        while (1);
    }

    printf("\n RTC Spare Register Read/Write Test: \n\n");

    // Enable spare register
    RTC_EnableSpareAccess();

    // Write spare register
    for (u32Len = 0; u32Len < SPARE_REG_SIZE_MAX; u32Len++)
    {

        RTC_WRITE_SPARE_REGISTER(u32Len, u32Len);
    }

    // Check spare register data
    for (u32Len = 0; u32Len < SPARE_REG_SIZE_MAX; u32Len++)
    {
        u32SpareRegData = RTC_READ_SPARE_REGISTER(u32Len);

        if (u32SpareRegData != u32Len)
        {
            printf(" SPARE_REGISTER[%u] = 0x%x \n", u32Len, u32SpareRegData);
            printf(" Test Fail!! \n");

            while (1);
        }
        else
            printf(" SPARE_REGISTER[%u] = 0x%x \n", u32Len, u32SpareRegData);
    }

    printf("\n Test Pass!! \n");

    while (1);

}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/



