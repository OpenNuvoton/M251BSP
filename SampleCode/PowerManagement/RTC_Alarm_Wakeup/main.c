/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC Wake-up in Power-down mode
 *           Please refer to the application note for details
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

#define PD_MODE   0     // Power-down mode
#define FWPD_MODE 1     // Fast wake up
#define DPD_MODE  2     // Deep Power-down mode

#define T_60SEC   60    //60Sec
#define T_60MIN   60    //60Min

/*---------------------------------------------------------------------------------------------------------*/
/*                                       Global variables                                                  */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t  g_i32Alarm  = FALSE;
volatile int32_t  g_i32WakeUp = FALSE;
S_RTC_TIME_DATA_T sInitTime, *sInitTime_ptr;
S_RTC_TIME_DATA_T sCurTime;

/*---------------------------------------------------------------------------------------------------------*/
/*                                Functions and variables declaration                                      */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
    uint32_t Power_Down_Mode;
    uint32_t Wakeup_AlarmTime;
    uint32_t Waiting_AlarmTime;
} CONFIG_INFO_T;

const CONFIG_INFO_T User_Config =
{
    PD_MODE,        /*!< Power Down Mode */
    10,             /*!< Wake-up Alarm Time Setting(Sec) */
    10,             /*!< Waiting Alarm Time Setting(Sec) */
};

/*---------------------------------------------------------------------------------------------------------*/
/*                                    Define functions prototype                                           */
/*---------------------------------------------------------------------------------------------------------*/
void EnterToPowerDown(uint32_t u32PDMode);
void Set_Alarm_time(uint32_t u32AlarmTime);
void PWRWU_IRQHandler(void);
void RTC_IRQHandler(void);
void LXT_Enable(void);
void SYS_Init(void);
int32_t main(void);

/*---------------------------------------------------------------------------------------------------------*/
/*                                    Select mode to enter power down                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EnterToPowerDown(uint32_t u32PDMode)
{
    g_i32WakeUp = FALSE;

    SYS_UnlockReg();

    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PMUCTL &= ~(CLK_PMUCTL_PDMSEL_Msk | CLK_PMUCTL_RTCWKEN_Msk);

    if (u32PDMode == PD_MODE)
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_PD;    //Power down
    else if ((u32PDMode == FWPD_MODE))
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_FWPD;  //fast up
    else if ((u32PDMode == DPD_MODE))
        CLK->PMUCTL |= CLK_PMUCTL_PDMSEL_DPD | CLK_PMUCTL_RTCWKEN_Msk; // DPD(Deep power dwon) Mode and RTC WK enable

    CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);
    CLK->PWRCTL |=  CLK_PWRCTL_PDWKIEN_Msk;

    CLK_PowerDown();

    SYS_LockReg();

    while (g_i32WakeUp == FALSE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         PWRWU  Handle                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void Set_Alarm_time(uint32_t u32AlarmTime)
{
    /* The alarm time setting */
    sCurTime.u32Second = sCurTime.u32Second + u32AlarmTime;

    /* Remainder processing of the seconds*/
    if (sCurTime.u32Second >= T_60SEC)
    {
        sCurTime.u32Minute = sCurTime.u32Minute + (sCurTime.u32Second / T_60SEC);
        sCurTime.u32Second = sCurTime.u32Second % T_60SEC;
    }

    /* Remainder processing of the mintunes*/
    if (sCurTime.u32Minute >= T_60MIN)
    {
        sCurTime.u32Hour = sCurTime.u32Hour + (sCurTime.u32Minute / T_60SEC);
        sCurTime.u32Minute = sCurTime.u32Minute % T_60MIN;
    }

    /* Set the alarm time */
    RTC_SetAlarmDateAndTime(&sCurTime);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         PWRWU  Handle                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    g_i32WakeUp = TRUE;

    CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         RTC  Handle                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_IRQHandler(void)
{
    /* alarm interrupt occurred */
    if ((RTC->INTEN & RTC_INTEN_ALMIEN_Msk) && (RTC->INTSTS & RTC_INTSTS_ALMIF_Msk))
    {
        g_i32Alarm = TRUE;

        RTC_CLEAR_ALARM_INT_FLAG();
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
/*                                          Init System Clock                                              */
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

    /*Enable external 32768Hz XTAL */
    LXT_Enable();

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                       Init UART                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                     MAIN function                                                       */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, peripheral clock */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|               RTC Alarm Wake-up Sample Code                 |\n");
    printf("+-------------------------------------------------------------+\n");

    /* Time setting */
    sInitTime.u32Year       = 2022;
    sInitTime.u32Month      = 5;
    sInitTime.u32Day        = 1;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 0;
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

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    /* Check Wake-up form RTC or System reset */
    if (CLK_GetPMUWKSrc() & CLK_PMUSTS_RTCWK_Msk)
    {
        printf("Wake-up from RTC...Current Time:%u/%02u/%02u %02u:%02u:%02u\n", sCurTime.u32Year, sCurTime.u32Month,
               sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);
    }
    else
    {
        printf("Wake-up from System reset...Current Time:%u/%02u/%02u %02u:%02u:%02u\n", sCurTime.u32Year, sCurTime.u32Month,
               sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);
    }

    /* Enable RTC Alarm Interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    NVIC_EnableIRQ(RTC_IRQn);

    NVIC_EnableIRQ(PWRWU_IRQn);

    while (1)
    {
        /* Set the waiting time for power down */
        Set_Alarm_time(User_Config.Waiting_AlarmTime);

        g_i32Alarm = FALSE;

        /* Clear interrupt status */
        RTC->INTSTS = RTC_INTSTS_ALMIF_Msk;

        /* Wait RTC interrupt occurred*/
        while (!g_i32Alarm) {};

        /* Get the current time */
        RTC_GetDateAndTime(&sCurTime);

        printf("Entering Power Down...Current Time:%u/%02u/%02u %02u:%02u:%02u\n", sCurTime.u32Year, sCurTime.u32Month,
               sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

        /* Wait the UART FIFO buffer is empty */
        while (!IsDebugFifoEmpty()) {};

        /* Set the alarm time for wake-up */
        Set_Alarm_time(User_Config.Wakeup_AlarmTime);

        g_i32Alarm = FALSE;

        /* Select mode to enter power down */
        EnterToPowerDown(User_Config.Power_Down_Mode);

        /* IOCTLSEL will automatically be set by hardware to 1 when system power is off and RTC_GPIOCTL0 had been configured. */
        /* when system wake up from PD mode, you need clear this bit if you want PF.4, 5, 6 pin I/O function is controlled by GPIO module. */
        RTC->LXTCTL &= ~(RTC_LXTCTL_IOCTLSEL_Msk);

        /* Wait RTC interrupt occurred*/
        while (!g_i32Alarm) {};

        /* Get the current time */
        RTC_GetDateAndTime(&sCurTime);

        printf("RTC Wake Up success...Current Time:%u/%02u/%02u %02u:%02u:%02u\n", sCurTime.u32Year, sCurTime.u32Month,
               sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);
    }
}

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
