/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to wake up system form DPD Power-down mode by Wake-up pin(PC.0)
 *           or Wake-up Timer or RTC Tick or RTC Alarm or RTC Tamper 0.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define CLK_HIRC    1
#define CLK_HXT     0
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up pin                         */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpPinFunction(uint32_t u32PDMode, uint32_t u32EdgeType)
{
    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Configure GPIO as Input mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);

    /* Set Wake-up pin trigger type at Deep Power down mode */
    CLK_EnableDPDWKPin(CLK_DPDWKPIN_0, u32EdgeType);

    printf("Enter to DPD Power-Down mode......\n");

    while (!IsDebugFifoEmpty()) {}; /* waits for TX empty */

    /* Disable LIRC clock for lower power consumption */
    CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for LIRC clock disable */
    CLK_WaitClockDisable(CLK_STATUS_LIRCSTB_Msk);

    /* Enter to Power-down mode */
    CLK_PowerDown();

    /* Wait for Power-down mode wake-up reset happen */
    while (1);
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up Timer                         */
/*-----------------------------------------------------------------------------------------------------------*/
void  WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval)
{
    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Set Wake-up Timer Time-out Interval */
    CLK_SET_WKTMR_INTERVAL(u32Interval);

    printf("Enter to DPD Power-Down mode......\n");

    while (!IsDebugFifoEmpty()) {}; /* waits for TX empty */

    /* Enable Wake-up Timer */
    CLK_ENABLE_WKTMR();

    /* Enter to Power-down mode */
    CLK_PowerDown();

    /* Wait for Power-down mode wake-up reset happen */
    while (1);
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by RTC Tick                              */
/*-----------------------------------------------------------------------------------------------------------*/
void  WakeUpRTCTickFunction(uint32_t u32PDMode)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; // 1 second timeout

    /* enable RTC peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;

    /* Select RTC clock source as LIRC */
    RTC->LXTCTL |= RTC_LXTCTL_C32KS_Msk;

    /* Open RTC and start counting */
    RTC->INIT = RTC_INIT_KEY;

    if (RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        RTC->INIT = RTC_INIT_KEY;

        while (RTC->INIT != RTC_INIT_ACTIVE_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Initialize RTC module and start counting failed\n");

                while (1);
            }
        }
    }


    /* clear tick status */
    RTC_CLEAR_TICK_INT_FLAG();

    /* Enable RTC Tick interrupt */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable Wake-up Timer */
    CLK_ENABLE_RTCWK();

    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    printf("Enter to DPD Power-Down mode......\n");

    while (!IsDebugFifoEmpty()) {}; /* waits for TX empty */

    /* Enter to Power-down mode */
    CLK_PowerDown();

    /* Wait for Power-down mode wake-up reset happen */
    while (1);
}


/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by RTC Alarm                             */
/*-----------------------------------------------------------------------------------------------------------*/
void  WakeUpRTCAlarmFunction(uint32_t u32PDMode)
{
    S_RTC_TIME_DATA_T sWriteRTC;
    uint32_t u32TimeOutCnt = SystemCoreClock; // 1 second timeout

    /* enable RTC peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;

    /* Select RTC clock source as LIRC */
    RTC->LXTCTL |= RTC_LXTCTL_C32KS_Msk;

    /* Open RTC and start counting */
    RTC->INIT = RTC_INIT_KEY;

    if (RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        RTC->INIT = RTC_INIT_KEY;

        while (RTC->INIT != RTC_INIT_ACTIVE_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Initialize RTC module and start counting failed\n");

                while (1);
            }
        }
    }

    /* Open RTC */
    sWriteRTC.u32Year       = 2017;
    sWriteRTC.u32Month      = 7;
    sWriteRTC.u32Day        = 26;
    sWriteRTC.u32DayOfWeek  = 3;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 4;
    sWriteRTC.u32Second     = 10;
    sWriteRTC.u32TimeScale  = 1;
    RTC_Open(&sWriteRTC);

    /* Set RTC alarm date/time */
    sWriteRTC.u32Year       = 2017;
    sWriteRTC.u32Month      = 7;
    sWriteRTC.u32Day        = 26;
    sWriteRTC.u32DayOfWeek  = 3;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 4;
    sWriteRTC.u32Second     = 15;
    RTC_SetAlarmDateAndTime(&sWriteRTC);

    printf("# Set RTC current date/time: 2017/07/26 15:04:10.\n");
    printf("# Set RTC alarm date/time:   2017/07/26 15:04:%u.\n", sWriteRTC.u32Second);


    /* clear alarm status */
    RTC_CLEAR_ALARM_INT_FLAG();

    /* Enable RTC alarm interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable Wake-up Timer */
    CLK_ENABLE_RTCWK();

    printf("Enter to DPD Power-Down mode......\n");

    while (!IsDebugFifoEmpty()) {}; /* waits for TX empty */

    /* Enter to Power-down mode */
    CLK_PowerDown();

    /* Wait for Power-down mode wake-up reset happen */
    while (1);
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by RTC Alarm                             */
/*-----------------------------------------------------------------------------------------------------------*/
void  WakeUpRTCTamperFunction(uint32_t u32PDMode)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; // 1 second timeout

    /* enable RTC peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;

    /* Open RTC and start counting */
    RTC->INIT = RTC_INIT_KEY;

    if (RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        RTC->INIT = RTC_INIT_KEY;

        while (RTC->INIT != RTC_INIT_ACTIVE_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Initialize RTC module and start counting failed\n");

                while (1);
            }
        }
    }

    RTC_StaticTamperEnable(RTC_TAMPER0_SELECT, RTC_TAMPER_LOW_LEVEL_DETECT, RTC_TAMPER_DEBOUNCE_DISABLE);

    /* Clear Tamper0 status */
    RTC_CLEAR_TAMPER_INT_FLAG(RTC_INTSTS_TAMP0IF_Msk);
    /* Disable Spare Register */
    RTC->SPRCTL = (1 << 5);

    RTC_EnableInt(RTC_INTEN_TAMP0IEN_Msk);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Enable Wake-up Timer */
    CLK_ENABLE_RTCWK();

    printf("Enter to DPD Power-Down mode......\n");

    while (!IsDebugFifoEmpty()) {}; /* waits for TX empty */

    /* Enter to Power-down mode */
    CLK_PowerDown();

    /* Wait for Power-down mode wake-up reset happen */
    while (1);
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                                                                  */
/*-----------------------------------------------------------------------------------------------------------*/
void CheckPowerSource(void)
{
    const uint32_t u32RegRstsrc = CLK_GetPMUWKSrc();

    printf("Power manager Power Manager Status 0x%x\n", u32RegRstsrc);

    if (u32RegRstsrc & CLK_PMUSTS_LVRWK_Msk)
        printf("Wake-up source is LVR.\n");

    if (u32RegRstsrc & CLK_PMUSTS_PINWK4_Msk)
        printf("Wake-up source is Wake-up Pin(GPF.6).\n");

    if (u32RegRstsrc & CLK_PMUSTS_PINWK3_Msk)
        printf("Wake-up source is Wake-up Pin(GPB.12).\n");

    if (u32RegRstsrc & CLK_PMUSTS_PINWK2_Msk)
        printf("Wake-up source is Wake-up Pin(GPB.2).\n");

    if (u32RegRstsrc & CLK_PMUSTS_PINWK1_Msk)
        printf("Wake-up source is Wake-up Pin(GPB.0).\n");

    if (u32RegRstsrc & CLK_PMUSTS_RTCWK_Msk)
        printf("Wake-up source is RTC.\n");

    if (u32RegRstsrc & CLK_PMUSTS_TMRWK_Msk)
        printf("Wake-up source is Wake-up Timer.\n");

    if (u32RegRstsrc & CLK_PMUSTS_PINWK0_Msk)
        printf("Wake-up source is Wake-up Pin(GPC.0).\n");
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for Check System Reset Status                                                                   */
/*-----------------------------------------------------------------------------------------------------------*/
void CheckResetStatus(void)
{
    static uint32_t u32ResetCounter = 1UL;

    const uint32_t u32RestStatus = SYS_GetResetSrc();

    /* Reset status register included PMURF,PINRF and PORF after reset */
    printf("Reset Status 0x%x,included...\n", u32RestStatus);

    if (u32RestStatus & SYS_RSTSTS_VBATLVRF_Msk)
        printf("[%d] VBAT LVR Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_CPULKRF_Msk)
        printf("[%d] CPU Lockup Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_CPURF_Msk)
        printf("[%d] CPU Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_PMURF_Msk)
        printf("[%d] PMU Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_SYSRF_Msk)
        printf("[%d] System Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_BODRF_Msk)
        printf("[%d] BOD Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_LVRF_Msk)
        printf("[%d] LVR Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_WDTRF_Msk)
        printf("[%d] WDT Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_PINRF_Msk)
        printf("[%d] nRESET Pin Reset Flag\n", u32ResetCounter++);

    if (u32RestStatus & SYS_RSTSTS_PORF_Msk)
        printf("[%d] POR Reset Flag\n", u32ResetCounter++);

    /* Clear all reset flag */
    SYS->RSTSTS = SYS->RSTSTS;
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
    GPIO_DISABLE_DIGITAL_PATH(PF, BIT4 | BIT5);
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

#else

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Select HCLK clock source as HXT and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Set PLL frequency */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, PLL_CLOCK);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, BIT2 | BIT3);
#endif

    /*Enable external 32768Hz XTAL */
    LXT_Enable();
    /* Enable LIRC clock (Internal RC 38.4KHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    /* Wait for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as PCLK0 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART0 multi-function pins, RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PF   multi-function pins for Tamper0 */
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF6MFP_Msk);
    SYS->GPF_MFPL |= SYS_GPF_MFPL_PF6MFP_TAMPER0 ;

    /* Set multi-function pins for CLKO(PA.03, PB.14, PD.12, PF.14) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA3MFP_Msk) | SYS_GPA_MFPL_PA3MFP_CLKO;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) | SYS_GPB_MFPH_PB14MFP_CLKO;
    SYS->GPD_MFPH = (SYS->GPD_MFPH & ~SYS_GPD_MFPH_PD12MFP_Msk) | SYS_GPD_MFPH_PD12MFP_CLKO;
    SYS->GPF_MFPH = (SYS->GPF_MFPH & ~SYS_GPF_MFPH_PF14MFP_Msk) | SYS_GPF_MFPH_PF14MFP_CLKO;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t u8Item;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* ---------- Turn off RTC  -------- */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;

    RTC->INTEN = 0;
    CLK->APBCLK0 &= ~CLK_APBCLK0_RTCCKEN_Msk;

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 0, 1);

    /* Get power manager wake up source */
    CheckPowerSource();

    /* Get Reset Status */
    CheckResetStatus();

    printf("+----------------------------------------------------------------+\n");
    printf("|    DPD Power-down Mode and Wake-up Sample Code.                |\n");
    printf("|    Please Select Wake up source.                               |\n");
    printf("+----------------------------------------------------------------+\n");
    printf("|[1] DPD Wake-up Pin(PC.0) trigger type is rising edge.          |\n");
    printf("|[2] DPD Wake-up TIMER time-out interval is 1024 OSC38.4K clocks.|\n");
    printf("|[3] DPD Wake-up by RTC Tick(1 second).                          |\n");
    printf("|[4] DPD Wake-up by RTC Alarm.                                   |\n");
    printf("|[5] DPD Wake-up by RTC Tamper0.                                 |\n");
    printf("+----------------------------------------------------------------+\n");
    u8Item = getchar();

    switch (u8Item)
    {
        case '1':
            WakeUpPinFunction(CLK_PMUCTL_PDMSEL_DPD, CLK_DPDWKPIN_RISING);
            break;

        case '2':
            WakeUpTimerFunction(CLK_PMUCTL_PDMSEL_DPD, CLK_PMUCTL_WKTMRIS_1024);
            break;

        case '3':
            WakeUpRTCTickFunction(CLK_PMUCTL_PDMSEL_DPD);
            break;

        case '4':
            WakeUpRTCAlarmFunction(CLK_PMUCTL_PDMSEL_DPD);
            break;

        case '5':
            WakeUpRTCTamperFunction(CLK_PMUCTL_PDMSEL_DPD);
            break;

        default:
            break;
    }

    while (1);
}
