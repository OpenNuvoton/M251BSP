/**************************************************************************//**
 * @file     TK_Main.c
 * @version  V1.00
 * @brief    Demonstrate how to TK14 in the NPD Mode to Wake Up for NuMaker-M258KE/M258KG board;
 *           TK1 for NuMaker-M256SD board.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "tklib.h"
#include "TK_Demo.h"

volatile uint8_t u8EventKeyScan = 0;
volatile int8_t i8SliderPercentage = 0;
volatile int8_t i8WheelPercentage = 0;
volatile int8_t i8KeyVal, i8KeyValPre;

#define TK1     (1)
#define TK14    (14)

#define eMAIN_APP_IDLE_STATE                    (0)
#define eMAIN_APP_IDLE_HOLD_STATE               (1)
#define eMAIN_APP_TK_POWERDOWN_STATE            (2)
#define eMAIN_APP_TK_POWERDOWN_HOLD_STATE       (3)

#ifdef MASS_FINETUNE
    void TK_MassProduction(int8_t *pai8Signal);
#endif

void TickCallback_KeyScan(void)
{
    u8EventKeyScan = 1;
}

int8_t SliderPercentage(int8_t *pu8SliderBuf, uint8_t u8Count)
{
    int8_t i;
    float i16M = 0.0, i16N = 0.0;

    for (i = 0; i < u8Count; i = i + 1)
    {
        if (pu8SliderBuf[i] > 1)
        {
            i16M += (i + 1) * pu8SliderBuf[i];
            i16N += pu8SliderBuf[i];
        }
    }

    return (int8_t)((((i16M * 10) / i16N) - 10) * 10) / ((u8Count - 1));
}

/**
  *  Report touching or un-touching state depends on debounce parameter you set on calibration stage
  *  For example,
  *      TK_ScanKey() may report someone key pressed but its signal is less than threshold of the key.
  *      The root cause is the key still under de-bounce stage.
  */
void TK_RawDataView(void)
{

    int8_t ai8Signal[TKLIB_TOL_NUM_KEY];

    if (u8EventKeyScan == 1)
    {
        u8EventKeyScan = 0;
        /**
          * TK_ScanKey() scan all enable key, slider and wheel channels.
          * i8Ret : Key/slider/wheel channel with max amplitude. -1: means no any key's amplitude over the key's threshold.
          * ai8Signal[]: The buffer size is equal to the M258 TK channels. It reports the signal amplitude on this round
          */
        int8_t i8Ret = TK_ScanKey(&ai8Signal[0]);

        i8KeyVal = i8Ret;

#ifdef MASS_FINETUNE
        TK_MassProduction(ai8Signal);
#endif

        int8_t ai8TmpSignal[TKLIB_TOL_NUM_KEY];

#if defined(OPT_SLIDER)

        {
            /** To save buffer size, re-used the ai8Signal[] buffer
              * Remember that the buffer will be destroied
              */
            uint16_t u16ChnMsk;
            static uint8_t updatecount = 0;

            updatecount = updatecount + 1;

            if (updatecount < 5)
                return;

            updatecount = 0;

            u16ChnMsk = TK_GetEnabledChannelMask(TK_SLIDER);

            if (TK_CheckSliderWheelPressed(TK_SLIDER) == 1)
            {

                uint8_t u8Count = 0, i;

                for (i = 0; i < TKLIB_TOL_NUM_KEY ; i++)
                {
                    if (u16ChnMsk & (1ul << i))
                    {
                        ai8TmpSignal[u8Count] = ai8Signal[i];
                        u8Count = u8Count + 1;
                    }
                }

                i8SliderPercentage = TK_SliderPercentage(ai8TmpSignal, u8Count);
            }

        }

#endif
#if defined(OPT_WHEEL)

        {
            /** To save buffer size, re-used the ai8Signal[] buffer
              * Remember that the buffer will be destroyed
              */

            uint32_t u32ChnMsk = TK_GetEnabledChannelMask(TK_WHEEL);

            if (TK_CheckSliderWheelPressed(TK_WHEEL)  == 1)
            {

                uint8_t i, u8Count = 0;

                for (i = 0; i < TKLIB_TOL_NUM_KEY ; i++)
                {
                    if (u32ChnMsk & (1ul << i))
                    {
                        ai8TmpSignal[u8Count] = ai8Signal[i];
                        u8Count = u8Count + 1;
                    }
                }

                i8WheelPercentage = TK_WheelPercentage(ai8TmpSignal, u8Count);
                DBG_PRINTF("Wheel %d\n", i8WheelPercentage);
            }
        }

#endif

    }

}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
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

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
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

    /* Select IP clock source */
    /* Select UART0 clock source is HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

#endif

    /* Enable LIRC clock (Internal RC 38.4Hz) */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

#ifdef UART1_DBG
    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
#endif

    /* Enable TIMER2 peripheral clock */
    CLK_EnableModuleClock(TMR2_MODULE);

    /* Enable TK peripheral clock */
    CLK_EnableModuleClock(TK_MODULE);

    /* Enable PA/B/C/D/E/F peripheral clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    //set all pin to input mode
    GPIO_SetMode(PA, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13 | BIT14 | BIT15), GPIO_MODE_INPUT);
    GPIO_SetMode(PB, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13 | BIT14 | BIT15), GPIO_MODE_INPUT);
    GPIO_SetMode(PC, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12), GPIO_MODE_INPUT);
    GPIO_SetMode(PD, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13 | BIT15), GPIO_MODE_INPUT);
    GPIO_SetMode(PE, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13 | BIT14 | BIT15), GPIO_MODE_INPUT);
    GPIO_SetMode(PF, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT14 | BIT15), GPIO_MODE_INPUT);
    //set all pin to pull-up
    GPIO_SetPullCtl(PA, (BIT0 | BIT1 | BIT2 | BIT4 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13 | BIT14 | BIT15), GPIO_PUSEL_PULL_UP);
    GPIO_SetPullCtl(PB, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13 | BIT14 | BIT15), GPIO_PUSEL_PULL_UP);
    GPIO_SetPullCtl(PC, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12), GPIO_PUSEL_PULL_UP);
    GPIO_SetPullCtl(PD, (BIT1 | BIT5 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13), GPIO_PUSEL_PULL_UP);
    GPIO_SetPullCtl(PE, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13 | BIT14 | BIT15), GPIO_PUSEL_PULL_UP);
    GPIO_SetPullCtl(PF, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 | BIT14 | BIT15), GPIO_PUSEL_PULL_UP);
    /* Set PB multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

#ifdef UART1_DBG
    /* Set GPB multi-function pins for UART1 RXD and TXD */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB2MFP_Msk) | SYS_GPB_MFPL_PB2MFP_UART1_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB3MFP_Msk) | SYS_GPB_MFPL_PB3MFP_UART1_TXD;
#endif
}

int32_t main(void)
{
    uint32_t u32ChanelMsk;

    SYS_Init();

#ifdef  DEMO_CALIBRATION
    UART0_Init();
#endif

#ifdef UART1_DBG
    UART1_Init();
    printf("UART Init\n");
#endif

    int8_t i8Ret = TK_LoadPara(&u32ChanelMsk);

#ifdef DEMO_CALIBRATION

    /* Initialize FMC to Load TK setting and calibration data from flash */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    if (i8Ret == -1)
    {
        /** i8Ret = -1 means that no any calibration data stored in flash
          * If no any data stored in flash. Get TK setting and calibration data from UART port
          * Program will be blocked in the function until received START_CALIBRATION command. The return value will be 1
          */
        i8Ret = TK_GetPacket(&u32ChanelMsk);
    }

    /* Init TK Controller */
    TK_Init();

    /* Initialize Multiple Function Pins for TK */
    SetTkMultiFun(u32ChanelMsk);

    /* Init systick 20ms/tick */
    Init_SysTick();

    /* Install Tick Event Handler To Drive Key Scan */
    TickSetTickEvent(1, (void *)TickCallback_KeyScan);

    do
    {
        if (i8Ret == 1)
        {
            /** Receive Start calibration command
              * The function will be blocked until calibration done
              */
            TK_Calibration_Untouch();
            /* Inform UART module calibration done */
            UART_SetCalibrationDone();
        }

        i8Ret = TK_GetPacket(&u32ChanelMsk);

        /** May change configurations through UART port
          * Init TK Controller again
          */
        TK_Init();

        /* Initialize Multiple Function Pins for TK again */
        SetTkMultiFun(u32ChanelMsk);
    } while (1);

#endif /* DEMO_CALIBRATION */

#ifdef DEMO_FREERUN
    uint8_t g_u8MainState = eMAIN_APP_IDLE_STATE;

    if (i8Ret < 0)
    {
        /* DBG_PRINTF("Please run target TK_Application first to calibrate touchkey\n"); */
        while (1);
    }

    /* Init TK Controller */
    TK_Init();

    /* Initialize Multiple Function Pins for TK */
    SetTkMultiFun(u32ChanelMsk);

    /* Init systick 20ms/tick */
    Init_SysTick();

    /* Install Tick Event Handler To Drive Key Scan */
    TickSetTickEvent(1, (void *)TickCallback_KeyScan);

    g_u8MainState = eMAIN_APP_IDLE_HOLD_STATE;
    i8KeyValPre = 0xff;
    printf("IDLE\r\n");

    do
    {
        TK_RawDataView();

        if (i8KeyVal != i8KeyValPre)
        {
#if defined (M256D)

            if ((i8KeyVal == TK1) && (i8KeyValPre == (int8_t)0xff))
#else
            if ((i8KeyVal == TK14) && (i8KeyValPre == (int8_t)0xff))
#endif
            {
                if (g_u8MainState == eMAIN_APP_IDLE_STATE)
                    g_u8MainState = eMAIN_APP_IDLE_HOLD_STATE;
                else if (g_u8MainState == eMAIN_APP_TK_POWERDOWN_STATE)
                    g_u8MainState = eMAIN_APP_TK_POWERDOWN_HOLD_STATE;
            }

#if defined (M256D)
            else if ((i8KeyVal == (int8_t)0xFF) && (i8KeyValPre == (int8_t)TK1))
#else
            else if ((i8KeyVal == (int8_t)0xFF) && (i8KeyValPre == (int8_t)TK14))
#endif
            {
                if (g_u8MainState == eMAIN_APP_IDLE_HOLD_STATE)
                    g_u8MainState = eMAIN_APP_TK_POWERDOWN_STATE;
                else if (g_u8MainState == eMAIN_APP_TK_POWERDOWN_HOLD_STATE)
                    g_u8MainState = eMAIN_APP_IDLE_STATE;
            }

            i8KeyValPre = i8KeyVal;

            switch (g_u8MainState)
            {
                case eMAIN_APP_IDLE_STATE:
                    printf("IDLE\r\n");
                    break;

                case eMAIN_APP_TK_POWERDOWN_STATE:
                    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_LIRC, 0);
                    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 50);
                    NVIC_DisableIRQ(TMR2_IRQn);
                    TIMER_ClearIntFlag(TIMER2);

                    TIMER2->INTSTS = TIMER2->INTSTS;
                    TK_ConfigPowerDown(0);
                    TIMER2->TRGCTL |= TIMER_TRGCTL_TRGTK_Msk;
                    TIMER_Start(TIMER2);

                    printf("Enter Power Down\r\n");

                    while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk) == 0) {}

                    CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_PD);
                    CLK_PowerDown();

                    /* Disable TMRTRG */
                    TIMER2->TRGCTL &= ~TIMER_TRGCTL_TRGTK_Msk;
                    printf("Wake Up\r\n");
                    FMC_ENABLE_ISP();
                    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
                    TIMER_ClearIntFlag(TIMER2);

                    /* Init systick 20ms/tick */
                    Init_SysTick();

                    /* Install Tick Event Handler To Drive Key Scan */
                    TickSetTickEvent(1, (void *)TickCallback_KeyScan);
                    break;
            }
        }
    } while (1);

#endif  /* DEMO_FREERUN */


}


