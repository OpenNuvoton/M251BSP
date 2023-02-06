/******************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Demonstrate how to implement a USB mouse device with BC1.2 (Battery Charging).
*            which shows different type of charging port after connected USB port.
 *           The mouse cursor will move automatically when this mouse device is connected to PC by USB.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"
#include "hid_mouse.h"
#include <stdio.h>

#define CRYSTAL_LESS 1 /* CRYSTAL_LESS must be 1 if USB clock source is HIRC*/
#define TRIM_INIT (SYS_BASE + 0x118)

volatile S_USBD_BC12_PD_STATUS g_sChargeStatus = USBD_BC12_VBUS_OFF;

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal HIRC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select module clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));

    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();
}

/**
 * @brief Utility for BC1.2 timing use(USBD_BC_Detect)
 *
 * @param[in] us
 * @return none
 */
int32_t systickDly(TIMER_T *dummy_for_compatible __attribute__((unused)), uint32_t us)
{
    uint32_t u32TimeOutCnt = SystemCoreClock;

    SysTick->CTRL = 0;
    SysTick->VAL = 0x00;

    SysTick->LOAD = us * (SystemCoreClock / 1000000); /*depend on core clock*/
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    g_CLK_i32ErrCode = 0;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            g_CLK_i32ErrCode = CLK_TIMEOUT_ERR;
            break;
        }
    }

    /* Disable SysTick counter */
    SysTick->CTRL = 0;

    return g_CLK_i32ErrCode;
}

/**
 * @brief BC1.2 Charge Port Detection
 *
 * @param[in] pu32Timer     NULL: timing source is systick timer;
 *                          TIMER0 ~ TIMERn: timing source is peripheral H/W timer
 * @return status code (S_USBD_BC12_PD_STATUS)
 *
 * @details We recommend that the H/W timer for timing counting if H/W resource is sufficient
 *          because the BSP APIs handle the setting details.
 *          User has to take cares the setting details if systick is used for timing counting.
 */
S_USBD_BC12_PD_STATUS USBD_BC_Detect(TIMER_T *pu32TimerSrc)
{

    /*
     * TDCD_TIMEOUT(BC1.2 SPEC): 300ms ~ 900ms
     * notice that overflow value in CPU@48MHz if systick is used as the timer
     * value range of Systick  : 300ms ~ 349 ms if CPU@48 MHz
     * value range of H/W timer: 300ms ~ 850 ms if CPU@48 MHz, TIMER using HIRC(48MHz)
     */
#define DCD_TIMEOUT_PERIOD_US 500000UL
#define ENABLE_BC12_DBG_MSG 0
#if ENABLE_BC12_DBG_MSG
#define DBG_MSG printf
#else
#define DBG_MSG(...)
#endif

#define BC_DELAY(us) pfnBC_Delay(pu32TimerSrc, us)

    int32_t (*pfnBC_Delay)(TIMER_T * dummy, uint32_t us);

    if (pu32TimerSrc == NULL)
    {
        pfnBC_Delay = systickDly;
    }
    else if ((pu32TimerSrc == TIMER0)
             || (pu32TimerSrc == TIMER1)
             || (pu32TimerSrc == TIMER2)
             || (pu32TimerSrc == TIMER3)
            )
    {
        pfnBC_Delay = TIMER_Delay;
    }
    else
    {
        DBG_MSG("Invalid delay timer source.\n");
        return USBD_BC12_ERROR; // Invalid delay timer source
    }

    if (USBD_IS_ATTACHED() == 0)
    {
        USBD->BCDC = 0;
        return USBD_BC12_VBUS_OFF;
    }
    else
    {
        BC_DELAY(1000); // 1ms
        DBG_MSG("VBUS Plug\n");
    }

    DBG_MSG("Check VBUS threshold voltage");
    USBD->BCDC = USBD_BCDC_BCDEN_Msk;

    BC_DELAY(30000); // 30 ms: wait PHY LDO stable
    USBD->BCDC |= USBD_BCDC_DETMOD_VBUS;

    while ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) == USBD_BCDC_DETSTS_VBUS_UNREACH) {}

    DBG_MSG("\nCheck VBUS OK\n");
    DBG_MSG("Check data pin contact status\n");
    USBD->BCDC = USBD_BCDC_BCDEN_Msk | USBD_BCDC_DETMOD_DCD;

    if (pu32TimerSrc == NULL) /*Use systick timer*/
    {

DCD_REPEAT_SYSTICK:
        SysTick->CTRL = 0;
        SysTick->VAL = (0x00);

        SysTick->LOAD = DCD_TIMEOUT_PERIOD_US * (SystemCoreClock / 1000000); /*Min: depend on core clock:max value 349 ms@48MHz clcok*/
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        while (1)
        {
            if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) ==
                    USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
            {

                BC_DELAY(5000); // 10 ms: SPEC TDCD_DBNC

                if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) !=
                        USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    goto DCD_REPEAT_SYSTICK;
                }

                BC_DELAY(4000); // 10 ms: SPEC TDCD_DBNC

                if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) !=
                        USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    goto DCD_REPEAT_SYSTICK;
                }

                BC_DELAY(1000); // 10 ms: SPEC TDCD_DBNC

                if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) !=
                        USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    goto DCD_REPEAT_SYSTICK;
                }

                DBG_MSG(" - DCD Data Contact\n");
                break;
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                SysTick->CTRL = 0;
                DBG_MSG(" - Timeout\n");
                break;
            }
        }
    }
    else   /*Use Hardware Timer*/
    {
DCD_REPEAT_TIMER:
        {
            uint32_t u32Usec = DCD_TIMEOUT_PERIOD_US;
            uint32_t u32Clk = TIMER_GetModuleClock(pu32TimerSrc);
            uint32_t u32Prescale = 0UL, u32Delay = (SystemCoreClock / u32Clk) + 1UL;
            uint32_t u32Cmpr, u32NsecPerTick;


            // Clear current timer configuration/
            pu32TimerSrc->CTL = 0UL;
            pu32TimerSrc->EXTCTL = 0UL;

            if (u32Clk <= 1000000UL) // min delay is 1000 us if timer clock source is <= 1 MHz
            {
                if (u32Usec < 1000UL)
                    u32Usec = 1000UL;

                if (u32Usec > 1000000UL)
                    u32Usec = 1000000UL;
            }
            else
            {
                if (u32Usec < 100UL)
                    u32Usec = 100UL;

                if (u32Usec > 1000000UL)
                    u32Usec = 1000000UL;
            }

            if (u32Clk <= 1000000UL)
            {
                u32Prescale = 0UL;
                u32NsecPerTick = 1000000000UL / u32Clk;
                u32Cmpr = (u32Usec * 1000UL) / u32NsecPerTick;
            }
            else
            {
                u32Cmpr = u32Usec * (u32Clk / 1000000UL);
                u32Prescale = (u32Cmpr >> 24); /* for 24 bits CMPDAT */

                if (u32Prescale > 0UL)
                    u32Cmpr = u32Cmpr / (u32Prescale + 1UL);
            }

            pu32TimerSrc->CMP = u32Cmpr;
            pu32TimerSrc->CTL = TIMER_CTL_CNTEN_Msk | TIMER_ONESHOT_MODE | u32Prescale;

            // When system clock is faster than timer clock, it is possible timer active
            // bit cannot set in time while we check it. And the while loop below return
            // immediately, so put a tiny delay here allowing timer start counting and
            // raise active flag.
            for (; u32Delay > 0UL; u32Delay--)
            {
                __NOP();
            }

            while (1)
            {
                //using S/W debounce, TDCD_DBNC is 10 ms totally.
                if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) ==
                        USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    BC_DELAY(5000); //5000 us debounce

                    if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) !=
                            USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                    {
                        goto DCD_REPEAT_TIMER;
                    }

                    BC_DELAY(3000); //3000 us debounce

                    if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) !=
                            USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                    {
                        goto DCD_REPEAT_TIMER;
                    }

                    BC_DELAY(2000); //2000 us debounce

                    if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) !=
                            USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                    {
                        goto DCD_REPEAT_TIMER;
                    }

                    DBG_MSG(" - DCD Data Contact\n");
                    break;
                }

                if (!(pu32TimerSrc->CTL & TIMER_CTL_ACTSTS_Msk))
                {
                    DBG_MSG(" - DCD Timeout\n");
                    break;
                }
            }
        }
    }

    USBD->BCDC = USBD_BCDC_BCDEN_Msk | USBD_BCDC_DETMOD_PD;
    DBG_MSG("BC1.2 - Primary Detect: ");

    /* Delay 40ms */
    BC_DELAY(40000); // SPEC: TVDPSRC_ON
    BC_DELAY(1000);  // tune

    if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) == USBD_BCDC_DETSTS_PD_SDP_NUSP)
    {
        USBD->BCDC = 0; //important: to prevent next loop un-expected paulse
        return USBD_BC12_SDP;
    }
    else
    {
        // switch back to IDLE than switch to USBD_BCDC_DETMOD_SD
        USBD->BCDC = USBD_BCDC_BCDEN_Msk | USBD_BCDC_DETMOD_IDLE; /* hardware limitation */
        BC_DELAY(10000);                                          /* TVDMSRC_DIS         */

        /* Port detect - Secondary detect */
        DBG_MSG("BC1.2 - Secondary Detect: ");
        USBD->BCDC = USBD_BCDC_BCDEN_Msk | USBD_BCDC_DETMOD_SD;
        BC_DELAY(40000); // SPEC: TVDMSRC_ON
        BC_DELAY(5000);  // tune

        if ((USBD->BCDC & USBD_BCDC_DETSTS_Msk) == USBD_BCDC_DETSTS_SD_CDP)
        {
            DBG_MSG("* CDP\n");
            USBD->BCDC = 0; //important: to prevent next loop un-expected pulse
            return USBD_BC12_CDP;
        }
        else
        {
            DBG_MSG("* DCP\n");
            return USBD_BC12_DCP;
        }
    }

#undef ENABLE_BC12_DBG_MSG
#undef DBG_MSG
#undef DCD_TIMEOUT_PERIOD_US
#undef BC_DELAY
}

void PowerDown()
{

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);
    SYS_UnlockReg();
    CLK_PowerDown();

    if (!(USBD->ATTR & USBD_ATTR_USBEN_Msk))
        USBD_ENABLE_USB();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if (CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();
    SYS_Init();
    SYS_LockReg();

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+-----------------------------------------------------+\n");
    printf("|     NuMicro USB HID Mouse Sample Code with BC1.2    |\n");
    printf("+-----------------------------------------------------+\n");


restart:

    while (1)
    {
        g_sChargeStatus = USBD_BC_Detect(TIMER0);

        if (USBD_BC12_SDP == g_sChargeStatus)
        {
            printf("==>is SDP\n");
            break;
        }

        if (USBD_BC12_CDP == g_sChargeStatus)
        {
            printf("==>is CDP\n");
            break;
        }

        if (USBD_BC12_DCP == g_sChargeStatus)
        {
            printf("==>is DCP\n");

            while (USBD_IS_ATTACHED()); // keep draw current until detach

            printf("==>VBUS detach\n");
            USBD->BCDC = 0; //important: PET shows that D+ must keep whole period during test.
            continue; // recognize
        }

        if (USBD_BC12_VBUS_OFF == g_sChargeStatus)
        {
            continue;
        }

        if (USBD_BC12_ERROR == g_sChargeStatus)
        {
            printf("parameter error\n");

            while (1);
        }
    }


    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 1, 0);

    /* Endpoint configuration */
    HID_Init();

    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim value */
    u32TrimInit = M32(TRIM_INIT);

    /* Clear SOF */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
#endif

    while (1)
    {
#if CRYSTAL_LESS

        /* Start USB trim function if it is not enabled. */
        if ((SYS->HIRCTRIMCTL & SYS_HIRCTRIMCTL_FREQSEL_Msk) != 0x1)
        {
            /* Start USB trim only when USB signal arrived */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /*
                    USB clock trim function:
                    HIRC Trimming with boundary function enhances robustility
                    and keeps HIRC in right frequency while receiving unstable USB
                   signal
                */
                SYS->HIRCTRIMCTL = (0x1 << SYS_HIRCTRIMCTL_REFCKSEL_Pos) |
                                   (0x1 << SYS_HIRCTRIMCTL_FREQSEL_Pos) |
                                   (0x0 << SYS_HIRCTRIMCTL_LOOPSEL_Pos) |
                                   (0x1 << SYS_HIRCTRIMCTL_BOUNDEN_Pos) |
                                   (20 << SYS_HIRCTRIMCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when any error found */
        if (SYS->HIRCTRIMSTS &
                (SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable USB clock trim function */
            SYS->HIRCTRIMCTL = 0;

            /* Clear trim error flags */
            SYS->HIRCTRIMSTS =
                SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
        }

#endif

        if ((USBD->VBUSDET & USBD_VBUSDET_VBUSDET_Msk) == 0x0)
        {
            printf("VBUS Un-Plug\n");
            USBD_SET_SE0();
            goto restart;
        }

        /* Enter power down when USB suspend */
        if (g_u8Suspend && (USBD->VBUSDET & USBD_VBUSDET_VBUSDET_Msk))
            PowerDown();

        //HID_UpdateMouseData();
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
