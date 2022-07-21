/**************************************************************************//**
 * @file     TK_SysTick.c
 * @version  V1.00
 * @brief    Touch key tick setting.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"
#include "TK_Demo.h"
/******************************************************************************
 * SysTick is used as system time base.
 * It provided upper to install/remove callback function.
 * Initial sysTick function: Init_SysTick()
 * Install callback function: TickSetTickEvent()
 * Remove callback function: TickClearTickEvent()
 ******************************************************************************/
typedef void (*sys_pvTimeFunPtr)(void);   /* function pointer */
typedef struct timeEvent_t
{
    uint8_t             active;
    unsigned long       initTick;
    unsigned long       curTick;
    sys_pvTimeFunPtr    funPtr;
} TimeEvent_T;

#define TICKEVENTCOUNT                                  (3) /* 1 ~ 3 */

static TimeEvent_T tTime0Event[TICKEVENTCOUNT];
volatile uint8_t  _sys_uTime0EventCount = 0;             /* Speed up interrupt response time if no callback function */
volatile uint32_t u32Tick = 0;


void TickClearTickEvent(uint8_t u8TimeEventID)
{
    if (u8TimeEventID >= TICKEVENTCOUNT)
        return;

    if (tTime0Event[u8TimeEventID].active == TRUE)
    {
        tTime0Event[u8TimeEventID].active = FALSE;
        _sys_uTime0EventCount--;
    }
}


char TickSetTickEvent(unsigned long uTimeTick, void *pvFun)
{
    int  i;
    int u8TimeEventID = 0;

    for (i = 0; i < TICKEVENTCOUNT; i++)
    {
        if (tTime0Event[i].active == FALSE)
        {
            tTime0Event[i].active = TRUE;
            tTime0Event[i].initTick = uTimeTick;
            tTime0Event[i].curTick = uTimeTick;
            tTime0Event[i].funPtr = (sys_pvTimeFunPtr)pvFun;
            u8TimeEventID = i;
            _sys_uTime0EventCount += 1;
            break;
        }
    }

    if (i == TICKEVENTCOUNT)
    {
        return -1;               /* -1 means invalid channel */
    }
    else
    {
        return u8TimeEventID;    /* Event ID start from 0*/
    }
}

void Init_SysTick(void)
{
    uint8_t i;

    _sys_uTime0EventCount = 0;

    /* Remove all callback function */
    for (i = 0; i < TICKEVENTCOUNT; i++)
        TickClearTickEvent(i);

    _sys_uTime0EventCount = 0;

    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, TK_SCAN_EVENT_FREQ);
    TIMER_EnableInt(TIMER2);
    NVIC_EnableIRQ(TMR2_IRQn);
    TIMER_Start(TIMER2);

}

void TMR2_IRQHandler(void)
{
    TIMER_ClearIntFlag(TIMER2);

    u32Tick += 1;

    if (u32Tick >= 0xfffffff)
    {
        u32Tick = 0;
    }

    if (_sys_uTime0EventCount)
    {
        uint8_t i;

        for (i = 0; i < TICKEVENTCOUNT; i++)
        {
            if (tTime0Event[i].active)
            {
                tTime0Event[i].curTick--;

                if (tTime0Event[i].curTick == 0)
                {
                    (*tTime0Event[i].funPtr)();
                    tTime0Event[i].curTick = tTime0Event[i].initTick;
                }
            }
        }
    }
}

