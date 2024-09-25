/**************************************************************************//**
 * @file     exeinsram.c
 * @version  V0.10
 * @brief    Implement a code and execute in SRAM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

void GPIO_SingleCycleIO_Test(void)
{
    uint32_t u32CounterTMR0 = 0, u32CounterTMR2 = 0, u32MicroSec, u32MHz;

    /* Configure TIMER2 to count PA.0 toggle event (falling edge) */
    TIMER2->CTL = TIMER_ONESHOT_MODE;
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);
    TIMER2->CTL |= TIMER_CTL_CNTEN_Msk;

    /* Configure TIMER0 to measure the elapsed time */
    TIMER0->CTL = TIMER_ONESHOT_MODE;
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;

    /* Toggle PA.0 state 50 times */
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;

    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;

    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;

    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;

    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;
    PA0 = 1;
    PA0 = 0;

    /* Get TIMER0 and TIMER2 counter */
    u32CounterTMR0 = TIMER0->CNT;
    /* avoid timer2 get gpio event too fast */
    __NOP();
    u32CounterTMR2 = TIMER2->CNT;

    /* Stop TIMER0 and TIMER2 */
    TIMER_Stop(TIMER0);
    TIMER_Stop(TIMER2);

    u32MicroSec = u32CounterTMR0 / CyclesPerUs;
    u32MHz = (u32CounterTMR2 * CyclesPerUs) / u32CounterTMR0;
    /* Print result */
    printf("Toggle speed measurement result\n");
    printf("=================================================\n");
    printf("GPIO Falling Edge Counts (A)        : %u\n",        u32CounterTMR2);
    printf("Total Elapsed Time       (B)        : %u.%02u(us)\n",  u32MicroSec, (u32CounterTMR0 - (u32MicroSec * CyclesPerUs)) * 100 / CyclesPerUs);
    printf("Average Toggle Speed     (C)=(A)/(B): %u.%02u(MHz)\n", u32MHz, ((u32CounterTMR2 * CyclesPerUs) - (u32MHz * u32CounterTMR0)) * 100 / u32CounterTMR0);
}
