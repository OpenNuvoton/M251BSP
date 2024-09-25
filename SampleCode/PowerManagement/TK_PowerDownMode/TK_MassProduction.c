/**************************************************************************//**
 * @file     TK_MassProduction.c
 * @version  V1.00
 * @brief    Touch key mass production function.
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

extern volatile uint8_t g_bIsFineTuneDone;
extern volatile uint8_t g_FineTuneDoneTimeOut;
int8_t i8MpTickEventID = 0;

/**
  * The function was called if time out only to make sure no risk condition
  *
  *
  */
void TK_MP_Close()
{
    TickClearTickEvent(i8MpTickEventID);                    /* Stop time out timer */
}

/**
  * The function is time out callback function.
  *
  *
  */
#if defined(MASS_FINETUNE)
void TickCallback_FineTuneTimeOut(void)
{
    g_FineTuneDoneTimeOut = 1;
    g_bIsFineTuneDone = 1;
    TK_MP_Close();
}


void TK_MP_Open()
{
    g_FineTuneDoneTimeOut = 0x0;
    g_bIsFineTuneDone = 0x0;
    i8MpTickEventID = TickSetTickEvent(150, (void *)TickCallback_FineTuneTimeOut);    /* 3 second time out */
}

#endif

#if defined(MASS_FINETUNE)
void TK_MassProduction(int8_t *pai8Signal)
{
    uint16_t u16ChnMsk;
    uint8_t i;
    S_TKFEAT *psTkFeat;

    psTkFeat = TK_GetFeaturePtr();


    if (g_FineTuneDoneTimeOut == 1)        /* Time out return directly */
        return;


    if (psTkFeat->u8BaseLineRound == 1)   /* Set by UART command - 'A': modify to update baseline time  */
    {
        u16ChnMsk = TK_GetEnabledChannelMask(TK_KEY);
        u16ChnMsk |= TK_GetEnabledChannelMask(TK_SLIDER);
        u16ChnMsk |= TK_GetEnabledChannelMask(TK_WHEEL);

        g_bIsFineTuneDone = 0xFF;

        for (i = 0; i < TKLIB_TOL_NUM_KEY ; i++)
        {
            if (u16ChnMsk & (1ul << i))
            {
                if (abs(pai8Signal[i]) > 4) /* Change to 4 from 2 @2020/09/02 */
                {
                    g_bIsFineTuneDone = 0;   /* If any channel's signal > 2, fine tune not yet complete */
                    break;
                }
            }
        }

        if (g_bIsFineTuneDone == 0xFF)
        {
            /* callback function if fine tune done */
            g_bIsFineTuneDone = 1;                                   /* Fine tune done */
        }
    }
}

#endif
