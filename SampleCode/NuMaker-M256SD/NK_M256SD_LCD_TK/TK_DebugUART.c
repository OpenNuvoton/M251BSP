/**************************************************************************//**
 * @file     TK_DebugUART.c
 * @version  V1.00
 * @brief    UART debug & connect tool setting.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"
#include "TK_Demo.h"

#ifdef DEMO_CALIBRATION
void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(UART0_IRQn);

}

#endif //DEMO_CALIBRATION

#ifdef UART1_DBG
//***********************************************************************************************************
//  Using UART1 for debug purpose.
//***********************************************************************************************************
void UART1_Init(void)
{
    /* Configure UART1 and set UART1 baud rate */
    UART_Open(UART1, 115200);
}

#endif /* OPT_UART1 */