/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Read the smartcard ATR from smartcard interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sclib.h"

#define SC_INTF 0  // Smartcard interface 0
#define PLL_CLOCK       FREQ_48MHZ

#define SW_CARD_DETECT_EN 0 /*  Card detection implemented for chips without card detection pin*/

#if SW_CARD_DETECT_EN
    volatile uint32_t g_u32CardIsInserted = 0; /*Card dectection pin status, 1: inserted; 0: removed*/
#endif

/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @retval None
  */
void SC0_IRQHandler(void)
{
    /* Please don't remove any of the function calls below */
#if !SW_CARD_DETECT_EN
    // Card insert/remove event occurred, no need to check other event...
    if (SCLIB_CheckCDEvent(SC_INTF))
        return;

#endif
    // Check if there's any timeout event occurs. If so, it usually indicates an error
    SCLIB_CheckTimeOutEvent(SC_INTF);

    // Check transmit and receive interrupt, all data transmission take place in this function
    SCLIB_CheckTxRxEvent(SC_INTF);

    /*
        Check if there's any transmission error occurred (e.g. parity error, frame error...)
        These errors will induce SCLIB to deactivation smartcard eventually.
    */
    SCLIB_CheckErrorEvent(SC_INTF);

    return;
}

#if SW_CARD_DETECT_EN
void GPB_IRQHandler(void)
{
    /* To check if PB.0 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT0))
    {
        SCLIB_Deactivate(SC_INTF);

        g_u32CardIsInserted = PB0; /*Assume level HIGH indicates card inserted*/
        GPIO_CLR_INT_FLAG(PB, BIT0);

        if (SCLIB_CheckCDEvent_ByVar(SC_INTF, &g_u32CardIsInserted))
        {
            // Card insert/remove event occurred, no need to check other event...
            return;
        }
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */

        printf("Un-expected interrupts.0x%08x\n", PB->INTSRC);
        PB->INTSRC = PB->INTSRC;
    }
}

#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable IP clock */
    CLK_EnableModuleClock(SC0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select SC0 clock source from HIRC and divide the clock to 3MHz*/
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(16));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk
                       | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SC0_CLK | SYS_GPA_MFPL_PA1MFP_SC0_DAT
                      | SYS_GPA_MFPL_PA2MFP_SC0_RST | SYS_GPA_MFPL_PA3MFP_SC0_PWR);

#if SW_CARD_DETECT_EN
    CLK_EnableModuleClock(GPB_MODULE);

    /*For Chip without CD pin*/
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB0MFP_Msk;
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_GPIO;
#else
    SYS->GPA_MFPL &= ~SYS_GPA_MFPL_PA4MFP_Msk;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA4MFP_SC0_nCD;
#endif

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    SCLIB_CARD_INFO_T sSclibCardInfo;
    uint32_t u32RetVal, u32Idx;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\nThis sample code reads ATR from smartcard\n");

    /*
        Open smartcard interface 0. CD pin state low indicates card insert and PWR pin low raise VCC pin to card
        The second and third parameter needs to be set according to the board design
    */
#if SW_CARD_DETECT_EN
    /*
        implement S/W card detection for chip without card detection pin
    */
    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 0, GPIO_INT_BOTH_EDGE);

    NVIC_EnableIRQ(GPB_IRQn);
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);

    SC_Open(SC0, SC_PIN_STATE_IGNORE, SC_PIN_STATE_HIGH);
#else
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
#endif

    NVIC_EnableIRQ(SC0_IRQn);




#if SW_CARD_DETECT_EN

    while ((!g_u32CardIsInserted)) {};

#else
    while (SC_IsCardInserted(SC0) == FALSE) {};

#endif

    /*
        Activate slot 0, and disable EMV2000 check during card activation
        EMV is a technical standard for smart payment cards and for payment terminals and automated teller
        machines that can accept them. It has a stricter checking rule than ISO 7816-3. If the second
        parameter set as TRUE, SCLIB will report activation failure for cards comply with ISO 7816 but not EMV2000
    */
    u32RetVal = SCLIB_Activate(SC_INTF, FALSE);

    if (u32RetVal == SCLIB_SUCCESS)
    {
        /*
            Use SCLIB_GetCardInfo to get information about the card, which includes ATR.

            An Answer To Reset (ATR) is a message output by a contact Smart Card conforming to
            ISO/IEC 7816 standards, following electrical reset of the card's chip by a card reader.
            The ATR conveys information about the communication parameters proposed by the card,
            and the card's nature and state.                                --Wikipedia
        */
        SCLIB_GetCardInfo(SC_INTF, &sSclibCardInfo);
        printf("ATR: ");

        for (u32Idx = 0; u32Idx < sSclibCardInfo.ATR_Len; u32Idx++)
            printf("%02x ", sSclibCardInfo.ATR_Buf[u32Idx]);

        printf("\n");
        SCLIB_Deactivate(SC_INTF);
    }
    else
        printf("Smartcard activate failed\n");


    // No operating system, so we have no where to go, just loop forever.

    while (1);

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/


