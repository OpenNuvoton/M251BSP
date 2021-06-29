/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use SC embedded timer
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sclib.h"

#define SC_INTF 0  // Smartcard interface 0
#define PLL_CLOCK       FREQ_48MHZ

/* The definition of commands used in this sample code and directory structures could
   be found in GSM 11.11 which is free for download from Internet.
   Different from the command defined in ISO 7816-4, CLS of SIM command is 0xA0,
   So the command defined below starting with 0xA0 */

// Select File
const uint8_t g_au8SelectMF[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
const uint8_t g_au8SelectDF_TELECOM[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x7F, 0x10};
const uint8_t g_au8SelectEF_ADN[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x6F, 0x3A};
//Get Response
uint8_t g_au8GetResp[] = {0xA0, 0xC0, 0x00, 0x00, 0x00};
//Read Record
uint8_t g_au8ReadRec[] = {0xA0, 0xB2, 0x01, 0x04, 0x00};
//Verify CHV, CHV = Card Holder Verification information
uint8_t g_au8VerifyChv[] = {0xA0, 0x20, 0x00, 0x01, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t g_au8Buf[300];
uint32_t g_u32Len;

/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @return None
  */
void SC0_IRQHandler(void)
{
    // Please don't remove any of the function calls below
    if (SCLIB_CheckCDEvent(SC_INTF))
        return; // Card insert/remove event occurred, no need to check other event...

    if (SC_INTSTS_TMR0IF_Msk == SC_CHECK_INTSTS(SC0, SC_INTSTS_TMR0IF_Msk))
    {
        static uint32_t u32Sec = 1;

        SC_CLEAR_INTSTS(SC0, SC_INTSTS_TMR0IF_Msk);
        printf("%u sec\n", u32Sec++);
    }

    return;
}

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

    /* Select SC clock source from HIRC */
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk
                       | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SC0_CLK | SYS_GPA_MFPL_PA1MFP_SC0_DAT
                      | SYS_GPA_MFPL_PA2MFP_SC0_RST | SYS_GPA_MFPL_PA3MFP_SC0_PWR);

    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA4MFP_SC0_nCD;

    /* Lock protected registers */
    SYS_LockReg();
}


int main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\nThis sample code demo how to use SC embedded timer. \n");

    /* Open smartcard interface 1. CD pin state low indicates card insert and PWR pin high raise VCC pin to card */
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);

    /* Enable SC TIMER0 interrupt */
    SC_ENABLE_INT(SC0, SC_INTEN_TMR0IEN_Msk);
    NVIC_EnableIRQ(SC0_IRQn);

    /* real ETU divider value is "47+1=48", and the duration of each ETU is 48/SC_CLK(48MHz) */
    SC_SET_ETUDIV(SC0, 47);

    /* each 1,000,000 ETU will generate TIMER0 timeout event */
    SC_StartTimer(SC0, 0, SC_TMR_MODE_4, 1000000);  // timer counter will be reloaded.

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
