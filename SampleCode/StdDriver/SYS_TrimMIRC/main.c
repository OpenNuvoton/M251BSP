/**************************************************************************//**
 * @file         main.c
 * @version      V3.00
 * @brief        Demonstrate how to use timer to trim MIRC
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TIMER       TIMER0
#define TRIM_LOOP   128

#define MIRC_WRITE_ADJ(u32Value) (outpw(0x40000110, u32Value))
#define MIRC_READ_ADJ() (inpw(0x40000110))
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*-----------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                   */
    /*-----------------------------------------------------------------------------------------------------*/

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

    /* Enable HIRC and MIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_MIRCEN_Msk);

    /* Wait for HIRC and MIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_MIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER module clock source as PCLK and TIMER module clock divider as 1 */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    /*-------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                               */
    /*-------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* Set PA multi-function pins for CLKO(PA.3) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA3MFP_Msk) | SYS_GPA_MFPL_PA3MFP_CLKO;

}

void UART0_Init(void)
{
    /*--------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                              */
    /*--------------------------------------------------------------------------------------------------------*/

    UART_Open(UART0, 115200); /* Configure UART0 and set UART0 Baud rate */
}

void TrimMIRC()
{
    uint32_t u32CapVal0, u32CapVal1, u32CapVal_Interval;
    uint32_t u32TrimLoopIndex, u32RCValue;
    uint32_t u32Freq_PCLK0_DIV_MIRC = ((CLK_GetPCLK0Freq() * 10) / __MIRC + 5) / 10;

    /* Set timer continuous counting mode */
    TIMER->CTL |= TIMER_CTL_OPMODE_Msk;

    /* Set timer prescale value */
    TIMER_SET_PRESCALE_VALUE(TIMER, 0x0);

    /*Set timer compared value*/
    TIMER_SET_CMP_VALUE(TIMER, 0xFFFFFF);

    /* Enable timer capture function */
    TIMER_EnableCapture(TIMER, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_FALLING);

    /* Select timer capture source from internal signal*/
    TIMER_CaptureSelect(TIMER, TIMER_CAPTURE_FROM_INTERNAL);

    /* Select timer capture source as MIRC and capture source divide 16 */
    TIMER->EXTCTL = (TIMER->EXTCTL & ~(TIMER_EXTCTL_INTERCAPSEL_Msk)) | TIMER_INTER_CAPTURE_FROM_MIRC;
    TIMER->EXTCTL = (TIMER->EXTCTL & ~(TIMER_EXTCTL_CAPDIVSCL_Msk)) | (4 << TIMER_EXTCTL_CAPDIVSCL_Pos);

    /* Enable CLKO and output frequency = MIRC  */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_MIRC, 0, 1);

    /* Start timer counting */
    TIMER_Start(TIMER);

    /* Start timer capture function */
    TIMER_StartCapture(TIMER);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Load MIRC ADJ value */
    u32RCValue = MIRC_READ_ADJ();

    while (1)
    {
        for (u32TrimLoopIndex = 0, u32CapVal_Interval = 0; u32TrimLoopIndex < TRIM_LOOP; u32TrimLoopIndex++)
        {
            /* Clear timer capture interrupt flag */
            TIMER_ClearCaptureIntFlag(TIMER);

            /* Get timer capture interrupt flag */
            while (TIMER_GetCaptureIntFlag(TIMER) == 0);

            /* Get capture value */
            u32CapVal0 = TIMER_GetCaptureData(TIMER);

            /* Clear timer capture interrupt flag */
            TIMER_ClearCaptureIntFlag(TIMER);

            /* Get timer capture interrupt flag */
            while (TIMER_GetCaptureIntFlag(TIMER) == 0);

            /* Get capture value */
            u32CapVal1 = TIMER_GetCaptureData(TIMER);

            /* Summary capture value */
            u32CapVal_Interval += u32CapVal1 - u32CapVal0;
        }

        /* Update MIRC ADJ value */
        if ((((u32CapVal_Interval * 10) / TRIM_LOOP + 5) / 10) / 16 < u32Freq_PCLK0_DIV_MIRC)
        {
            u32RCValue++;
        }
        else if ((((u32CapVal_Interval * 10) / TRIM_LOOP + 5) / 10) / 16 > u32Freq_PCLK0_DIV_MIRC)
        {
            u32RCValue--;
        }
        else
        {
            printf("[MIRCADJ]0x%x, MIRC Trim PASS!\n", MIRC_READ_ADJ());
            break;
        }

        if (u32RCValue > 0x3FF)
        {
            MIRC_WRITE_ADJ(0x200);
            printf("[MIRCADJ]0x%x, MIRC Trim Fail!\n", MIRC_READ_ADJ());
            break;
        }
        else
        {
            /* Update MIRC ADJ value */
            MIRC_WRITE_ADJ(u32RCValue);
            printf("Update MIRC ADJ: 0x%x\n", MIRC_READ_ADJ());
        }
    }

    /* Lock protected registers */
    SYS_LockReg();

    /* Stop timer capture function */
    TIMER_StopCapture(TIMER);

    /* Stop timer counting */
    TIMER_Stop(TIMER);

}

int32_t main(void)
{
    SYS_UnlockReg(); /* Unlock protected registers */

    /* Init System, IP clock and multi-function I/O
    In the end of SYS_Init() will issue SYS_LockReg()
    to lock protected register. If user want to write
    protected register, please issue SYS_UnlockReg()
    to unlock protected register if necessary */
    SYS_Init();

    SYS_LockReg(); /* Lock protected registers */

    UART0_Init(); /* Init UART0 for printf */

    /* Trim MIRC to 4.032MHz */
    TrimMIRC();

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
