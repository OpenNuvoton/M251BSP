/****************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use LXT to trim HIRC
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/**
 * @brief       RCTrim IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RCTrim default IRQ, declared in startup_M251.s.
 */
void IRCTRIM_IRQHandler()
{
    if (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_TFAILIF_Msk)   /* Get Trim Failure Interrupt */
    {
        /* Display HIRC trim status */
        printf("HIRC Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_TFAILIF_Msk;
    }

    if (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_CLKERIF_Msk)   /* Get LXT Clock Error Interrupt */
    {
        /* Display HIRC trim status */
        printf("LXT Clock Error Interrupt\n");
        /* Clear LXT Clock Error Interrupt */
        SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_CLKERIF_Msk;
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*-----------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                   */
    /*-----------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk);

    /* Disable digital input path of analog pin X32_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 4));

    /* Disable digital input path of analog pin XT32_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 5));

    /* Enable external 32768Hz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

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

void TrimHIRC()
{
    uint32_t u32TimeOutCnt = SystemCoreClock;// 1 second timeout

    /*  Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->HIRCTRIMIEN |= (SYS_HIRCTRIMIEN_CLKEIEN_Msk | SYS_HIRCTRIMIEN_TFALIEN_Msk);
    SYS->HIRCTRIMCTL = (SYS->HIRCTRIMCTL & ~SYS_HIRCTRIMCTL_FREQSEL_Msk) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    while (1)
    {
        if (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_FREQLOCK_Msk)
        {
            printf("HIRC Frequency Lock\n");
            SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_FREQLOCK_Msk;     /* Clear Trim Lock */
            break;
        }
        else
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("HIRC Trim failed\n");
                /* Disable IRC Trim */
                SYS->HIRCTRIMCTL &= (~SYS_HIRCTRIMCTL_FREQSEL_Msk);
                printf("Disable IRC Trim\n");

                while (1);
            }
        }
    }

    /* Enable CLKO and output frequency = HIRC  */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HIRC, 1, 0);

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

    UART0_Init(); /* Init UART0 for printf */

    /* Enable Interrupt */
    NVIC_EnableIRQ(IRCTRIM_IRQn);

    /* Trim HIRC to 48MHz */
    TrimHIRC();

    /* Disable IRC Trim */
    SYS->HIRCTRIMCTL = (SYS->HIRCTRIMCTL & ~SYS_HIRCTRIMCTL_FREQSEL_Msk);
    printf("Disable IRC Trim\n");

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
