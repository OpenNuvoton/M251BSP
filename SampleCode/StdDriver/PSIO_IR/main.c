/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement NEC IR protocol by PSIO.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "NEC_IR_driver.h"


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

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

    /* Set PCLK1 clock divider as 32 */
    CLK->PCLKDIV = (CLK->PCLKDIV & ~CLK_PCLKDIV_APB1DIV_Msk) | CLK_PCLKDIV_APB1DIV_DIV32;

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO_MODULE);

    /* Select PSIO module clock source as PCLK1 and PSIO module clock divider as 93 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_PCLK1, CLK_CLKDIV1_PSIO(93));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* PSIO multi-function pin CH0(PB.15) */
    SYS->GPB_MFPH &= ~SYS_GPB_MFPH_PB15MFP_Msk;
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB15MFP_PSIO0_CH0;
}


void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_PSIO_NEC_CFG sConfig;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|               NEC IR sample code                     | \n");
    printf("|      Please check waveform on PSIO_CH0(PB.15)        | \n");
    printf("+------------------------------------------------------+ \n");

    /* Use slot controller 0 and pin 0 */
    sConfig.u8SlotCtrl   = PSIO_SC0;
    sConfig.u8TxPin      = PSIO_PIN0;

    /* Initialize PSIO setting for NEC IR protocol */
    PSIO_NEC_Open(&sConfig);

    /* Send  0x1(Address), ~0x1(/Address), 0x2(Command), ~0x2(/Command) */
    PSIO_NEC_Send(&sConfig, 0x1, ~0x1, 0x2, ~0x2);

    /* Wait transfer done */
    while (PSIO_NEC_TransferDone(&sConfig));

    /* Send  Repeat signal */
    PSIO_NEC_Repeat(&sConfig);

    /* Wait transfer done */
    while (PSIO_NEC_TransferDone(&sConfig));

    /* Release PSIO setting */
    PSIO_NEC_Close(&sConfig);

    printf("Complete!\n");

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
