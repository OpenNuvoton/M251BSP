/***************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       Demonstrate how to implement DMX512 protocol by PSIO.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "DMX512_driver.h"


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

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO_MODULE);

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 4 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_HIRC, CLK_CLKDIV1_PSIO(4));

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

    /* PSIO multi-function pin CH0(PB.15) and CH1(PC.4)*/
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_PSIO0_CH0);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC4MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC4MFP_PSIO0_CH1);
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
    S_PSIO_DMX512_CFG sConfig;
    uint16_t au16RxBuf[5];
    uint32_t i;

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
    printf("+--------------------------------------------------------+ \n");
    printf("|   DMX512 Protocol Test Code                            | \n");
    printf("|   Please connected PSIO_CH0(PB.15) to PSIO_CH1(PC.4)   | \n");
    printf("+--------------------------------------------------------+ \n");

    /* Reset PSIO */
    SYS->IPRST2 |= SYS_IPRST2_PSIORST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PSIORST_Msk;

    /* Use slot controller 0 and pin 0  for TX */
    /* Use slot controller 1 and pin 1  for RX */
    sConfig.u8TxSlotCounter      = PSIO_SC0;
    sConfig.u8RxSlotCounter      = PSIO_SC1;
    sConfig.u8TxPin              = PSIO_PIN0;
    sConfig.u8RxPin              = PSIO_PIN1;

    /* Initialize PSIO setting for DMX512 */
    PSIO_DMX512_Open(&sConfig);
    NVIC_EnableIRQ(PSIO_IRQn);

    while (1)
    {

        for (i = 1 ; i < 6 ; i++)
        {
            PSIO_DMX512_getChannelData(&sConfig, i, &au16RxBuf[0]);
            printf("press any key to continue\n");
            getchar();

            PSIO_DMX512_Tx(&sConfig, 0, eDMX512_BREAK_START);
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x59, eDMX512_DATA);     /* Channel 1 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x67, eDMX512_DATA);     /* Channel 2 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x79, eDMX512_DATA);     /* Channel 3 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x88, eDMX512_DATA);     /* Channel 4 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x55, eDMX512_DATA);     /* Channel 5 data */
            CLK_SysTickDelay(50);

            while (!(*sConfig.pu8RcvDone));

            printf("%u: 0x%02X\n", i, (uint8_t)DMX512_GET_DATA(au16RxBuf[0]));
        }
    }
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
