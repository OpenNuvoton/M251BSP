/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> Power-down Mode
//      <0=> PD
//      <2=> FWPD
//      <6=> DPD
*/
#define SET_PDMSEL       0

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC       0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0

#define GPIO_P0_TO_P15      0xFFFF

void PowerDownFunction(void);
void GPB_IRQHandler(void);
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
void SYS_Init(void);
void UART0_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART0)

    if (--u32TimeOutCnt == 0) break;

    /* Select Power-down mode */
    CLK_SetPowerDownMode(SET_PDMSEL << CLK_PMUCTL_PDMSEL_Pos);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ, declared in startup_M251.s
 */
void GPB_IRQHandler(void)
{
    volatile uint32_t u32temp;

    /* To check if PB.2 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT2))
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
        printf("PB2 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        u32temp = PB->INTSRC;
        PB->INTSRC = u32temp;
        printf("Un-expected interrupts.\n");
    }
}

void PorSetting(void)
{
    if (SET_POR == 0)
    {
        SYS_DISABLE_POR();
    }
    else
    {
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LIRC == 0)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LIRCSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

        if (CLK_WaitClockReady(CLK_PWRCTL_LIRCEN_Msk) == 0)
        {
            printf("Wait for LIRC enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t LxtSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LXT == 0)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LXTSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

        if (CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk) == 0)
        {
            printf("Wait for LXT enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init System                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable HIRC, HXT and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable all GPIO clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPACKEN_Msk | CLK_AHBCLK_GPBCKEN_Msk | CLK_AHBCLK_GPCCKEN_Msk | CLK_AHBCLK_GPDCKEN_Msk |
                   CLK_AHBCLK_GPECKEN_Msk | CLK_AHBCLK_GPFCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Init UART0 multi-function pins, RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCnt, u32PMUSTS;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Clear DPD mode wake-up status for enterning DPD mode again */
    u32PMUSTS = CLK->PMUSTS;

    if (CLK->PMUSTS)
    {
        /* Clear DPD mode wake-up status */
        CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->PMUSTS)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for DPD mode wake-up status is cleared time-out!\n");
                goto lexit;
            }
        }
    }

    /* PB.2 falling-edge wake-up event */
    if (u32PMUSTS & CLK_PMUSTS_PINWK2_Msk)
    {
        printf("System waken-up done.\n\n");

        while (1);
    }

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.2 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("|  Operating sequence                                               |\n");
    printf("|  1. Remove R16                                                    |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  3. Disable analog function, e.g. POR module                      |\n");
    printf("|  4. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  5. Enter to Power-Down                                           |\n");
    printf("|  6. Wait for PB.2 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /* To measure Power-down current on NuMaker-M258KE board, remove R16 and measure the current on Ammeter Connector */

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART0)

    if (--u32TimeOutCnt == 0) break;

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFPL = 0;
    SYS->GPA_MFPH = 0;
    SYS->GPB_MFPL = 0;
    SYS->GPB_MFPH = (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    SYS->GPC_MFPL = 0;
    SYS->GPC_MFPH = 0;
    SYS->GPD_MFPL = 0;
    SYS->GPD_MFPH = 0;
    SYS->GPE_MFPL = 0;
    SYS->GPE_MFPH = 0;
    SYS->GPF_MFPL = 0;
    SYS->GPF_MFPH = 0;

    /* Configure all GPIO as Quasi-bidirectional Mode */
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if (LircSetting() < 0) goto lexit;

    /* LXT setting */
    if (LxtSetting() < 0) goto lexit;

    /* Wake-up source configuraion */
    if ((SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD) || (SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD))
    {
        /* Configure PB.2 as Quasi mode and enable interrupt by falling edge trigger */
        GPIO_SetMode(PB, BIT2, GPIO_MODE_QUASI);
        GPIO_EnableInt(PB, 2, GPIO_INT_FALLING);
        NVIC_EnableIRQ(GPB_IRQn);
    }
    else if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_DPD)
    {
        CLK_EnableDPDWKPin(CLK_DPDWKPIN_2, CLK_DPDWKPIN_FALLING);
    }
    else
    {
        printf("Unknown Power-down mode!\n");
        goto lexit;
    }

    printf("Press any key to enter Power-down mode.\n");
    getchar();

    /* Enter to Power-down mode */
    if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD)        printf("Enter to PD Power-Down ......\n");
    else if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD) printf("Enter to FWPD Power-Down ......\n");
    else if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_DPD)  printf("Enter to DPD Power-Down ......\n");

    PowerDownFunction();

    /* Waiting for PB.2 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while (1);

}
