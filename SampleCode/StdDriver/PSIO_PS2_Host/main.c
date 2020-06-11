/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement PS/2 host protocol by PSIO.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "PS2_Host_driver.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

S_PSIO_PS2 g_sConfig;
volatile uint32_t g_u32TxData = 0, g_u32RxData = 0;
volatile uint32_t g_u32RxACK = 1;


void PSIO_IRQHandler(void)
{
    static uint8_t u8BitNumber = 0;
    static uint32_t u32RxBuffer = 0;
    uint8_t u8INT0Flag;

    /* Get INT0 interrupt flag */
    u8INT0Flag = PSIO_GET_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk);

    if (u8INT0Flag)
    {
        /* Clear INT0 interrupt flag */
        PSIO_CLEAR_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk);
    }
    else
    {
        printf("Unknown interrupt occur!!!\n");
    }

    if ((PSIO_PS2_GET_STATUS() == eHOST_READ) | (PSIO_PS2_GET_STATUS() == eHOST_READY_TO_READ))
    {
        /* Trigger slot controller */
        PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

        /* Wait input buffer full */
        while (!PSIO_GET_TRANSFER_STATUS(PSIO, PSIO_TRANSTS_INFULL0_Msk << (g_sConfig.u8DataPin * 4)));

        /* Receive 11 bit */
        u32RxBuffer |= (PSIO_GET_INPUT_DATA(PSIO, g_sConfig.u8DataPin) << u8BitNumber);

        if (u8BitNumber == 10)
        {
            PSIO_PS2_SET_STATUS(eHOST_IDLE);
            u8BitNumber = 0;
            g_u32RxData = u32RxBuffer;
            u32RxBuffer = 0;
        }
        else
        {
            PSIO_PS2_SET_STATUS(eHOST_READ);
            u8BitNumber++;
        }
    }
    else if (PSIO_PS2_GET_STATUS() == eHOST_WRITE)
    {
        if (u8BitNumber == 10)
        {
            /* Trigger slot controller */
            PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

            /* Wait slot controller is not busy */
            while (PSIO_GET_BUSY_FLAG(PSIO, g_sConfig.u8DataSC));

            /* Read data from buffer */
            g_u32RxACK = PSIO_GET_INPUT_DATA(PSIO, g_sConfig.u8DataPin);

            /* Set pin interval status as high */
            PSIO->GNCT[g_sConfig.u8DataPin].GENCTL = (PSIO->GNCT[g_sConfig.u8DataPin].GENCTL & ~PSIO_GNCT_GENCTL_INTERVAL_Msk)
                                                     | (PSIO_HIGH_LEVEL << PSIO_GNCT_GENCTL_INTERVAL_Pos);
            PSIO_PS2_SET_STATUS(eHOST_IDLE);
            u8BitNumber = 0;
        }
        else if (u8BitNumber == 9)
        {

            PSIO_SET_OUTPUT_DATA(PSIO, g_sConfig.u8DataPin, g_u32TxData >> u8BitNumber);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

            /* Wait slot controller is not busy */
            while (PSIO_GET_BUSY_FLAG(PSIO, g_sConfig.u8DataSC));

            /* Set check point action */
            /* For more efficient, accessing register directly */
            PSIO->GNCT[g_sConfig.u8DataPin].CPCTL1   = PSIO_IN_BUFFER;     //input buffer
            u8BitNumber++;
        }
        else
        {
            /* Send 9 bit */
            PSIO_SET_OUTPUT_DATA(PSIO, g_sConfig.u8DataPin, g_u32TxData >> u8BitNumber);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

            u8BitNumber++;
        }
    }
}


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

    /* Set PCLK1 clock divider as 4 */
    CLK->PCLKDIV = (CLK->PCLKDIV & ~CLK_PCLKDIV_APB1DIV_Msk) | CLK_PCLKDIV_APB1DIV_DIV4;

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO_MODULE);

    /* Select PSIO module clock source as PCLK1 and PSIO module clock divider as 0x1 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_PCLK1, CLK_CLKDIV1_PSIO(1));

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


int main()
{
    uint32_t u32Data = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("******************************************************\n");
    printf("|               PS/2 Host Sample Code                |\n");
    printf("|      Please connected PSIO_CH0(PB.15)(Clock)       |\n");
    printf("|      and PSIO_CH1(PC.4)(Data).                     |\n");
    printf("******************************************************\n");

    /* Use slot controller 0/1 and pin 0/1 */
    g_sConfig.u8ClockSC        = PSIO_SC0;
    g_sConfig.u8DataSC         = PSIO_SC1;
    g_sConfig.u8ClockPin       = PSIO_PIN0;
    g_sConfig.u8DataPin        = PSIO_PIN1;
    g_sConfig.p32ClockMFP      = &PB15;
    g_sConfig.p32DataMFP       = &PC4;

    /* Initialize PSIO setting for PS/2 host protocol */
    PSIO_PS2_Open(&g_sConfig);

    while (1)
    {
        /* Set PSIO on read signal state */
        if (PSIO_PS2_GET_STATUS() != eHOST_READY_TO_READ)
        {
            PSIO_PS2_HostRead(&g_sConfig);
        }

        /* Receiving data */
        while (PSIO_PS2_GET_STATUS() == eHOST_READ);

        /* Data was received */
        if (PSIO_PS2_GET_STATUS() == eHOST_IDLE)
        {
            PSIO_PS2_SET_STATUS(eHOST_READY_TO_READ);
            printf("[Start]0x%x, [Data]0x%x, [Parity]0x%x, [Stop]0x%x\n",
                   g_u32RxData & 0x1, (g_u32RxData >> 1) & 0xFF, (g_u32RxData >> 9) & 0x1, (g_u32RxData >> 10) & 0x1);
        }

        /* Send data */
        if (!(UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            u32Data = UART0->DAT;
            g_u32TxData = PSIO_Encode_TxData(&u32Data);
            printf("[Host send to device]0x%x, 0x%x\n", u32Data, g_u32TxData);
            PSIO_PS2_HostSend(&g_sConfig);

            while (PSIO_PS2_GET_STATUS() == eHOST_WRITE);
        }
    }

    /* Close setting */
    PSIO_PS2_Close(&g_sConfig);

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
