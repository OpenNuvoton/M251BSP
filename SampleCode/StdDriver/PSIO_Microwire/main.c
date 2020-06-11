/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement Microwire protocol by PSIO.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "AT93C46D_driver_EEPROM.h"

volatile uint32_t *pu32ChipSelectPin, *pu32InputPin;

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

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO_MODULE);

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 12 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_HIRC, CLK_CLKDIV1_PSIO(12));

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

    /* PSIO multi-function pin CH0(PB.15), CH1(PC.4), CH2(PC.3) and CH3(PC.2) */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_PSIO0_CH0);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC4MFP_PSIO0_CH1 | SYS_GPC_MFPL_PC3MFP_PSIO0_CH2 | SYS_GPC_MFPL_PC2MFP_PSIO0_CH3);
}


void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void SetCSPinToPSIO(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_PSIO0_CH0);
}

void SetCSPinToGPIO(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_GPIO);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_PSIO_AT93C46D sConfig;
    uint8_t u8TxData = 0;
    uint8_t u8RxData = 0;
    uint8_t u8Address = 0;

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
    printf("|      AT93C46D Microwire EEPROM  Test Code            | \n");
    printf("| Please connected PSIO_CH0(PB.15), PSIO_CH1(PC.4)     | \n");
    printf("| ,PSIO_CH2(PC.3), PSIO_CH4(PC.2)to device.            | \n");
    printf("+------------------------------------------------------+ \n");

    /* Use slot controller 0, pin 0, pin1, pin 2, pin 3 */
    sConfig.u8SlotCtrl      = PSIO_SC0;
    sConfig.u8ChipSelectPin = PSIO_PIN0;
    sConfig.u8ClockPin      = PSIO_PIN1;
    sConfig.u8DO            = PSIO_PIN2;
    sConfig.u8DI            = PSIO_PIN3;

    /* Initialize PIN config */
    pu32ChipSelectPin   = &PB15;
    pu32InputPin        = &PC3;

    /* Set CS pin output mode when GPIO function */
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);

    /* Initialize PSIO setting for AT93C46D */
    PSIO_AT93C46D_Init(&sConfig);

    /* Send write enable command */
    PSIO_AT93C46D_EraseWrite_Enable(&sConfig);

    printf("Read/Write data to AT93C46D\n");

    for (u8Address = 0; u8Address < EEPROM_SIZE / DATA_WIDTH; u8Address++, u8TxData = u8Address + 2)
    {
        /* Send erase command */
        PSIO_AT93C46D_Erase(&sConfig, u8Address);

        /* Write data to AT93C46D */
        PSIO_AT93C46D_Write(&sConfig, u8Address, &u8TxData);

        /* Read data from AT93C46D */
        PSIO_AT93C46D_Read(&sConfig, u8Address, &u8RxData);

        /* Compare data is correct */
        if (u8TxData != u8RxData)
        {
            printf("[Error] TxData:0x%x, RxData:0x%x\n", u8TxData, u8RxData);

            while (1);
        }
        else
        {
            printf(".");
        }
    }

    printf("PASS!\n");

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
