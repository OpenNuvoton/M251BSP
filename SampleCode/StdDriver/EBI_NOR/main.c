/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Configure EBI interface to access MX29LV320T (NOR Flash) on EBI interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern uint16_t NOR_MX29LV320T_READ(uint32_t u32Bank, uint32_t u32DstAddr);
extern int32_t NOR_MX29LV320T_WRITE(uint32_t u32Bank, uint32_t u32DstAddr, uint16_t u16Data);
extern void NOR_MX29LV320T_GET_ID(uint32_t u32Bank, uint16_t *pu16IDTable);
extern int32_t NOR_MX29LV320T_EraseChip(uint32_t u32Bank, uint32_t u32IsCheckBlank);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

void Configure_EBI_16BIT_Pins(void)
{
    /* EBI AD0~5 pins on PC.0~5 */
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_EBI_AD0 | SYS_GPC_MFPL_PC1MFP_EBI_AD1 |
                     SYS_GPC_MFPL_PC2MFP_EBI_AD2 | SYS_GPC_MFPL_PC3MFP_EBI_AD3 |
                     SYS_GPC_MFPL_PC4MFP_EBI_AD4 | SYS_GPC_MFPL_PC5MFP_EBI_AD5;

    /* EBI AD6, AD7 pins on PA.6, PA.7 */
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA6MFP_EBI_AD6 | SYS_GPA_MFPL_PA7MFP_EBI_AD7;

    /* EBI AD8, AD9 pins on PC.6, PC.7 */
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC6MFP_EBI_AD8 | SYS_GPC_MFPL_PC7MFP_EBI_AD9;

    /* EBI AD10 pins on PD.13 */
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD13MFP_EBI_AD10;

    /* EBI AD11 pins on PC.14 */
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC14MFP_EBI_AD11;

    /* EBI AD12, AD13 pins on PD.1, PD.0 */
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD1MFP_EBI_AD12 | SYS_GPD_MFPL_PD0MFP_EBI_AD13;

    /* EBI AD14, AD15 pins on PB.13, PB.12 */
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB13MFP_EBI_AD14 | SYS_GPB_MFPH_PB12MFP_EBI_AD15;

    /* EBI ADR0~3 pins on PB.5~2 */
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB5MFP_EBI_ADR0 | SYS_GPB_MFPL_PB4MFP_EBI_ADR1 |
                     SYS_GPB_MFPL_PB3MFP_EBI_ADR2 | SYS_GPB_MFPL_PB2MFP_EBI_ADR3;

    /* EBI ADR4~7 pins on PC.9~12 */
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC12MFP_EBI_ADR4 | SYS_GPC_MFPH_PC11MFP_EBI_ADR5 |
                     SYS_GPC_MFPH_PC10MFP_EBI_ADR6 | SYS_GPC_MFPH_PC9MFP_EBI_ADR7;

    /* EBI ADR8~9 pins on PB.0~1 */
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB1MFP_EBI_ADR8 | SYS_GPB_MFPL_PB0MFP_EBI_ADR9;

    /* EBI ADR10~15 pins on PE.8~13 */
    SYS->GPE_MFPH |= SYS_GPE_MFPH_PE8MFP_EBI_ADR10 | SYS_GPE_MFPH_PE9MFP_EBI_ADR11 |
                     SYS_GPE_MFPH_PE10MFP_EBI_ADR12 | SYS_GPE_MFPH_PE11MFP_EBI_ADR13 |
                     SYS_GPE_MFPH_PE12MFP_EBI_ADR14 | SYS_GPE_MFPH_PE13MFP_EBI_ADR15;

    /* EBI ADR16~17 pins on PB.11, PB.10 */
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB11MFP_EBI_ADR16 | SYS_GPB_MFPH_PB10MFP_EBI_ADR17;

    /* EBI ADR18~19 pins on PF.6~7 */
    SYS->GPF_MFPL |= SYS_GPF_MFPL_PF7MFP_EBI_ADR18 | SYS_GPF_MFPL_PF6MFP_EBI_ADR19;

    /* EBI RD and WR pins on PA.11 and PA.10 */
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA10MFP_EBI_nWR | SYS_GPA_MFPH_PA11MFP_EBI_nRD;

    /* EBI WRL and WRH pins on PB.7 and PB.6 */
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB6MFP_EBI_nWRH | SYS_GPB_MFPL_PB7MFP_EBI_nWRL;

    /* EBI CS0 pin on PD.12 */
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD12MFP_EBI_nCS0;

    /* EBI CS1 pin on PD.11 */
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD11MFP_EBI_nCS1;

    /* EBI CS2 pin on PD.10 */
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD10MFP_EBI_nCS2;

    /* EBI ALE pin on PA.8 */
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA8MFP_EBI_ALE;

    /* EBI MCLK pin on PA.9 */
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA9MFP_EBI_MCLK;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(EBI_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
#if !(defined(DEBUG_ENABLE_SEMIHOST))
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA4MFP_Msk) | SYS_GPA_MFPL_PA4MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA5MFP_Msk) | SYS_GPA_MFPL_PA5MFP_UART0_TXD;
#endif

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
#if !(defined(DEBUG_ENABLE_SEMIHOST))
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
#endif

}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Addr, u32MaxEBISize;
    uint16_t u16WData;
    uint16_t au16IDTable[2];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    EBI Nor Flash Sample Code on Bank1   |\n");
    printf("+-----------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect MX29LV320T nor flash to EBI bank1 before accessing !! *\n");
    printf("* EBI pins settings:                                                   *\n");
    printf("*   - AD0 ~ AD5     on PC.0 ~ PC.5                                     *\n");
    printf("*   - AD6 ~ AD7     on PA.6 ~ PA.7                                     *\n");
    printf("*   - AD8 ~ AD9     on PC.6 ~ PC.7                                     *\n");
    printf("*   - AD10 ~ AD11   on PD.13 ~ PC.14                                   *\n");
    printf("*   - AD12 ~ AD13   on PD.1 ~ PD.0                                     *\n");
    printf("*   - AD14 ~ AD15   on PB.13 ~ PB.12                                   *\n");
    printf("*   - ADR0 ~ ADR3   on PB.5 ~ PB.2                                     *\n");
    printf("*   - ADR4 ~ ADR7   on PC.12 ~ PC.9                                    *\n");
    printf("*   - ADR8 ~ ADR9   on PB.1 ~ PB.0                                     *\n");
    printf("*   - ADR10 ~ ADR15 on PE.8 ~ PE.13                                    *\n");
    printf("*   - ADR16 ~ ADR17 on PB.11 ~ PB.10                                   *\n");
    printf("*   - ADR18 ~ ADR19 on PF.7 ~ PF.6                                     *\n");
    printf("*   - nWR           on PA.10                                           *\n");
    printf("*   - nRD           on PA.11                                           *\n");
    printf("*   - nWRL          on PB.7                                            *\n");
    printf("*   - nWRH          on PB.6                                            *\n");
    printf("*   - nCS0          on PD.12                                           *\n");
    printf("*   - nCS1          on PD.11                                           *\n");
    printf("*   - nCS2          on PF.2                                            *\n");
    printf("*   - ALE           on PA.8                                            *\n");
    printf("*   - MCLK          on PA.9                                            *\n");
    printf("**********************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank1 to access external nor */
    EBI_Open(EBI_BANK1, EBI_BUSWIDTH_16BIT, EBI_TIMING_VERYSLOW, 0, EBI_CS_ACTIVE_LOW);


    /* Step 1, check ID */
    NOR_MX29LV320T_GET_ID(EBI_BANK1, (uint16_t *)au16IDTable);
    printf(">> Manufacture ID: 0x%X, Device ID: 0x%X .... ", au16IDTable[0], au16IDTable[1]);

    if ((au16IDTable[0] != 0xC2) || (au16IDTable[1] != 0x22A8))
    {
        printf("FAIL !!!\n\n");

        while (1);
    }
    else
    {
        printf("PASS !!!\n\n");
    }


    /* Step 2, erase chip */
    if (NOR_MX29LV320T_EraseChip(EBI_BANK1, TRUE) < 0)
        while (1);


    /* Step 3, program flash and compare data */
    printf(">> Run program flash test ......\n");
    u32MaxEBISize = EBI_MAX_SIZE;

    for (u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;

        if (NOR_MX29LV320T_WRITE(EBI_BANK1, u32Addr, u16WData) < 0)
        {
            printf("Program [0x%08X]: [0x%08X] FAIL !!!\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16WData);

            while (1);
        }
        else
        {
            /* Show UART message ...... */
            if ((u32Addr % 256) == 0)
                printf("Program [0x%08X]:[0x%08X] !!!       \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16WData);
        }
    }

    for (u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        uint16_t    u16RData;

        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;
        u16RData = NOR_MX29LV320T_READ(EBI_BANK1, u32Addr);

        if (u16WData != u16RData)
        {
            printf("Compare [0x%08X] FAIL !!! (W:0x%08X, R:0x%08X)\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16WData, u16RData);

            while (1);
        }
        else
        {
            /* Show UART message ...... */
            if ((u32Addr % 256) == 0)
                printf("Read [0x%08X]: [0x%08X] !!!         \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16RData);
        }
    }

    printf(">> Program flash OK !!!                             \n\n");

    /* Disable EBI function */
    EBI_Close(EBI_BANK1);

    /* Disable EBI clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_EBICKEN_Msk;

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
