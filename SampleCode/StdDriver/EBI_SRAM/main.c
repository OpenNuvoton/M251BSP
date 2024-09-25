/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Configure EBI interface to access BS616LV4017 (SRAM) on EBI interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern void SRAM_BS616LV4017(uint32_t u32MaxSize);
void AccessEBIWithPDMA(void);

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

    /* Configure UART0 and set UART0 Baudrate */
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
    printf("+--------------------------------------------------------+\n");
    printf("|    EBI SRAM Sample Code on Bank0 with PDMA transfer    |\n");
    printf("+--------------------------------------------------------+\n\n");

    printf("********************************************************************\n");
    printf("* Please connect BS616LV4017 SRAM to EBI bank0 before accessing !! *\n");
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

    /* Initialize EBI bank0 to access external SRAM */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_16BIT, EBI_TIMING_SLOWEST, 0, EBI_CS_ACTIVE_LOW);

    /* Start to test EBI SRAM */
    SRAM_BS616LV4017(512 * 1024);

    /* EBI SRAM with PDMA test */
    AccessEBIWithPDMA();

    /* Disable EBI function */
    EBI_Close(EBI_BANK0);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI_MODULE);

    printf("*** SRAM Test OK ***\n");

    while (1);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables for PDMA                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t g_u32PdmaTestLength = 64;
uint32_t g_au32SrcArray[64];
uint32_t g_au32DestArray[64];
uint32_t volatile g_u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M480.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if (PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void AccessEBIWithPDMA(void)
{
    uint32_t u32LoopCnt;
    uint32_t u32Result0 = 0x5A5A, u32Result1 = 0x5A5A;

    printf("[[ Access EBI with PDMA ]]\n");

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    for (u32LoopCnt = 0; u32LoopCnt < 64; u32LoopCnt++)
    {
        g_au32SrcArray[u32LoopCnt] = 0x76570000 + u32LoopCnt;
        u32Result0 += g_au32SrcArray[u32LoopCnt];
    }

    /* Open Channel 2 */
    PDMA_Open(PDMA, (1 << 2));

    //burst size is 4
    PDMA_SetBurstType(PDMA, 2, PDMA_REQ_BURST, PDMA_BURST_4);

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA, 2, PDMA_WIDTH_32, g_u32PdmaTestLength);
    PDMA_SetTransferAddr(PDMA, 2, (uint32_t)g_au32SrcArray, PDMA_SAR_INC, EBI_BANK0_BASE_ADDR, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA, 2, PDMA_MEM, FALSE, 0);

    PDMA_EnableInt(PDMA, 2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);

    g_u32IsTestOver = 0;
    PDMA_Trigger(PDMA, 2);

    while (g_u32IsTestOver == 0);

    /* Transfer internal SRAM to EBI SRAM done */

    /* Clear internal SRAM data */
    for (u32LoopCnt = 0; u32LoopCnt < 64; u32LoopCnt++)
    {
        g_au32SrcArray[u32LoopCnt] = 0x0;
    }

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA, 2, PDMA_WIDTH_32, g_u32PdmaTestLength);
    PDMA_SetTransferAddr(PDMA, 2, EBI_BANK0_BASE_ADDR, PDMA_SAR_INC, (uint32_t)g_au32SrcArray, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA, 2, PDMA_MEM, FALSE, 0);

    g_u32IsTestOver = 0;
    PDMA_Trigger(PDMA, 2);

    while (g_u32IsTestOver == 0);

    /* Transfer EBI SRAM to internal SRAM done */
    for (u32LoopCnt = 0; u32LoopCnt < 64; u32LoopCnt++)
    {
        u32Result1 += g_au32SrcArray[u32LoopCnt];
    }

    if (g_u32IsTestOver == 1)
    {
        if ((u32Result0 == u32Result1) && (u32Result0 != 0x5A5A))
        {
            printf("        PASS (0x%X)\n\n", u32Result0);
        }
        else
        {
            printf("        FAIL - data matched (0x%X)\n\n", u32Result0);

            while (1);
        }
    }
    else
    {
        printf("        PDMA fail\n\n");

        while (1);
    }

    PDMA_Close(PDMA);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
