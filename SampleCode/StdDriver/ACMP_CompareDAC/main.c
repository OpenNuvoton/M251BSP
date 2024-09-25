/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how ACMP compare DAC output with ACMP1_P1 value.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/* Generates a Square wave*/
const uint16_t g_au16Square[] = { 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050,
                                  3900, 3900, 3900, 3900, 3900, 3900, 3900, 3900, 3900, 3900
                                };

static uint32_t g_u32Index = 0;
const uint32_t g_u32ArraySize = sizeof(g_au16Square) / sizeof(uint16_t);

/*---------------------------------------------------------------------------------------------------------*/
/*                                 Define functions prototype                                              */
/*---------------------------------------------------------------------------------------------------------*/
void DAC_IRQHandler(void);
void ACMP01_IRQHandler(void);
void SYS_Init(void);
int32_t main(void);


#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


void DAC_IRQHandler(void)
{
    if (DAC_GET_INT_FLAG(DAC0, 0))
    {

        if (g_u32Index == g_u32ArraySize)
        {
            g_u32Index = 0;
        }
        else
        {
            DAC_WRITE_DATA(DAC0, 0, g_au16Square[g_u32Index++]);
            DAC_START_CONV(DAC0);
            /* Clear the DAC conversion complete finish flag */
            DAC_CLR_INT_FLAG(DAC0, 0);
        }
    }

}

void ACMP01_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    /* Check Comparator 0 Output Status */
    if (ACMP_GET_OUTPUT(ACMP01, 1))
    {
        printf("[%d] ACMP1_P voltage >  DAC voltage, ACMP1_O = %d.\n", u32Cnt, PC0);
    }
    else
    {
        printf("[%d] ACMP1_P voltage <= DAC voltage, ACMP1_O = %d.\n", u32Cnt, PC0);
    }

    u32Cnt++;
}


void SYS_Init(void)
{
#if (CLK_SOURCE == CLK_HIRC )

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select HIRC as the clock source of UART */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

#else

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Select HXT as the clock source of UART */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Disable digital input path of analog pin XT1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 2));

    /* Disable digital input path of analog pin XT1_IN to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PF, (1ul << 3));

#endif

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);
    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPB_MODULE);
    /* Enable GPC peripheral clock */
    CLK_EnableModuleClock(GPC_MODULE);

    /* Enable DAC module clock */
    CLK_EnableModuleClock(DAC_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Note: ACMP negative input source does not support DAC.
     * (Part no. M254KG6AE, M258KG6AE, M256QG6AE, M258QG6AE, M254SG6AE, M258SG6AE)
     */
    if ((SYS->PDID & 0x01920000) == 0x01920000)
    {
        /* Set PA.8 to input mode */
        PA->MODE &= ~(GPIO_MODE_MODE8_Msk) ;
        /* Set PB.4 and PB.5 and PC.0  to input mode */
        PB->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk);
        PC->MODE &= ~(GPIO_MODE_MODE0_Msk);

        /* Set PB4 multi-function pin for ACMP1 positive input pin and PB5 multi-function pin for ACMP1 negative input pin*/
        SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) | (SYS_GPB_MFPL_PB4MFP_ACMP1_P1 | SYS_GPB_MFPL_PB5MFP_ACMP1_N);
        /* Set PC0 multi-function pin for ACMP1 output pin*/
        SYS->GPC_MFPL = (SYS->GPC_MFPL & ~SYS_GPC_MFPL_PC0MFP_Msk) | SYS_GPC_MFPL_PC0MFP_ACMP1_O;
        /* Set PA multi-function pin for DAC voltage output */
        SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk) | SYS_GPA_MFPH_PA8MFP_DAC0_OUT;

    }
    else
    {
        /* Set PB.4 and PC.0 to input mode */
        PB->MODE &= ~(GPIO_MODE_MODE4_Msk);
        PC->MODE &= ~(GPIO_MODE_MODE0_Msk);

        /* Set PB4 multi-function pin for ACMP1 positive input pin and PC0 multi-function pin for ACMP1 output pin*/
        SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk) | SYS_GPB_MFPL_PB4MFP_ACMP1_P1;
        SYS->GPC_MFPL = (SYS->GPC_MFPL & ~SYS_GPC_MFPL_PC0MFP_Msk) | SYS_GPC_MFPL_PC0MFP_ACMP1_O;
    }

    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA4MFP_Msk) | SYS_GPA_MFPL_PA4MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA5MFP_Msk) | SYS_GPA_MFPL_PA5MFP_UART0_TXD;

    /* Note: ACMP negative input source does not support DAC.
     * (Part no. M254KG6AE, M258KG6AE, M256QG6AE, M258QG6AE, M254SG6AE, M258SG6AE)
     */
    if ((SYS->PDID & 0x01920000) == 0x01920000)
    {
        /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PA, (1ul << 8));
        /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));
        /* Disable digital input path of analog pin ACMP1_N to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 5));
    }
    else
    {
        /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
        GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));
    }

}

/*
 * When the voltage of the positive input is greater than the voltage of the
 * negative input, the analog comparator outputs logical one; otherwise, it outputs
 * logical zero. This sample code will show the expression of the comparator's
 * number when detecting a transition of analog comparator's output.
 */

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("This sample code demonstrates [ACMP1] function.\n");

    /* Note: ACMP negative input source does not support DAC.
     * (Part no. M254KG6AE, M258KG6AE, M256QG6AE, M258QG6AE, M254SG6AE, M258SG6AE)
     */
    if ((SYS->PDID & 0x01920000) == 0x01920000)
    {
        printf("  ACMP1_N:  Using PB5 as negative input.\n");
        printf("  ACMP1_P1: Using PB4 as positive input.\n");
        printf("  ACMP1_O:  Using PC0 as compare output.\n");
        printf("Note: Becasue ACMP negative input source does not support DAC.\n");
        printf("Please connect ACMP1_N (PB5) to DAC output (PA8).\n");
    }
    else
    {
        printf("  ACMP1_N:  Using DAC output as negative input.\n");
        printf("  ACMP1_P1: Using PB4 as positive input.\n");
        printf("  ACMP1_O:  Using PC0 as compare output.\n");
    }

    printf("Please connect ACMP1_P1 (PB4) to 1.5 v.\n");
    printf("\nThe compare result reflects on ACMP1_O (PC0).\n");
    printf("Press any key to start ...\n");
    getchar();

    /* Set the software trigger, enable DAC even trigger mode and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_SOFTWARE_TRIGGER);

    /* The DAC conversion settling time is 1ms */
    DAC_SetDelayTime(DAC0, 100);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC0, 0, 0x0);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the DAC interrupt */
    DAC_ENABLE_INT(DAC0, 0);

    NVIC_EnableIRQ(DAC_IRQn);

    /* Start A/D conversion */
    DAC_START_CONV(DAC0);

    /* Note:ACMP  negative input source is not support DAC (Part no. M258KG6AE,M254KG6AE,M258QG6AE,M256QG6AE,M258SG6AE M254SG6AE) */
    if ((SYS->PDID & 0x01920000) == 0x01920000)
    {
        /* Configure ACMP1. Enable ACMP1 and selecting the voltage of ACMP negative input pin as the source of ACMP negative input.*/
        ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_PIN, ACMP_CTL_HYSTERESIS_DISABLE);
    }
    else
    {
        /* Configure ACMP1. Enable ACMP1 and select DAC voltage as the source of ACMP negative input. */
        ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_DAC, ACMP_CTL_HYSTERESIS_DISABLE);
    }

    /* Select P1 as ACMP positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);

    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 1);

    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);


    while (1);

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
