/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to control OPA.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

/*---------------------------------------------------------------------------------------------------------*/
/*                                 Define functions prototype                                              */
/*---------------------------------------------------------------------------------------------------------*/
void OPA_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);
int main(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

void OPA_IRQHandler(void)
{
    static uint32_t u32Cnt = 0ul;

    /* Clear OPA0 interrupt flag */
    OPA_CLR_INT_FLAG(OPA, 0);

    /* Check OPA0 digital output state */
    if (OPA_GET_DIGITAL_OUTPUT(OPA, 0))
        printf("OP0_P voltage > OP0_N voltage (%d)\n", (int)u32Cnt);
    else
        printf("OP0_P voltage <= OP0_N voltage (%d)\n", (int)u32Cnt);

    u32Cnt++;
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                             Init System Clock                                           */
/*---------------------------------------------------------------------------------------------------------*/

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

    /* Enable IP module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(OPA_MODULE);



    /*---------------------------------------------------------------------------------------------------------*/
    /*                                  Init I/O Multi-function                                                */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();


    /* Set PB.0,PB.1 and PB.2 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk);
    //Set I/O multi-function pins to OPA
    //PB0: OP0_P
    //PB1: OP0_N
    //PB2: OP0_O
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_OPA0_O | SYS_GPB_MFPL_PB1MFP_OPA0_N |  SYS_GPB_MFPL_PB0MFP_OPA0_P);

    /* Disable digital input path of analog pin OPA0_N to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 1));

    /* Disable digital input path of analog pin OPA_P to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 0));

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                             Init UART                                                   */
/*---------------------------------------------------------------------------------------------------------*/

void UART0_Init(void)
{

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                     MAIN function                                                       */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i32CalResult;
    uint32_t u32DelayCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init UART0 for printf */
    UART0_Init();
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|            OPA Sample Code            |\n");
    printf("+---------------------------------------+\n");

    printf("\nThis sample code demonstrates OPA digital output function.\n");
    printf("OPA0_P (PB.0) is the positive input pin, OPA0_N (PB.1) is the negative input and OPA0_O (PB.2) is the OPA output pin.\n");
    printf("When the voltage of the positive input is greater than the voltage of the negative input,\n");
    printf("the digital output state, OPDO0 (OPA_STATUS[0]), will be set to 1; otherwise, it will be cleared to 0.\n");
    printf("This sample code will show the relation between the OPA's inputs and show a sequence \n");
    printf("number when detecting a transition of OPA's digital output.\n");
    printf("Press any key to start ...\n");
    getchar();


    /* Enable OPA0 schmitt trigger buffer */
    OPA_ENABLE_SCH_TRIGGER(OPA, 0);

    /* Power on the OPA0 circuit */
    OPA_POWER_ON(OPA, 0);

    /* Delay for OPA stable time */
    for (u32DelayCnt = 0; u32DelayCnt < 9600ul; u32DelayCnt++) __NOP();

    /* Clear OPA0 interrupt flag */
    OPA_CLR_INT_FLAG(OPA, 0);

    /* Enable OPA0 interrupt function */
    OPA_ENABLE_INT(OPA, 0);

    /* Enable OPA interrupt */
    NVIC_EnableIRQ(OPA_IRQn);

    /* Start OPA0 calibration */
    i32CalResult = OPA_Calibration(OPA, 0, OPA_CALIBRATION_CLK_1K, OPA_CALIBRATION_RV_1_2_AVDD);

    if (i32CalResult == 0)
        printf("OPA0 calibration result is Pass\n");
    else
        printf("OPA0 calibration result is Fail\n");

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
