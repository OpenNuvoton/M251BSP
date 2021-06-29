/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement HDQ protocol by PSIO.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "BQ2028_driver_EEPROM.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

PSIO_BQ2028_CFG_T g_Config;

typedef void (*PSIO_FUNC)(PSIO_BQ2028_CFG_T *pConfig);
PSIO_FUNC s_pfnPSIOHandler = NULL;


void PSIO_IRQHandler(void)
{
    /* Get slot controller done interrupt flag */
    if (PSIO_GET_INT_FLAG(PSIO, PSIO_INTSTS_SC0IF_Msk))
    {
        /* Clear slot controller done interrupt flag */
        PSIO_CLEAR_INT_FLAG(PSIO, PSIO_INTSTS_SC0IF_Msk);

        if (s_pfnPSIOHandler != NULL)
        {
            s_pfnPSIOHandler(&g_Config);
        }
    }
    else
    {
        printf("Invalid interrupt occur \n");
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

    /* Select PSIO module clock source as PCLK1(48Mhz/4) and PSIO module clock divider as 0x7D */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_PCLK1, CLK_CLKDIV1_PSIO(0x7Du));

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
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB15MFP_Msk) | SYS_GPB_MFPH_PB15MFP_PSIO0_CH0;
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
    uint8_t u8RxData = 0, u8DataCnt = 0;
    uint8_t u8Col, u8Row, u8Page, u8CRC;

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
    printf("|           HDQ TI BQ2028 EEPROM Test Code           |\n");
    printf("|      Please connected PSIO_CH0(PB.15) to device    |\n");
    printf("******************************************************\n");

    /* Use slot controller 0 and pin 0 */
    g_Config.u8SlotCtrl   = PSIO_SC0;
    g_Config.u8Data0Pin   = PSIO_PIN0;

    /* Initialize PSIO setting for BQ2028 */
    PSIO_BQ2028_Open(&g_Config);

    /* Send "Read ID" command and read device ID */
    PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_READID, &u8RxData);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    printf("BQ2028 ID is %x.\n", u8RxData);

    /* Send "Read ID" command and read device revision */
    PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_READREV, &u8RxData);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    printf("REV is %x.\n", u8RxData);

    /* Send "Read device status" command and read device status */
    PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_STATUS, &u8RxData);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    /* Clear reset flag if reset flag is 1 */
    if (u8RxData & STATUS_RST_MSK)
    {
        /* Send "Read control 0 register" command and read control 0 register value */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_CTL0, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Send "Write control 0 register" command and write STATUS_RST_MSK to clear reset flag */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_CTL0, u8RxData | STATUS_RST_MSK);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }
    }

    printf("/***********************************/\n");
    printf("|       Buffer R/W Test             |\n");
    printf("/***********************************/\n");

    printf("Buffer 0 R/W Testing...");

    /* Write 0x0 ~ 0xFF to buffer 0 */
    for (u8DataCnt = 0; u8DataCnt < 0xFF; u8DataCnt++)
    {
        /* Write 1 Byte data to buffer 0 */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_BUF0, u8DataCnt);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Read 1 Byte from buffer 0 */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_BUF0, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Check write data correct or not */
        if (u8DataCnt != u8RxData)
        {
            printf("[Error] Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

            while (1);
        }
    }

    printf("PASS!\n");

    printf("Buffer 1 R/W Testing...");

    /* Write 0x0 ~ 0xFF to buffer 1 */
    for (u8DataCnt = 0; u8DataCnt < 0xFF; u8DataCnt++)
    {
        /* Write 1 Byte data to buffer 1 */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_BUF1, u8DataCnt);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Read 1 Byte from buffer 1 */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_BUF1, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Check write data correct or not */
        if (u8DataCnt != u8RxData)
        {
            printf("[Error] Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

            while (1);
        }
    }

    printf("PASS!\n");

    printf("Buffer 2 R/W Testing...");

    /* Write 0x0 ~ 0xFF to buffer 2 */
    for (u8DataCnt = 0; u8DataCnt < 0xFF; u8DataCnt++)
    {
        /* Write 1 Byte data to buffer 2 */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_BUF2, u8DataCnt);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Read 1 Byte from buffer 2 */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_BUF2, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Check data correct or not */
        if (u8DataCnt != u8RxData)
        {
            printf("[Error] Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

            while (1);
        }
    }

    printf("PASS!\n");

    printf("Buffer 3 R/W Testing...");

    /* Write 0x0 ~ 0xFF to buffer 3 */
    for (u8DataCnt = 0; u8DataCnt < 0xFF; u8DataCnt++)
    {
        /* Write 1 Byte data to buffer 3 */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_BUF3, u8DataCnt);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Read 1 Byte from buffer 3 */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_BUF3, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Check data correct or not */
        if (u8DataCnt != u8RxData)
        {
            printf("[Error] Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

            while (1);
        }
    }

    printf("PASS!\n");

    printf("/***********************************/\n");
    printf("|       EEPROM R/W Test             |\n");
    printf("/***********************************/\n");

    /* Enable to write manufacturer area registers */
    PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_CTL2, 0x01);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    /* Enable page */
    PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_PAGE_EN, 0xFF);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    /* Update EEPROM page 0 ~ page 7 */
    for (u8Page = 0; u8Page < 8; u8Page++)
    {
        /* Setting current page which we want to access */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_PAGE, u8Page);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        printf("Page %d write pattern\n", u8Page);

        /* Access row 0x0 ~0xF */
        for (u8Row = 0; u8Row < 0x10; u8Row++)
        {
            /* Access column 0x0 ~0x3 */
            for (u8Col = 0; u8Col < 4; u8Col++)
            {
                /* Prepare data and calculating the CRC value */
                u8DataCnt = (u8Page * 0x10 * 4) + (0x10 * u8Row) + u8Col;
                u8CRC   =   PSIO_BQ2028_CRC8(u8DataCnt);

                /* Send "Write and EEPROM address" command and write data to EEPROM */
                PSIO_BQ2028_Write_OneByte(&g_Config, (u8Row << 2) | u8Col | HDQ_W | HDQ_MAP, u8DataCnt);

                while (PSIO_BQ2028_BUSY())
                {
                    /* Do something here */
                }

                /* Send "Check CRC" command and write CRC value */
                PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_CRCT, u8CRC);

                while (PSIO_BQ2028_BUSY())
                {
                    /* Do something here */
                }

                /* Check EEPROM status is normal */
                while (1)
                {
                    PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_STATUS, &u8RxData);

                    while (PSIO_BQ2028_BUSY())
                    {
                        /* Do something here */
                    }

                    if (u8RxData & (STATUS_PAGEER_MSK | STATUS_MEMER_MSK | STATUS_CRCER_MSK))
                    {
                        printf("status error 0x%x", u8RxData);
                    }
                    else if (!(u8RxData & STATUS_BUSY_MSK))
                        break;
                }
            }
        }
    }

    /* Verify EEPROM page 0 ~ page 7 */
    for (u8Page = 0; u8Page < 8; u8Page++)
    {
        /* Setting current page which we want to access */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_PAGE, u8Page);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        printf("Page %d read pattern\n", u8Page);

        /* Access row 0x0 ~0xF */
        for (u8Row = 0; u8Row < 0x10; u8Row++)
        {
            /* Access column 0x0 ~0x3 */
            for (u8Col = 0; u8Col < 4; u8Col++)
            {
                u8DataCnt = (u8Page * 0x10 * 4) + (0x10 * u8Row) + u8Col;

                /* Send "Read and EEPROM address" command and read data from EEPROM */
                PSIO_BQ2028_Read_OneByte(&g_Config, (u8Row << 2) | u8Col | HDQ_R | HDQ_MAP, &u8RxData);

                while (PSIO_BQ2028_BUSY())
                {
                    /* Do something here */
                }

                /* Check data correct or not */
                if (u8DataCnt != u8RxData)
                {
                    printf("[Error]Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

                    while (1);
                }
            }
        }
    }

    printf("PASS...\n");

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
