/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show how to use USCI_I2C interface to access EEPROM.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_48MHZ

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8TxData[3];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLenM;
volatile uint8_t g_u8EndFlagM = 0;
volatile enum UI2C_MASTER_EVENT g_eMEvent;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

volatile static UI2C_FUNC s_UI2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = (UI2C0->PROTSTS);

    if (s_UI2C0HandlerFn != NULL)
        s_UI2C0HandlerFn(u32Status);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C Tx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_EEPROM_MasterTx(uint32_t u32Status)
{
    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);                    /* Clear START INT Flag */
        UI2C_SET_DATA(UI2C0, (g_u8DeviceAddr << 1) | 0x00);                        /* Write SLA+W to Register TXDAT */
        g_eMEvent = MASTER_SEND_ADDRESS;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);                     /* Clear ACK INT Flag */

        if (g_eMEvent == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8TxData[g_u8DataLenM++]);                     /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            g_eMEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (g_eMEvent == MASTER_SEND_DATA)
        {
            if (g_u8DataLenM != 3)
            {
                UI2C_SET_DATA(UI2C0, g_au8TxData[g_u8DataLenM++]);                 /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                g_eMEvent = MASTER_STOP;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));       /* Send STOP signal */
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);                    /* Clear NACK INT Flag */
        g_u8EndFlagM = 0;

        if (g_eMEvent == MASTER_SEND_ADDRESS)
        {
            /* SLA+W has been transmitted and NACK has been received */
            g_eMEvent = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));           /* Send START signal */
        }
        else if (g_eMEvent == MASTER_SEND_DATA)
        {
            /* ADDRESS has been transmitted and NACK has been received */
            g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));           /* Send STOP signal */
        }
        else
            printf("Get Wrong NACK Event\n");
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);    /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        g_u8EndFlagM = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C Rx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_EEPROM_MasterRx(uint32_t u32Status)
{
    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);    /* Clear START INT Flag */

        if (g_eMEvent == MASTER_SEND_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceAddr << 1) | 0x00);    /* Write SLA+W to Register TXDAT */
            g_eMEvent = MASTER_SEND_ADDRESS;
        }
        else if (g_eMEvent == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceAddr << 1) | 0x01);    /* Write SLA+R to Register TXDAT */
            g_eMEvent = MASTER_SEND_H_RD_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);    /* Clear ACK INT Flag */

        if (g_eMEvent == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8TxData[g_u8DataLenM++]);    /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            g_eMEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (g_eMEvent == MASTER_SEND_DATA)
        {
            if (g_u8DataLenM != 2)
            {
                UI2C_SET_DATA(UI2C0, g_au8TxData[g_u8DataLenM++]);    /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                g_eMEvent = MASTER_SEND_REPEAT_START;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send repeat START signal */
            }
        }
        else if (g_eMEvent == MASTER_SEND_H_RD_ADDRESS)
        {
            g_eMEvent = MASTER_READ_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);    /* Clear NACK INT Flag */

        if (g_eMEvent == MASTER_SEND_ADDRESS)
        {
            g_eMEvent = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send START signal */
        }
        else if (g_eMEvent == MASTER_READ_DATA)
        {
            g_u8RxData = (unsigned char) UI2C_GET_DATA(UI2C0) & 0xFF;
            g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */
        }
        else
            printf("Get Wrong NACK Event\n");
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);    /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        g_u8EndFlagM = 1;
    }
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

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* using semihost to show message so we don't set multi-function pins for uart*/
    //Uart0DefaultMPF();

    /* Set UI2C0 multi-function pins */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_USCI0_CLK | SYS_GPB_MFPH_PB13MFP_USCI0_DAT0);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void UI2C0_Init(void)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, 100000);

    /* Get USCI_I2C0 Bus Clock */
    printf("USCI I2C clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */

    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN func                                                                                                                        tion                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Idx;
    uint8_t u8Temp;

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

    /*
        This sample code sets I2C bus clock to 100kHz. Then, accesses EEPROM 24LC64 with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|     USCI_I2C Driver Sample Code with EEPROM 24LC64    |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init USCI_I2C0 to access EEPROM */
    UI2C0_Init();

    g_u8DeviceAddr = 0x50;

    for (u32Idx = 0; u32Idx < 2; u32Idx++)
    {
        g_au8TxData[0] = (uint8_t)((u32Idx & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(u32Idx & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);

        g_u8DataLenM = 0;
        g_u8EndFlagM = 0;

        /* USCI_I2C function to write data to slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)USCI_I2C_EEPROM_MasterTx;

        /* USCI_I2C as master sends START signal */
        g_eMEvent = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Tx Finish */
        while (g_u8EndFlagM == 0);

        g_u8EndFlagM = 0;

        /* USCI_I2C function to read data from slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)USCI_I2C_EEPROM_MasterRx;

        g_u8DataLenM = 0;
        g_u8DeviceAddr = 0x50;

        g_eMEvent = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Rx Finish */
        while (g_u8EndFlagM == 0);

        g_u8EndFlagM = 0;

        /* Compare data */
        u8Temp = g_au8TxData[2];

        if (g_u8RxData != u8Temp)
        {
            printf("USCI_I2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
            return -1;
        }
    }

    printf("USCI_I2C Access EEPROM Test OK\n");

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
