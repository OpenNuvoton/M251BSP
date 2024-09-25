/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demo USCI_I2C Monitor Mode
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define I2C_DATA_MAX    20
#define MONITOR_ADDR    0x16

enum UI2C_Monitor_State
{
    GET_Mon_START = 1,
    GET_Mon_ACK,
    GET_Mon_SLV_W,
    GET_Mon_Data,
    GET_Mon_RESTART,
    GET_Mon_SLV_R,
    GET_MON_NACK,
    GET_MON_STOP
};

volatile uint8_t    g_au8MstTxData[I2C_DATA_MAX];
volatile uint8_t    g_au8SlvRxData[I2C_DATA_MAX];
volatile uint8_t    g_u8DeviceAddr;
volatile uint32_t   g_u32SlaveBuffAddr = 0;
volatile uint8_t    g_u8MstDataLen;
volatile uint8_t    g_u8SlvDataLen;
volatile uint32_t   g_u32ProtOn;
volatile uint8_t    g_au8MonRxData[(I2C_DATA_MAX + 1) * 2] = {0};
volatile uint8_t    g_u8MonDataCnt = 0;
volatile uint32_t   g_u32PCLKClock = 0;

volatile uint8_t g_u8EndFlag = 0;

uint8_t g_u8RxDataTmp;
uint8_t g_au8SlvData[256];

enum UI2C_MASTER_EVENT g_eMEvent;
enum UI2C_SLAVE_EVENT g_eSEvent;

typedef void (*I2C_FUNC)(uint32_t u32Status);
static I2C_FUNC s_I2C0HandlerFn = NULL;
static I2C_FUNC s_I2C1HandlerFn = NULL;

typedef void (*UI2C_FUNC)(uint32_t u32Status);
static UI2C_FUNC s_UI2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    uint32_t u32Status;

    //UI2C0 Interrupt
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);

    if (s_UI2C0HandlerFn != NULL)
        s_UI2C0HandlerFn(u32Status);
}

void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C1);

    if (I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
    }
    else
    {
        if (s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Set I2C 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C1, 100000);

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C1, 0, MONITOR_ADDR, 0);   /* Slave Address : MONITOR_ADDR */
    I2C_SetSlaveAddr(I2C1, 1, 0x35, 0);                 /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C1, 2, 0x55, 0);                 /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C1, 3, 0x75, 0);                 /* Slave Address : 0x75 */

    /* Set I2C 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C1, 0, 0x04);
    I2C_SetSlaveAddrMask(I2C1, 1, 0x02);
    I2C_SetSlaveAddrMask(I2C1, 2, 0x04);
    I2C_SetSlaveAddrMask(I2C1, 3, 0x02);

    /* Enable I2C1 interrupt and set corresponding NVIC bit */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);

    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, MONITOR_ADDR, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x16 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x36, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x36 */

    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x04);                    /* Slave Address : 0x4 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x02);                    /* Slave Address : 0x2 */

    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI0_IRQn);

    //UI2C0 be a Slave
    g_eSEvent = SLAVE_ADDRESS_ACK;

    UI2C0->PROTCTL |= (UI2C_PROTCTL_MONEN_Msk | UI2C_PROTCTL_SCLOUTEN_Msk);
}

void UI2C_SLV_7bit_Monitor(uint32_t u32Status)
{
    uint32_t u32Rxdata;

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);

        if (g_u32ProtOn == 0)
        {
            g_u32ProtOn = 1;
        }

        g_eSEvent = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        if (g_eSEvent == SLAVE_ADDRESS_ACK)
        {
            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                u32Rxdata = (uint8_t)UI2C_GET_DATA(UI2C0);
                g_au8MonRxData[g_u8MonDataCnt++] = u32Rxdata;
                g_eSEvent = SLAVE_SEND_DATA;
            }
            else
            {
                u32Rxdata = (uint8_t)UI2C_GET_DATA(UI2C0);
                g_au8MonRxData[g_u8MonDataCnt++] = u32Rxdata;
                g_eSEvent = SLAVE_GET_DATA;
            }

            if (((u32Rxdata >> 1) != (UI2C0->DEVADDR0 & 0xFF)) && ((u32Rxdata >> 1) != (UI2C0->DEVADDR1 & 0xFF)))
            {
                /* Check Receive Adddress not match */
                printf("\n[Error]Receive address(0x%x) not match!\n", u32Rxdata);

                while (1);
            }

            g_eSEvent = SLAVE_GET_DATA;
        }
        else if (g_eSEvent == SLAVE_GET_DATA)
        {
            u32Rxdata = (uint8_t)UI2C_GET_DATA(UI2C0);
            g_au8MonRxData[g_u8MonDataCnt++] = u32Rxdata;
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);
        u32Rxdata = (uint8_t)UI2C_GET_DATA(UI2C0);
        g_au8MonRxData[g_u8MonDataCnt++] = u32Rxdata;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STO INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);
        g_u32ProtOn = 0;
        g_eSEvent = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
}

void I2C_SlaveTRx_7bit(uint32_t u32Status)
{
    if (u32Status == 0x60) /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        g_u8RxDataTmp = (unsigned char)I2C_GET_DATA(I2C1);
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80) /* Previously address with own SLA address
                                  Data has been received; ACK has been returned*/
    {
        g_u8RxDataTmp = (unsigned char)I2C_GET_DATA(I2C1);
        g_au8SlvRxData[g_u8SlvDataLen] = g_u8RxDataTmp;
        g_u8SlvDataLen++;

        if (g_u8SlvDataLen == 2)
        {
            g_u32SlaveBuffAddr = g_au8SlvRxData[1];
            g_u32SlaveBuffAddr += (g_au8SlvRxData[0] << 8);
        }

        if (g_u8SlvDataLen == I2C_DATA_MAX - 2)
        {
            g_au8SlvData[g_u32SlaveBuffAddr] = g_au8SlvRxData[I2C_DATA_MAX - 2];
            g_u8SlvDataLen = 0;
        }

        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8) /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C1, g_au8SlvData[g_u32SlaveBuffAddr]);
        g_u32SlaveBuffAddr++;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xC0) /* Data byte or last data in I2CDAT has been transmitted NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88) /* Previously addressed with own SLA address; NACK has been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)  /* A STOP or repeated START has been received while still
                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08) /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1));/* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18) /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20) /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28) /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MstDataLen != I2C_DATA_MAX/*-1*/)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8EndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08) /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1));  /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18) /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20) /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28) /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MstDataLen != (I2C_DATA_MAX - 2))
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if (u32Status == 0x10) /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));     /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x40) /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x58) /* DATA has been received and NACK has been returned */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
        g_u8EndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

int32_t Read_Write_SLAVE_Mon(uint8_t slvaddr)
{
    uint32_t u32Idx;

    g_u8DeviceAddr = slvaddr;

    printf("Dump transmitted data:\n");

    for (u32Idx = 0; u32Idx < I2C_DATA_MAX; u32Idx++)
    {
        g_au8MstTxData[u32Idx] = 5 + u32Idx;
        printf("[0x%X]\t", g_au8MstTxData[u32Idx]);
    }

    g_u8MonDataCnt = 0;
    g_u8MstDataLen = 0;
    g_u8EndFlag = 0;

    /* I2C function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Wait I2C Tx Finish */
    while (g_u8EndFlag == 0);

    g_u8EndFlag = 0;

    /* I2C function to read data from slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

    g_u8MstDataLen = 0;
    g_u8DeviceAddr = slvaddr;

    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Wait I2C Rx Finish */
    while (g_u8EndFlag == 0);

    while (g_u32ProtOn);

    return 0;
}

int32_t UI2C_Monitor_7bit_test()
{
    int32_t err = 0;
    uint32_t u32Idx;

    UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_AA);

    for (u32Idx = 0; u32Idx < 0x100; u32Idx++)
    {
        g_au8SlvData[u32Idx] = 0;
    }

    g_u32ProtOn = 0;

    /* I2C function to Slave receive/transmit data */
    s_UI2C0HandlerFn = UI2C_SLV_7bit_Monitor;

    I2C1_Init();
    /* I2C1 enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

    for (u32Idx = 0; u32Idx < 0x100; u32Idx++)
    {
        g_au8SlvData[u32Idx] = 0;
    }

    /* I2C function to Slave receive/transmit data */
    s_I2C1HandlerFn = I2C_SlaveTRx_7bit;

    /* I2C IP as Master */
    I2C0_Init();

    err = Read_Write_SLAVE_Mon(MONITOR_ADDR);

    printf("\nDump Monitor data: \n");

    for (u32Idx = 0; u32Idx < (I2C_DATA_MAX + 1); u32Idx++)
    {
        if (u32Idx == 0)
            printf("Monitor address: [0x%X]", g_au8MonRxData[u32Idx] >> 1);
        else
            printf("[0x%X]\t", g_au8MonRxData[u32Idx]);

        if (u32Idx % 8 == 0)
            printf("\n");
    }

    printf("\n\n");

    for (u32Idx = 0; u32Idx < I2C_DATA_MAX; u32Idx++)
        g_au8MonRxData[u32Idx] = 0;

    return err;
}

void SYS_Init(void)
{

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable USCI0 clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPA multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA1MFP_Msk) | SYS_GPA_MFPL_PA1MFP_UART0_TXD;

    /* Set UI2C0 multi-function pins */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_USCI0_CLK | SYS_GPB_MFPH_PB13MFP_USCI0_DAT0);

    /* Set I2C0 multi-function pins */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_I2C0_SDA | SYS_GPB_MFPL_PB5MFP_I2C0_SCL);

    /* Set I2C1 multi-function pins */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk)) |
                    (SYS_GPA_MFPL_PA2MFP_I2C1_SDA | SYS_GPA_MFPL_PA3MFP_I2C1_SCL);

    /* I2C pins enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk;
    PB->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;
    PB->SMTEN |= GPIO_SMTEN_SMTEN12_Msk | GPIO_SMTEN_SMTEN13_Msk;

}

int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Open(UART0, 115200);

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for Monitor Mode         |\n");
    printf("|  7-bit Monitor mode test                              |\n");
    printf("|  I2C0(Master)  <----> UI2C0(Monitor) & I2C1(Slave)    |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a monitor mode.\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PB.13), UI2C0_SCL(PB.12)\n");
    printf("\n");
    printf("Configure I2C0 as Master, and I2C1 as a slave.\n");
    printf("The I/O connection I2C0 to I2C1:\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5)\n");
    printf("I2C1_SDA(PA.2), I2C1_SCL(PA.3)\n\n");

    /* Init USCI_I2C0 */
    UI2C0_Init(100000);

    g_eSEvent = SLAVE_ADDRESS_ACK;

    while (1)
    {
        printf("Monitor test ....\n");
        UI2C_Monitor_7bit_test();
        printf("Press any key to continue\n");
        getchar();
    }

}


