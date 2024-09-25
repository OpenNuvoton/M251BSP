/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Demonstrate how to set I2C Master mode and Slave mode.
 *           And show how a master access a slave on a chip.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SLV_10BIT_ADDR (0x1E<<2)             //1111+0xx+r/w

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32Slave_buff_addr;
volatile uint8_t  g_au8SlvData[256];
volatile uint8_t  g_au8SlvRxData[3];
volatile uint8_t  g_u8DeviceHAddr;
volatile uint8_t  g_u8DeviceLAddr;
volatile uint8_t  g_u8DeviceAddr;
volatile uint8_t  g_au8MstTxData[3];
volatile uint8_t  g_u8MstRxData;
volatile uint8_t  g_u8MstDataLen;
volatile uint8_t  g_u8SlvDataLen;
volatile uint8_t  g_u8MstEndFlag = 0;
volatile uint8_t  g_u8MstTxAbortFlag = 0;
volatile uint8_t  g_u8MstRxAbortFlag = 0;
volatile uint8_t  g_u8MstReStartFlag = 0;
volatile uint8_t  g_u8Enable10BitMode = 0;
volatile uint8_t  g_u8TimeoutFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_pfnI2C0Handler = NULL;
static volatile I2C_FUNC s_pfnI2C1Handler = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
        g_u8TimeoutFlag = 1;
    }
    else
    {
        if (s_pfnI2C0Handler != NULL)
        {
            s_pfnI2C0Handler(u32Status);
        }
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
        if (s_pfnI2C1Handler != NULL)
        {
            s_pfnI2C1Handler(u32Status);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Master Rx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
        if (g_u8Enable10BitMode)
        {
            I2C_SET_DATA(I2C0, (g_u8DeviceHAddr << 1)); /* Write SLA+W to Register I2CDAT */
        }
        else
        {
            I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1)); /* Write SLA+W to Register I2CDAT */
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        if (g_u8Enable10BitMode)
        {
            I2C_SET_DATA(I2C0, g_u8DeviceLAddr);
        }
        else
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MstDataLen != 2)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        if (g_u8Enable10BitMode)
        {
            I2C_SET_DATA(I2C0, ((g_u8DeviceHAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        }
        else
        {
            I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
        g_u8MstRxData = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* Error condition process */
        printf("[MasterRx] Status [0x%x] Unexpected abort!! Press any key to re-start\n", u32Status);

        if (u32Status == 0x38)                /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if (u32Status == 0x30)           /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if (u32Status == 0x48)           /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if (u32Status == 0x00)           /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }

        /*Setting MasterRx abort flag for re-start mechanism*/
        g_u8MstRxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        u32TimeOutCnt = SystemCoreClock;

        while (I2C0->CTL0 & I2C_CTL0_SI_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                break;
            }
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Master Tx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        if (g_u8Enable10BitMode)
        {
            I2C_SET_DATA(I2C0, g_u8DeviceHAddr << 1);  /* Write SLA+W to Register I2CDAT */
        }
        else
        {
            I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);  /* Write SLA+W to Register I2CDAT */
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        if (g_u8Enable10BitMode)
        {
            I2C_SET_DATA(I2C0, g_u8DeviceLAddr);
        }
        else
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MstDataLen != 3)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* Error condition process */
        printf("[MasterTx] Status [0x%x] Unexpected abort!! Press any key to re-start\n", u32Status);

        if (u32Status == 0x38)                  /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if (u32Status == 0x00)             /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if (u32Status == 0x30)             /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if (u32Status == 0x48)             /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if (u32Status == 0x10)             /* Master repeat start, clear SI */
        {
            if (g_u8Enable10BitMode)
            {
                I2C_SET_DATA(I2C0, (uint32_t)((g_u8DeviceHAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            }
            else
            {
                I2C_SET_DATA(I2C0, (uint32_t)((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            }

            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }

        /*Setting MasterTRx abort flag for re-start mechanism*/
        g_u8MstTxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        u32TimeOutCnt = SystemCoreClock;

        while (I2C0->CTL0 & I2C_CTL0_SI_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                break;
            }
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 Slave TRx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8Data;
    uint32_t temp;

    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8Data = (unsigned char) I2C_GET_DATA(I2C1);

        if (g_u8SlvDataLen < 2)
        {
            g_au8SlvRxData[g_u8SlvDataLen++] = u8Data;
            temp = (uint32_t)(g_au8SlvRxData[0] << 8);
            temp += g_au8SlvRxData[1];
            g_u32Slave_buff_addr =  temp;
        }
        else
        {
            g_au8SlvData[g_u32Slave_buff_addr++] = u8Data;

            if (g_u32Slave_buff_addr == 256)
            {
                g_u32Slave_buff_addr = 0;
            }
        }

        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C1, g_au8SlvData[g_u32Slave_buff_addr]);
        g_u32Slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xB8)                 /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        I2C_SET_DATA(I2C1, g_au8SlvData[g_u32Slave_buff_addr++]);
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        printf("[SlaveTRx] Status [0x%x] Unexpected abort!!\n", u32Status);

        if (u32Status == 0x68)              /* Slave receive arbitration lost, clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        }
        else if (u32Status == 0xB0)         /* Address transmit arbitration lost, clear SI  */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        }
        else                                /* Slave bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable I2C0, I2C1 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);
    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Update System Core Clock */
    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();
    /* Set I2C0 multi-function pins */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_I2C0_SDA | SYS_GPB_MFPL_PB5MFP_I2C0_SCL);
    /* Set I2C1 multi-function pins */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk)) |
                    (SYS_GPA_MFPL_PA2MFP_I2C1_SDA | SYS_GPA_MFPL_PA3MFP_I2C1_SCL);
    /* I2C pins enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk;
    PB->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;
}


void I2C0_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);
    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    /* Open I2C1 module and set bus clock */
    I2C_Open(I2C1, 100000);
    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

    if (g_u8Enable10BitMode)
    {
        /* Enable I2C 10-bit address mode */
        I2C1->CTL1 |= I2C_CTL1_ADDR10EN_Msk;
        /* Set I2C1 4 Slave Addresses */
        I2C_SetSlaveAddr(I2C1, 0, 0x116, 0);
        I2C_SetSlaveAddr(I2C1, 1, 0x136, 0);
        I2C_SetSlaveAddr(I2C1, 2, 0x156, 0);
        I2C_SetSlaveAddr(I2C1, 3, 0x176, 0);
        /* Set I2C1 4 Slave Addresses Mask */
        I2C_SetSlaveAddrMask(I2C1, 0, 0x04);
        I2C_SetSlaveAddrMask(I2C1, 1, 0x02);
        I2C_SetSlaveAddrMask(I2C1, 2, 0x04);
        I2C_SetSlaveAddrMask(I2C1, 3, 0x02);
    }
    else
    {
        /* Set I2C1 4 Slave Addresses */
        I2C_SetSlaveAddr(I2C1, 0, 0x16, I2C_GCMODE_DISABLE);   /* Slave Address : 0x16 */
        I2C_SetSlaveAddr(I2C1, 1, 0x36, I2C_GCMODE_DISABLE);   /* Slave Address : 0x36 */
        I2C_SetSlaveAddr(I2C1, 2, 0x56, I2C_GCMODE_DISABLE);   /* Slave Address : 0x56 */
        I2C_SetSlaveAddr(I2C1, 3, 0x76, I2C_GCMODE_DISABLE);   /* Slave Address : 0x76 */
        /* Set I2C1 4 Slave Addresses Mask */
        I2C_SetSlaveAddrMask(I2C1, 0, 0x04);
        I2C_SetSlaveAddrMask(I2C1, 1, 0x02);
        I2C_SetSlaveAddrMask(I2C1, 2, 0x04);
        I2C_SetSlaveAddrMask(I2C1, 3, 0x02);
    }

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);
    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);
}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C1);
    NVIC_DisableIRQ(I2C1_IRQn);
    /* Disable I2C1 and close I2C1 clock */
    I2C_Close(I2C1);
    CLK_DisableModuleClock(I2C1_MODULE);
}

int32_t Read_Write_SLAVE(uint16_t u16SlvAddr)
{
    uint32_t i;

    if (g_u8Enable10BitMode)
    {
        /* Init Send 10-bit Addr */
        g_u8DeviceHAddr = (u16SlvAddr >> 8) | SLV_10BIT_ADDR;
        g_u8DeviceLAddr = u16SlvAddr & 0x7F;
    }
    else
    {
        g_u8DeviceAddr = (u16SlvAddr & 0x7F);
    }

    do
    {
        /* Enable I2C timeout */
        I2C_EnableTimeout(I2C0, 0);
        g_u8MstReStartFlag = 0;
        g_u8TimeoutFlag = 0;

        for (i = 0; i < 0x100; i++)
        {
            uint8_t u8MstTxData;
            g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
            g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
            g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);
            u8MstTxData = g_au8MstTxData[2];
            g_u8MstDataLen = 0;
            g_u8MstEndFlag = 0;
            /* I2C0 function to write data to slave */
            s_pfnI2C0Handler = (I2C_FUNC)I2C_MasterTx;
            /* I2C0 as master sends START signal */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Tx Finish or Unexpected Abort*/
            do
            {
                if (g_u8TimeoutFlag)
                {
                    printf(" MasterTx time out!! Press any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init();
                    /* Set MasterTx abort flag*/
                    g_u8MstTxAbortFlag = 1;
                }
            } while (g_u8MstEndFlag == 0 && g_u8MstTxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if (g_u8MstTxAbortFlag)
            {
                /* Clear MasterTx abort flag*/
                g_u8MstTxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }

            /* I2C0 function to read data from slave */
            s_pfnI2C0Handler = (I2C_FUNC)I2C_MasterRx;
            g_u8MstDataLen = 0;
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Rx Finish or Unexpected Abort*/
            do
            {
                if (g_u8TimeoutFlag)
                {
                    /* When I2C timeout, reset IP*/
                    printf(" MasterRx time out!! Press any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init();
                    /* Set MasterRx abort flag*/
                    g_u8MstRxAbortFlag = 1;
                }
            } while (g_u8MstEndFlag == 0 && g_u8MstRxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if (g_u8MstRxAbortFlag)
            {
                /* Clear MasterRx abort flag*/
                g_u8MstRxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }

            /* Compare data */
            if (g_u8MstRxData != u8MstTxData)
            {
                /* Disable I2C timeout */
                I2C_DisableTimeout(I2C0);
                printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
                return -1;
            }
        }
    } while (g_u8MstReStartFlag); /*If unexpected abort happens, re-start the transmition*/

    /* Disable I2C timeout */
    I2C_DisableTimeout(I2C0);
    printf("Master Access Slave (0x%X) Test OK\n", u16SlvAddr);
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i, ch;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);
    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+-------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code for loopback test              |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C1)               |\n");
    printf("+-------------------------------------------------------+\n");
    printf("\n");
    printf("Configure I2C0 as Master, and I2C1 as a slave.\n");
    printf(" - Mode Select, 0: 7-Bit Mode, 1: 10-Bit Mode\n");
    ch = getchar();

    if ('1' == ch)
    {
        g_u8Enable10BitMode = 1;
        printf(" -- 10-Bit Mode is selected. (0x115, 0x135)\n");
    }
    else
    {
        g_u8Enable10BitMode = 0;
        printf(" -- 7-Bit Mode is selected. (0x15, 0x35, 0x55, 0x75)\n");
    }

    /* Init I2C0 */
    I2C0_Init();
    /* Init I2C1 */
    I2C1_Init();
    /* I2C1 enter non address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

    for (i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C1 function to Slave receive/transmit data */
    s_pfnI2C1Handler = I2C_SlaveTRx;
    printf("\n");
    printf("I2C1 Slave Mode is Running.\n");

    if (g_u8Enable10BitMode)
    {
        /* Access Slave with no address */
        printf("\n");
        printf(" == No Mask Address ==\n");
        Read_Write_SLAVE(0x116);
        Read_Write_SLAVE(0x136);
        Read_Write_SLAVE(0x156);
        Read_Write_SLAVE(0x176);
        printf("Slave Address test OK.\n");
        /* Access Slave with address mask */
        printf("\n");
        printf(" == Mask Address ==\n");
        Read_Write_SLAVE(0x116 & ~0x04);
        Read_Write_SLAVE(0x136 & ~0x02);
        Read_Write_SLAVE(0x156 & ~0x04);
        Read_Write_SLAVE(0x176 & ~0x02);
        printf("Slave Address Mask test OK.\n");
    }
    else
    {
        /* Access Slave with no address */
        printf("\n");
        printf(" == No Mask Address ==\n");
        Read_Write_SLAVE(0x16);
        Read_Write_SLAVE(0x36);
        Read_Write_SLAVE(0x56);
        Read_Write_SLAVE(0x76);
        printf("Slave Address test OK.\n");
        /* Access Slave with address mask */
        printf("\n");
        printf(" == Mask Address ==\n");
        Read_Write_SLAVE(0x16 & ~0x04);
        Read_Write_SLAVE(0x36 & ~0x02);
        Read_Write_SLAVE(0x56 & ~0x04);
        Read_Write_SLAVE(0x76 & ~0x02);
        printf("Slave Address Mask test OK.\n");
    }

    s_pfnI2C0Handler = NULL;
    s_pfnI2C1Handler = NULL;
    /* Close I2C0,1 */
    I2C0_Close();
    I2C1_Close();

    while (1);
}

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/


