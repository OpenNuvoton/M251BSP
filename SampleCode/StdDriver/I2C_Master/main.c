/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Show a Master how to access Slave.
 *           This sample code needs to work with I2C_Slave.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t s_u8DeviceAddr;
static volatile uint8_t s_au8MstTxData[3];
static volatile uint8_t s_u8MstRxData;
static volatile uint8_t s_u8MstDataLen;
static volatile uint8_t s_u8MstEndFlag = 0;
static volatile uint8_t s_u8MstTxAbortFlag = 0;
static volatile uint8_t s_u8MstRxAbortFlag = 0;
static volatile uint8_t s_u8MstReStartFlag = 0;
static volatile uint8_t s_u8TimeoutFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_pfnI2C0Handler = NULL;

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
        s_u8TimeoutFlag = 1;
    }
    else
    {
        if (s_pfnI2C0Handler != NULL)
            s_pfnI2C0Handler(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (s_u8DeviceAddr << 1)); /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, s_au8MstTxData[s_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA | I2C_CTL_STO | I2C_CTL_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (s_u8MstDataLen != 2)
        {
            I2C_SET_DATA(I2C0, s_au8MstTxData[s_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, ((s_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
        s_u8MstRxData = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
        s_u8MstEndFlag = 1;
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
        s_u8MstRxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);

        while (I2C0->CTL0 & I2C_CTL0_SI_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, s_u8DeviceAddr << 1);  /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, s_au8MstTxData[s_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA | I2C_CTL_STO | I2C_CTL_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (s_u8MstDataLen != 3)
        {
            I2C_SET_DATA(I2C0, s_au8MstTxData[s_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            s_u8MstEndFlag = 1;
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
            I2C_SET_DATA(I2C0, (uint32_t)((s_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }

        /*Setting MasterTRx abort flag for re-start mechanism*/
        s_u8MstTxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);

        while (I2C0->CTL0 & I2C_CTL0_SI_Msk);
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

    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable GPIO clock */
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

    /* I2C pins enable schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
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

int32_t I2C0_Read_Write_Slave(uint8_t slvaddr)
{

    do
    {
        uint32_t i = 0;
        /* Enable I2C timeout */
        I2C_EnableTimeout(I2C0, 0);
        s_u8MstReStartFlag = 0;
        s_u8DeviceAddr = slvaddr;
        s_u8TimeoutFlag = 0;

        for (i = 0; i < 0x100; i++)
        {
            s_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
            s_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
            s_au8MstTxData[2] = (uint8_t)(s_au8MstTxData[1] + 3);

            s_u8MstDataLen = 0;
            s_u8MstEndFlag = 0;

            /* I2C function to write data to slave */
            s_pfnI2C0Handler = (I2C_FUNC)I2C_MasterTx;

            /* I2C as master sends START signal */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Tx Finish or Unexpected Abort*/
            do
            {
                if (s_u8TimeoutFlag)
                {
                    printf(" MasterTx time out!! Press any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init();
                    /* Set MasterTx abort flag*/
                    s_u8MstTxAbortFlag = 1;
                }
            } while (s_u8MstEndFlag == 0 && s_u8MstTxAbortFlag == 0);

            s_u8MstEndFlag = 0;

            if (s_u8MstTxAbortFlag)
            {
                /* Clear MasterTx abort flag*/
                s_u8MstTxAbortFlag = 0;
                /* Set Master re-start flag*/
                s_u8MstReStartFlag = 1;
                break;
            }

            /* I2C function to read data from slave */
            s_pfnI2C0Handler = (I2C_FUNC)I2C_MasterRx;

            s_u8MstDataLen = 0;
            s_u8DeviceAddr = slvaddr;

            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Rx Finish or Unexpected Abort*/
            do
            {
                if (s_u8TimeoutFlag)
                {
                    /* When I2C timeout, reset IP*/
                    printf(" MasterRx time out!! Press any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init();
                    /* Set MasterRx abort flag*/
                    s_u8MstRxAbortFlag = 1;
                }
            } while (s_u8MstEndFlag == 0 && s_u8MstRxAbortFlag == 0);

            s_u8MstEndFlag = 0;

            if (s_u8MstRxAbortFlag)
            {
                /* Clear MasterRx abort flag*/
                s_u8MstRxAbortFlag = 0;
                /* Set Master re-start flag*/
                s_u8MstReStartFlag = 1;
                break;
            }

            /* Compare data */
            if (s_u8MstRxData != s_au8MstTxData[2])
            {
                printf("I2C Byte Write/Read Failed, Data 0x%x\n", s_u8MstRxData);
                return -1;
            }

        }
    } while (s_u8MstReStartFlag); /*If unexpected abort happens, re-start the transmition*/



    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|       I2C Driver Sample Code(Master) for access Slave |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Configure I2C0 as a master.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5)\n");

    /* Init I2C0 */
    I2C0_Init();

    printf("\n");
    printf("Check I2C Slave(I2C0) is running first!\n");
    printf("Press any key to continue.\n");
    getchar();

    /* Access Slave with no address */
    printf("\n");
    printf(" == No Mask Address ==\n");
    I2C0_Read_Write_Slave(0x15);
    I2C0_Read_Write_Slave(0x35);
    I2C0_Read_Write_Slave(0x55);
    I2C0_Read_Write_Slave(0x75);
    printf("Slave Address test OK.\n");

    /* Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    I2C0_Read_Write_Slave(0x15 & ~0x01);
    I2C0_Read_Write_Slave(0x35 & ~0x04);
    I2C0_Read_Write_Slave(0x55 & ~0x01);
    I2C0_Read_Write_Slave(0x75 & ~0x04);
    printf("Slave Address Mask test OK.\n");

    s_pfnI2C0Handler = NULL;

    /* Close I2C0 */
    I2C0_Close();

    while (1);
}
