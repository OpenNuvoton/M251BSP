/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Access SPI flash using QSPI quad mode
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Fast
#define SlewRateMode        1
// <c1> Enable QSPI Optimize
// <i> Use FIFO mechanism and enable PDMA for QSPI RX to maximize throughput.
#define ENABLE_QSPI_OPTIMIZE
// </c>
// *** <<< end of configuration section >>> ***

#define TEST_NUMBER         1   /* page numbers */
#define TEST_LENGTH         256 /* length */

#define SPI_FLASH_PORT      QSPI0
#define PDMA_PORT           PDMA

#ifdef ENABLE_QSPI_OPTIMIZE
    #define QSPI_RX_PDMA_CH 0
#endif

uint8_t g_au8SrcArray[TEST_LENGTH];
uint8_t g_au8DestArray[TEST_LENGTH];

//------------------------------------------------------------------------------
#ifdef ENABLE_QSPI_OPTIMIZE
static void QSPI_PDMA_Rx_Init(QSPI_T *module, uint32_t *u32RxTargetAddr, uint32_t u32TransferCount)
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    PDMA_Open(PDMA_PORT, (1 << QSPI_RX_PDMA_CH));

    /* --- RX PDMA  --- */
    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA_PORT, QSPI_RX_PDMA_CH, PDMA_WIDTH_32, u32TransferCount);
    //:APB to MEM (QSPI RX RAM)
    PDMA_SetTransferMode(PDMA_PORT, QSPI_RX_PDMA_CH, PDMA_QSPI0_RX, FALSE, 0);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA_PORT, QSPI_RX_PDMA_CH,
                         (uint32_t) & (module)->RX,
                         PDMA_SAR_FIX,
                         (uint32_t)u32RxTargetAddr,
                         PDMA_DAR_INC);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA_PORT, QSPI_RX_PDMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA_PORT->DSCT[QSPI_RX_PDMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

static void QSPI_PDMA_Rx_Polling(void)
{
    uint32_t u32RetryTimes = SystemCoreClock;

    while (1)
    {
        /* Polling status flag. */
        if (PDMA_GET_TD_STS(PDMA_PORT) & (1 << QSPI_RX_PDMA_CH))
        {
            break;
        }

        u32RetryTimes--;

        if (u32RetryTimes == 0)
        {
            break;
        }
    }

    PDMA_CLR_TD_FLAG(PDMA_PORT, (1 << QSPI_RX_PDMA_CH));

    /* Disable SPI master's PDMA transfer function */
    QSPI_DISABLE_RX_PDMA(SPI_FLASH_PORT);
}
#endif

void B0B1_SwitchToNormalMode(void)
{
    SYS->GPB_MFPL =  SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT1, GPIO_MODE_OUTPUT);
    PB0 = 1;
    PB1 = 1;
}

void B0B1_SwitchToQuadMode(void)
{
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_QSPI0_MOSI1 | SYS_GPB_MFPL_PB1MFP_QSPI0_MISO1;
}

uint16_t SpiFlash_ReadMidDid(void)
{
    uint8_t u8RxData[6], u8IDCnt = 0;

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x90, Read Manufacturer/Device ID
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x90);

    // send 24-bit '0', dummy
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // receive 16-bit
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    while (!QSPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
        u8RxData[u8IDCnt ++] = QSPI_READ_RX(SPI_FLASH_PORT);

    return ((u8RxData[4] << 8) | u8RxData[5]);
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    QSPI_ClearRxFIFO(SPI_FLASH_PORT);
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    uint8_t u8Val;

    QSPI_ClearRxFIFO(SPI_FLASH_PORT);

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

    // read status
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    u8Val = QSPI_READ_RX(SPI_FLASH_PORT);
    u8Val = QSPI_READ_RX(SPI_FLASH_PORT);

    return u8Val;
}

uint8_t SpiFlash_ReadStatusReg2(void)
{
    uint8_t u8Val;

    QSPI_ClearRxFIFO(SPI_FLASH_PORT);

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x35, Read status register
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x35);

    // read status
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    u8Val = QSPI_READ_RX(SPI_FLASH_PORT);
    u8Val = QSPI_READ_RX(SPI_FLASH_PORT);

    return u8Val;
}

void SpiFlash_WriteStatusReg(uint8_t u8Value1, uint8_t u8Value2)
{
    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x01);

    // write status
    QSPI_WRITE_TX(SPI_FLASH_PORT, u8Value1);
    QSPI_WRITE_TX(SPI_FLASH_PORT, u8Value2);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

void SpiFlash_WaitReady(void)
{
    volatile uint8_t u8ReturnValue;

    do
    {
        u8ReturnValue = SpiFlash_ReadStatusReg();
        u8ReturnValue = u8ReturnValue & 1;
    } while (u8ReturnValue != 0); // check the BUSY bit
}

void SpiFlash_NormalPageProgram(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt = 0;

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);


    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 16) & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 8)  & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    // write data
    while (1)
    {
        if (!QSPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT))
        {
            QSPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[u32Cnt++]);

            if (u32Cnt > 255) break;
        }
    }

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    QSPI_ClearRxFIFO(SPI_FLASH_PORT);
}

void spiFlash_EnableQEBit(void)
{
    uint8_t u8Status1 = SpiFlash_ReadStatusReg();
    uint8_t u8Status2 = SpiFlash_ReadStatusReg2();

    u8Status2 |= 0x2;
    SpiFlash_WriteStatusReg(u8Status1, u8Status2);
    SpiFlash_WaitReady();
}

void spiFlash_DisableQEBit(void)
{
    uint8_t u8Status1 = SpiFlash_ReadStatusReg();
    uint8_t u8Status2 = SpiFlash_ReadStatusReg2();

    u8Status2 &= ~0x2;
    SpiFlash_WriteStatusReg(u8Status1, u8Status2);
    SpiFlash_WaitReady();
}

void SpiFlash_QuadFastRead(uint32_t StartAddress, uint8_t *u8DataBuffer)
{

#if !defined (ENABLE_QSPI_OPTIMIZE)
    uint32_t u32Cnt;
#else
    uint8_t u8TXS;
    uint32_t u32TxDataCount = 0;

    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA_PORT, QSPI_RX_PDMA_CH, PDMA_WIDTH_32, 64);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA_PORT, QSPI_RX_PDMA_CH,
                         (uint32_t) & ((QSPI_T *)SPI_FLASH_PORT)->RX,
                         PDMA_SAR_FIX,
                         (uint32_t)u8DataBuffer,
                         PDMA_DAR_INC);
#endif

    // enable quad mode
    spiFlash_EnableQEBit();

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // Command: 0xEB, Fast Read quad data
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0xEB);

    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // enable SPI dual IO mode and set direction to input
    B0B1_SwitchToQuadMode();
    QSPI_ENABLE_QUAD_OUTPUT_MODE(SPI_FLASH_PORT);

    // send 24-bit start address
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 16) & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 8)  & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    // dummy byte
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    QSPI_ENABLE_QUAD_INPUT_MODE(SPI_FLASH_PORT);

    // clear RX buffer
    QSPI_ClearRxFIFO(SPI_FLASH_PORT);

#ifdef ENABLE_QSPI_OPTIMIZE
    QSPI_ENABLE_BYTE_REORDER(SPI_FLASH_PORT);
    QSPI_SET_DATA_WIDTH(SPI_FLASH_PORT, 32);

    /* Enable SPI master PDMA function */
    QSPI_TRIGGER_RX_PDMA(QSPI0);

    while (u32TxDataCount < 64)
    {
        /* Check TX FIFO count. The Maximum FIFO is 8 layers. */
        u8TXS = (8 - QSPI_GET_TX_FIFO_COUNT(SPI_FLASH_PORT));
        u32TxDataCount += u8TXS;

        while (u8TXS--)
        {
            QSPI_WRITE_TX(SPI_FLASH_PORT, 0xFFFFFFFF);
        }
    }

    QSPI_PDMA_Rx_Polling();
#else

    // read data
    for (u32Cnt = 0; u32Cnt < 256; u32Cnt++)
    {
        QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

        while (QSPI_IS_BUSY(SPI_FLASH_PORT));

        u8DataBuffer[u32Cnt] = QSPI_READ_RX(SPI_FLASH_PORT);
    }

#endif

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

#ifdef ENABLE_QSPI_OPTIMIZE
    QSPI_DISABLE_BYTE_REORDER(SPI_FLASH_PORT);
    QSPI_SET_DATA_WIDTH(SPI_FLASH_PORT, 8);
#endif

    QSPI_DISABLE_QUAD_MODE(SPI_FLASH_PORT);
    B0B1_SwitchToNormalMode();

    // disable quad mode
    spiFlash_DisableQEBit();
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select PCLK0 as the clock source of QSPI0 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable QSPI0 peripheral clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

#ifdef ENABLE_QSPI_OPTIMIZE
    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA_MODULE);
#endif

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set UART0 Default MPF */
    Uart0DefaultMPF() ;
    /* Setup QSPI0 multi-function pins */
    SYS->GPA_MFPL = SYS_GPA_MFPL_PA0MFP_QSPI0_MOSI0 | SYS_GPA_MFPL_PA1MFP_QSPI0_MISO0 | SYS_GPA_MFPL_PA2MFP_QSPI0_CLK | SYS_GPA_MFPL_PA3MFP_QSPI0_SS ;
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_QSPI0_MOSI1 | SYS_GPB_MFPL_PB1MFP_QSPI0_MISO1 ;

    /* Enable QSPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

#if (SlewRateMode == 0)
    /* Enable QSPI0 I/O normal slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5, GPIO_SLEWCTL_NORMAL);
#elif (SlewRateMode == 1)
    /* Enable QSPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5, GPIO_SLEWCTL_HIGH);
#elif (SlewRateMode == 2)
    /* Enable QSPI0 I/O fast slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5, GPIO_SLEWCTL_FAST);
#endif
}

/* Main */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t u32Error = 0;
    uint16_t u16ID;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    B0B1_SwitchToNormalMode();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 20MHz */
    QSPI_Open(SPI_FLASH_PORT, QSPI_MASTER, QSPI_MODE_0, 8, 20000000);

    /* Disable auto SS function, control SS signal manually. */
    QSPI_DisableAutoSS(SPI_FLASH_PORT);

#ifdef ENABLE_QSPI_OPTIMIZE
    QSPI_PDMA_Rx_Init(SPI_FLASH_PORT, 0, 0);
#endif

    printf("\n+------------------------------------------------------------------------+\n");
    printf("|                SPI Quad Mode with Flash Sample Code                    |\n");
    printf("+------------------------------------------------------------------------+\n");

    u16ID = SpiFlash_ReadMidDid();

    if (u16ID == 0xEF13)
        printf("Flash found: W25Q80 ...\n");
    else if (u16ID == 0xEF14)
        printf("Flash found: W25Q16 ...\n");
    else if (u16ID == 0xEF15)
        printf("Flash found: W25Q32 ...\n");
    else if (u16ID == 0xEF16)
        printf("Flash found: W25Q64 ...\n");
    else if (u16ID == 0xEF17)
        printf("Flash found: W25Q128 ...\n");
    else
    {
        printf("Wrong ID, 0x%x\n", u16ID);

        while (1);
    }

    printf("Erase chip ...");

    /* Erase SPI flash */
    SpiFlash_ChipErase();

    /* Wait ready */
    SpiFlash_WaitReady();

    printf("[OK]\n");

    /* init source data buffer */
    for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        g_au8SrcArray[u32ByteCount] = u32ByteCount;
    }

    printf("Start to write data to Flash ...");
    /* Program SPI flash */
    u32FlashAddress = 0;

    for (u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page program */
        SpiFlash_NormalPageProgram(u32FlashAddress, g_au8SrcArray);
        SpiFlash_WaitReady();
        u32FlashAddress += 0x100;
    }

    printf("[OK]\n");

    /* clear destination data buffer */
    for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        g_au8DestArray[u32ByteCount] = 0;
    }

    printf("Quad Read & Compare ...");

    /* Read SPI flash */
    u32FlashAddress = 0;

    for (u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page read */
        SpiFlash_QuadFastRead(u32FlashAddress, g_au8DestArray);
        u32FlashAddress += 0x100;

        for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
        {
            if (g_au8DestArray[u32ByteCount] != g_au8SrcArray[u32ByteCount])
                u32Error ++;
        }
    }

    if (u32Error == 0)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/


