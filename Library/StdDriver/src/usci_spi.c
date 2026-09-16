/***************************************************************************//**
 * @file    usci_spi.c
 * @version V1.00
 * @brief   USCI_SPI driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup USCI_SPI_Driver USCI_SPI Driver
  @{
*/


/** @addtogroup USCI_SPI_EXPORTED_FUNCTIONS USCI_SPI Exported Functions
  @{
*/

#define USPI_BRGEN_CLKDIV_MAX  (USPI_BRGEN_CLKDIV_Msk >> USPI_BRGEN_CLKDIV_Pos)

/**
 * @brief      This function is used to enable USCI_SPI protocol.
 *
 * @param[in]  uspi The pointer of the specified USCI_SPI module.
 *
 * @return     None.
 */
static uint32_t USPI_IsValidModule(const USPI_T *uspi)
{
    return (((uspi == USPI0) || (uspi == USPI1) || (uspi == USPI2)) ? 1UL : 0UL);
}

/**
 * @brief      Check USCI_SPI master/slave mode is valid or not
 *
 * @param[in]  u32MasterSlave The master/slave mode.
 *
 * @return     1 = Valid master/slave mode; 0 = Invalid master/slave mode.
 */
static uint32_t USPI_IsValidMasterSlave(uint32_t u32MasterSlave)
{
    return (((u32MasterSlave == USPI_MASTER) || (u32MasterSlave == USPI_SLAVE)) ? 1UL : 0UL);
}

/**
 * @brief      Check USCI_SPI mode is valid or not
 *
 * @param[in]  u32SPIMode The SPI mode.
 *
 * @return     1 = Valid SPI mode; 0 = Invalid SPI mode.
 */
static uint32_t USPI_IsValidMode(uint32_t u32SPIMode)
{
    return (((u32SPIMode == USPI_MODE_0) || (u32SPIMode == USPI_MODE_1) ||
             (u32SPIMode == USPI_MODE_2) || (u32SPIMode == USPI_MODE_3)) ? 1UL : 0UL);
}

/**
 * @brief      Check USCI_SPI automatic slave select configuration is valid or not
 *
 * @param[in]  u32SSPinMask   The slave select pin mask.
 * @param[in]  u32ActiveLevel The active level of the slave select signal.
 *
 * @return     1 = Valid auto SS configuration; 0 = Invalid auto SS configuration.
 */
static uint32_t USPI_IsValidAutoSSConfig(uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    return ((((u32SSPinMask == 0UL) || (u32SSPinMask == USPI_SS)) &&
             ((u32ActiveLevel == USPI_SS_ACTIVE_HIGH) || (u32ActiveLevel == USPI_SS_ACTIVE_LOW))) ? 1UL : 0UL);
}

/**
 * @brief   Get the clock source frequency for the specified USPI peripheral.
 *
 * @param   uspi Pointer to the USPI_T structure that represents the USPI peripheral.
 *
 * @return  The clock frequency in Hz for the specified USPI peripheral.
 */
static uint32_t USPI_GetPCLKSrcFreq(const USPI_T *uspi)
{
    uint32_t u32Pclk = 0UL;

    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return 0UL;
    }

    if ((uspi == (USPI_T *)USPI0) || (uspi == (USPI_T *)USPI2))
    {
        u32Pclk = CLK_GetPCLK0Freq();
    }
    else
    {
        u32Pclk = CLK_GetPCLK1Freq();
    }

    return u32Pclk;
}

/**
 * @brief      This function make USCI_SPI module be ready to transfer.
 *             By default, the USCI_SPI transfer sequence is MSB first, the slave selection
 *             signal is active low and the automatic slave select function is disabled. In
 *             Slave mode, the u32BusClock must be 0 and the USCI_SPI clock
 *             divider setting will be 0. The returned clock rate is the USCI_SPI peripheral
 *             clock source frequency.
 *
 * @param[in]  uspi           The pointer of the specified USCI_SPI module.
 * @param[in]  u32MasterSlave Decide the USCI_SPI module is operating in master mode or in slave mode. Valid values are:
 *                            - \ref USPI_SLAVE
 *                            - \ref USPI_MASTER
 * @param[in]  u32SPIMode     Decide the transfer timing. Valid values are:
 *                            - \ref USPI_MODE_0
 *                            - \ref USPI_MODE_1
 *                            - \ref USPI_MODE_2
 *                            - \ref USPI_MODE_3
 * @param[in]  u32DataWidth   The data width of a USCI_SPI transaction.
 * @param[in]  u32BusClock    The expected frequency of USCI_SPI bus clock in Hz.
 *
 * @return     Actual frequency of USCI_SPI peripheral clock.
 */
uint32_t USPI_Open(USPI_T *uspi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock)
{
    uint32_t u32UspiClk = 0UL;
    uint32_t u32Ctl = 0UL;
    uint32_t u32LineCtl = 0UL;
    uint32_t u32CtlIn0 = 0UL;
    uint32_t u32ProtCtl = 0UL;
    uint32_t u32Brgen = 0UL;

    if ((USPI_IsValidModule(uspi) == 0UL) ||
            (USPI_IsValidMasterSlave(u32MasterSlave) == 0UL) ||
            (USPI_IsValidMode(u32SPIMode) == 0UL))
    {
        return 0UL;
    }

    u32Ctl = uspi->CTL;
    u32LineCtl = uspi->LINECTL;
    u32CtlIn0 = uspi->CTLIN0;
    u32ProtCtl = uspi->PROTCTL;
    u32Brgen = uspi->BRGEN;

    /* Enable USCI_SPI protocol */
    uspi->CTL &= ~(USPI_CTL_FUNMODE_Msk);
    uspi->CTL |= (1UL << USPI_CTL_FUNMODE_Pos);

    /* Data format configuration */
    USPI_SET_DATA_WIDTH(uspi, u32DataWidth);

    /* MSB data format */
    uspi->LINECTL &= ~(USPI_LINECTL_LSB_Msk);

    /* Set slave selection signal active low */
    if (u32MasterSlave == USPI_MASTER)
    {
        uspi->LINECTL |= USPI_LINECTL_CTLOINV_Msk;
    }
    else
    {
        uspi->CTLIN0 |= USPI_CTLIN0_ININV_Msk;
    }

    /* Set operating mode and transfer timing */
    uspi->PROTCTL &= ~(USPI_PROTCTL_SCLKMODE_Msk |
                       USPI_PROTCTL_AUTOSS_Msk |
                       USPI_PROTCTL_SLAVE_Msk);
    uspi->PROTCTL |= (u32MasterSlave | u32SPIMode);

    if (u32MasterSlave == USPI_MASTER)
    {
        /* Set USCI_SPI bus clock */
        u32UspiClk = USPI_SetBusClock(uspi, u32BusClock);

        if (u32UspiClk == 0UL)
        {
            uspi->CTL = u32Ctl;
            uspi->LINECTL = u32LineCtl;
            uspi->CTLIN0 = u32CtlIn0;
            uspi->PROTCTL = u32ProtCtl;
            uspi->BRGEN = u32Brgen;
            return 0UL;
        }
    }
    else
    {
        /* In slave mode the divider is not used and remains cleared. */
        uspi->BRGEN &= ~USPI_BRGEN_CLKDIV_Msk;
        u32UspiClk = USPI_GetPCLKSrcFreq(uspi);
    }

    uspi->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;

    return u32UspiClk;
}

/**
 * @brief      Disable USCI_SPI function mode.
 *
 * @param[in]  uspi The pointer of the specified USCI_SPI module.
 *
 * @return     None
 */
void USPI_Close(USPI_T *uspi)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    if (u32RegLockLevel != 0UL)
    {
        /* Unlock protected registers */
        SYS_UnlockReg();
    }

    uspi->CTL &= ~(USPI_CTL_FUNMODE_Msk);

    if (u32RegLockLevel != 0UL)
    {
        /* Lock protected registers */
        SYS_LockReg();
    }
}

/**
 * @brief      Clear Rx buffer.
 *
 * @param[in]  uspi The pointer of the specified USCI_SPI module.
 *
 * @return     None
 */
void USPI_ClearRxBuf(USPI_T *uspi)
{
    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    uspi->BUFCTL |= USPI_BUFCTL_RXCLR_Msk;
}

/**
 * @brief      Clear Tx buffer.
 *
 * @param[in]  uspi The pointer of the specified USCI_SPI module.
 *
 * @return     None
 */
void USPI_ClearTxBuf(USPI_T *uspi)
{
    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    uspi->BUFCTL |= USPI_BUFCTL_TXCLR_Msk;
}

/**
 * @brief      Disable the automatic slave select function.
 *
 * @param[in]  uspi The pointer of the specified USCI_SPI module.
 *
 * @return     None
 */
void USPI_DisableAutoSS(USPI_T *uspi)
{
    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    uspi->PROTCTL &= ~(USPI_PROTCTL_AUTOSS_Msk | USPI_PROTCTL_SS_Msk);
}

/**
 * @brief      Enable the automatic slave select function. Only available in Master mode.
 *
 * @param[in]  uspi           The pointer of the specified USCI_SPI module.
 * @param[in]  u32SSPinMask   This parameter is not used.
 * @param[in]  u32ActiveLevel The active level of slave select signal. Valid values are:
 *                            - \ref USPI_SS_ACTIVE_HIGH
 *                            - \ref USPI_SS_ACTIVE_LOW
 *
 * @return     None
 */
void USPI_EnableAutoSS(USPI_T *uspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    if ((USPI_IsValidModule(uspi) == 0UL) ||
            (USPI_IsValidAutoSSConfig(u32SSPinMask, u32ActiveLevel) == 0UL))
    {
        return;
    }

    uspi->LINECTL = (uspi->LINECTL & ~USPI_LINECTL_CTLOINV_Msk) | u32ActiveLevel;
    uspi->PROTCTL |= USPI_PROTCTL_AUTOSS_Msk;
}

/**
 * @brief      Set USCI_SPI data width.
 *
 * @param[in]  uspi     The pointer of the specified USCI_SPI module.
 * @param[in]  u32Width Requested data width.
 *
 * @return     None
 *
 * @details
 *             The SPI data-width register encoding is:
 *             - Register value 0 represents 16-bit width.
 *             - Values from 4 to 16 represent the corresponding data width.
 *             - Requested widths from 1 to 4 are clamped to 4 bits.
 *             - Requested widths greater than or equal to 16 are configured as 16 bits.
 */
void USPI_SetDataWidth(USPI_T *uspi, uint32_t u32Width)
{
    uint32_t u32RegWidth;

    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    if (u32Width == 0UL)
    {
        /*
         * Register value 0 represents 16-bit data width.
         */
        u32RegWidth = 0UL;
    }
    else if (u32Width < 4UL)
    {
        /*
         * Clamp unsupported widths from 1 to 4
         * to the minimum supported width of 4 bits.
         */
        u32RegWidth = 4UL;
    }
    else if (u32Width < 16UL)
    {
        /*
         * Register directly encodes data widths
         * from 4 to 15 bits.
         */
        u32RegWidth = u32Width;
    }
    else
    {
        /*
         * Widths greater than or equal to 16
         * are configured as 16-bit width.
         */
        u32RegWidth = 0UL;
    }

    uspi->LINECTL = (uspi->LINECTL & ~USPI_LINECTL_DWIDTH_Msk) |
                    (u32RegWidth << USPI_LINECTL_DWIDTH_Pos);
}

/**
 * @brief      Set the USCI_SPI bus clock. Only available in Master mode.
 *
 * @param[in]  uspi        The pointer of the specified USCI_SPI module.
 * @param[in]  u32BusClock The expected frequency of USCI_SPI bus clock.
 *
 * @return     Actual frequency of USCI_SPI peripheral clock.
 */
uint32_t USPI_SetBusClock(USPI_T *uspi, uint32_t u32BusClock)
{
    uint32_t u32ClkDiv = 0UL;
    uint32_t u32Pclk = 0UL;
    uint32_t u32UspiClk = 0UL;

    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return 0UL;
    }

    if (u32BusClock == 0UL)
    {
        return 0UL;
    }

    u32Pclk = USPI_GetPCLKSrcFreq(uspi);

    if (u32Pclk == 0UL)
    {
        return 0UL;
    }

    if (u32BusClock >= (u32Pclk / 2UL))
    {
        u32ClkDiv = 0UL;
        u32UspiClk = (u32Pclk / 2UL);
    }
    else
    {
        /* Compute proper divider for USCI_SPI clock */
        u32ClkDiv = (uint32_t)((((((u32Pclk / 2UL) * 10UL) / (u32BusClock)) + 5UL) / 10UL) - 1UL);

        if (u32ClkDiv > USPI_BRGEN_CLKDIV_MAX)
        {
            u32ClkDiv = USPI_BRGEN_CLKDIV_MAX;
        }

        u32UspiClk = (u32Pclk / ((u32ClkDiv + 1UL) << 1UL));
    }

    /* Set USCI_SPI bus clock */
    uspi->BRGEN &= ~USPI_BRGEN_CLKDIV_Msk;
    uspi->BRGEN |= (u32ClkDiv << USPI_BRGEN_CLKDIV_Pos);

    return u32UspiClk;
}

/**
 * @brief      Get the actual frequency of USCI_SPI bus clock. Only available in Master mode.
 *
 * @param[in]  uspi The pointer of the specified USCI_SPI module.
 *
 * @return     Actual USCI_SPI bus clock frequency.
 */
uint32_t USPI_GetBusClock(const USPI_T *uspi)
{
    uint32_t u32ClkDiv = 0UL;
    uint32_t u32Pclk = 0UL;

    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return 0UL;
    }

    u32Pclk = USPI_GetPCLKSrcFreq(uspi);

    u32ClkDiv = ((uspi->BRGEN & USPI_BRGEN_CLKDIV_Msk) >> USPI_BRGEN_CLKDIV_Pos);

    return (u32Pclk / ((u32ClkDiv + 1UL) << 1));
}

/**
 * @brief      Enable related interrupts specified by u32Mask parameter.
 *
 * @param[in]  uspi    The pointer of the specified USCI_SPI module.
 * @param[in]  u32Mask The combination of all related interrupt enable bits.
 *                     Each bit corresponds to a interrupt bit.
 *                     This parameter decides which interrupts will be enabled. Valid values are:
 *                     - \ref USPI_SSINACT_INT_MASK
 *                     - \ref USPI_SSACT_INT_MASK
 *                     - \ref USPI_SLVTO_INT_MASK
 *                     - \ref USPI_SLVBE_INT_MASK
 *                     - \ref USPI_TXUDR_INT_MASK
 *                     - \ref USPI_RXOV_INT_MASK
 *                     - \ref USPI_TXST_INT_MASK
 *                     - \ref USPI_TXEND_INT_MASK
 *                     - \ref USPI_RXST_INT_MASK
 *                     - \ref USPI_RXEND_INT_MASK
 *
 * @return     None
 */
void USPI_EnableInt(USPI_T *uspi, uint32_t u32Mask)
{
    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    /* Enable slave selection signal inactive interrupt flag */
    if ((u32Mask & USPI_SSINACT_INT_MASK) == USPI_SSINACT_INT_MASK)
    {
        uspi->PROTIEN |= USPI_PROTIEN_SSINAIEN_Msk;
    }

    /* Enable slave selection signal active interrupt flag */
    if ((u32Mask & USPI_SSACT_INT_MASK) == USPI_SSACT_INT_MASK)
    {
        uspi->PROTIEN |= USPI_PROTIEN_SSACTIEN_Msk;
    }

    /* Enable slave time-out interrupt flag */
    if ((u32Mask & USPI_SLVTO_INT_MASK) == USPI_SLVTO_INT_MASK)
    {
        uspi->PROTIEN |= USPI_PROTIEN_SLVTOIEN_Msk;
    }

    /* Enable slave bit count error interrupt flag */
    if ((u32Mask & USPI_SLVBE_INT_MASK) == USPI_SLVBE_INT_MASK)
    {
        uspi->PROTIEN |= USPI_PROTIEN_SLVBEIEN_Msk;
    }

    /* Enable TX under run interrupt flag */
    if ((u32Mask & USPI_TXUDR_INT_MASK) == USPI_TXUDR_INT_MASK)
    {
        uspi->BUFCTL |= USPI_BUFCTL_TXUDRIEN_Msk;
    }

    /* Enable RX overrun interrupt flag */
    if ((u32Mask & USPI_RXOV_INT_MASK) == USPI_RXOV_INT_MASK)
    {
        uspi->BUFCTL |= USPI_BUFCTL_RXOVIEN_Msk;
    }

    /* Enable TX start interrupt flag */
    if ((u32Mask & USPI_TXST_INT_MASK) == USPI_TXST_INT_MASK)
    {
        uspi->INTEN |= USPI_INTEN_TXSTIEN_Msk;
    }

    /* Enable TX end interrupt flag */
    if ((u32Mask & USPI_TXEND_INT_MASK) == USPI_TXEND_INT_MASK)
    {
        uspi->INTEN |= USPI_INTEN_TXENDIEN_Msk;
    }

    /* Enable RX start interrupt flag */
    if ((u32Mask & USPI_RXST_INT_MASK) == USPI_RXST_INT_MASK)
    {
        uspi->INTEN |= USPI_INTEN_RXSTIEN_Msk;
    }

    /* Enable RX end interrupt flag */
    if ((u32Mask & USPI_RXEND_INT_MASK) == USPI_RXEND_INT_MASK)
    {
        uspi->INTEN |= USPI_INTEN_RXENDIEN_Msk;
    }
}

/**
 * @brief      Disable related interrupts specified by u32Mask parameter.
 *
 * @param[in]  uspi    The pointer of the specified USCI_SPI module.
 * @param[in]  u32Mask The combination of all related interrupt enable bits.
 *                     Each bit corresponds to a interrupt bit.
 *                     This parameter decides which interrupts will be disabled. Valid values are:
 *                     - \ref USPI_SSINACT_INT_MASK
 *                     - \ref USPI_SSACT_INT_MASK
 *                     - \ref USPI_SLVTO_INT_MASK
 *                     - \ref USPI_SLVBE_INT_MASK
 *                     - \ref USPI_TXUDR_INT_MASK
 *                     - \ref USPI_RXOV_INT_MASK
 *                     - \ref USPI_TXST_INT_MASK
 *                     - \ref USPI_TXEND_INT_MASK
 *                     - \ref USPI_RXST_INT_MASK
 *                     - \ref USPI_RXEND_INT_MASK
 *
 * @return     None
 */
void USPI_DisableInt(USPI_T *uspi, uint32_t u32Mask)
{
    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    /* Disable slave selection signal inactive interrupt flag */
    if ((u32Mask & USPI_SSINACT_INT_MASK) == USPI_SSINACT_INT_MASK)
    {
        uspi->PROTIEN &= ~USPI_PROTIEN_SSINAIEN_Msk;
    }

    /* Disable slave selection signal active interrupt flag */
    if ((u32Mask & USPI_SSACT_INT_MASK) == USPI_SSACT_INT_MASK)
    {
        uspi->PROTIEN &= ~USPI_PROTIEN_SSACTIEN_Msk;
    }

    /* Disable slave time-out interrupt flag */
    if ((u32Mask & USPI_SLVTO_INT_MASK) == USPI_SLVTO_INT_MASK)
    {
        uspi->PROTIEN &= ~USPI_PROTIEN_SLVTOIEN_Msk;
    }

    /* Disable slave bit count error interrupt flag */
    if ((u32Mask & USPI_SLVBE_INT_MASK) == USPI_SLVBE_INT_MASK)
    {
        uspi->PROTIEN &= ~USPI_PROTIEN_SLVBEIEN_Msk;
    }

    /* Disable TX under run interrupt flag */
    if ((u32Mask & USPI_TXUDR_INT_MASK) == USPI_TXUDR_INT_MASK)
    {
        uspi->BUFCTL &= ~USPI_BUFCTL_TXUDRIEN_Msk;
    }

    /* Disable RX overrun interrupt flag */
    if ((u32Mask & USPI_RXOV_INT_MASK) == USPI_RXOV_INT_MASK)
    {
        uspi->BUFCTL &= ~USPI_BUFCTL_RXOVIEN_Msk;
    }

    /* Disable TX start interrupt flag */
    if ((u32Mask & USPI_TXST_INT_MASK) == USPI_TXST_INT_MASK)
    {
        uspi->INTEN &= ~USPI_INTEN_TXSTIEN_Msk;
    }

    /* Disable TX end interrupt flag */
    if ((u32Mask & USPI_TXEND_INT_MASK) == USPI_TXEND_INT_MASK)
    {
        uspi->INTEN &= ~USPI_INTEN_TXENDIEN_Msk;
    }

    /* Disable RX start interrupt flag */
    if ((u32Mask & USPI_RXST_INT_MASK) == USPI_RXST_INT_MASK)
    {
        uspi->INTEN &= ~USPI_INTEN_RXSTIEN_Msk;
    }

    /* Disable RX end interrupt flag */
    if ((u32Mask & USPI_RXEND_INT_MASK) == USPI_RXEND_INT_MASK)
    {
        uspi->INTEN &= ~USPI_INTEN_RXENDIEN_Msk;
    }
}

/**
 * @brief      Get interrupt flag.
 *
 * @param[in]  uspi    The pointer of the specified USCI_SPI module.
 * @param[in]  u32Mask The combination of all related interrupt sources.
 *                     Each bit corresponds to a interrupt source.
 *                     This parameter decides which interrupt flags will be read. It is combination of:
 *                     - \ref USPI_SSINACT_INT_MASK
 *                     - \ref USPI_SSACT_INT_MASK
 *                     - \ref USPI_SLVTO_INT_MASK
 *                     - \ref USPI_SLVBE_INT_MASK
 *                     - \ref USPI_TXUDR_INT_MASK
 *                     - \ref USPI_RXOV_INT_MASK
 *                     - \ref USPI_TXST_INT_MASK
 *                     - \ref USPI_TXEND_INT_MASK
 *                     - \ref USPI_RXST_INT_MASK
 *                     - \ref USPI_RXEND_INT_MASK
 *
 * @return     Interrupt flags of selected sources.
 */
uint32_t USPI_GetIntFlag(const USPI_T *uspi, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0UL;
    uint32_t u32ProtSts = 0UL;
    uint32_t u32BufSts = 0UL;

    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return 0UL;
    }

    u32ProtSts = uspi->PROTSTS;
    u32BufSts = uspi->BUFSTS;

    /* Check slave selection signal inactive interrupt flag */
    if (((u32Mask & USPI_SSINACT_INT_MASK) == USPI_SSINACT_INT_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_SSINAIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_SSINACT_INT_MASK;
    }

    /* Check slave selection signal active interrupt flag */
    if (((u32Mask & USPI_SSACT_INT_MASK) == USPI_SSACT_INT_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_SSACTIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_SSACT_INT_MASK;
    }

    /* Check slave time-out interrupt flag */
    if (((u32Mask & USPI_SLVTO_INT_MASK) == USPI_SLVTO_INT_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_SLVTOIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_SLVTO_INT_MASK;
    }

    /* Check slave bit count error interrupt flag */
    if (((u32Mask & USPI_SLVBE_INT_MASK) == USPI_SLVBE_INT_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_SLVBEIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_SLVBE_INT_MASK;
    }

    /* Check TX under run interrupt flag */
    if (((u32Mask & USPI_TXUDR_INT_MASK) == USPI_TXUDR_INT_MASK) &&
            ((u32BufSts & USPI_BUFSTS_TXUDRIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_TXUDR_INT_MASK;
    }

    /* Check RX overrun interrupt flag */
    if (((u32Mask & USPI_RXOV_INT_MASK) == USPI_RXOV_INT_MASK) &&
            ((u32BufSts & USPI_BUFSTS_RXOVIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_RXOV_INT_MASK;
    }

    /* Check TX start interrupt flag */
    if (((u32Mask & USPI_TXST_INT_MASK) == USPI_TXST_INT_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_TXSTIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_TXST_INT_MASK;
    }

    /* Check TX end interrupt flag */
    if (((u32Mask & USPI_TXEND_INT_MASK) == USPI_TXEND_INT_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_TXENDIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_TXEND_INT_MASK;
    }

    /* Check RX start interrupt flag */
    if (((u32Mask & USPI_RXST_INT_MASK) == USPI_RXST_INT_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_RXSTIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_RXST_INT_MASK;
    }

    /* Check RX end interrupt flag */
    if (((u32Mask & USPI_RXEND_INT_MASK) == USPI_RXEND_INT_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_RXENDIF_Msk) != 0UL))
    {
        u32IntFlag |= USPI_RXEND_INT_MASK;
    }

    return u32IntFlag;
}

/**
 * @brief      Clear interrupt flag.
 *
 * @param[in]  uspi    The pointer of the specified USCI_SPI module.
 * @param[in]  u32Mask The combination of all related interrupt sources.
 *                     Each bit corresponds to a interrupt source.
 *                     This parameter decides which interrupt flags will be cleared. It could be the combination of:
 *                     - \ref USPI_SSINACT_INT_MASK
 *                     - \ref USPI_SSACT_INT_MASK
 *                     - \ref USPI_SLVTO_INT_MASK
 *                     - \ref USPI_SLVBE_INT_MASK
 *                     - \ref USPI_TXUDR_INT_MASK
 *                     - \ref USPI_RXOV_INT_MASK
 *                     - \ref USPI_TXST_INT_MASK
 *                     - \ref USPI_TXEND_INT_MASK
 *                     - \ref USPI_RXST_INT_MASK
 *                     - \ref USPI_RXEND_INT_MASK
 *
 * @return     None
 */
void USPI_ClearIntFlag(USPI_T *uspi, uint32_t u32Mask)
{
    uint32_t u32ProtClrMask = 0UL;
    uint32_t u32BufClrMask = 0UL;

    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    /* Clear slave selection signal inactive interrupt flag */
    if ((u32Mask & USPI_SSINACT_INT_MASK) == USPI_SSINACT_INT_MASK)
    {
        u32ProtClrMask |= USPI_PROTSTS_SSINAIF_Msk;
    }

    /* Clear slave selection signal active interrupt flag */
    if ((u32Mask & USPI_SSACT_INT_MASK) == USPI_SSACT_INT_MASK)
    {
        u32ProtClrMask |= USPI_PROTSTS_SSACTIF_Msk;
    }

    /* Clear slave time-out interrupt flag */
    if ((u32Mask & USPI_SLVTO_INT_MASK) == USPI_SLVTO_INT_MASK)
    {
        u32ProtClrMask |= USPI_PROTSTS_SLVTOIF_Msk;
    }

    /* Clear slave bit count error interrupt flag */
    if ((u32Mask & USPI_SLVBE_INT_MASK) == USPI_SLVBE_INT_MASK)
    {
        u32ProtClrMask |= USPI_PROTSTS_SLVBEIF_Msk;
    }

    /* Clear TX under run interrupt flag */
    if ((u32Mask & USPI_TXUDR_INT_MASK) == USPI_TXUDR_INT_MASK)
    {
        u32BufClrMask |= USPI_BUFSTS_TXUDRIF_Msk;
    }

    /* Clear RX overrun interrupt flag */
    if ((u32Mask & USPI_RXOV_INT_MASK) == USPI_RXOV_INT_MASK)
    {
        u32BufClrMask |= USPI_BUFSTS_RXOVIF_Msk;
    }

    /* Clear TX start interrupt flag */
    if ((u32Mask & USPI_TXST_INT_MASK) == USPI_TXST_INT_MASK)
    {
        u32ProtClrMask |= USPI_PROTSTS_TXSTIF_Msk;
    }

    /* Clear TX end interrupt flag */
    if ((u32Mask & USPI_TXEND_INT_MASK) == USPI_TXEND_INT_MASK)
    {
        u32ProtClrMask |= USPI_PROTSTS_TXENDIF_Msk;
    }

    /* Clear RX start interrupt flag */
    if ((u32Mask & USPI_RXST_INT_MASK) == USPI_RXST_INT_MASK)
    {
        u32ProtClrMask |= USPI_PROTSTS_RXSTIF_Msk;
    }

    /* Clear RX end interrupt flag */
    if ((u32Mask & USPI_RXEND_INT_MASK) == USPI_RXEND_INT_MASK)
    {
        u32ProtClrMask |= USPI_PROTSTS_RXENDIF_Msk;
    }

    if (u32ProtClrMask != 0UL)
    {
        uspi->PROTSTS = u32ProtClrMask;
    }

    if (u32BufClrMask != 0UL)
    {
        uspi->BUFSTS = u32BufClrMask;
    }
}

/**
 * @brief      Get USCI_SPI status.
 *
 * @param[in]  uspi    The pointer of the specified USCI_SPI module.
 * @param[in]  u32Mask The combination of all related sources.
 *                     Each bit corresponds to a source.
 *                     This parameter decides which flags will be read. It is combination of:
 *                     - \ref USPI_BUSY_MASK
 *                     - \ref USPI_RX_EMPTY_MASK
 *                     - \ref USPI_RX_FULL_MASK
 *                     - \ref USPI_TX_EMPTY_MASK
 *                     - \ref USPI_TX_FULL_MASK
 *                     - \ref USPI_SSLINE_STS_MASK
 *
 * @return     Flags of selected sources.
 */
uint32_t USPI_GetStatus(const USPI_T *uspi, uint32_t u32Mask)
{
    uint32_t u32Flag = 0UL;
    uint32_t u32ProtSts = 0UL;
    uint32_t u32BufSts = 0UL;

    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return 0UL;
    }

    u32ProtSts = uspi->PROTSTS;
    u32BufSts = uspi->BUFSTS;

    /* Check busy status */
    if (((u32Mask & USPI_BUSY_MASK) == USPI_BUSY_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_BUSY_Msk) != 0UL))
    {
        u32Flag |= USPI_BUSY_MASK;
    }

    /* Check RX empty flag */
    if (((u32Mask & USPI_RX_EMPTY_MASK) == USPI_RX_EMPTY_MASK) &&
            ((u32BufSts & USPI_BUFSTS_RXEMPTY_Msk) != 0UL))
    {
        u32Flag |= USPI_RX_EMPTY_MASK;
    }

    /* Check RX full flag */
    if (((u32Mask & USPI_RX_FULL_MASK) == USPI_RX_FULL_MASK) &&
            ((u32BufSts & USPI_BUFSTS_RXFULL_Msk) != 0UL))
    {
        u32Flag |= USPI_RX_FULL_MASK;
    }

    /* Check TX empty flag */
    if (((u32Mask & USPI_TX_EMPTY_MASK) == USPI_TX_EMPTY_MASK) &&
            ((u32BufSts & USPI_BUFSTS_TXEMPTY_Msk) != 0UL))
    {
        u32Flag |= USPI_TX_EMPTY_MASK;
    }

    /* Check TX full flag */
    if (((u32Mask & USPI_TX_FULL_MASK) == USPI_TX_FULL_MASK) &&
            ((u32BufSts & USPI_BUFSTS_TXFULL_Msk) != 0UL))
    {
        u32Flag |= USPI_TX_FULL_MASK;
    }

    /* Check USCI_SPI_SS line status */
    if (((u32Mask & USPI_SSLINE_STS_MASK) == USPI_SSLINE_STS_MASK) &&
            ((u32ProtSts & USPI_PROTSTS_SSLINE_Msk) != 0UL))
    {
        u32Flag |= USPI_SSLINE_STS_MASK;
    }

    return u32Flag;
}

/**
 * @brief      Enable USCI_SPI Wake-up Function.
 *
 * @param[in]  uspi The pointer of the specified USCI_SPI module.
 *
 * @return     None
 */
void USPI_EnableWakeup(USPI_T *uspi)
{
    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    uspi->WKCTL |= USPI_WKCTL_WKEN_Msk;
}

/**
 * @brief      Disable USCI_SPI Wake-up Function.
 *
 * @param[in]  uspi The pointer of the specified USCI_SPI module.
 *
 * @return     None
 */
void USPI_DisableWakeup(USPI_T *uspi)
{
    if (USPI_IsValidModule(uspi) == 0UL)
    {
        return;
    }

    uspi->WKCTL &= ~(USPI_WKCTL_WKEN_Msk);
}

/** @} end of group USCI_SPI_EXPORTED_FUNCTIONS */
/** @} end of group USCI_SPI_Driver */
/** @} end of group Standard_Driver */

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
