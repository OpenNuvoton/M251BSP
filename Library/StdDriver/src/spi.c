/***************************************************************************//**
 * @file    spi.c
 * @version V1.00
 * @brief   SPI driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPI_Driver SPI Driver
  @{
*/

/** @addtogroup SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/
#define SPI_CONFIG_TIMEOUT  (0x100000UL)

static uint32_t SPII2S_GetSourceClockFreq(const SPI_T *i2s);

/**
 * @brief      Check SPI module is valid or not
 *
 * @param[in]  spi The pointer of the specified SPI module.
 *
 * @return     1 = Valid SPI module; 0 = Invalid SPI module.
 */
static uint32_t SPI_IsValidModule(const SPI_T *spi)
{
    return (((spi == SPI0) || (spi == SPI1)) ? 1UL : 0UL);
}

/**
 * @brief      Check SPI master/slave mode is valid or not
 *
 * @param[in]  u32MasterSlave The master/slave mode.
 *
 * @return     1 = Valid master/slave mode; 0 = Invalid master/slave mode.
 */
static uint32_t SPI_IsValidMasterSlave(uint32_t u32MasterSlave)
{
    return (((u32MasterSlave == SPI_MASTER) || (u32MasterSlave == SPI_SLAVE)) ? 1UL : 0UL);
}

/**
 * @brief      Check SPI mode is valid or not
 *
 * @param[in]  u32SPIMode The SPI mode.
 *
 * @return     1 = Valid SPI mode; 0 = Invalid SPI mode.
 */
static uint32_t SPI_IsValidMode(uint32_t u32SPIMode)
{
    return (((u32SPIMode == SPI_MODE_0) || (u32SPIMode == SPI_MODE_1) ||
             (u32SPIMode == SPI_MODE_2) || (u32SPIMode == SPI_MODE_3)) ? 1UL : 0UL);
}

/**
 * @brief      Wait SPI module to be disabled.
 *
 * @param[in]  spi The pointer of the specified SPI module.
 *
 * @return     SPI_OK = SPI module is disabled; SPI_ERR_TIMEOUT = SPI module disable timeout.
 */
static int32_t SPI_WaitDisableReady(const SPI_T *spi)
{
    uint32_t u32Timeout = SPI_CONFIG_TIMEOUT;

    while (((spi->STATUS & SPI_STATUS_SPIENSTS_Msk) != 0UL) && (u32Timeout > 0UL))
    {
        u32Timeout--;
    }

    return (u32Timeout == 0UL) ? SPI_ERR_TIMEOUT : SPI_OK;
}

/**
 * @brief       Enter SPI configuration mode.
 *
 * @param[in]   spi            The pointer of the specified SPI module.
 *
 * @param[out]  pu32WasEnabled The pointer to store the SPI module was enabled or not.
 *
 * @return      SPI_OK = SPI module is ready to configure; SPI_ERR_FAIL = Invalid parameter; SPI_ERR_TIMEOUT = SPI module disable timeout.
 */
static int32_t SPI_EnterConfigMode(SPI_T *spi, uint32_t *pu32WasEnabled)
{
    if ((SPI_IsValidModule(spi) == 0UL) || (pu32WasEnabled == (uint32_t *)NULL))
    {
        return SPI_ERR_FAIL;
    }

    *pu32WasEnabled = spi->CTL & SPI_CTL_SPIEN_Msk;

    if (*pu32WasEnabled != 0UL)
    {
        spi->CTL &= ~SPI_CTL_SPIEN_Msk;
    }

    return SPI_WaitDisableReady(spi);
}

/**
 * @brief      Exit SPI configuration mode.
 *
 * @param[in]  spi           The pointer of the specified SPI module.
 * @param[in]  u32WasEnabled The SPI module was enabled or not.
 *
 * @return     None
 */
static void SPI_ExitConfigMode(SPI_T *spi, uint32_t u32WasEnabled)
{
    if (u32WasEnabled != 0UL)
    {
        spi->CTL |= SPI_CTL_SPIEN_Msk;
    }
}

/**
 * @brief      Select PCLK as the clock source of SPI
 *
 * @param[in]  spi The pointer of the specified SPI module.
 *
 * @return     None
 */
static void SPI_SetPCLKSrc(const SPI_T *spi)
{
    if (SPI_IsValidModule(spi) == 0UL)
    {
        return;
    }

    /* Select PCLK as the clock source of SPI */
    if (spi == SPI0)
    {
        /* Set the peripheral clock rate to equal APB clock rate */
        CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK1;
    }
    else
    {
        /* Set the peripheral clock rate to equal APB clock rate */
        CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;
    }
}

/**
 * @brief      Get PCLK frequency of the specified SPI module.
 *
 * @param[in]  spi The pointer of the specified SPI module.
 *
 * @return     PCLK frequency of the specified SPI module.
 */
static uint32_t SPI_GetPCLKFreq(const SPI_T *spi)
{
    uint32_t u32RetValue = 0UL;

    if (SPI_IsValidModule(spi) == 0UL)
    {
        return 0UL;
    }

    /* Select PCLK as the clock source of SPI */
    if (spi == SPI0)
    {
        /* Return slave peripheral clock rate */
        u32RetValue = CLK_GetPCLK1Freq();
    }
    else
    {
        /* Return slave peripheral clock rate */
        u32RetValue = CLK_GetPCLK0Freq();
    }

    return u32RetValue;
}

/**
 * @brief   Check SPI Clock Source Frequency.
 *
 * @param   spi The pointer of the specified SPI module.
 *
 * @return  uint32_t Clock Frequency
 */
static uint32_t SPI_GetModuleClkSrcFreq(const SPI_T *spi)
{
    uint32_t u32RetValue = 0UL;

    if (SPI_IsValidModule(spi) == 0UL)
    {
        return 0UL;
    }

    /* Get SPI clock source selection */
    if (spi == SPI0)
    {
        if ((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_HXT)
        {
            u32RetValue = __HXT; /* Clock source is HXT */
        }
        else if ((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PLL)
        {
            u32RetValue = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        }
        else if ((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PCLK1)
        {
            /* Clock source is PCLK1 */
            u32RetValue = SPI_GetPCLKFreq(spi);
        }
        else
        {
            u32RetValue = __HIRC; /* Clock source is HIRC */
        }
    }
    else if (spi == SPI1)
    {
        if ((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_HXT)
        {
            u32RetValue = __HXT; /* Clock source is HXT */
        }
        else if ((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PLL)
        {
            u32RetValue = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        }
        else if ((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PCLK0)
        {
            /* Clock source is PCLK0 */
            u32RetValue = SPI_GetPCLKFreq(spi);
        }
        else
        {
            u32RetValue = __HIRC; /* Clock source is HIRC */
        }
    }
    else
    {

    }

    return u32RetValue;
}

/**
 * @brief      This function make SPI module be ready to transfer.
 *
 * @param[in]  spi            The pointer of the specified SPI module.
 * @param[in]  u32MasterSlave Decides the SPI module is operating in master mode or in slave mode. (SPI_SLAVE, SPI_MASTER)
 * @param[in]  u32SPIMode     Decides the transfer timing. (SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3)
 * @param[in]  u32DataWidth   Decides the data width of a SPI transaction.
 * @param[in]  u32BusClock    The expected frequency of SPI bus clock in Hz.
 *
 * @return     Actual frequency of SPI peripheral clock.
 *
 * @details    By default, the SPI transfer sequence is MSB first, the slave selection signal is active low and the automatic
 *             slave selection function is disabled.
 *             In Slave mode, the u32BusClock shall be 0 and the SPI clock divider setting will be 0.
 *             The actual clock rate may be different from the target SPI clock rate.
 *             For example, if the SPI source clock rate is 12 MHz and the target SPI bus clock rate is 7 MHz, the
 *             actual SPI clock rate will be 6MHz.
 * @note   If u32DataWidth is 1 ~ 7, the data width will be configured as 8 bits.
 * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
 * @note   If u32BusClock >= system clock frequency, SPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
 * @note   If u32BusClock >= SPI peripheral clock source, DIVIDER will be set to 0.
 * @note   In slave mode, the SPI peripheral clock rate will be equal to APB clock rate.
 */
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock)
{
    uint32_t u32RetValue = 0UL;

    if ((SPI_IsValidModule(spi) == 0UL) ||
            (SPI_IsValidMasterSlave(u32MasterSlave) == 0UL) ||
            (SPI_IsValidMode(u32SPIMode) == 0UL))
    {
        return 0UL;
    }

    /* Disable I2S mode */
    spi->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;

    /* Default setting: slave selection signal is active low.
       In Master mode, disable the automatic slave selection function. */
    spi->SSCTL = SPI_SS_ACTIVE_LOW;

    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    spi->CTL = (u32SPIMode) | (u32MasterSlave);
    SPI_SET_DATA_WIDTH(spi, u32DataWidth);

    if (u32MasterSlave == SPI_MASTER)
    {
        /* Set the bus clock for the SPI module and store the actual frequency in u32RetValue. */
        u32RetValue = SPI_SetBusClock(spi, u32BusClock);
    }
    else /* For slave mode, force the SPI peripheral clock rate to equal APB clock rate. */
    {
        /* Force slave peripheral clock source back to PCLK. */
        SPI_SetPCLKSrc(spi);

        /* Set DIVIDER = 0 */
        spi->CLKDIV = 0UL;

        /* Select PCLK as the clock source of SPI */
        u32RetValue = SPI_GetPCLKFreq(spi);
    }

    spi->CTL |= SPI_CTL_SPIEN_Msk;

    return u32RetValue;
}

/**
 * @brief      Disable SPI controller.
 *
 * @param[in]  spi The pointer of the specified SPI module.
 *
 * @return     None
 *
 * @details    This function will reset SPI controller.
 */
void SPI_Close(const SPI_T *spi)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (SPI_IsValidModule(spi) == 0UL)
    {
        return;
    }

    /* Unlock protected registers */
    if (u32RegLockLevel != 0UL)
    {
        SYS_UnlockReg();
    }

    /* Reset SPI */
    if (spi == SPI0)
    {
        /* Reset SPI */
        SYS->IPRST1 |= SYS_IPRST1_SPI0RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_SPI0RST_Msk;
    }
    else
    {
        /* Reset SPI */
        SYS->IPRST1 |= SYS_IPRST1_SPI1RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_SPI1RST_Msk;
    }

    /* Lock protected registers */
    if (u32RegLockLevel != 0UL)
    {
        SYS_LockReg();
    }
}

/**
 * @brief      Clear RX FIFO buffer.
 *
 * @param[in]  spi The pointer of the specified SPI module.
 *
 * @return     None
 *
 * @details    This function will clear SPI RX FIFO buffer. The RXEMPTY (SPI_STATUS[8]) will be set to 1.
 */
void SPI_ClearRxFIFO(SPI_T *spi)
{
    if (SPI_IsValidModule(spi) == 0UL)
    {
        return;
    }

    spi->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
}

/**
 * @brief      Clear TX FIFO buffer.
 *
 * @param[in]  spi The pointer of the specified SPI module.
 *
 * @return     None
 *
 * @details    This function will clear SPI TX FIFO buffer. The TXEMPTY (SPI_STATUS[16]) will be set to 1.
 * @note The TX shift register will not be cleared.
 */
void SPI_ClearTxFIFO(SPI_T *spi)
{
    if (SPI_IsValidModule(spi) == 0UL)
    {
        return;
    }

    spi->FIFOCTL |= SPI_FIFOCTL_TXFBCLR_Msk;
}

/**
 * @brief      Disable the automatic slave selection function.
 *
 * @param[in]  spi The pointer of the specified SPI module.
 *
 * @return     None
 *
 * @details    This function will disable the automatic slave selection function and set slave selection signal to inactive state.
 */
void SPI_DisableAutoSS(SPI_T *spi)
{
    uint32_t u32WasEnabled = 0UL;

    if (SPI_EnterConfigMode(spi, &u32WasEnabled) != SPI_OK)
    {
        return;
    }

    spi->SSCTL &= ~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk);

    SPI_ExitConfigMode(spi, u32WasEnabled);
}

/**
 * @brief      Enable the automatic slave selection function.
 *
 * @param[in]  spi            The pointer of the specified SPI module.
 * @param[in]  u32SSPinMask   Specifies slave selection pins. (SPI_SS)
 * @param[in]  u32ActiveLevel Specifies the active level of slave selection signal. (SPI_SS_ACTIVE_HIGH, SPI_SS_ACTIVE_LOW)
 *
 * @return     None
 *
 * @details    This function will enable the automatic slave selection function. Only available in Master mode.
 *             The slave selection pin and the active level will be set in this function.
 */
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    uint32_t u32WasEnabled = 0UL;
    uint32_t u32PinMask = (u32SSPinMask & SPI_SSCTL_SS_Msk);
    uint32_t u32ActLevel = (u32ActiveLevel & SPI_SSCTL_SSACTPOL_Msk);

    if (u32PinMask == 0UL)
    {
        return;
    }

    if (SPI_EnterConfigMode(spi, &u32WasEnabled) != SPI_OK)
    {
        return;
    }

    spi->SSCTL = (spi->SSCTL & (~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS_Msk))) |
                 (u32PinMask | u32ActLevel | SPI_SSCTL_AUTOSS_Msk);

    SPI_ExitConfigMode(spi, u32WasEnabled);
}

/**
 * @brief      Set the SPI bus clock.
 *
 * @param[in]  spi         The pointer of the specified SPI module.
 * @param[in]  u32BusClock The expected frequency of SPI bus clock in Hz.
 *
 * @return     Actual frequency of SPI bus clock.
 *
 * @details    This function is only available in Master mode. The actual clock rate may be different from the target SPI bus clock rate.
 *             For example, if the SPI source clock rate is 12 MHz and the target SPI bus clock rate is 7 MHz, the actual SPI bus clock
 *             rate will be 6 MHz.
 * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
 * @note   If u32BusClock >= system clock frequency, SPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
 * @note   If u32BusClock >= SPI peripheral clock source, DIVIDER will be set to 0.
 */
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock)
{
    uint32_t u32ClkSrc = 0UL;
    uint32_t u32HCLKFreq = 0UL;
    uint32_t u32RetValue = 0UL;
    uint32_t u32WasEnabled = 0UL;

    if (SPI_IsValidModule(spi) == 0UL)
    {
        return 0UL;
    }

    if (SPI_EnterConfigMode(spi, &u32WasEnabled) != SPI_OK)
    {
        return SPI_GetBusClock(spi);
    }

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    if (u32BusClock >= u32HCLKFreq)
    {
        /* Select PCLK as the clock source of SPI */
        SPI_SetPCLKSrc(spi);
    }

    /* Check clock source of SPI */
    u32ClkSrc = SPI_GetModuleClkSrcFreq(spi);

    if (u32BusClock >= u32HCLKFreq)
    {
        /* Set DIVIDER = 0 */
        spi->CLKDIV = 0UL;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    }
    else if (u32BusClock >= u32ClkSrc)
    {
        /* Set DIVIDER = 0 */
        spi->CLKDIV = 0UL;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    }
    else if (u32BusClock == 0U)
    {
        /* Set DIVIDER to the maximum value. f_spi = f_spi_clk_src / (DIVIDER + 1) */
        spi->CLKDIV = (spi->CLKDIV & ~SPI_CLKDIV_DIVIDER_Msk) | SPI_CLKDIV_DIVIDER_Msk;
        /* Return master peripheral clock rate */
        u32RetValue = (u32ClkSrc / ((SPI_CLKDIV_DIVIDER_Msk >> SPI_CLKDIV_DIVIDER_Pos) + 1U));
    }
    else
    {
        uint32_t u32Div = 0UL;
        u32Div = ((((u32ClkSrc * 10U) / u32BusClock) + 5U) / 10U) - 1U; /* Round to the nearest integer */

        // Ensure the calculated divider does not exceed the maximum allowed value
        u32Div = ((u32Div > (SPI_CLKDIV_DIVIDER_Msk >> SPI_CLKDIV_DIVIDER_Pos)) ?
                  (SPI_CLKDIV_DIVIDER_Msk >> SPI_CLKDIV_DIVIDER_Pos) : u32Div);

        spi->CLKDIV = (spi->CLKDIV & ~(SPI_CLKDIV_DIVIDER_Msk)) | (u32Div << SPI_CLKDIV_DIVIDER_Pos);

        /* Return master peripheral clock rate */
        u32RetValue = (u32ClkSrc / (u32Div + 1U));
    }

    SPI_ExitConfigMode(spi, u32WasEnabled);

    return u32RetValue;
}

/**
 * @brief      Set SPI data width.
 *
 * @param[in]  spi      Pointer to SPI module.
 * @param[in]  u32Width Requested data width.
 *
 * @return     None
 *
 * @details
 *             The SPI data-width register encoding is:
 *             - Register value 0 represents 32-bit width.
 *             - Values from 8 to 31 represent the corresponding data width.
 *             - Requested widths from 1 to 8 are clamped to 8 bits.
 *             - Requested widths greater than or equal to 32 are configured as 32 bits.
 */
void SPI_SetDataWidth(SPI_T *spi, uint32_t u32Width)
{
    uint32_t u32RegWidth;
    uint32_t u32RegValue;

    if (u32Width == 0UL)
    {
        /*
         * Register value 0 represents 32-bit data width.
         */
        u32RegWidth = 0UL;
    }
    else if (u32Width < 8UL)
    {
        /*
         * Clamp unsupported widths from 1 to 8
         * to the minimum supported width of 8 bits.
         */
        u32RegWidth = 8UL;
    }
    else if (u32Width < 32UL)
    {
        /*
         * Register directly encodes data widths
         * from 8 to 31 bits.
         */
        u32RegWidth = u32Width;
    }
    else
    {
        /*
         * Widths greater than or equal to 32
         * are configured as 32-bit width.
         */
        u32RegWidth = 0UL;
    }

    u32RegValue = spi->CTL;
    u32RegValue &= ~SPI_CTL_DWIDTH_Msk;
    u32RegValue |= ((u32RegWidth << SPI_CTL_DWIDTH_Pos) & SPI_CTL_DWIDTH_Msk);

    spi->CTL = u32RegValue;
}

/**
 * @brief      Configure FIFO threshold setting.
 *
 * @param[in]  spi            The pointer of the specified SPI module.
 * @param[in]  u32TxThreshold Decides the TX FIFO threshold. It could be 0 ~ 3. If data width is 8~16 bits, it could be 0 ~ 7.
 * @param[in]  u32RxThreshold Decides the RX FIFO threshold. It could be 0 ~ 3. If data width is 8~16 bits, it could be 0 ~ 7.
 *
 * @return     None
 *
 * @details    Set TX FIFO threshold and RX FIFO threshold configurations.
 */
void SPI_SetFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    uint32_t u32WasEnabled = 0UL;

    if (SPI_EnterConfigMode(spi, &u32WasEnabled) != SPI_OK)
    {
        return;
    }

    spi->FIFOCTL = (spi->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk)) |
                   ((u32TxThreshold << SPI_FIFOCTL_TXTH_Pos) & SPI_FIFOCTL_TXTH_Msk) |
                   ((u32RxThreshold << SPI_FIFOCTL_RXTH_Pos) & SPI_FIFOCTL_RXTH_Msk);

    SPI_ExitConfigMode(spi, u32WasEnabled);
}

/**
 * @brief      Get the actual frequency of SPI bus clock. Only available in Master mode.
 *
 * @param[in]  spi The pointer of the specified SPI module.
 *
 * @return     Actual SPI bus clock frequency in Hz.
 *
 * @details    This function will calculate the actual SPI bus clock rate according to the SPInSEL and DIVIDER settings. Only available in Master mode.
 */
uint32_t SPI_GetBusClock(const SPI_T *spi)
{
    uint32_t u32Div = 0UL;
    uint32_t u32ClkSrc = 0UL;

    if (SPI_IsValidModule(spi) == 0UL)
    {
        return 0UL;
    }

    /* Get DIVIDER setting */
    u32Div = ((spi->CLKDIV & SPI_CLKDIV_DIVIDER_Msk) >> SPI_CLKDIV_DIVIDER_Pos);

    /* Check clock source of SPI */
    u32ClkSrc = SPI_GetModuleClkSrcFreq(spi);

    /* Return SPI bus clock rate */
    return (u32ClkSrc / (u32Div + 1U));
}

/**
 * @brief      Enable interrupt function.
 *
 * @param[in]  spi     The pointer of the specified SPI module.
 * @param[in]  u32Mask The combination of all related interrupt enable bits.
 *                     Each bit corresponds to a interrupt enable bit.
 *                     This parameter decides which interrupts will be enabled. It is combination of:
 *                     - \ref SPI_UNIT_INT_MASK
 *                     - \ref SPI_SSACT_INT_MASK
 *                     - \ref SPI_SSINACT_INT_MASK
 *                     - \ref SPI_SLVUR_INT_MASK
 *                     - \ref SPI_SLVBE_INT_MASK
 *                     - \ref SPI_TXUF_INT_MASK
 *                     - \ref SPI_FIFO_TXTH_INT_MASK
 *                     - \ref SPI_FIFO_RXTH_INT_MASK
 *                     - \ref SPI_FIFO_RXOV_INT_MASK
 *                     - \ref SPI_FIFO_RXTO_INT_MASK
 *
 * @return     None
 *
 * @details    Enable SPI related interrupts specified by u32Mask parameter.
 */
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask)
{
    if (SPI_IsValidModule(spi) == 0UL)
    {
        return;
    }

    /* Enable unit transfer interrupt flag */
    if ((u32Mask & SPI_UNIT_INT_MASK) == SPI_UNIT_INT_MASK)
    {
        spi->CTL |= SPI_CTL_UNITIEN_Msk;
    }

    /* Enable slave selection signal active interrupt flag */
    if ((u32Mask & SPI_SSACT_INT_MASK) == SPI_SSACT_INT_MASK)
    {
        spi->SSCTL |= SPI_SSCTL_SSACTIEN_Msk;
    }

    /* Enable slave selection signal inactive interrupt flag */
    if ((u32Mask & SPI_SSINACT_INT_MASK) == SPI_SSINACT_INT_MASK)
    {
        spi->SSCTL |= SPI_SSCTL_SSINAIEN_Msk;
    }

    /* Enable slave TX under run interrupt flag */
    if ((u32Mask & SPI_SLVUR_INT_MASK) == SPI_SLVUR_INT_MASK)
    {
        spi->SSCTL |= SPI_SSCTL_SLVURIEN_Msk;
    }

    /* Enable slave bit count error interrupt flag */
    if ((u32Mask & SPI_SLVBE_INT_MASK) == SPI_SLVBE_INT_MASK)
    {
        spi->SSCTL |= SPI_SSCTL_SLVBEIEN_Msk;
    }

    /* Enable slave TX underflow interrupt flag */
    if ((u32Mask & SPI_TXUF_INT_MASK) == SPI_TXUF_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Enable TX threshold interrupt flag */
    if ((u32Mask & SPI_FIFO_TXTH_INT_MASK) == SPI_FIFO_TXTH_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Enable RX threshold interrupt flag */
    if ((u32Mask & SPI_FIFO_RXTH_INT_MASK) == SPI_FIFO_RXTH_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Enable RX overrun interrupt flag */
    if ((u32Mask & SPI_FIFO_RXOV_INT_MASK) == SPI_FIFO_RXOV_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Enable RX time-out interrupt flag */
    if ((u32Mask & SPI_FIFO_RXTO_INT_MASK) == SPI_FIFO_RXTO_INT_MASK)
    {
        spi->FIFOCTL |= SPI_FIFOCTL_RXTOIEN_Msk;
    }
}

/**
 * @brief      Disable interrupt function.
 *
 * @param[in]  spi     The pointer of the specified SPI module.
 * @param[in]  u32Mask The combination of all related interrupt enable bits.
 *                     Each bit corresponds to a interrupt bit.
 *                     This parameter decides which interrupts will be disabled. It is combination of:
 *                     - \ref SPI_UNIT_INT_MASK
 *                     - \ref SPI_SSACT_INT_MASK
 *                     - \ref SPI_SSINACT_INT_MASK
 *                     - \ref SPI_SLVUR_INT_MASK
 *                     - \ref SPI_SLVBE_INT_MASK
 *                     - \ref SPI_TXUF_INT_MASK
 *                     - \ref SPI_FIFO_TXTH_INT_MASK
 *                     - \ref SPI_FIFO_RXTH_INT_MASK
 *                     - \ref SPI_FIFO_RXOV_INT_MASK
 *                     - \ref SPI_FIFO_RXTO_INT_MASK
 *
 * @return     None
 *
 * @details    Disable SPI related interrupts specified by u32Mask parameter.
 */
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask)
{
    if (SPI_IsValidModule(spi) == 0UL)
    {
        return;
    }

    /* Disable unit transfer interrupt flag */
    if ((u32Mask & SPI_UNIT_INT_MASK) == SPI_UNIT_INT_MASK)
    {
        spi->CTL &= ~SPI_CTL_UNITIEN_Msk;
    }

    /* Disable slave selection signal active interrupt flag */
    if ((u32Mask & SPI_SSACT_INT_MASK) == SPI_SSACT_INT_MASK)
    {
        spi->SSCTL &= ~SPI_SSCTL_SSACTIEN_Msk;
    }

    /* Disable slave selection signal inactive interrupt flag */
    if ((u32Mask & SPI_SSINACT_INT_MASK) == SPI_SSINACT_INT_MASK)
    {
        spi->SSCTL &= ~SPI_SSCTL_SSINAIEN_Msk;
    }

    /* Disable slave TX under run interrupt flag */
    if ((u32Mask & SPI_SLVUR_INT_MASK) == SPI_SLVUR_INT_MASK)
    {
        spi->SSCTL &= ~SPI_SSCTL_SLVURIEN_Msk;
    }

    /* Disable slave bit count error interrupt flag */
    if ((u32Mask & SPI_SLVBE_INT_MASK) == SPI_SLVBE_INT_MASK)
    {
        spi->SSCTL &= ~SPI_SSCTL_SLVBEIEN_Msk;
    }

    /* Disable slave TX underflow interrupt flag */
    if ((u32Mask & SPI_TXUF_INT_MASK) == SPI_TXUF_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Disable TX threshold interrupt flag */
    if ((u32Mask & SPI_FIFO_TXTH_INT_MASK) == SPI_FIFO_TXTH_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Disable RX threshold interrupt flag */
    if ((u32Mask & SPI_FIFO_RXTH_INT_MASK) == SPI_FIFO_RXTH_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Disable RX overrun interrupt flag */
    if ((u32Mask & SPI_FIFO_RXOV_INT_MASK) == SPI_FIFO_RXOV_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Disable RX time-out interrupt flag */
    if ((u32Mask & SPI_FIFO_RXTO_INT_MASK) == SPI_FIFO_RXTO_INT_MASK)
    {
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTOIEN_Msk;
    }
}

/**
 * @brief      Get interrupt flag.
 *
 * @param[in]  spi     The pointer of the specified SPI module.
 * @param[in]  u32Mask The combination of all related interrupt sources.
 *                     Each bit corresponds to a interrupt source.
 *                     This parameter decides which interrupt flags will be read. It is combination of:
 *                     - \ref SPI_UNIT_INT_MASK
 *                     - \ref SPI_SSACT_INT_MASK
 *                     - \ref SPI_SSINACT_INT_MASK
 *                     - \ref SPI_SLVUR_INT_MASK
 *                     - \ref SPI_SLVBE_INT_MASK
 *                     - \ref SPI_TXUF_INT_MASK
 *                     - \ref SPI_FIFO_TXTH_INT_MASK
 *                     - \ref SPI_FIFO_RXTH_INT_MASK
 *                     - \ref SPI_FIFO_RXOV_INT_MASK
 *                     - \ref SPI_FIFO_RXTO_INT_MASK
 *
 * @return     Interrupt flags of selected sources.
 *
 * @details    Get SPI related interrupt flags specified by u32Mask parameter.
 */
uint32_t SPI_GetIntFlag(const SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0UL;
    uint32_t u32Status = 0UL;

    if (SPI_IsValidModule(spi) == 0UL)
    {
        return 0UL;
    }

    u32Status = spi->STATUS;

    /* Check unit transfer interrupt flag */
    if (((u32Mask & SPI_UNIT_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_UNITIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_UNIT_INT_MASK;
    }

    /* Check slave selection signal active interrupt flag */
    if (((u32Mask & SPI_SSACT_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_SSACTIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_SSACT_INT_MASK;
    }

    /* Check slave selection signal inactive interrupt flag */
    if (((u32Mask & SPI_SSINACT_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_SSINAIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_SSINACT_INT_MASK;
    }

    /* Check slave TX under run interrupt flag */
    if (((u32Mask & SPI_SLVUR_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_SLVURIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_SLVUR_INT_MASK;
    }

    /* Check slave bit count error interrupt flag */
    if (((u32Mask & SPI_SLVBE_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_SLVBEIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_SLVBE_INT_MASK;
    }

    /* Check slave TX underflow interrupt flag */
    if (((u32Mask & SPI_TXUF_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_TXUFIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_TXUF_INT_MASK;
    }

    /* Check TX threshold interrupt flag */
    if (((u32Mask & SPI_FIFO_TXTH_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_TXTHIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_FIFO_TXTH_INT_MASK;
    }

    /* Check RX threshold interrupt flag */
    if (((u32Mask & SPI_FIFO_RXTH_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_RXTHIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_FIFO_RXTH_INT_MASK;
    }

    /* Check RX overrun interrupt flag */
    if (((u32Mask & SPI_FIFO_RXOV_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_RXOVIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_FIFO_RXOV_INT_MASK;
    }

    /* Check RX time-out interrupt flag */
    if (((u32Mask & SPI_FIFO_RXTO_INT_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_RXTOIF_Msk) != 0UL))
    {
        u32IntFlag |= SPI_FIFO_RXTO_INT_MASK;
    }

    return u32IntFlag;
}

/**
 * @brief      Clear interrupt flag.
 *
 * @param[in]  spi     The pointer of the specified SPI module.
 * @param[in]  u32Mask The combination of all related interrupt sources.
 *                     Each bit corresponds to a interrupt source.
 *                     This parameter decides which interrupt flags will be cleared. It could be the combination of:
 *                     - \ref SPI_UNIT_INT_MASK
 *                     - \ref SPI_SSACT_INT_MASK
 *                     - \ref SPI_SSINACT_INT_MASK
 *                     - \ref SPI_SLVUR_INT_MASK
 *                     - \ref SPI_SLVBE_INT_MASK
 *                     - \ref SPI_TXUF_INT_MASK
 *                     - \ref SPI_FIFO_RXOV_INT_MASK
 *                     - \ref SPI_FIFO_RXTO_INT_MASK
 *
 * @return     None
 *
 * @details    Clear SPI related interrupt flags specified by u32Mask parameter.
 */
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32ClearMask = 0UL;

    if (SPI_IsValidModule(spi) == 0UL)
    {
        return;
    }

    if ((u32Mask & SPI_UNIT_INT_MASK) != 0UL)
    {
        u32ClearMask |= SPI_STATUS_UNITIF_Msk; /* Clear unit transfer interrupt flag */
    }

    if ((u32Mask & SPI_SSACT_INT_MASK) != 0UL)
    {
        u32ClearMask |= SPI_STATUS_SSACTIF_Msk; /* Clear slave selection signal active interrupt flag */
    }

    if ((u32Mask & SPI_SSINACT_INT_MASK) != 0UL)
    {
        u32ClearMask |= SPI_STATUS_SSINAIF_Msk; /* Clear slave selection signal inactive interrupt flag */
    }

    if ((u32Mask & SPI_SLVUR_INT_MASK) != 0UL)
    {
        u32ClearMask |= SPI_STATUS_SLVURIF_Msk; /* Clear slave TX under run interrupt flag */
    }

    if ((u32Mask & SPI_SLVBE_INT_MASK) != 0UL)
    {
        u32ClearMask |= SPI_STATUS_SLVBEIF_Msk; /* Clear slave bit count error interrupt flag */
    }

    if ((u32Mask & SPI_TXUF_INT_MASK) != 0UL)
    {
        u32ClearMask |= SPI_STATUS_TXUFIF_Msk; /* Clear slave TX underflow interrupt flag */
    }

    if ((u32Mask & SPI_FIFO_RXOV_INT_MASK) != 0UL)
    {
        u32ClearMask |= SPI_STATUS_RXOVIF_Msk; /* Clear RX overrun interrupt flag */
    }

    if ((u32Mask & SPI_FIFO_RXTO_INT_MASK) != 0UL)
    {
        u32ClearMask |= SPI_STATUS_RXTOIF_Msk; /* Clear RX time-out interrupt flag */
    }

    if (u32ClearMask != 0UL)
    {
        spi->STATUS = u32ClearMask;
    }
}

/**
 * @brief      Get SPI status.
 *
 * @param[in]  spi     The pointer of the specified SPI module.
 * @param[in]  u32Mask The combination of all related sources.
 *                     Each bit corresponds to a source.
 *                     This parameter decides which flags will be read. It is combination of:
 *                     - \ref SPI_BUSY_MASK
 *                     - \ref SPI_RX_EMPTY_MASK
 *                     - \ref SPI_RX_FULL_MASK
 *                     - \ref SPI_TX_EMPTY_MASK
 *                     - \ref SPI_TX_FULL_MASK
 *                     - \ref SPI_TXRX_RESET_MASK
 *                     - \ref SPI_SPIEN_STS_MASK
 *                     - \ref SPI_SSLINE_STS_MASK
 *
 * @return     Flags of selected sources.
 *
 * @details    Get SPI related status specified by u32Mask parameter.
 */
uint32_t SPI_GetStatus(const SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32Flag = 0UL;
    uint32_t u32Status = 0UL;

    if (SPI_IsValidModule(spi) == 0UL)
    {
        return 0UL;
    }

    u32Status = spi->STATUS;

    /* Check busy status */
    if (((u32Mask & SPI_BUSY_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_BUSY_Msk) != 0UL))
    {
        u32Flag |= SPI_BUSY_MASK;
    }

    /* Check RX empty flag */
    if (((u32Mask & SPI_RX_EMPTY_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_RXEMPTY_Msk) != 0UL))
    {
        u32Flag |= SPI_RX_EMPTY_MASK;
    }

    /* Check RX full flag */
    if (((u32Mask & SPI_RX_FULL_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_RXFULL_Msk) != 0UL))
    {
        u32Flag |= SPI_RX_FULL_MASK;
    }

    /* Check TX empty flag */
    if (((u32Mask & SPI_TX_EMPTY_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_TXEMPTY_Msk) != 0UL))
    {
        u32Flag |= SPI_TX_EMPTY_MASK;
    }

    /* Check TX full flag */
    if (((u32Mask & SPI_TX_FULL_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_TXFULL_Msk) != 0UL))
    {
        u32Flag |= SPI_TX_FULL_MASK;
    }

    /* Check TX/RX reset flag */
    if (((u32Mask & SPI_TXRX_RESET_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_TXRXRST_Msk) != 0UL))
    {
        u32Flag |= SPI_TXRX_RESET_MASK;
    }

    /* Check SPIEN flag */
    if (((u32Mask & SPI_SPIEN_STS_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_SPIENSTS_Msk) != 0UL))
    {
        u32Flag |= SPI_SPIEN_STS_MASK;
    }

    /* Check SPIx_SS line status */
    if (((u32Mask & SPI_SSLINE_STS_MASK) != 0UL) &&
            ((u32Status & SPI_STATUS_SSLINE_Msk) != 0UL))
    {
        u32Flag |= SPI_SSLINE_STS_MASK;
    }

    return u32Flag;
}

/**
 * @brief      Get SPI status2.
 *
 * @param[in]  spi     The pointer of the specified SPI module.
 * @param[in]  u32Mask The combination of all related sources.
 *                     Each bit corresponds to a source.
 *                     This parameter decides which flags will be read. It is combination of:
 *                     - \ref SPI_SLVBENUM_MASK
 *
 * @return     Flags of selected sources.
 *
 * @details    Get SPI related status specified by u32Mask parameter.
 */
uint32_t SPI_GetStatus2(const SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32TmpStatus = 0UL;

    if (SPI_IsValidModule(spi) == 0UL)
    {
        return 0UL;
    }

    u32TmpStatus = (spi->STATUS2 & SPI_STATUS2_SLVBENUM_Msk) >> SPI_STATUS2_SLVBENUM_Pos;

    /* Check effective bit number of uncompleted RX data status */
    if ((u32Mask & SPI_SLVBENUM_MASK) != 0UL)
    {
        return u32TmpStatus;
    }

    return 0UL;
}

/**
 * @brief      This function is used to get I2S source clock frequency.
 *
 * @param[in]  i2s The pointer of the specified I2S module.
 *
 * @return     I2S source clock frequency (Hz).
 *
 * @details    Return the source clock frequency according to the setting of SPI0SEL (CLKSEL2[27:26]).
 */
static uint32_t SPII2S_GetSourceClockFreq(const SPI_T *i2s)
{
    return SPI_GetModuleClkSrcFreq(i2s);
}

/**
 * @brief      This function configures some parameters of I2S interface for general purpose use.
 *
 * @param[in]  i2s            The pointer of the specified I2S module.
 * @param[in]  u32MasterSlave I2S operation mode. Valid values are listed below.
 *                            - \ref SPII2S_MODE_MASTER
 *                            - \ref SPII2S_MODE_SLAVE
 * @param[in]  u32SampleRate  Sample rate
 * @param[in]  u32WordWidth   Data length. Valid values are listed below.
 *                            - \ref SPII2S_DATABIT_8
 *                            - \ref SPII2S_DATABIT_16
 *                            - \ref SPII2S_DATABIT_24
 *                            - \ref SPII2S_DATABIT_32
 * @param[in]  u32Channels    Audio format. Valid values are listed below.
 *                            - \ref SPII2S_MONO
 *                            - \ref SPII2S_STEREO
 * @param[in]  u32DataFormat  Data format. Valid values are listed below.
 *                            - \ref SPII2S_FORMAT_I2S
 *                            - \ref SPII2S_FORMAT_MSB
 *                            - \ref SPII2S_FORMAT_PCMA
 *                            - \ref SPII2S_FORMAT_PCMB
 *
 * @return     Real sample rate of master mode or peripheral clock rate of slave mode.
 *
 * @details    This function will reset SPI/I2S controller and configure I2S controller according to the input parameters.
 *             Set TX FIFO threshold to 2 and RX FIFO threshold to 1. Both the TX and RX functions will be enabled.
 *             The actual sample rate may be different from the target sample rate. The real sample rate will be returned for reference.
 * @note   In slave mode, the SPI peripheral clock rate will be equal to APB clock rate.
 */
uint32_t SPII2S_Open(SPI_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat)
{
    uint32_t u32RetValue = 0UL;
    uint32_t u32BitRate = 0UL;

    if (SPI_IsValidModule(i2s) == 0UL)
    {
        return 0UL;
    }

    if (u32MasterSlave == SPI_MASTER)
    {
        u32BitRate = u32SampleRate * ((u32WordWidth >> SPI_I2SCTL_WDWIDTH_Pos) + 1U) * 16UL;

        if (u32BitRate == 0UL)
        {
            return 0UL;
        }
    }

    /* Reset SPI/I2S */
    SPI_Close(i2s);

    /* Configure I2S controller */
    i2s->I2SCTL = u32MasterSlave | u32WordWidth | u32Channels | u32DataFormat;
    /* Set TX FIFO threshold to 2 and RX FIFO threshold to 1 */
    SPII2S_SetFIFO(i2s, 2, 1);

    if (u32MasterSlave == SPI_MASTER)
    {
        uint32_t u32Divider = 0UL;
        uint32_t u32SrcClk = 0UL;

        /* Get the source clock rate */
        u32SrcClk = SPII2S_GetSourceClockFreq(i2s);

        u32Divider = ((((u32SrcClk * 10UL / u32BitRate) >> 1U) + 5UL) / 10UL) - 1UL;
        //u32Divider = ((u32SrcClk / u32BitRate) >> 1U) - 1UL;
        /* Set BCLKDIV setting */
        i2s->I2SCLK = (i2s->I2SCLK & ~SPI_I2SCLK_BCLKDIV_Msk) | (u32Divider << SPI_I2SCLK_BCLKDIV_Pos);
        /* Enable I2S mode for the frequency of peripheral clock. */
        i2s->I2SCLK |= SPI_I2SCLK_I2SMODE_Msk;

        /* Calculate bit clock rate */
        u32BitRate = u32SrcClk / ((u32Divider + 1U) * 2U);
        /* Calculate real sample rate and return the real sample rate */
        u32RetValue = u32BitRate / (((u32WordWidth >> SPI_I2SCTL_WDWIDTH_Pos) + 1U) * 16U);
    }
    else
    {
        /* Set BCLKDIV = 0 */
        i2s->I2SCLK &= ~SPI_I2SCLK_BCLKDIV_Msk;

        /* Enable I2S slave mode and I2S mode for the frequency of peripheral clock. */
        i2s->I2SCLK |= (SPI_I2SCLK_I2SSLAVE_Msk | SPI_I2SCLK_I2SMODE_Msk);

        SPI_SetPCLKSrc(i2s);

        u32RetValue = SPI_GetPCLKFreq(i2s);
    }

    /* Enable TX function, RX function and I2S mode. */
    i2s->I2SCTL |= (SPI_I2SCTL_RXEN_Msk | SPI_I2SCTL_TXEN_Msk | SPI_I2SCTL_I2SEN_Msk);

    return u32RetValue;
}

/**
 * @brief      Disable I2S function.
 *
 * @param[in]  i2s The pointer of the specified I2S module.
 *
 * @return     None
 *
 * @details    Disable I2S function.
 */
void SPII2S_Close(SPI_T *i2s)
{
    if (SPI_IsValidModule(i2s) == 0UL)
    {
        return;
    }

    i2s->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;
}

/**
 * @brief      Enable interrupt function.
 *
 * @param[in]  i2s     The pointer of the specified I2S module.
 * @param[in]  u32Mask The combination of all related interrupt enable bits.
 *                     Each bit corresponds to a interrupt source. Valid values are listed below.
 *                     - \ref SPII2S_FIFO_TXTH_INT_MASK
 *                     - \ref SPII2S_FIFO_RXTH_INT_MASK
 *                     - \ref SPII2S_FIFO_RXOV_INT_MASK
 *                     - \ref SPII2S_FIFO_RXTO_INT_MASK
 *                     - \ref SPII2S_TXUF_INT_MASK
 *                     - \ref SPII2S_RIGHT_ZC_INT_MASK
 *                     - \ref SPII2S_LEFT_ZC_INT_MASK
 *                     - \ref SPII2S_SLV_CLKERR_INT_MASK
 *
 * @return     None
 *
 * @details    This function enables the interrupt according to the u32Mask parameter.
 */
void SPII2S_EnableInt(SPI_T *i2s, uint32_t u32Mask)
{
    if (SPI_IsValidModule(i2s) == 0UL)
    {
        return;
    }

    /* Enable TX threshold interrupt flag */
    if ((u32Mask & SPII2S_FIFO_TXTH_INT_MASK) == SPII2S_FIFO_TXTH_INT_MASK)
    {
        i2s->FIFOCTL |= SPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Enable RX threshold interrupt flag */
    if ((u32Mask & SPII2S_FIFO_RXTH_INT_MASK) == SPII2S_FIFO_RXTH_INT_MASK)
    {
        i2s->FIFOCTL |= SPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Enable RX overrun interrupt flag */
    if ((u32Mask & SPII2S_FIFO_RXOV_INT_MASK) == SPII2S_FIFO_RXOV_INT_MASK)
    {
        i2s->FIFOCTL |= SPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Enable RX time-out interrupt flag */
    if ((u32Mask & SPII2S_FIFO_RXTO_INT_MASK) == SPII2S_FIFO_RXTO_INT_MASK)
    {
        i2s->FIFOCTL |= SPI_FIFOCTL_RXTOIEN_Msk;
    }

    /* Enable TX underflow interrupt flag */
    if ((u32Mask & SPII2S_TXUF_INT_MASK) == SPII2S_TXUF_INT_MASK)
    {
        i2s->FIFOCTL |= SPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Enable right channel zero cross interrupt flag */
    if ((u32Mask & SPII2S_RIGHT_ZC_INT_MASK) == SPII2S_RIGHT_ZC_INT_MASK)
    {
        i2s->I2SCTL |= SPI_I2SCTL_RZCIEN_Msk;
    }

    /* Enable left channel zero cross interrupt flag */
    if ((u32Mask & SPII2S_LEFT_ZC_INT_MASK) == SPII2S_LEFT_ZC_INT_MASK)
    {
        i2s->I2SCTL |= SPI_I2SCTL_LZCIEN_Msk;
    }

    /* Enable slave mode bit clock loss interrupt flag */
    if ((u32Mask & SPII2S_SLV_CLKERR_INT_MASK) == SPII2S_SLV_CLKERR_INT_MASK)
    {
        i2s->I2SCTL |= SPI_I2SCTL_SLVERRIEN_Msk;
    }
}

/**
 * @brief      Disable interrupt function.
 *
 * @param[in]  i2s     The pointer of the specified I2S module.
 * @param[in]  u32Mask The combination of all related interrupt enable bits.
 *                     Each bit corresponds to a interrupt source. Valid values are listed below.
 *                     - \ref SPII2S_FIFO_TXTH_INT_MASK
 *                     - \ref SPII2S_FIFO_RXTH_INT_MASK
 *                     - \ref SPII2S_FIFO_RXOV_INT_MASK
 *                     - \ref SPII2S_FIFO_RXTO_INT_MASK
 *                     - \ref SPII2S_TXUF_INT_MASK
 *                     - \ref SPII2S_RIGHT_ZC_INT_MASK
 *                     - \ref SPII2S_LEFT_ZC_INT_MASK
 *                     - \ref SPII2S_SLV_CLKERR_INT_MASK
 *
 * @return     None
 *
 * @details    This function disables the interrupt according to the u32Mask parameter.
 */
void SPII2S_DisableInt(SPI_T *i2s, uint32_t u32Mask)
{
    if (SPI_IsValidModule(i2s) == 0UL)
    {
        return;
    }

    /* Disable TX threshold interrupt flag */
    if ((u32Mask & SPII2S_FIFO_TXTH_INT_MASK) == SPII2S_FIFO_TXTH_INT_MASK)
    {
        i2s->FIFOCTL &= ~SPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Disable RX threshold interrupt flag */
    if ((u32Mask & SPII2S_FIFO_RXTH_INT_MASK) == SPII2S_FIFO_RXTH_INT_MASK)
    {
        i2s->FIFOCTL &= ~SPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Disable RX overrun interrupt flag */
    if ((u32Mask & SPII2S_FIFO_RXOV_INT_MASK) == SPII2S_FIFO_RXOV_INT_MASK)
    {
        i2s->FIFOCTL &= ~SPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Disable RX time-out interrupt flag */
    if ((u32Mask & SPII2S_FIFO_RXTO_INT_MASK) == SPII2S_FIFO_RXTO_INT_MASK)
    {
        i2s->FIFOCTL &= ~SPI_FIFOCTL_RXTOIEN_Msk;
    }

    /* Disable TX underflow interrupt flag */
    if ((u32Mask & SPII2S_TXUF_INT_MASK) == SPII2S_TXUF_INT_MASK)
    {
        i2s->FIFOCTL &= ~SPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Disable right channel zero cross interrupt flag */
    if ((u32Mask & SPII2S_RIGHT_ZC_INT_MASK) == SPII2S_RIGHT_ZC_INT_MASK)
    {
        i2s->I2SCTL &= ~SPI_I2SCTL_RZCIEN_Msk;
    }

    /* Disable left channel zero cross interrupt flag */
    if ((u32Mask & SPII2S_LEFT_ZC_INT_MASK) == SPII2S_LEFT_ZC_INT_MASK)
    {
        i2s->I2SCTL &= ~SPI_I2SCTL_LZCIEN_Msk;
    }

    /* Disable slave mode bit clock loss interrupt flag */
    if ((u32Mask & SPII2S_SLV_CLKERR_INT_MASK) == SPII2S_SLV_CLKERR_INT_MASK)
    {
        i2s->I2SCTL &= ~SPI_I2SCTL_SLVERRIEN_Msk;
    }
}

/**
 * @brief      Enable master clock (MCLK).
 *
 * @param[in]  i2s         The pointer of the specified I2S module.
 * @param[in]  u32BusClock The target MCLK clock rate.
 *
 * @return     Actual MCLK clock rate
 *
 * @details    Set the master clock rate according to u32BusClock parameter and enable master clock output.
 *             The actual master clock rate may be different from the target master clock rate. The real master clock rate will be returned for reference.
 */
uint32_t SPII2S_EnableMCLK(SPI_T *i2s, uint32_t u32BusClock)
{
    uint32_t u32Divider = 0UL;
    uint32_t u32SrcClk = 0UL;
    uint32_t u32RetValue = 0UL;

    if ((SPI_IsValidModule(i2s) == 0UL) || (u32BusClock == 0UL))
    {
        return 0UL;
    }

    u32SrcClk = SPII2S_GetSourceClockFreq(i2s);

    if (u32BusClock == u32SrcClk)
    {
        u32Divider = 0UL;
    }
    else
    {
        u32Divider = ((u32SrcClk / u32BusClock) >> 1U);

        /* MCLKDIV is a 7-bit width configuration. */
        if (u32Divider > SPI_I2SCLK_MCLKDIV_Msk)
        {
            u32Divider = SPI_I2SCLK_MCLKDIV_Msk;
        }
    }

    /* Write u32Divider to MCLKDIV (SPI_I2SCLK[5:0]) */
    i2s->I2SCLK = (i2s->I2SCLK & ~SPI_I2SCLK_MCLKDIV_Msk) | (u32Divider << SPI_I2SCLK_MCLKDIV_Pos);

    /* Enable MCLK output */
    i2s->I2SCTL |= SPI_I2SCTL_MCLKEN_Msk;

    if (u32Divider == 0U)
    {
        u32RetValue = u32SrcClk; /* If MCLKDIV=0, master clock rate is equal to the source clock rate. */
    }
    else
    {
        u32RetValue = ((u32SrcClk >> 1U) / u32Divider); /* If MCLKDIV>0, master clock rate = source clock rate / (MCLKDIV * 2) */
    }

    return u32RetValue;
}

/**
 * @brief      Disable master clock (MCLK).
 *
 * @param[in]  i2s The pointer of the specified I2S module.
 *
 * @return     None
 *
 * @details    Clear MCLKEN bit of SPI_I2SCTL register to disable master clock output.
 */
void SPII2S_DisableMCLK(SPI_T *i2s)
{
    if (SPI_IsValidModule(i2s) == 0UL)
    {
        return;
    }

    i2s->I2SCTL &= ~SPI_I2SCTL_MCLKEN_Msk;
}

/**
 * @brief      Configure FIFO threshold setting.
 *
 * @param[in]  i2s            The pointer of the specified I2S module.
 * @param[in]  u32TxThreshold Decides the TX FIFO threshold. It could be 0 ~ 3.
 * @param[in]  u32RxThreshold Decides the RX FIFO threshold. It could be 0 ~ 3.
 *
 * @return     None
 *
 * @details    Set TX FIFO threshold and RX FIFO threshold configurations.
 */
void SPII2S_SetFIFO(SPI_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    if (SPI_IsValidModule(i2s) == 0UL)
    {
        return;
    }

    i2s->FIFOCTL = (i2s->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk)) |
                   ((u32TxThreshold << SPI_FIFOCTL_TXTH_Pos) & SPI_FIFOCTL_TXTH_Msk) |
                   ((u32RxThreshold << SPI_FIFOCTL_RXTH_Pos) & SPI_FIFOCTL_RXTH_Msk);
}

/** @} end of group SPI_EXPORTED_FUNCTIONS */
/** @} end of group SPI_Driver */
/** @} end of group Standard_Driver */

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
