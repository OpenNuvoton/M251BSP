/***************************************************************************//**
 * @file    qspi.c
 * @version V1.00
 * @brief   QSPI driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup QSPI_Driver QSPI Driver
  @{
*/

/** @addtogroup QSPI_EXPORTED_FUNCTIONS QSPI Exported Functions
  @{
*/
#define QSPI_CONFIG_TIMEOUT     (0x100000UL)

/**
 * @brief      This function make sure the module is valid QSPI module.
 *
 * @param[in]  qspi The pointer of the specified QSPI module.
 *
 * @return     1 = QSPI module; 0 = not QSPI module.
 */
static uint32_t QSPI_IsValidModule(const QSPI_T *qspi)
{
    return ((qspi == QSPI0) ? 1UL : 0UL);
}

/**
 * @brief      This function make sure the master/slave setting is valid.
 *
 * @param[in]  u32MasterSlave The master/slave setting.
 *
 * @return     1 = valid; 0 = invalid.
 */
static uint32_t QSPI_IsValidMasterSlave(uint32_t u32MasterSlave)
{
    return (((u32MasterSlave == QSPI_MASTER) || (u32MasterSlave == QSPI_SLAVE)) ? 1UL : 0UL);
}

/**
 * @brief      This function make sure the QSPI mode setting is valid.
 *
 * @param[in]  u32QSPIMode The QSPI mode setting.
 *
 * @return     1 = valid; 0 = invalid.
 */
static uint32_t QSPI_IsValidMode(uint32_t u32QSPIMode)
{
    return (((u32QSPIMode == QSPI_MODE_0) || (u32QSPIMode == QSPI_MODE_1)
             || (u32QSPIMode == QSPI_MODE_2) || (u32QSPIMode == QSPI_MODE_3)) ? 1UL : 0UL);
}

/**
 * @brief      This function set PCLK as the clock source of QSPI.
 *
 * @param[in]  qspi The pointer of the specified QSPI module.
 *
 * @return     None.
 */
static void QSPI_SetPCLKSrc(const QSPI_T *qspi)
{
    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return;
    }

    if (qspi == QSPI0)
    {
        CLK->CLKSEL2 = (CLK->CLKSEL2 & ~(CLK_CLKSEL2_QSPI0SEL_Msk)) | (CLK_CLKSEL2_QSPI0SEL_PCLK0);
    }
}

/**
 * @brief      This function get the PCLK clock frequency of QSPI.
 *
 * @param[in]  qspi The pointer of the specified QSPI module.
 *
 * @return     The PCLK clock frequency of QSPI.
 */
static uint32_t QSPI_GetPCLKSrcFreq(const QSPI_T *qspi)
{
    uint32_t u32RetValue = 0UL;

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return 0UL;
    }

    if (qspi == QSPI0)
    {
        u32RetValue = CLK_GetPCLK0Freq();
    }

    return u32RetValue;
}

/**
 * @brief   Get QSPI Clock Source Frequency.
 *
 * @param   qspi The pointer of the specified QSPI module.
 *
 * @return  uint32_t Clock Frequency
 */
static uint32_t QSPI_GetModuleClkSrcFrq(const QSPI_T *qspi)
{
    uint32_t u32RetValue = 0UL;

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return 0UL;
    }

    /* Get QSPI clock source selection */
    if (qspi == QSPI0)
    {
        /* Check clock source of QSPI */
        if ((CLK->CLKSEL2 & CLK_CLKSEL2_QSPI0SEL_Msk) == CLK_CLKSEL2_QSPI0SEL_HXT)
        {
            u32RetValue = __HXT; /* Clock source is HXT */
        }
        else if ((CLK->CLKSEL2 & CLK_CLKSEL2_QSPI0SEL_Msk) == CLK_CLKSEL2_QSPI0SEL_PLL)
        {
            u32RetValue = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        }
        else if ((CLK->CLKSEL2 & CLK_CLKSEL2_QSPI0SEL_Msk) == CLK_CLKSEL2_QSPI0SEL_PCLK0)
        {
            /* Clock source is PCLK0 */
            u32RetValue = CLK_GetPCLK0Freq();
        }
        else
        {
            u32RetValue = __HIRC; /* Clock source is HIRC */
        }
    }

    return u32RetValue;
}

/**
 * @brief      This function waits for the QSPI module to be disabled.
 *
 * @param[in]  qspi The pointer of the specified QSPI module.
 *
 * @return     QSPI_OK if the module is disabled; QSPI_ERR_TIMEOUT if the operation times out; QSPI_ERR_FAIL if the module is invalid.
 */
static int32_t QSPI_WaitDisableReady(const QSPI_T *qspi)
{
    uint32_t u32Timeout = QSPI_CONFIG_TIMEOUT;

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return QSPI_ERR_FAIL;
    }

    while (((qspi->STATUS & QSPI_STATUS_QSPIENSTS_Msk) != 0UL) && (u32Timeout > 0UL))
    {
        u32Timeout--;
    }

    return ((u32Timeout == 0UL) ? QSPI_ERR_TIMEOUT : QSPI_OK);
}

/**
 * @brief      This function waits for the QSPI module to be ready.
 *
 * @param[in]  qspi The pointer of the specified QSPI module.
 * @param[in]  pu32WasEnabled Pointer to a variable that indicates whether the QSPI module was enabled before entering configuration mode.
 *
 * @return     QSPI_OK if the module is ready; QSPI_ERR_TIMEOUT if the operation times out; QSPI_ERR_FAIL if the module is invalid.
 */
static int32_t QSPI_EnterConfigMode(QSPI_T *qspi, uint32_t *pu32WasEnabled)
{
    if ((QSPI_IsValidModule(qspi) == 0UL) || (pu32WasEnabled == (uint32_t *)NULL))
    {
        return QSPI_ERR_FAIL;
    }

    *pu32WasEnabled = (qspi->CTL & QSPI_CTL_QSPIEN_Msk);

    if (*pu32WasEnabled != 0U)
    {
        qspi->CTL &= ~QSPI_CTL_QSPIEN_Msk;
    }

    return QSPI_WaitDisableReady(qspi);
}

/**
 * @brief      This function exits the QSPI configuration mode.
 *
 * @param[in]  qspi          The pointer of the specified QSPI module.
 * @param[in]  u32WasEnabled The QSPI module was enabled or not.
 *
 * @return     None
 */
static void QSPI_ExitConfigMode(QSPI_T *qspi, uint32_t u32WasEnabled)
{
    if (u32WasEnabled != 0U)
    {
        qspi->CTL |= QSPI_CTL_QSPIEN_Msk;
    }
}

/**
 * @brief      This function make QSPI module be ready to transfer.
 *
 * @param[in]  qspi           The pointer of the specified QSPI module.
 * @param[in]  u32MasterSlave Decides the QSPI module is operating in master mode or in slave mode. (QSPI_SLAVE, QSPI_MASTER)
 * @param[in]  u32QSPIMode    Decides the transfer timing. (QSPI_MODE_0, QSPI_MODE_1, QSPI_MODE_2, QSPI_MODE_3)
 * @param[in]  u32DataWidth   Decides the data width of a QSPI transaction.
 * @param[in]  u32BusClock    The expected frequency of QSPI bus clock in Hz.
 *
 * @return     Actual frequency of QSPI peripheral clock.
 *
 * @details    By default, the QSPI transfer sequence is MSB first, the slave selection signal is active low and the automatic
 *             slave selection function is disabled.
 *             In Slave mode, the u32BusClock shall be 0U and the QSPI clock divider setting will be 0.
 *             The actual clock rate may be different from the target QSPI clock rate.
 *             For example, if the QSPI source clock rate is 12 MHz and the target QSPI bus clock rate is 7 MHz, the
 *             actual QSPI clock rate will be 6MHz.
 * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
 * @note   If u32BusClock >= system clock frequency, QSPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
 * @note   If u32BusClock >= QSPI peripheral clock source, DIVIDER will be set to 0.
 * @note   In slave mode, the QSPI peripheral clock rate will be equal to APB clock rate.
 */
uint32_t QSPI_Open(QSPI_T *qspi, uint32_t u32MasterSlave, uint32_t u32QSPIMode, uint32_t u32DataWidth, uint32_t u32BusClock)
{
    uint32_t u32RetValue = 0UL;

    if ((QSPI_IsValidModule(qspi) == 0UL) ||
            (QSPI_IsValidMasterSlave(u32MasterSlave) == 0UL) ||
            (QSPI_IsValidMode(u32QSPIMode) == 0UL))
    {
        return 0UL;
    }

    qspi->CTL &= ~QSPI_CTL_QSPIEN_Msk;

    if (QSPI_WaitDisableReady(qspi) != QSPI_OK)
    {
        return 0UL;
    }

    /* Default setting: slave selection signal is active low.
       In Master mode, disable the automatic slave selection function. */
    qspi->SSCTL = QSPI_SS_ACTIVE_LOW;

    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    qspi->CTL = (u32QSPIMode | u32MasterSlave);
    QSPI_SET_DATA_WIDTH(qspi, u32DataWidth);

    if (u32MasterSlave == QSPI_MASTER)
    {
        /* Set the bus clock for the QSPI module and store the actual frequency in u32RetValue. */
        u32RetValue = QSPI_SetBusClock(qspi, u32BusClock);
    }
    else /* For slave mode, force the QSPI peripheral clock rate to equal APB clock rate. */
    {
        /* Force slave peripheral clock source back to PCLK. */
        QSPI_SetPCLKSrc(qspi);

        /* Set DIVIDER = 0 */
        qspi->CLKDIV = 0UL;

        /* Get the PCLK clock frequency of QSPI */
        u32RetValue = QSPI_GetPCLKSrcFreq(qspi);
    }

    qspi->CTL |= QSPI_CTL_QSPIEN_Msk;

    return u32RetValue;
}

/**
 * @brief      Disable QSPI controller.
 *
 * @param[in]  qspi The pointer of the specified QSPI module.
 *
 * @return     None
 *
 * @details    This function will reset QSPI controller.
 */
void QSPI_Close(const QSPI_T *qspi)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return;
    }

    /* Unlock protected registers */
    if (u32RegLockLevel != 0UL)
    {
        SYS_UnlockReg();
    }

    /* Reset QSPI */
    if (qspi == QSPI0)
    {
        SYS->IPRST1 |= SYS_IPRST1_QSPI0RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_QSPI0RST_Msk;
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
 * @param[in]  qspi The pointer of the specified QSPI module.
 *
 * @return     None
 *
 * @details    This function will clear QSPI RX FIFO buffer. The RXEMPTY (QSPI_STATUS[8]) will be set to 1.
 */
void QSPI_ClearRxFIFO(QSPI_T *qspi)
{
    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return;
    }

    qspi->FIFOCTL |= QSPI_FIFOCTL_RXFBCLR_Msk;
}

/**
 * @brief      Clear TX FIFO buffer.
 *
 * @param[in]  qspi The pointer of the specified QSPI module.
 *
 * @return     None
 *
 * @details    This function will clear QSPI TX FIFO buffer. The TXEMPTY (QSPI_STATUS[16]) will be set to 1.
 * @note The TX shift register will not be cleared.
 */
void QSPI_ClearTxFIFO(QSPI_T *qspi)
{
    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return;
    }

    qspi->FIFOCTL |= QSPI_FIFOCTL_TXFBCLR_Msk;
}

/**
 * @brief      Disable the automatic slave selection function.
 *
 * @param[in]  qspi The pointer of the specified QSPI module.
 *
 * @return     None
 *
 * @details    This function will disable the automatic slave selection function and set slave selection signal to inactive state.
 */
void QSPI_DisableAutoSS(QSPI_T *qspi)
{
    uint32_t u32WasEnabled = 0UL;

    if (QSPI_EnterConfigMode(qspi, &u32WasEnabled) != QSPI_OK)
    {
        return;
    }

    qspi->SSCTL &= ~(QSPI_SSCTL_AUTOSS_Msk | QSPI_SSCTL_SS_Msk);

    QSPI_ExitConfigMode(qspi, u32WasEnabled);
}

/**
 * @brief      Enable the automatic slave selection function.
 *
 * @param[in]  qspi           The pointer of the specified QSPI module.
 * @param[in]  u32SSPinMask   Specifies slave selection pins. (QSPI_SS)
 * @param[in]  u32ActiveLevel Specifies the active level of slave selection signal. (QSPI_SS_ACTIVE_HIGH, QSPI_SS_ACTIVE_LOW)
 *
 * @return     None
 *
 * @details    This function will enable the automatic slave selection function. Only available in Master mode.
 *             The slave selection pin and the active level will be set in this function.
 */
void QSPI_EnableAutoSS(QSPI_T *qspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    uint32_t u32WasEnabled = 0UL;

    if (QSPI_EnterConfigMode(qspi, &u32WasEnabled) != QSPI_OK)
    {
        return;
    }

    qspi->SSCTL = (qspi->SSCTL & (~(QSPI_SSCTL_AUTOSS_Msk | QSPI_SSCTL_SSACTPOL_Msk | QSPI_SSCTL_SS_Msk))) |
                  ((u32SSPinMask & QSPI_SSCTL_SS_Msk) | (u32ActiveLevel & QSPI_SSCTL_SSACTPOL_Msk) | QSPI_SSCTL_AUTOSS_Msk);

    QSPI_ExitConfigMode(qspi, u32WasEnabled);
}

/**
 * @brief      Set the QSPI bus clock.
 *
 * @param[in]  qspi        The pointer of the specified QSPI module.
 * @param[in]  u32BusClock The expected frequency of QSPI bus clock in Hz.
 *
 * @return     Actual frequency of QSPI bus clock.
 *
 * @details    This function is only available in Master mode. The actual clock rate may be different from the target QSPI bus clock rate.
 *             For example, if the QSPI source clock rate is 12 MHz and the target QSPI bus clock rate is 7 MHz, the actual QSPI bus clock
 *             rate will be 6 MHz.
 * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
 * @note   If u32BusClock >= system clock frequency, QSPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
 * @note   If u32BusClock >= QSPI peripheral clock source, DIVIDER will be set to 0.
 */
uint32_t QSPI_SetBusClock(QSPI_T *qspi, uint32_t u32BusClock)
{
    uint32_t u32ClkSrc = 0UL;
    uint32_t u32HCLKFreq = 0UL;
    uint32_t u32RetValue = 0UL;
    uint32_t u32WasEnabled = 0UL;

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return 0UL;
    }

    if (QSPI_EnterConfigMode(qspi, &u32WasEnabled) != QSPI_OK)
    {
        return QSPI_GetBusClock(qspi);
    }

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    if (u32BusClock >= u32HCLKFreq)
    {
        /* Select PCLK as the clock source of QSPI */
        QSPI_SetPCLKSrc(qspi);
    }

    /* Get clock source of QSPI */
    u32ClkSrc = QSPI_GetModuleClkSrcFrq(qspi);

    if (u32BusClock >= u32HCLKFreq)
    {
        /* Set DIVIDER = 0 */
        qspi->CLKDIV = 0UL;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    }
    else if (u32BusClock >= u32ClkSrc)
    {
        /* Set DIVIDER = 0 */
        qspi->CLKDIV = 0UL;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    }
    else if (u32BusClock == 0U)
    {
        /* Set DIVIDER to the maximum value. f_qspi = f_qspi_clk_src / (DIVIDER + 1) */
        qspi->CLKDIV = (qspi->CLKDIV & ~QSPI_CLKDIV_DIVIDER_Msk) | QSPI_CLKDIV_DIVIDER_Msk;
        /* Return master peripheral clock rate */
        u32RetValue = (u32ClkSrc / ((QSPI_CLKDIV_DIVIDER_Msk >> QSPI_CLKDIV_DIVIDER_Pos) + 1U));
    }
    else
    {
        uint32_t u32Div = 0UL;

        u32Div = (((((u32ClkSrc * 10U) / u32BusClock) + 5U) / 10U) - 1U); /* Round to the nearest integer */

        /* Ensure the divider does not exceed the maximum allowed value. */
        u32Div = ((u32Div > (QSPI_CLKDIV_DIVIDER_Msk >> QSPI_CLKDIV_DIVIDER_Pos)) ?
                  (QSPI_CLKDIV_DIVIDER_Msk >> QSPI_CLKDIV_DIVIDER_Pos) : u32Div);

        /* Update the CLKDIV register with the new divider value. */
        qspi->CLKDIV = (qspi->CLKDIV & ~(QSPI_CLKDIV_DIVIDER_Msk)) | (u32Div << QSPI_CLKDIV_DIVIDER_Pos);

        /* Return master peripheral clock rate */
        u32RetValue = (u32ClkSrc / (u32Div + 1U));
    }

    QSPI_ExitConfigMode(qspi, u32WasEnabled);

    return u32RetValue;
}

/**
 * @brief      Set QSPI data width.
 *
 * @param[in]  qspi     Pointer to QSPI module.
 * @param[in]  u32Width Requested data width.
 *
 * @return     None
 *
 * @details
 *             The QSPI data-width register encoding is:
 *             - Register value 0 represents 32-bit width.
 *             - Values from 8 to 31 represent the corresponding data width.
 *             - Requested widths from 1 to 7 are clamped to 8 bits.
 *             - Requested widths greater than or equal to 32 are configured as 32 bits.
 */
void QSPI_SetDataWidth(QSPI_T *qspi, uint32_t u32Width)
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
         * Clamp unsupported widths from 1 to 7
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

    u32RegValue = qspi->CTL;
    u32RegValue &= ~QSPI_CTL_DWIDTH_Msk;
    u32RegValue |= ((u32RegWidth << QSPI_CTL_DWIDTH_Pos) & QSPI_CTL_DWIDTH_Msk);

    qspi->CTL = u32RegValue;
}

/**
 * @brief      Configure FIFO threshold setting.
 *
 * @param[in]  qspi           The pointer of the specified QSPI module.
 * @param[in]  u32TxThreshold Decides the TX FIFO threshold. It could be 0 ~ 7.
 * @param[in]  u32RxThreshold Decides the RX FIFO threshold. It could be 0 ~ 7.
 *
 * @return     None
 *
 * @details    Set TX FIFO threshold and RX FIFO threshold configurations.
 */
void QSPI_SetFIFO(QSPI_T *qspi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    uint32_t u32WasEnabled = 0UL;

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return;
    }

    if (QSPI_EnterConfigMode(qspi, &u32WasEnabled) != QSPI_OK)
    {
        return;
    }

    qspi->FIFOCTL = ((qspi->FIFOCTL & ~(QSPI_FIFOCTL_TXTH_Msk | QSPI_FIFOCTL_RXTH_Msk)) |
                     ((u32TxThreshold << QSPI_FIFOCTL_TXTH_Pos) & QSPI_FIFOCTL_TXTH_Msk) |
                     ((u32RxThreshold << QSPI_FIFOCTL_RXTH_Pos) & QSPI_FIFOCTL_RXTH_Msk));

    QSPI_ExitConfigMode(qspi, u32WasEnabled);
}

/**
 * @brief      Get the actual frequency of QSPI bus clock. Only available in Master mode.
 *
 * @param[in]  qspi The pointer of the specified QSPI module.
 *
 * @return     Actual QSPI bus clock frequency in Hz.
 *
 * @details    This function will calculate the actual QSPI bus clock rate according to the QSPInSEL and DIVIDER settings. Only available in Master mode.
 */
uint32_t QSPI_GetBusClock(const QSPI_T *qspi)
{
    uint32_t u32Div = 0UL;
    uint32_t u32ClkSrc = 0UL;

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return 0UL;
    }

    /* Get DIVIDER setting */
    u32Div = ((qspi->CLKDIV & QSPI_CLKDIV_DIVIDER_Msk) >> QSPI_CLKDIV_DIVIDER_Pos);

    /* Get clock source of QSPI */
    u32ClkSrc = QSPI_GetModuleClkSrcFrq(qspi);

    /* Return QSPI bus clock rate */
    return (u32ClkSrc / (u32Div + 1U));
}

/**
 * @brief      Enable interrupt function.
 *
 * @param[in]  qspi    The pointer of the specified QSPI module.
 * @param[in]  u32Mask The combination of all related interrupt enable bits.
 *                     Each bit corresponds to a interrupt enable bit.
 *                     This parameter decides which interrupts will be enabled. It is combination of:
 *                     - \ref QSPI_UNIT_INT_MASK
 *                     - \ref QSPI_SSACT_INT_MASK
 *                     - \ref QSPI_SSINACT_INT_MASK
 *                     - \ref QSPI_SLVUR_INT_MASK
 *                     - \ref QSPI_SLVBE_INT_MASK
 *                     - \ref QSPI_SLVTO_INT_MASK
 *                     - \ref QSPI_TXUF_INT_MASK
 *                     - \ref QSPI_FIFO_TXTH_INT_MASK
 *                     - \ref QSPI_FIFO_RXTH_INT_MASK
 *                     - \ref QSPI_FIFO_RXOV_INT_MASK
 *                     - \ref QSPI_FIFO_RXTO_INT_MASK
 *
 * @return     None
 *
 * @details    Enable QSPI related interrupts specified by u32Mask parameter.
 */
void QSPI_EnableInt(QSPI_T *qspi, uint32_t u32Mask)
{
    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return;
    }

    /* Enable unit transfer interrupt flag */
    if ((u32Mask & QSPI_UNIT_INT_MASK) == QSPI_UNIT_INT_MASK)
    {
        qspi->CTL |= QSPI_CTL_UNITIEN_Msk;
    }

    /* Enable slave selection signal active interrupt flag */
    if ((u32Mask & QSPI_SSACT_INT_MASK) == QSPI_SSACT_INT_MASK)
    {
        qspi->SSCTL |= QSPI_SSCTL_SSACTIEN_Msk;
    }

    /* Enable slave selection signal inactive interrupt flag */
    if ((u32Mask & QSPI_SSINACT_INT_MASK) == QSPI_SSINACT_INT_MASK)
    {
        qspi->SSCTL |= QSPI_SSCTL_SSINAIEN_Msk;
    }

    /* Enable slave TX under run interrupt flag */
    if ((u32Mask & QSPI_SLVUR_INT_MASK) == QSPI_SLVUR_INT_MASK)
    {
        qspi->SSCTL |= QSPI_SSCTL_SLVURIEN_Msk;
    }

    /* Enable slave bit count error interrupt flag */
    if ((u32Mask & QSPI_SLVBE_INT_MASK) == QSPI_SLVBE_INT_MASK)
    {
        qspi->SSCTL |= QSPI_SSCTL_SLVBEIEN_Msk;
    }

    /* Enable slave mode time-out interrupt flag */
    if ((u32Mask & QSPI_SLVTO_INT_MASK) == QSPI_SLVTO_INT_MASK)
    {
        qspi->SSCTL |= QSPI_SSCTL_SLVTOIEN_Msk;
    }

    /* Enable slave TX underflow interrupt flag */
    if ((u32Mask & QSPI_TXUF_INT_MASK) == QSPI_TXUF_INT_MASK)
    {
        qspi->FIFOCTL |= QSPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Enable TX threshold interrupt flag */
    if ((u32Mask & QSPI_FIFO_TXTH_INT_MASK) == QSPI_FIFO_TXTH_INT_MASK)
    {
        qspi->FIFOCTL |= QSPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Enable RX threshold interrupt flag */
    if ((u32Mask & QSPI_FIFO_RXTH_INT_MASK) == QSPI_FIFO_RXTH_INT_MASK)
    {
        qspi->FIFOCTL |= QSPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Enable RX overrun interrupt flag */
    if ((u32Mask & QSPI_FIFO_RXOV_INT_MASK) == QSPI_FIFO_RXOV_INT_MASK)
    {
        qspi->FIFOCTL |= QSPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Enable RX time-out interrupt flag */
    if ((u32Mask & QSPI_FIFO_RXTO_INT_MASK) == QSPI_FIFO_RXTO_INT_MASK)
    {
        qspi->FIFOCTL |= QSPI_FIFOCTL_RXTOIEN_Msk;
    }
}

/**
 * @brief      Disable interrupt function.
 *
 * @param[in]  qspi    The pointer of the specified QSPI module.
 * @param[in]  u32Mask The combination of all related interrupt enable bits.
 *                     Each bit corresponds to a interrupt bit.
 *                     This parameter decides which interrupts will be disabled. It is combination of:
 *                     - \ref QSPI_UNIT_INT_MASK
 *                     - \ref QSPI_SSACT_INT_MASK
 *                     - \ref QSPI_SSINACT_INT_MASK
 *                     - \ref QSPI_SLVUR_INT_MASK
 *                     - \ref QSPI_SLVBE_INT_MASK
 *                     - \ref QSPI_SLVTO_INT_MASK
 *                     - \ref QSPI_TXUF_INT_MASK
 *                     - \ref QSPI_FIFO_TXTH_INT_MASK
 *                     - \ref QSPI_FIFO_RXTH_INT_MASK
 *                     - \ref QSPI_FIFO_RXOV_INT_MASK
 *                     - \ref QSPI_FIFO_RXTO_INT_MASK
 *
 * @return     None
 *
 * @details    Disable QSPI related interrupts specified by u32Mask parameter.
 */
void QSPI_DisableInt(QSPI_T *qspi, uint32_t u32Mask)
{
    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return;
    }

    /* Disable unit transfer interrupt flag */
    if ((u32Mask & QSPI_UNIT_INT_MASK) == QSPI_UNIT_INT_MASK)
    {
        qspi->CTL &= ~QSPI_CTL_UNITIEN_Msk;
    }

    /* Disable slave selection signal active interrupt flag */
    if ((u32Mask & QSPI_SSACT_INT_MASK) == QSPI_SSACT_INT_MASK)
    {
        qspi->SSCTL &= ~QSPI_SSCTL_SSACTIEN_Msk;
    }

    /* Disable slave selection signal inactive interrupt flag */
    if ((u32Mask & QSPI_SSINACT_INT_MASK) == QSPI_SSINACT_INT_MASK)
    {
        qspi->SSCTL &= ~QSPI_SSCTL_SSINAIEN_Msk;
    }

    /* Disable slave TX under run interrupt flag */
    if ((u32Mask & QSPI_SLVUR_INT_MASK) == QSPI_SLVUR_INT_MASK)
    {
        qspi->SSCTL &= ~QSPI_SSCTL_SLVURIEN_Msk;
    }

    /* Disable slave bit count error interrupt flag */
    if ((u32Mask & QSPI_SLVBE_INT_MASK) == QSPI_SLVBE_INT_MASK)
    {
        qspi->SSCTL &= ~QSPI_SSCTL_SLVBEIEN_Msk;
    }

    /* Disable slave mode time-out interrupt flag */
    if ((u32Mask & QSPI_SLVTO_INT_MASK) == QSPI_SLVTO_INT_MASK)
    {
        qspi->SSCTL &= ~QSPI_SSCTL_SLVTOIEN_Msk;
    }

    /* Disable slave TX underflow interrupt flag */
    if ((u32Mask & QSPI_TXUF_INT_MASK) == QSPI_TXUF_INT_MASK)
    {
        qspi->FIFOCTL &= ~QSPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Disable TX threshold interrupt flag */
    if ((u32Mask & QSPI_FIFO_TXTH_INT_MASK) == QSPI_FIFO_TXTH_INT_MASK)
    {
        qspi->FIFOCTL &= ~QSPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Disable RX threshold interrupt flag */
    if ((u32Mask & QSPI_FIFO_RXTH_INT_MASK) == QSPI_FIFO_RXTH_INT_MASK)
    {
        qspi->FIFOCTL &= ~QSPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Disable RX overrun interrupt flag */
    if ((u32Mask & QSPI_FIFO_RXOV_INT_MASK) == QSPI_FIFO_RXOV_INT_MASK)
    {
        qspi->FIFOCTL &= ~QSPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Disable RX time-out interrupt flag */
    if ((u32Mask & QSPI_FIFO_RXTO_INT_MASK) == QSPI_FIFO_RXTO_INT_MASK)
    {
        qspi->FIFOCTL &= ~QSPI_FIFOCTL_RXTOIEN_Msk;
    }
}

/**
 * @brief      Get interrupt flag.
 *
 * @param[in]  qspi    The pointer of the specified QSPI module.
 * @param[in]  u32Mask The combination of all related interrupt sources.
 *                     Each bit corresponds to a interrupt source.
 *                     This parameter decides which interrupt flags will be read. It is combination of:
 *                     - \ref QSPI_UNIT_INT_MASK
 *                     - \ref QSPI_SSACT_INT_MASK
 *                     - \ref QSPI_SSINACT_INT_MASK
 *                     - \ref QSPI_SLVUR_INT_MASK
 *                     - \ref QSPI_SLVBE_INT_MASK
 *                     - \ref QSPI_SLVTO_INT_MASK
 *                     - \ref QSPI_TXUF_INT_MASK
 *                     - \ref QSPI_FIFO_TXTH_INT_MASK
 *                     - \ref QSPI_FIFO_RXTH_INT_MASK
 *                     - \ref QSPI_FIFO_RXOV_INT_MASK
 *                     - \ref QSPI_FIFO_RXTO_INT_MASK
 *
 * @return     Interrupt flags of selected sources.
 *
 * @details    Get QSPI related interrupt flags specified by u32Mask parameter.
 */
uint32_t QSPI_GetIntFlag(const QSPI_T *qspi, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0UL;
    uint32_t u32Status = 0UL;

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return 0UL;
    }

    u32Status = qspi->STATUS;

    /* Check unit transfer interrupt flag */
    if (((u32Mask & QSPI_UNIT_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_UNITIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_UNIT_INT_MASK;
    }

    /* Check slave selection signal active interrupt flag */
    if (((u32Mask & QSPI_SSACT_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_SSACTIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_SSACT_INT_MASK;
    }

    /* Check slave selection signal inactive interrupt flag */
    if (((u32Mask & QSPI_SSINACT_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_SSINAIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_SSINACT_INT_MASK;
    }

    /* Check slave TX under run interrupt flag */
    if (((u32Mask & QSPI_SLVUR_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_SLVURIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_SLVUR_INT_MASK;
    }

    /* Check slave bit count error interrupt flag */
    if (((u32Mask & QSPI_SLVBE_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_SLVBEIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_SLVBE_INT_MASK;
    }

    /* Check slave mode time-out interrupt flag */
    if (((u32Mask & QSPI_SLVTO_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_SLVTOIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_SLVTO_INT_MASK;
    }

    /* Check slave TX underflow interrupt flag */
    if (((u32Mask & QSPI_TXUF_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_TXUFIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_TXUF_INT_MASK;
    }

    /* Check TX threshold interrupt flag */
    if (((u32Mask & QSPI_FIFO_TXTH_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_TXTHIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_FIFO_TXTH_INT_MASK;
    }

    /* Check RX threshold interrupt flag */
    if (((u32Mask & QSPI_FIFO_RXTH_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_RXTHIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_FIFO_RXTH_INT_MASK;
    }

    /* Check RX overrun interrupt flag */
    if (((u32Mask & QSPI_FIFO_RXOV_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_RXOVIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_FIFO_RXOV_INT_MASK;
    }

    /* Check RX time-out interrupt flag */
    if (((u32Mask & QSPI_FIFO_RXTO_INT_MASK) != 0U) &&
            ((u32Status & QSPI_STATUS_RXTOIF_Msk) != 0U))
    {
        u32IntFlag |= QSPI_FIFO_RXTO_INT_MASK;
    }

    return u32IntFlag;
}

/**
 * @brief      Clear interrupt flag.
 *
 * @param[in]  qspi    The pointer of the specified QSPI module.
 * @param[in]  u32Mask The combination of all related interrupt sources.
 *                     Each bit corresponds to a interrupt source.
 *                     This parameter decides which interrupt flags will be cleared. It could be the combination of:
 *                     - \ref QSPI_UNIT_INT_MASK
 *                     - \ref QSPI_SSACT_INT_MASK
 *                     - \ref QSPI_SSINACT_INT_MASK
 *                     - \ref QSPI_SLVUR_INT_MASK
 *                     - \ref QSPI_SLVBE_INT_MASK
 *                     - \ref QSPI_SLVTO_INT_MASK
 *                     - \ref QSPI_TXUF_INT_MASK
 *                     - \ref QSPI_FIFO_RXOV_INT_MASK
 *                     - \ref QSPI_FIFO_RXTO_INT_MASK
 *
 * @return     None
 *
 * @details    Clear QSPI related interrupt flags specified by u32Mask parameter.
 */
void QSPI_ClearIntFlag(QSPI_T *qspi, uint32_t u32Mask)
{
    uint32_t u32ClearMask = 0UL;

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return;
    }

    if ((u32Mask & QSPI_UNIT_INT_MASK) != 0UL)
    {
        u32ClearMask |= QSPI_STATUS_UNITIF_Msk; /* Clear unit transfer interrupt flag */
    }

    if ((u32Mask & QSPI_SSACT_INT_MASK) != 0UL)
    {
        u32ClearMask |= QSPI_STATUS_SSACTIF_Msk; /* Clear slave selection signal active interrupt flag */
    }

    if ((u32Mask & QSPI_SSINACT_INT_MASK) != 0UL)
    {
        u32ClearMask |= QSPI_STATUS_SSINAIF_Msk; /* Clear slave selection signal inactive interrupt flag */
    }

    if ((u32Mask & QSPI_SLVUR_INT_MASK) != 0UL)
    {
        u32ClearMask |= QSPI_STATUS_SLVURIF_Msk; /* Clear slave TX under run interrupt flag */
    }

    if ((u32Mask & QSPI_SLVBE_INT_MASK) != 0UL)
    {
        u32ClearMask |= QSPI_STATUS_SLVBEIF_Msk; /* Clear slave bit count error interrupt flag */
    }

    if ((u32Mask & QSPI_SLVTO_INT_MASK) != 0UL)
    {
        u32ClearMask |= QSPI_STATUS_SLVTOIF_Msk; /* Clear slave mode time-out interrupt flag */
    }

    if ((u32Mask & QSPI_TXUF_INT_MASK) != 0UL)
    {
        u32ClearMask |= QSPI_STATUS_TXUFIF_Msk; /* Clear slave TX underflow interrupt flag */
    }

    if ((u32Mask & QSPI_FIFO_RXOV_INT_MASK) != 0UL)
    {
        u32ClearMask |= QSPI_STATUS_RXOVIF_Msk; /* Clear RX overrun interrupt flag */
    }

    if ((u32Mask & QSPI_FIFO_RXTO_INT_MASK) != 0UL)
    {
        u32ClearMask |= QSPI_STATUS_RXTOIF_Msk; /* Clear RX time-out interrupt flag */
    }

    if (u32ClearMask != 0UL)
    {
        qspi->STATUS = u32ClearMask;
    }
}

/**
 * @brief      Get QSPI status.
 *
 * @param[in]  qspi    The pointer of the specified QSPI module.
 * @param[in]  u32Mask The combination of all related sources.
 *                     Each bit corresponds to a source.
 *                     This parameter decides which flags will be read. It is combination of:
 *                     - \ref QSPI_BUSY_MASK
 *                     - \ref QSPI_RX_EMPTY_MASK
 *                     - \ref QSPI_RX_FULL_MASK
 *                     - \ref QSPI_TX_EMPTY_MASK
 *                     - \ref QSPI_TX_FULL_MASK
 *                     - \ref QSPI_TXRX_RESET_MASK
 *                     - \ref QSPI_QSPIEN_STS_MASK
 *                     - \ref QSPI_SSLINE_STS_MASK
 *
 * @return     Flags of selected sources.
 *
 * @details    Get QSPI related status specified by u32Mask parameter.
 */
uint32_t QSPI_GetStatus(const QSPI_T *qspi, uint32_t u32Mask)
{
    uint32_t u32Flag = 0UL;
    uint32_t u32Status = 0UL;

    if (QSPI_IsValidModule(qspi) == 0UL)
    {
        return 0UL;
    }

    u32Status = qspi->STATUS;

    /* Check busy status */
    if (((u32Mask & QSPI_BUSY_MASK) != 0U)
            && ((u32Status & QSPI_STATUS_BUSY_Msk) != 0U))
    {
        u32Flag |= QSPI_BUSY_MASK;
    }

    /* Check RX empty flag */
    if (((u32Mask & QSPI_RX_EMPTY_MASK) != 0UL) &&
            ((u32Status & QSPI_STATUS_RXEMPTY_Msk) != 0UL))
    {
        u32Flag |= QSPI_RX_EMPTY_MASK;
    }

    /* Check RX full flag */
    if (((u32Mask & QSPI_RX_FULL_MASK) != 0UL) &&
            ((u32Status & QSPI_STATUS_RXFULL_Msk) != 0UL))
    {
        u32Flag |= QSPI_RX_FULL_MASK;
    }

    /* Check TX empty flag */
    if (((u32Mask & QSPI_TX_EMPTY_MASK) != 0UL) &&
            ((u32Status & QSPI_STATUS_TXEMPTY_Msk) != 0UL))
    {
        u32Flag |= QSPI_TX_EMPTY_MASK;
    }

    /* Check TX full flag */
    if (((u32Mask & QSPI_TX_FULL_MASK) != 0UL) &&
            ((u32Status & QSPI_STATUS_TXFULL_Msk) != 0UL))
    {
        u32Flag |= QSPI_TX_FULL_MASK;
    }

    /* Check TX/RX reset flag */
    if (((u32Mask & QSPI_TXRX_RESET_MASK) != 0UL) &&
            ((u32Status & QSPI_STATUS_TXRXRST_Msk) != 0UL))
    {
        u32Flag |= QSPI_TXRX_RESET_MASK;
    }

    /* Check QSPIEN flag */
    if (((u32Mask & QSPI_QSPIEN_STS_MASK) != 0UL) &&
            ((u32Status & QSPI_STATUS_QSPIENSTS_Msk) != 0UL))
    {
        u32Flag |= QSPI_QSPIEN_STS_MASK;
    }

    /* Check QSPIx_SS line status */
    if (((u32Mask & QSPI_SSLINE_STS_MASK) != 0UL) &&
            ((u32Status & QSPI_STATUS_SSLINE_Msk) != 0UL))
    {
        u32Flag |= QSPI_SSLINE_STS_MASK;
    }

    return u32Flag;
}

/** @} end of group QSPI_EXPORTED_FUNCTIONS */
/** @} end of group QSPI_Driver */
/** @} end of group Standard_Driver */

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
