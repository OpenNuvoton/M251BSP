/**************************************************************************//**
 * @file     fmc.c
 * @version  V3.00
 * @brief    M251 series Flash Memory Controller(FMC) driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FMC_Driver FMC Driver
  @{
*/

/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

int32_t  g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS; /*!< FMC global error code */

/**
  * @brief      Set boot source from LDROM or APROM after next software reset
  *
  * @param[in]  i32BootSrc
  *                1: Boot from LDROM
  *                0: Boot from APROM
  *
  *
  * @details   This function is used to switch APROM boot or LDROM boot. User need to call
  *            FMC_SetBootSource to select boot source first, then use CPU reset or
  *            System Reset Request to reset system.
  *
  */
void FMC_SetBootSource(int32_t i32BootSrc)
{
    FMC_SELECT_NEXT_BOOT(i32BootSrc & 0x1u);
}


/**
  * @brief    Disable ISP Functions
  *
  *
  *
  * @details  This function will clear ISPEN bit of ISPCON to disable ISP function
  *
  */
void FMC_Close(void)
{
    FMC_DISABLE_ISP();
}


/**
  * @brief    Get the current boot source
  *
  *
  * @retval   0 This chip is currently booting from APROM
  * @retval   1 This chip is currently booting from LDROM
  *
  * @note     This function only show the boot source.
  *           User need to read ISPSTA register to know if IAP mode supported or not in relative boot.
  */
int32_t FMC_GetBootSource(void)
{
    return FMC_GET_BOOT_STATUS();
}


/**
  * @brief    Enable FMC ISP function
  *
  *
  *
  * @details  ISPEN bit of ISPCON must be set before we can use ISP commands.
  *           Therefore, To use all FMC function APIs, user needs to call FMC_Open() first to enable ISP functions.
  *
  * @note     ISP functions are write-protected. user also needs to unlock it by calling SYS_UnlockReg() before using all ISP functions.
  *
  */
void FMC_Open(void)
{
    FMC_ENABLE_ISP();
}


/**
  * @brief       Read the User Configuration words.
  *
  * @param[out]  u32Config  The word buffer to store the User Configuration data.
  * @param[in]   u32Count   The word count to be read.
  *
  * @retval       0 Success
  * @retval      -1 Failed
  *
  * @details     This function is used to read the settings of user configuration.
  *              if u32Count = 1, Only CONFIG0 will be returned to the buffer specified by u32Config.
  *              if u32Count = 2, Both CONFIG0 and CONFIG1 will be returned.
  */
int32_t FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count)
{
    uint32_t i;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;

    for (i = 0u; i < u32Count; i++)
    {
        u32Config[i] = FMC_Read(FMC_CONFIG_BASE + i * 4u);
    }

    return g_FMC_i32ErrCode;
}


/**
  * @brief    Write User Configuration
  *
  * @param[in]  u32Config The word buffer to store the User Configuration data.
  * @param[in]  u32Count The word count to program to User Configuration.
  *
  * @retval     0 Success
  * @retval    -1 Failed
  *
  * @details  User must enable User Configuration update before writing it.
  *           User must erase User Configuration before writing it.
  *           User Configuration is also be page erase. User needs to backup necessary data
  *           before erase User Configuration.
  */
int32_t FMC_WriteConfig(uint32_t u32Config[], uint32_t u32Count)
{
    int32_t  ret = 0;
    uint32_t i;

    for (i = 0u; i < u32Count; i++)
    {
        FMC_Write(FMC_CONFIG_BASE + i * 4u, u32Config[i]);

        if (FMC_Read(FMC_CONFIG_BASE + i * 4u) != u32Config[i])
        {
            ret = -1;
        }
    }

    return ret;
}


/**
  * @brief      Run CRC32 checksum calculation and get result.
  *
  * @param[in]  u32addr    Starting flash address. It must be a page aligned address.
  * @param[in]  u32count   Byte count of flash to be calculated. It must be multiple of 512 bytes.
  *
  * @retval     0xFFFFFFFF  Invalid parameter or command timeout
  * @retval     Others      Checksum value of specify area
  *
  * @details    Run ISP checksum command to calculate specify area
  *
  * @note       g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT if timeout.
  *             g_FMC_i32ErrCode will be set to eFMC_ERRCODE_INVALID_PARAM if not 512 aligned.
  *
  */
uint32_t  FMC_GetChkSum(uint32_t u32addr, uint32_t u32count)
{
    return FMC_GetCheckSum(u32addr, u32count);
}


/**
  * @brief Run flash all one verification and get result.
  *
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
  *
  * @retval   READ_ALLONE_YES      The contents of verified flash area are 0xFFFFFFFF.
  * @retval   READ_ALLONE_NOT  Some contents of verified flash area are not 0xFFFFFFFF.
  * @retval   READ_ALLONE_CMD_FAIL  Unexpected error occurred.
  *
  * @details  Run ISP check all one command to check specify area is all one or not.
  *
  * @note     g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT if timeout.
  *           g_FMC_i32ErrCode will be set to eFMC_ERRCODE_INVALID_PARAM if not 512 aligned.
  *
  */
uint32_t  FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count)
{
    int32_t  i32TimeOutCnt;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;

    if ((u32addr % 512UL) || (u32count % 512UL))
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_INVALID_PARAM;
        return READ_ALLONE_CMD_FAIL;
    }

    FMC->ISPSTS |= FMC_ISPSTS_ALLONE_Msk;   /* clear check all one bit */

    FMC->ISPCMD   = FMC_ISPCMD_RUN_ALL1;
    FMC->ISPADDR  = u32addr;
    FMC->ISPDAT   = u32count;
    FMC->ISPTRG   = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_CHKALLONE;

    while ((i32TimeOutCnt-- > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) {}

    if (i32TimeOutCnt <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return READ_ALLONE_CMD_FAIL;
    }

    i32TimeOutCnt = FMC_TIMEOUT_CHKALLONE;

    do
    {
        FMC->ISPCMD  = FMC_ISPCMD_READ_ALL1;
        FMC->ISPADDR = u32addr;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        while ((i32TimeOutCnt-- > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) {}

        if (i32TimeOutCnt <= 0)
        {
            g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
            return READ_ALLONE_CMD_FAIL;
        }
    } while (FMC->ISPDAT == 0UL);

    if (FMC->ISPDAT == READ_ALLONE_YES || FMC->ISPDAT == READ_ALLONE_NOT)
    {
        return FMC->ISPDAT;
    }

    return READ_ALLONE_CMD_FAIL;
}


/**
  * @brief  Check the XOM is actived or not.
  *
  * @param[in] xom_num    The XOM number. The value will always be 0 in M251 series.
  *
  * @retval   1   XOM is actived.
  * @retval   0   XOM is not actived.
  * @retval   -2  Invalid XOM number.
  *
  * @details To get specify XOM active status
  */
int32_t FMC_Is_XOM_Actived(uint32_t xom_num)
{
    int32_t  ret = 0;

    if (xom_num >= 1UL)
    {
        ret = -2;
    }

    if (ret >= 0)
    {
        uint32_t u32act;
        u32act = (((FMC->XOMSTS) & 0xful) & (1ul << xom_num)) >> xom_num;
        ret = (int32_t)u32act;
    }

    return ret;
}


/**
  * @brief     Config XOM Region
  * @param[in] xom_num    The XOM number. The value will always be 0 in M251 series.
  * @param[in] xom_base   The XOM region base address.
  * @param[in] xom_page   The XOM page number of region size.
  *
  * @retval   0   Success
  * @retval   1   XOM is has already actived.
  * @retval   -1  Program failed.
  * @retval   -2  Invalid XOM number.
  *
  * @details  Program XOM base address and XOM size(page)
  *
  * @note     g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT if timeout.
  *           g_FMC_i32ErrCode will be set to eFMC_ERRCODE_INVALID_PARAM if wrong xom number is set.
  *           g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_FAIL if command fail.
  */
int32_t FMC_Config_XOM(uint32_t xom_num, uint32_t xom_base, uint8_t xom_page)
{
    int32_t  i32TimeOutCnt;
    int32_t  ret = 0;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;

    if (xom_num >= 1UL)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_INVALID_PARAM;
        ret = -2;
    }

    if (ret == 0)
    {
        ret = FMC_Is_XOM_Actived(xom_num);
    }

    if (ret == 0)
    {
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + (xom_num * 0x10u);
        FMC->ISPDAT  = xom_base;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while ((i32TimeOutCnt-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

        if (i32TimeOutCnt <= 0)
        {
            g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
            return -1;
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
            ret = -1;
        }
    }

    if (ret == 0)
    {
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + (xom_num * 0x10u + 0x04u);
        FMC->ISPDAT  = xom_page;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while ((i32TimeOutCnt-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

        if (i32TimeOutCnt <= 0)
        {
            g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
            return -1;
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
            ret = -1;
        }
    }

    if (ret == 0)
    {
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + (xom_num * 0x10u + 0x08u);
        FMC->ISPDAT  = 0u;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while ((i32TimeOutCnt-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

        if (i32TimeOutCnt <= 0)
        {
            g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
            return -1;
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
            ret = -1;
        }
    }

    return ret;
}


/**
  * @brief  Execute Erase XOM Region
  *
  * @param[in]  xom_num  The XOM number. The value will always be 0 in M251 series.
  *
  * @return   XOM erase success or not.
  * @retval    0  Success
  * @retval   -1  Erase failed
  * @retval   -2  Invalid XOM number.
  *
  * @details Execute FMC_ISPCMD_PAGE_ERASE command to erase XOM.
  *
  * @note    g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT if timeout.
  *          g_FMC_i32ErrCode will be set to eFMC_ERRCODE_INVALID_PARAM if wrong xom number is set.
  *          g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_FAIL if command fail.
  */
int32_t FMC_Erase_XOM(uint32_t xom_num)
{
    int32_t i32Active, err = 0;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;

    if (xom_num >= 1UL)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_INVALID_PARAM;
        err = -2;
    }

    i32Active = FMC_Is_XOM_Actived(xom_num);

    if (i32Active)
    {
        uint32_t u32Addr;
        u32Addr =  FMC->XOMR0STS0;
        FMC->ISPCMD  = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPADDR = u32Addr;
        FMC->ISPDAT  = 0x55aa03u;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
        __ISB();
#endif
        i32TimeOutCnt = FMC_TIMEOUT_ERASE;

        while ((i32TimeOutCnt-- > 0) && FMC->ISPTRG) {}

        if (i32TimeOutCnt <= 0)
        {
            g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
            err = -1;
        }

        /* Check ISPFF flag to know whether erase OK or fail. */
        if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
        {
            FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
            g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
            err = -1;
        }
    }
    else
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
        err = -1;
    }

    return err;
}


/**
 * @brief      Flash page erase
 *
 * @param[in]  u32Addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @retval      0 Success
 * @retval     -1 Erase failed
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 512 bytes.
 *
 * @note       g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT if timeout.
 *             g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_FAIL if command fail.
 *
 */
int32_t FMC_Erase(uint32_t u32Addr)
{
    int32_t  ret = 0;
    int32_t  i32TimeOutCnt;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;

    FMC->ISPCMD  = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif

    i32TimeOutCnt = FMC_TIMEOUT_ERASE;

    while ((i32TimeOutCnt-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32TimeOutCnt <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return -1;
    }

    /* Check ISPFF flag to know whether erase OK or fail. */
    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
        ret = -1;
    }

    return ret;
}


/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note        g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT if timeout.
 *
 */
uint32_t FMC_Read(uint32_t u32Addr)
{
    int32_t  i32TimeOutCnt;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD  = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT  = 0u;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif

    i32TimeOutCnt = FMC_TIMEOUT_READ;

    while ((i32TimeOutCnt-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32TimeOutCnt <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFF;
    }

    return FMC->ISPDAT;
}



/**
 * @brief      Program 32-bit data into specified address of flash
 *
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32Data  32-bit Data to program
 *
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *
 * @note       g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT if timeout.
 *             g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_FAIL if command fail.
 *
 */
void FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t  i32TimeOutCnt;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT  = u32Data;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while ((i32TimeOutCnt-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32TimeOutCnt <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
    }
}


/**
 * @brief      Program Multi-Word data into specified address of flash
 *
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  pu32Buf  A data pointer is point to a data buffer start address;
 *
 * @retval     eFMC_ERRCODE_SUCCESS: Write data success
 * @retval     eFMC_ERRCODE_INVALID_PARAM: Invalid buffer or address
 * @retval     eFMC_ERRCODE_CMD_TIMEOUT: Program timeout
 * @retval     eFMC_ERRCODE_CMD_FAIL: Program failed
 *
 * @details    To program multi-words data into Flash include APROM, LDROM, and CONFIG.
 *             The function needs to execute in SRAM.
 *
 * @note       Global error code g_FMC_i32ErrCode
 *             eFMC_ERRCODE_INVALID_PARAM: Invalid buffer or address
 *             eFMC_ERRCODE_CMD_TIMEOUT: Program timeout
 *             eFMC_ERRCODE_CMD_FAIL: Program failed
 *
 * @details    If multi-word programming is interrupted during state transition,
 *             this function restores the write pointer from MPADDR and retries from
 *             the interrupted position until the chunk is completed or an error occurs.
 *
 */
int32_t FMC_Write128(uint32_t u32Addr, uint32_t pu32Buf[])
{
    int32_t  i32TimeOutCnt;
    uint32_t i, u32BufIdx, u32OnProg;
    uint32_t u32MPStatus = 0;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;

    /* pu32Buf must be valid, u32Addr must be multiple of 16. */
    if ((pu32Buf == NULL) || (u32Addr % 16) != 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_INVALID_PARAM;
        return eFMC_ERRCODE_INVALID_PARAM;
    }

    u32BufIdx = 0;
    FMC->ISPCMD  = FMC_ISPCMD_MULTI_PROG;
    FMC->ISPADDR = u32Addr;

    do
    {
        int32_t i32ErrStatus = eFMC_ERRCODE_SUCCESS;

        u32OnProg = TRUE;
        FMC->MPDAT0 = pu32Buf[u32BufIdx + 0];
        FMC->MPDAT1 = pu32Buf[u32BufIdx + 1];
        FMC->MPDAT2 = pu32Buf[u32BufIdx + 2];
        FMC->MPDAT3 = pu32Buf[u32BufIdx + 3];
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        u32BufIdx += 4;

        /* Max data length is FMC_MULTI_WORD_PROG_LEN bytes (FMC_MULTI_WORD_PROG_LEN/4 words) */
        for (i = u32BufIdx; i < (FMC_MULTI_WORD_PROG_LEN / 4); i += 4)
        {
            i32TimeOutCnt = FMC_TIMEOUT_WRITE;

            do
            {
                if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0)
                {
                    /* ISP multi-word programming is interrupted.
                     * Resume address is derived from MPADDR and retry starts from this position.
                     */
                    FMC->ISPADDR = (FMC->MPADDR + 8) & (~0xF);
                    u32BufIdx = (FMC->ISPADDR - u32Addr) / 4;
                    i32ErrStatus = eFMC_ERRCODE_PROG_INTERRUPTED;
                    break;
                }

                if (i32TimeOutCnt-- <= 0)
                {
                    g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
                    return eFMC_ERRCODE_CMD_TIMEOUT;
                }
            } while (FMC->MPSTS & (FMC_MPSTS_D0_Msk | FMC_MPSTS_D1_Msk));

            if (i32ErrStatus == eFMC_ERRCODE_SUCCESS)
            {
                /* Update new data for D0 and D1 */
                FMC->MPDAT0 = pu32Buf[i + 0];
                FMC->MPDAT1 = pu32Buf[i + 1];
                i32TimeOutCnt = FMC_TIMEOUT_WRITE;

                do
                {
                    if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0)
                    {
                        /* ISP multi-word programming is interrupted.
                         * Resume address is derived from MPADDR and retry starts from this position.
                         */
                        FMC->ISPADDR = (FMC->MPADDR + 8) & (~0xF);
                        u32BufIdx = (FMC->ISPADDR - u32Addr) / 4;
                        i32ErrStatus = eFMC_ERRCODE_PROG_INTERRUPTED;
                        break;
                    }

                    if (i32TimeOutCnt-- <= 0)
                    {
                        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
                        return eFMC_ERRCODE_CMD_TIMEOUT;
                    }
                } while (FMC->MPSTS & (FMC_MPSTS_D2_Msk | FMC_MPSTS_D3_Msk));

                if (i32ErrStatus == eFMC_ERRCODE_SUCCESS)
                {
                    /* Update new data for D2 and D3 */
                    FMC->MPDAT2 = pu32Buf[i + 2];
                    FMC->MPDAT3 = pu32Buf[i + 3];

                    if ((i + 4) >= (FMC_MULTI_WORD_PROG_LEN / 4))
                    {
                        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

                        do
                        {
                            u32MPStatus = FMC->MPSTS;

                            if (((u32MPStatus & FMC_MPSTS_MPBUSY_Msk) == 0) && (u32MPStatus & (0xF << FMC_MPSTS_D0_Pos)))
                            {
                                /* Final in-flight data exists but MP engine is interrupted.
                                 * Recover address and continue in the next do-while iteration.
                                 */
                                FMC->ISPADDR = (FMC->MPADDR + 8) & (~0xF);
                                u32BufIdx = (FMC->ISPADDR - u32Addr) / 4;
                                i32ErrStatus = eFMC_ERRCODE_PROG_INTERRUPTED;
                                break;
                            }

                            if (i32TimeOutCnt-- <= 0)
                            {
                                g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
                                return eFMC_ERRCODE_CMD_TIMEOUT;
                            }
                        } while (u32MPStatus & (0xF << FMC_MPSTS_D0_Pos));
                    }
                }
            }

            if (i32ErrStatus != eFMC_ERRCODE_SUCCESS)
            {
                if (FMC_GET_FAIL_FLAG())
                {
                    FMC_CLR_FAIL_FLAG();
                    i32ErrStatus = eFMC_ERRCODE_CMD_FAIL;
                    g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
                }

                break;
            }
        }

        /* Only stop retry loop when this chunk is not in interrupted-retry state. */
        if (i32ErrStatus != (int32_t)eFMC_ERRCODE_PROG_INTERRUPTED)
        {
            u32OnProg = FALSE;
            i32TimeOutCnt = FMC_TIMEOUT_WRITE;

            while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
            {
                if (i32TimeOutCnt-- <= 0)
                {
                    g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
                    return eFMC_ERRCODE_CMD_TIMEOUT;
                }
            }
        }
    } while (u32OnProg);

    if (FMC_GET_FAIL_FLAG())
    {
        FMC_CLR_FAIL_FLAG();
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
        return eFMC_ERRCODE_CMD_FAIL;
    }

    return eFMC_ERRCODE_SUCCESS;
}

/** @} end of group FMC_EXPORTED_FUNCTIONS */

/** @} end of group FMC_Driver */

/** @} end of group Standard_Driver */
