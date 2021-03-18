/**************************************************************************//**
 * @file     TK_MultPinFunctions.c
 * @version  V1.00
 * @brief    Touch key Multi-Function setting.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"
#include "tklib.h"
#include "TK_Demo.h"

/**************************************************************************//**
 * TK0      PD.15
 * TK1      PA.5
 * TK2      PA.4
 * TK3      PA.3
 * TK4      PA.2
 * TK5      PA.1
 * TK6      PA.0
 * TK7      PF.15
 * TK8      PE.14
 * TK9      PC.5
 * TK10     PC.4
 * TK11     PC.3
 * TK12     PC.2
 * TK13     PD.7/3
 * TK14     PD.6/2
 * TK15     PD.5/1
 * TK16     PD.4/0
 ******************************************************************************/

/**************************************************************************//**
  * @brief      Configure multi-function pin to touch key function
  * @param[in]  u16TkMsk Combination of enabled scan keys. Each bit corresponds to a touch key.
  *             Bit 0 represents touch key 0, bit 1 represents touch key 1...
  * @retval     None.
  * @details    This function is used to configure multi-function pin to touch key function
 ******************************************************************************/
void SetTkMultiFun(uint32_t u32TkMsk)
{
    /* Avoid using the pointer to set multiple pin function registers */
    unsigned int i;

    for (i = 0; i < TKLIB_TOL_NUM_KEY; i++)
    {
        if ((1ul << i) & u32TkMsk)
        {
            switch (i)
            {
                case 0: /* HAG040 : PD.15 */
                    SYS->GPD_MFPH = (SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD15MFP_Msk)) | SYS_GPD_MFPH_PD15MFP_TK_TK0;
                    break;

                case 1: /* HAG040 : PA.5 */
                    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA5MFP_Msk)) | SYS_GPA_MFPL_PA5MFP_TK_TK1;
                    break;

                case 2: /* HAG040 : PA.4 */
                    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA4MFP_Msk)) | SYS_GPA_MFPL_PA4MFP_TK_TK2;
                    break;

                case 3: /* HAG040 : PA.3 */
                    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_TK_TK3;
                    break;

                case 4: /* HAG040 : PA.2 */
                    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_TK_TK4;
                    break;

                case 5: /* HAG040 : PA.1 */
                    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_TK_TK5;
                    break;

                case 6: /* HAG040 : PA.0 */
                    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_TK_TK6;
                    break;

                case 7: /* HAG040 : PF.15 */
                    SYS->GPF_MFPH = (SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF15MFP_Msk)) | SYS_GPF_MFPH_PF15MFP_TK_TK7;
                    break;

                case 8: /* HAG040 : PE.14 */
                    SYS->GPE_MFPH = (SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE14MFP_Msk)) | SYS_GPE_MFPH_PE14MFP_TK_TK8;
                    break;

                case 9: /* HAG040 : PC.5 */
                    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_TK_TK9;
                    break;

                case 10: /* HAG040 : PC.4 */
                    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_TK_TK10;
                    break;

                case 11: /* HAG040 : PC.3 */
                    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_TK_TK11;
                    break;

                case 12: /* HAG040 : PC.2 */
                    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk)) | SYS_GPC_MFPL_PC2MFP_TK_TK12;
                    break;

                case 13: /* HAG040 : PD.7/3 */
                    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD7MFP_Msk)) | SYS_GPD_MFPL_PD7MFP_TK_TK13;
                    break;

                case 14: /* HAG040 : PD.6/2 */
                    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD6MFP_Msk)) | SYS_GPD_MFPL_PD6MFP_TK_TK14;
                    break;

                case 15: /* HAG040 : PD.5/1 */
                    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD5MFP_Msk)) | SYS_GPD_MFPL_PD5MFP_TK_TK15;
                    break;

                case 16: /* HAG040 : PD.4/0 */
                    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD4MFP_Msk)) | SYS_GPD_MFPL_PD4MFP_TK_TK16;
                    break;

                default:
                    break;
            }
        }
    }

}