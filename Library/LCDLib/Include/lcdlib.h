/**************************************************************************//**
 * @file     lcdlib.h
 * @version  V3.00
 * @brief    RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD library header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __LCDLIB_H
#define __LCDLIB_H

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Library Library
  @{
*/

/** @addtogroup LCDLIB LCD Library
  @{
*/

/** @addtogroup LCDLIB_EXPORTED_CONSTANTS LCD Library Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  Digit Zone Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define ZONE_MAIN_DIGIT             0   /*!< Main digit display zone index  */
#define ZONE_PPM_DIGIT              1   /*!< PPM or percentage digit display zone index  */
#define ZONE_TEMP_DIGIT             2   /*!< Temperature digit display zone index  */
#define ZONE_VER_DIGIT              3   /*!< Version number digit display zone index  */
#define ZONE_TIME_DIGIT             4   /*!< Time display on COM 4, SEG 10 */
#define ZONE_NUMICRO_DIGIT          5   /*!< NuMicro VER. display on COM 4, SEG 10 */

/*---------------------------------------------------------------------------------------------------------*/
/*  COM and SEG Position of Symbol Constant Definitions                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define SYMBOL_NVT              ((10)<<4 | (4)<<0)  /*!< T1 display on COM 4, SEG 10 */
#define SYMBOL_WIFI             ((10)<<4 | (5)<<0)  /*!< T2 display on COM 5, SEG 10 */
#define SYMBOL_SOUND            ((10)<<4 | (6)<<0)  /*!< T3 display on COM 6, SEG 10 */
#define SYMBOL_NUMICRO          ((9)<<4  | (3)<<0)  /*!< Y3 display on COM 3, SEG 9 */
#define SYMBOL_BAT_FRAME        ((10)<<4 | (0)<<0)  /*!< T7 display on COM 0, SEG 10 */
#define SYMBOL_BAT_1            ((10)<<4 | (2)<<0)  /*!< T4 display on COM 2, SEG 10 */
#define SYMBOL_BAT_2            ((10)<<4 | (3)<<0)  /*!< T5 display on COM 3, SEG 10 */
#define SYMBOL_BAT_3            ((10)<<4 | (1)<<0)  /*!< T6 display on COM 1, SEG 10 */
#define SYMBOL_PLUS             ((3)<<4  | (1)<<0)  /*!< T12 display on COM 1, SEG 3 */
#define SYMBOL_MINUS            ((3)<<4  | (3)<<0)  /*!< T13 display on COM 3, SEG 3 */
#define SYMBOL_V                ((39)<<4 | (4)<<0)  /*!< T26 display on COM 4, SEG 39 */
#define SYMBOL_A                ((37)<<4 | (4)<<0)  /*!< T27 display on COM 4, SEG 37 */
#define SYMBOL_W                ((35)<<4 | (4)<<0)  /*!< T28 display on COM 4, SEG 35 */
#define SYMBOL_ARROW_UP         ((1)<<4  | (4)<<0)  /*!< T29 display on COM 4, SEG 1 */
#define SYMBOL_ARROW_LEFT       ((1)<<4  | (6)<<0)  /*!< T30 display on COM 6, SEG 1 */
#define SYMBOL_ARROW_DOWN       ((0)<<4  | (7)<<0)  /*!< T31 display on COM 7, SEG 0 */
#define SYMBOL_ARROW_RIGHT      ((0)<<4  | (4)<<0)  /*!< T32 display on COM 4, SEG 0 */
#define SYMBOL_CIRCLE_UP        ((1)<<4  | (5)<<0)  /*!< T33 display on COM 5, SEG 1 */
#define SYMBOL_CIRCLE_LEFT      ((0)<<4  | (6)<<0)  /*!< T34 display on COM 6, SEG 0 */
#define SYMBOL_CIRCLE_RIGHT     ((0)<<4  | (5)<<0)  /*!< T35 display on COM 5, SEG 0 */
#define SYMBOL_PERCENTAGE       ((23)<<4 | (4)<<0)  /*!< Y2 display on COM 4, SEG 23 */
#define SYMBOL_PPM              ((21)<<4 | (4)<<0)  /*!< Y1 display on COM 4, SEG 21 */
#define SYMBOL_TEMP_C           ((25)<<4 | (4)<<0)  /*!< T37 display on COM 4, SEG 25 */
#define SYMBOL_TEMP_F           ((27)<<4 | (4)<<0)  /*!< T38 display on COM 4, SEG 27 */
#define SYMBOL_VERSION          ((31)<<4 | (4)<<0)  /*!< T41 display on COM 4, SEG 31 */
#define SYMBOL_S(x)             (((((x)-1)%5)+11)<<4 | (((x)-1)/5)<<0)  /*!< S[x] display on COM x, SEG x. [x] range is 1 ~ 40. */
#define SYMBOL_MAIN_DIG_COL1    ((16)<<4 | (1)<<0)  /*!< T14 display on COM 1, SEG 16 */
#define SYMBOL_MAIN_DIG_COL2    ((20)<<4 | (1)<<0)  /*!< T16 display on COM 1, SEG 20 */
#define SYMBOL_MAIN_DIG_COL3    ((24)<<4 | (1)<<0)  /*!< T18 display on COM 1, SEG 24 */
#define SYMBOL_MAIN_DIG_COL4    ((28)<<4 | (1)<<0)  /*!< T20 display on COM 1, SEG 28 */
#define SYMBOL_MAIN_DIG_COL5    ((32)<<4 | (1)<<0)  /*!< T22 display on COM 1, SEG 32 */
#define SYMBOL_MAIN_DIG_COL6    ((36)<<4 | (1)<<0)  /*!< T24 display on COM 1, SEG 36 */
#define SYMBOL_MAIN_DIG_P1      ((16)<<4 | (3)<<0)  /*!< T15 display on COM 3, SEG 16 */
#define SYMBOL_MAIN_DIG_P2      ((20)<<4 | (3)<<0)  /*!< T17 display on COM 3, SEG 20 */
#define SYMBOL_MAIN_DIG_P3      ((24)<<4 | (3)<<0)  /*!< T19 display on COM 3, SEG 24 */
#define SYMBOL_MAIN_DIG_P4      ((28)<<4 | (3)<<0)  /*!< T21 display on COM 3, SEG 28 */
#define SYMBOL_MAIN_DIG_P5      ((32)<<4 | (3)<<0)  /*!< T23 display on COM 3, SEG 32 */
#define SYMBOL_MAIN_DIG_P6      ((36)<<4 | (3)<<0)  /*!< T25 display on COM 3, SEG 36 */
#define SYMBOL_VER_DIG_P1       ((29)<<4 | (4)<<0)  /*!< T39 display on COM 4, SEG 29 */
#define SYMBOL_VER_DIG_P2       ((33)<<4 | (4)<<0)  /*!< T40 display on COM 4, SEG 33 */
#define SYMBOL_TIME_DIG_COL1    ((5)<<4  | (7)<<0)  /*!< T9 display on COM 7, SEG 5 */
#define SYMBOL_TIME_DIG_P1      ((3)<<4  | (7)<<0)  /*!< T8 display on COM 7, SEG 3 */
#define SYMBOL_TIME_DIG_P2      ((5)<<4  | (3)<<0)  /*!< T10 display on COM 3, SEG 5 */
#define SYMBOL_TIME_DIG_P3      ((7)<<4  | (3)<<0)  /*!< T11 display on COM 3, SEG 7 */

/*@}*/ /* end of group LCDLib_EXPORTED_CONSTANTS */


/** @addtogroup LCDLIB_EXPORTED_STRUCTS LCD Library Exported Structs
  @{
*/
typedef struct
{
    uint32_t u32DigitCnt;   /*!< Digit counts */
    uint32_t u32MaxSegNum;  /*!< Maximum segment number */
} LCD_ZONE_INFO_T;

/*@}*/ /* end of group LCDLIB_EXPORTED_STRUCTS */


/** @addtogroup LCDLIB_EXPORTED_FUNCTIONS LCD Library Exported Functions
  @{
*/

void LCDLIB_Printf(uint32_t u32Zone, char *InputStr);
void LCDLIB_PutChar(uint32_t u32Zone, uint32_t u32Index, uint8_t u8Ch);
void LCDLIB_PrintNumber(uint32_t u32Zone, uint32_t InputNum);
void LCDLIB_SetSymbol(uint32_t u32Symbol, uint32_t u32OnOff);

/*@}*/ /* end of group LCDLIB_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group LCDLIB */

/*@}*/ /* end of group Library */

#ifdef __cplusplus
}
#endif

#endif  /* __LCDLIB_H */
