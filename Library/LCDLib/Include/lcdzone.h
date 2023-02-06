/**************************************************************************//**
 * @file     lcdzone.h
 * @version  V1.00
 * @brief    RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD zone header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __LCDZONE_H
#define __LCDZONE_H

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

/** @addtogroup LCDLIB_EXPORTED_CONSTANTS LCD Zone Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Digit Zone Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define ZONE_MAIN_DIGIT                                     0
#define ZONE_TIME_DIGIT                                     1
#define ZONE_NUMICRO_DIGIT                                  2
#define ZONE_PPM_DIGIT                                      3
#define ZONE_TEMP_DIGIT                                     4
#define ZONE_VER_DIGIT                                      5


/*---------------------------------------------------------------------------------------------------------*/
/*  COM and SEG Position of Symbol Constant Definitions                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define SYMBOL_PLUS_1                                       ((3)<<4 | (1)<<0)
#define SYMBOL_MINUS_2                                      ((3)<<4 | (3)<<0)
#define SYMBOL_S01_10                                       ((11)<<4 | (0)<<0)
#define SYMBOL_S02_10                                       ((12)<<4 | (0)<<0)
#define SYMBOL_S03_10                                       ((13)<<4 | (0)<<0)
#define SYMBOL_S04_10                                       ((14)<<4 | (0)<<0)
#define SYMBOL_S05_10                                       ((15)<<4 | (0)<<0)
#define SYMBOL_S06_10                                       ((11)<<4 | (1)<<0)
#define SYMBOL_S07_10                                       ((12)<<4 | (1)<<0)
#define SYMBOL_S08_10                                       ((13)<<4 | (1)<<0)
#define SYMBOL_S09_10                                       ((14)<<4 | (1)<<0)
#define SYMBOL_S10_10                                       ((15)<<4 | (1)<<0)
#define SYMBOL_S11_10                                       ((11)<<4 | (2)<<0)
#define SYMBOL_S12_10                                       ((12)<<4 | (2)<<0)
#define SYMBOL_S13_10                                       ((13)<<4 | (2)<<0)
#define SYMBOL_S14_10                                       ((14)<<4 | (2)<<0)
#define SYMBOL_S15_10                                       ((15)<<4 | (2)<<0)
#define SYMBOL_S16_10                                       ((11)<<4 | (3)<<0)
#define SYMBOL_S17_10                                       ((12)<<4 | (3)<<0)
#define SYMBOL_S18_10                                       ((13)<<4 | (3)<<0)
#define SYMBOL_S19_10                                       ((14)<<4 | (3)<<0)
#define SYMBOL_S20_10                                       ((15)<<4 | (3)<<0)
#define SYMBOL_S21_10                                       ((11)<<4 | (4)<<0)
#define SYMBOL_S22_10                                       ((12)<<4 | (4)<<0)
#define SYMBOL_S23_10                                       ((13)<<4 | (4)<<0)
#define SYMBOL_S24_10                                       ((14)<<4 | (4)<<0)
#define SYMBOL_S25_10                                       ((15)<<4 | (4)<<0)
#define SYMBOL_S26_10                                       ((11)<<4 | (5)<<0)
#define SYMBOL_S27_10                                       ((12)<<4 | (5)<<0)
#define SYMBOL_S28_10                                       ((13)<<4 | (5)<<0)
#define SYMBOL_S29_10                                       ((14)<<4 | (5)<<0)
#define SYMBOL_S30_10                                       ((15)<<4 | (5)<<0)
#define SYMBOL_S31_10                                       ((11)<<4 | (6)<<0)
#define SYMBOL_S32_10                                       ((12)<<4 | (6)<<0)
#define SYMBOL_S33_10                                       ((13)<<4 | (6)<<0)
#define SYMBOL_S34_10                                       ((14)<<4 | (6)<<0)
#define SYMBOL_S35_10                                       ((15)<<4 | (6)<<0)
#define SYMBOL_S36_10                                       ((11)<<4 | (7)<<0)
#define SYMBOL_S37_10                                       ((12)<<4 | (7)<<0)
#define SYMBOL_S38_10                                       ((13)<<4 | (7)<<0)
#define SYMBOL_S39_10                                       ((14)<<4 | (7)<<0)
#define SYMBOL_S40_10                                       ((15)<<4 | (7)<<0)
#define SYMBOL_V_11                                         ((39)<<4 | (4)<<0)
#define SYMBOL_A_12                                         ((37)<<4 | (4)<<0)
#define SYMBOL_W_13                                         ((35)<<4 | (4)<<0)
#define SYMBOL_NVT_14                                       ((10)<<4 | (4)<<0)
#define SYMBOL_NUMICRO_15                                   ((9)<<4 | (3)<<0)
#define SYMBOL_WIFI_16                                      ((10)<<4 | (5)<<0)
#define SYMBOL_SOUND_17                                     ((10)<<4 | (6)<<0)
#define SYMBOL_BAT_FRAME_18                                 ((10)<<4 | (0)<<0)
#define SYMBOL_BAT_1_18                                     ((10)<<4 | (2)<<0)
#define SYMBOL_BAT_2_18                                     ((10)<<4 | (3)<<0)
#define SYMBOL_BAT_3_18                                     ((10)<<4 | (1)<<0)
#define SYMBOL_ARROW_UP_26                                  ((1)<<4 | (4)<<0)
#define SYMBOL_ARROW_LEFT_26                                ((1)<<4 | (6)<<0)
#define SYMBOL_ARROW_DOWN_26                                ((0)<<4 | (7)<<0)
#define SYMBOL_ARROW_RIGHT_26                               ((0)<<4 | (4)<<0)
#define SYMBOL_CIRCLE_UP_26                                 ((1)<<4 | (5)<<0)
#define SYMBOL_CIRCLE_LEFT_26                               ((0)<<4 | (6)<<0)
#define SYMBOL_CIRCLE_RIGHT_26                              ((0)<<4 | (5)<<0)
#define SYMBOL_PERCENTAGE_30                                ((23)<<4 | (4)<<0)
#define SYMBOL_PPM_31                                       ((21)<<4 | (4)<<0)
#define SYMBOL_TEMP_C_35                                    ((25)<<4 | (4)<<0)
#define SYMBOL_TEMP_F_36                                    ((27)<<4 | (4)<<0)
#define SYMBOL_VERSION_37                                   ((31)<<4 | (4)<<0)
#define SYMBOL_P_44                                         ((3)<<4 | (7)<<0)
#define SYMBOL_P_45                                         ((5)<<4 | (3)<<0)
#define SYMBOL_P_46                                         ((7)<<4 | (3)<<0)
#define SYMBOL_COL_47                                       ((5)<<4 | (7)<<0)
#define SYMBOL_P_49                                         ((16)<<4 | (3)<<0)
#define SYMBOL_P_50                                         ((20)<<4 | (3)<<0)
#define SYMBOL_P_51                                         ((24)<<4 | (3)<<0)
#define SYMBOL_P_52                                         ((28)<<4 | (3)<<0)
#define SYMBOL_P_53                                         ((32)<<4 | (3)<<0)
#define SYMBOL_P_54                                         ((36)<<4 | (3)<<0)
#define SYMBOL_COL_55                                       ((16)<<4 | (1)<<0)
#define SYMBOL_COL_57                                       ((20)<<4 | (1)<<0)
#define SYMBOL_COL_59                                       ((24)<<4 | (1)<<0)
#define SYMBOL_COL_61                                       ((28)<<4 | (1)<<0)
#define SYMBOL_COL_63                                       ((32)<<4 | (1)<<0)
#define SYMBOL_COL_65                                       ((36)<<4 | (1)<<0)
#define SYMBOL_P_67                                         ((29)<<4 | (4)<<0)
#define SYMBOL_P_68                                         ((33)<<4 | (4)<<0)


/** @} end of group LCDLIB_EXPORTED_CONSTANTS */



/** @addtogroup LCDLIB_EXPORTED_STRUCTS LCD Zone Exported Structs
  @{
 */
typedef struct
{
    unsigned char   u8LCDDispTableNum;          /*!< LCD Display Table Number */
    unsigned char   u8GetLCDComSegNum;          /*!< LCD Com Seg Table Number */
    unsigned short  *pu16LCDDispTable;          /*!< LCD Display Table Pointer */
    unsigned char   *pu8GetLCDComSeg;           /*!< LCD Com Seg Table Pointer */

} LCD_ZONE_INFO_T;

/** @} end of group LCDLIB_EXPORTED_STRUCTS */



/** @addtogroup LCDLIB_EXPORTED_CONSTANTS LCD Zone Exported Constants
  @{
*/

#define ZONE_MAIN_DIG_CNT                                       7
#define ZONE_MAIN_SEG_NUM                                       14

#define ZONE_TIME_DIG_CNT                                       4
#define ZONE_TIME_SEG_NUM                                       7

#define ZONE_NUMICRO_DIG_CNT                                    3
#define ZONE_NUMICRO_SEG_NUM                                    7

#define ZONE_PPM_DIG_CNT                                        3
#define ZONE_PPM_SEG_NUM                                        7

#define ZONE_TEMP_DIG_CNT                                       3
#define ZONE_TEMP_SEG_NUM                                       7

#define ZONE_VER_DIG_CNT                                        6
#define ZONE_VER_SEG_NUM                                        7



/** @} end of group LCDLIB_EXPORTED_CONSTANTS */


/** @addtogroup LCDLIB_EXPORTED_STRUCTS LCD Zone Exported Structs
  @{
 */

/**************************************************************************//**
*
* Defines each text's segment (alphabet+numeric) in terms of COM and SEG numbers,
* Using this way that text segment can be consisted of each bit in the
* following bit pattern:
*
*              A
*         -----------
*         |\   |   /|
*         F G  H  I B
*         |  \ | /  |
*         --J-- --K--
*         |   /| \  |
*         E  L M  N C
*         | /  |   \|
*         -----------
*              D
*
*              0
*         -----------
*         |\   |   /|
*        5| 6  7  8 |1
*         |  \ | /  |
*         --9-- -10--
*         |   /| \  |
*        4| 11 12 13|2
*         | /  |   \|
*         -----------
*              3
*
*****************************************************************************/

static const char acMAINDigitRawData[ZONE_MAIN_DIG_CNT][ZONE_MAIN_SEG_NUM][2] =
{
    {
        {0, 1},         {0, 0},         {3, 0},         {3, 1},         {2, 3},         {0, 3},         {0, 2},         {1, 1},         {1, 0},         {1, 2},         {2, 0},         {3, 2},         {2, 2},         {2, 1},
    },
    {
        {0, 18},        {0, 19},        {3, 19},        {3, 18},        {2, 16},        {0, 16},        {0, 17},        {1, 18},        {1, 19},        {1, 17},        {2, 19},        {3, 17},        {2, 17},        {2, 18},
    },
    {
        {0, 22},        {0, 23},        {3, 23},        {3, 22},        {2, 20},        {0, 20},        {0, 21},        {1, 22},        {1, 23},        {1, 21},        {2, 23},        {3, 21},        {2, 21},        {2, 22},
    },
    {
        {0, 26},        {0, 27},        {3, 27},        {3, 26},        {2, 24},        {0, 24},        {0, 25},        {1, 26},        {1, 27},        {1, 25},        {2, 27},        {3, 25},        {2, 25},        {2, 26},
    },
    {
        {0, 30},        {0, 31},        {3, 31},        {3, 30},        {2, 28},        {0, 28},        {0, 29},        {1, 30},        {1, 31},        {1, 29},        {2, 31},        {3, 29},        {2, 29},        {2, 30},
    },
    {
        {0, 34},        {0, 35},        {3, 35},        {3, 34},        {2, 32},        {0, 32},        {0, 33},        {1, 34},        {1, 35},        {1, 33},        {2, 35},        {3, 33},        {2, 33},        {2, 34},
    },
    {
        {0, 38},        {0, 39},        {3, 39},        {3, 38},        {2, 36},        {0, 36},        {0, 37},        {1, 38},        {1, 39},        {1, 37},        {2, 39},        {3, 37},        {2, 37},        {2, 38},
    },
};

/**************************************************************************//**
*
* Defines each text's segment (numeric) in terms of COM and BIT numbers,
* Using this way that text segment can be consisted of each bit in the
* following bit pattern:
*
*         ---A---
*         |     |
*         F     B
*         |     |
*         ---G---
*         |     |
*         E     C
*         |     |
*         ---D---
*
*         ---0---
*         |     |
*         5     1
*         |     |
*         ---6---
*         |     |
*         4     2
*         |     |
*         ---3---
*
*****************************************************************************/

static const char acTIMEDigitRawData[ZONE_TIME_DIG_CNT][ZONE_TIME_SEG_NUM][2] =
{
    {
        {7, 2},         {6, 3},         {4, 3},         {4, 2},         {5, 2},         {6, 2},         {5, 3},
    },
    {
        {7, 4},         {6, 5},         {4, 5},         {4, 4},         {5, 4},         {6, 4},         {5, 5},
    },
    {
        {7, 6},         {6, 7},         {4, 7},         {4, 6},         {5, 6},         {6, 6},         {5, 7},
    },
    {
        {7, 8},         {6, 9},         {4, 9},         {4, 8},         {5, 8},         {6, 8},         {5, 9},
    },
};
static const char acNUMICRODigitRawData[ZONE_NUMICRO_DIG_CNT][ZONE_NUMICRO_SEG_NUM][2] =
{
    {
        {3, 4},         {2, 5},         {0, 5},         {0, 4},         {1, 4},         {2, 4},         {1, 5},
    },
    {
        {3, 6},         {2, 7},         {0, 7},         {0, 6},         {1, 6},         {2, 6},         {1, 7},
    },
    {
        {3, 8},         {2, 9},         {0, 9},         {0, 8},         {1, 8},         {2, 8},         {1, 9},
    },
};
static const char acPPMDigitRawData[ZONE_PPM_DIG_CNT][ZONE_PPM_SEG_NUM][2] =
{
    {
        {4, 16},        {5, 17},        {7, 17},        {7, 16},        {6, 16},        {5, 16},        {6, 17},
    },
    {
        {4, 18},        {5, 19},        {7, 19},        {7, 18},        {6, 18},        {5, 18},        {6, 19},
    },
    {
        {4, 20},        {5, 21},        {7, 21},        {7, 20},        {6, 20},        {5, 20},        {6, 21},
    },
};
static const char acTEMPDigitRawData[ZONE_TEMP_DIG_CNT][ZONE_TEMP_SEG_NUM][2] =
{
    {
        {4, 22},        {5, 23},        {7, 23},        {7, 22},        {6, 22},        {5, 22},        {6, 23},
    },
    {
        {4, 24},        {5, 25},        {7, 25},        {7, 24},        {6, 24},        {5, 24},        {6, 25},
    },
    {
        {4, 26},        {5, 27},        {7, 27},        {7, 26},        {6, 26},        {5, 26},        {6, 27},
    },
};
static const char acVERDigitRawData[ZONE_VER_DIG_CNT][ZONE_VER_SEG_NUM][2] =
{
    {
        {4, 28},        {5, 29},        {7, 29},        {7, 28},        {6, 28},        {5, 28},        {6, 29},
    },
    {
        {4, 30},        {5, 31},        {7, 31},        {7, 30},        {6, 30},        {5, 30},        {6, 31},
    },
    {
        {4, 32},        {5, 33},        {7, 33},        {7, 32},        {6, 32},        {5, 32},        {6, 33},
    },
    {
        {4, 34},        {5, 35},        {7, 35},        {7, 34},        {6, 34},        {5, 34},        {6, 35},
    },
    {
        {4, 36},        {5, 37},        {7, 37},        {7, 36},        {6, 36},        {5, 36},        {6, 37},
    },
    {
        {4, 38},        {5, 39},        {7, 39},        {7, 38},        {6, 38},        {5, 38},        {6, 39},
    },
};

/**************************************************************************//**
*
* Defines segments for the alphabet - ASCII table 0x20 to 0x7A
* Bit pattern below defined for alphabet (text segments)
*
*****************************************************************************/

static const unsigned short auMAINDigitMap[] =
{
    0x0000, /* space */
    0x1100, /* ! */
    0x0280, /* " */
    0x0000, /* # */
    0x0000, /* $ */
    0x0000, /* % */
    0x0000, /* & */
    0x0000, /* ? */
    0x0039, /* ( */
    0x000f, /* ) */
    0x3fc0, /* * */
    0x1540, /* + */
    0x0000, /* , */
    0x0440, /* - */
    0x8000, /* . */
    0x2200, /* / */

    0x003F, /* 0 */
    0x0006, /* 1 */
    0x061B, /* 2 */
    0x060F, /* 3 */
    0x0626, /* 4 */
    0x062D, /* 5 */
    0x063D, /* 6 */
    0x0007, /* 7 */
    0x063F, /* 8 */
    0x062F, /* 9 */

    0x0000, /* : */
    0x0000, /* ; */
    0x2100, /* < */
    0x0000, /* = */
    0x0840, /* > */
    0x1403, /* ? */
    0x3FFF, /* @ */

    0x0637, /* A */
    0x2339, /* B */
    0x0039, /* C */
    0x2139, /* D */
    0x0639, /* E */
    0x0631, /* F */
    0x043D, /* G */
    0x0636, /* H */
    0x1080, /* I */
    0x000E, /* J */
    0x2330, /* K */
    0x0038, /* L */
    0x0176, /* M */
    0x2076, /* N */
    0x003F, /* O */
    0x0633, /* P */
    0x203F, /* Q */
    0x2331, /* R */
    0x062D, /* S */
    0x1081, /* T */
    0x003E, /* U */
    0x0930, /* V */
    0x2836, /* W */
    0x2940, /* X */
    0x1140, /* Y */
    0x0909, /* Z */

    0x0039, /* [ */
    0x0900, /* backslash */
    0x000F, /* ] */
    0x2800, /* ^ */
    0x0008, /* _ */
    0x0040, /* ` */

    0x1218, /* a */
    0x063C, /* b */
    0x0618, /* c */
    0x061E, /* d */
    0x0A18, /* e */
    0x0231, /* f */
    0x048F, /* g */
    0x1230, /* h */
    0x1000, /* i */
    0x000E, /* j */
    0x2330, /* k */
    0x0038, /* l */
    0x1614, /* m */
    0x1404, /* n */
    0x061C, /* o */
    0x0331, /* p */
    0x0447, /* q */
    0x1400, /* r */
    0x2408, /* s */
    0x0238, /* t */
    0x1018, /* u */
    0x0810, /* v */
    0x2814, /* w */
    0x2940, /* x */
    0x0446, /* y */
    0x0A08, /* z */

    0x0000,
};

/**************************************************************************//**
* Defines segments for the numeric display
*****************************************************************************/

static const unsigned short auTIMEDigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};
static const unsigned short auNUMICRODigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};
static const unsigned short auPPMDigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};
static const unsigned short auTEMPDigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};
static const unsigned short auVERDigitMap[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
};

static const LCD_ZONE_INFO_T g_LCDZoneInfo[] =
{
    {ZONE_MAIN_DIG_CNT,         ZONE_MAIN_SEG_NUM, (unsigned short *)auMAINDigitMap, (unsigned char *)acMAINDigitRawData},
    {ZONE_TIME_DIG_CNT,         ZONE_TIME_SEG_NUM, (unsigned short *)auTIMEDigitMap, (unsigned char *)acTIMEDigitRawData},
    {ZONE_NUMICRO_DIG_CNT,      ZONE_NUMICRO_SEG_NUM, (unsigned short *)auNUMICRODigitMap, (unsigned char *)acNUMICRODigitRawData},
    {ZONE_PPM_DIG_CNT,          ZONE_PPM_SEG_NUM, (unsigned short *)auPPMDigitMap, (unsigned char *)acPPMDigitRawData},
    {ZONE_TEMP_DIG_CNT,         ZONE_TEMP_SEG_NUM, (unsigned short *)auTEMPDigitMap, (unsigned char *)acTEMPDigitRawData},
    {ZONE_VER_DIG_CNT,          ZONE_VER_SEG_NUM, (unsigned short *)auVERDigitMap, (unsigned char *)acVERDigitRawData},

};

/** @} end of group LCDLIB_EXPORTED_STRUCTS */
/** @} end of group LCDLIB */
/** @} end of group Library */

#ifdef __cplusplus
}
#endif

#endif  /* __LCDZONE_H */
