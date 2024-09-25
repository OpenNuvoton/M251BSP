/**************************************************************************//**
 * @file     lcdzone.h
 * @version  V1.00
 * @brief    RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD zone header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
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

/** @addtogroup M256D_LCDLIB_EXPORTED_CONSTANTS M256D LCD Zone Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  Digit Zone Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define ZONE_MAIN_DIGIT                                     0
#define ZONE_PPM_DIGIT                                      1
#define ZONE_TEMP_DIGIT                                     2
#define ZONE_VER_DIGIT                                      3


/*---------------------------------------------------------------------------------------------------------*/
/*  COM and SEG Position of Symbol Constant Definitions                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define SYMBOL_V_11                                     ((27)<<4 | (7)<<0)
#define SYMBOL_A_12                                     ((25)<<4 | (7)<<0)
#define SYMBOL_W_13                                     ((23)<<4 | (7)<<0)
#define SYMBOL_NVT_14                                   ((4)<<4 | (7)<<0)
#define SYMBOL_WIFI_16                                  ((4)<<4 | (6)<<0)
#define SYMBOL_SOUND_17                                 ((4)<<4 | (5)<<0)
#define SYMBOL_BAT_FRAME_18                             ((4)<<4 | (3)<<0)
#define SYMBOL_BAT_1_18                                 ((4)<<4 | (1)<<0)
#define SYMBOL_BAT_2_18                                 ((4)<<4 | (0)<<0)
#define SYMBOL_BAT_3_18                                 ((4)<<4 | (2)<<0)
#define SYMBOL_PERCENTAGE_30                            ((8)<<4 | (7)<<0)
#define SYMBOL_PPM_31                                   ((1)<<4 | (7)<<0)
#define SYMBOL_TEMP_C_35                                ((37)<<4 | (7)<<0)
#define SYMBOL_TEMP_F_36                                ((36)<<4 | (7)<<0)
#define SYMBOL_VERSION_37                               ((18)<<4 | (7)<<0)
#define SYMBOL_P_50                                     ((0)<<4 | (0)<<0)
#define SYMBOL_P_51                                     ((9)<<4 | (0)<<0)
#define SYMBOL_P_52                                     ((13)<<4 | (0)<<0)
#define SYMBOL_P_53                                     ((19)<<4 | (0)<<0)
#define SYMBOL_P_54                                     ((24)<<4 | (0)<<0)
#define SYMBOL_COL_57                                   ((0)<<4 | (2)<<0)
#define SYMBOL_COL_59                                   ((9)<<4 | (2)<<0)
#define SYMBOL_COL_61                                   ((13)<<4 | (2)<<0)
#define SYMBOL_COL_63                                   ((19)<<4 | (2)<<0)
#define SYMBOL_COL_65                                   ((24)<<4 | (2)<<0)
#define SYMBOL_P_67                                     ((14)<<4 | (7)<<0)
#define SYMBOL_P_68                                     ((20)<<4 | (7)<<0)
#define SYMBOL_COL_69                                   ((12)<<4 | (2)<<0)
#define SYMBOL_P_72                                     ((12)<<4 | (0)<<0)


/** @} end of group M256D_LCDLIB_EXPORTED_CONSTANTS */



/** @addtogroup M256D_LCDLIB_EXPORTED_STRUCTS M256D LCD Zone Exported Structs
  @{
 */
typedef struct
{
    unsigned char   u8LCDDispTableNum;          /*!< LCD Display Table Number */
    unsigned char   u8GetLCDComSegNum;          /*!< LCD Com Seg Table Number */
    unsigned short  *pu16LCDDispTable;          /*!< LCD Display Table Pointer */
    unsigned char   *pu8GetLCDComSeg;           /*!< LCD Com Seg Table Pointer */

} LCD_ZONE_INFO_T;

/** @} end of group M256D_LCDLIB_EXPORTED_STRUCTS */



/** @addtogroup M256D_LCDLIB_EXPORTED_CONSTANTS M256D LCD Zone Exported Constants
  @{
*/

#define ZONE_MAIN_DIG_CNT                                       6
#define ZONE_MAIN_SEG_NUM                                       14

#define ZONE_PPM_DIG_CNT                                        3
#define ZONE_PPM_SEG_NUM                                        7

#define ZONE_TEMP_DIG_CNT                                       3
#define ZONE_TEMP_SEG_NUM                                       7

#define ZONE_VER_DIG_CNT                                        6
#define ZONE_VER_SEG_NUM                                        7



/** @} end of group M256D_LCDLIB_EXPORTED_CONSTANTS */


/** @addtogroup M256D_LCDLIB_EXPORTED_STRUCTS M256D LCD Zone Exported Structs
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
        {3, 38},        {3, 39},        {0, 39},        {0, 38},        {1, 12},        {3, 12},        {3, 5},         {2, 38},        {2, 39},        {2, 5},         {1, 39},        {0, 5},         {1, 5},         {1, 38},
    },
    {
        {3, 7},         {3, 8},         {0, 8},         {0, 7},         {1, 0},         {3, 0},         {3, 1},         {2, 7},         {2, 8},         {2, 1},         {1, 8},         {0, 1},         {1, 1},         {1, 7},
    },
    {
        {3, 6},         {3, 36},        {0, 36},        {0, 6},         {1, 9},         {3, 9},         {3, 37},        {2, 6},         {2, 36},        {2, 37},        {1, 36},        {0, 37},        {1, 37},        {1, 6},
    },
    {
        {3, 17},        {3, 18},        {0, 18},        {0, 17},        {1, 13},        {3, 13},        {3, 14},        {2, 17},        {2, 18},        {2, 14},        {1, 18},        {0, 14},        {1, 14},        {1, 17},
    },
    {
        {3, 22},        {3, 23},        {0, 23},        {0, 22},        {1, 19},        {3, 19},        {3, 20},        {2, 22},        {2, 23},        {2, 20},        {1, 23},        {0, 20},        {1, 20},        {1, 22},
    },
    {
        {3, 26},        {3, 27},        {0, 27},        {0, 26},        {1, 24},        {3, 24},        {3, 25},        {2, 26},        {2, 27},        {2, 25},        {1, 27},        {0, 25},        {1, 25},        {1, 26},
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

static const char acPPMDigitRawData[ZONE_PPM_DIG_CNT][ZONE_PPM_SEG_NUM][2] =
{
    {
        {7, 12},        {6, 5},         {4, 5},         {4, 12},        {5, 12},        {6, 12},        {5, 5},
    },
    {
        {7, 38},        {6, 39},        {4, 39},        {4, 38},        {5, 38},        {6, 38},        {5, 39},
    },
    {
        {7, 0},         {6, 1},         {4, 1},         {4, 0},         {5, 0},         {6, 0},         {5, 1},
    },
};
static const char acTEMPDigitRawData[ZONE_TEMP_DIG_CNT][ZONE_TEMP_SEG_NUM][2] =
{
    {
        {7, 7},         {6, 8},         {4, 8},         {4, 7},         {5, 7},         {6, 7},         {5, 8},
    },
    {
        {7, 9},         {6, 37},        {4, 37},        {4, 9},         {5, 9},         {6, 9},         {5, 37},
    },
    {
        {7, 6},         {6, 36},        {4, 36},        {4, 6},         {5, 6},         {6, 6},         {5, 36},
    },
};
static const char acVERDigitRawData[ZONE_VER_DIG_CNT][ZONE_VER_SEG_NUM][2] =
{
    {
        {7, 13},        {6, 14},        {4, 14},        {4, 13},        {5, 13},        {6, 13},        {5, 14},
    },
    {
        {7, 17},        {6, 18},        {4, 18},        {4, 17},        {5, 17},        {6, 17},        {5, 18},
    },
    {
        {7, 19},        {6, 20},        {4, 20},        {4, 19},        {5, 19},        {6, 19},        {5, 20},
    },
    {
        {7, 22},        {6, 23},        {4, 23},        {4, 22},        {5, 22},        {6, 22},        {5, 23},
    },
    {
        {7, 24},        {6, 25},        {4, 25},        {4, 24},        {5, 24},        {6, 24},        {5, 25},
    },
    {
        {7, 26},        {6, 27},        {4, 27},        {4, 26},        {5, 26},        {6, 26},        {5, 27},
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
    {ZONE_PPM_DIG_CNT,          ZONE_PPM_SEG_NUM, (unsigned short *)auPPMDigitMap, (unsigned char *)acPPMDigitRawData},
    {ZONE_TEMP_DIG_CNT,         ZONE_TEMP_SEG_NUM, (unsigned short *)auTEMPDigitMap, (unsigned char *)acTEMPDigitRawData},
    {ZONE_VER_DIG_CNT,          ZONE_VER_SEG_NUM, (unsigned short *)auVERDigitMap, (unsigned char *)acVERDigitRawData},

};

/** @} end of group M256D_LCDLIB_EXPORTED_STRUCTS */
/** @} end of group LCDLIB */
/** @} end of group Library */

#ifdef __cplusplus
}

#endif

#endif  /* __LCDZONE_H */
