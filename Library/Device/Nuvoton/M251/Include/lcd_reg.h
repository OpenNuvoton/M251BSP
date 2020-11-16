/**************************************************************************//**
 * @file     lcd_reg.h
 * @version  V1.00
 * @brief    LCD register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __LCD_REG_H__
#define __LCD_REG_H__


/** @addtogroup REGISTER Control Register

  @{

*/


/*---------------------- Liquid-Crystal Display -------------------------*/
/**
    @addtogroup LCD Liquid-Crystal Display(LCD)
    Memory Mapped Structure for LCD Controller
@{ */

typedef struct
{


    /**
     * @var LCD_T::CTL
     * Offset: 0x00  LCD Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EN        |LCD Display Enable Bit
     * |        |          |This field is used to select the bias voltage level of LCD controller.
     * |        |          |0 = LCD display function disabled.
     * |        |          |1 = LCD display function enabled.
     * |        |          |Note 1: When software writes 1 to this bit, the LCD Controller needs some synchronizing time to completely enable the LCD display function. Before that,
     * |        |          |the read value of this bit is still 0.
     * |        |          |Note 2: When software writes 0 to this bit, the LCD Controller needs some synchronizing time to completely disable the LCD display function. Before that,
     * |        |          |the read value of this bit is still 1.
     * |[31]    |SYNC      |LCD Enable/Disable Synchronizing Indicator (Read Only)
     * |        |          |When software writes 0/1 to EN bit (LCD_CTL[0]), the LCD Controller needs some synchronizing time to completely disable/enable the LCD display function.
     * |        |          |During this time, this bit keeps at 1.
     * |        |          |0 = LCD display function is completely disabled/enabled.
     * |        |          |1 = LCD display function is not yet completely disabled/enabled.
     * |        |          |Note 1: The synchronizing time to enable LCD display function is not constant. It is between one and two cycles of CLKLCD.
     * |        |          |Note 2: The LCD display function cannot be disabled until the end of a frame. So the maximum synchronizing time to disable LCD display function could be as long as one frame time.
     * @var LCD_T::PCTL
     * Offset: 0x04  LCD Panel Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |BIAS      |LCD Bias Level Selection
     * |        |          |This field is used to select the bias level.
     * |        |          |0 = Reserved.
     * |        |          |1 = 1/2 Bias.
     * |        |          |2 = 1/3 Bias.
     * |        |          |3 = 1/4 Bias.
     * |[4:2]   |DUTY      |LCD Duty Ratio Selection
     * |        |          |This field is used to select the duty ratio.
     * |        |          |0 = 1/1 Duty.
     * |        |          |1 = 1/2 Duty.
     * |        |          |2 = 1/3 Duty.
     * |        |          |3 = 1/4 Duty.
     * |        |          |4 = 1/5 Duty.
     * |        |          |5 = 1/6 Duty.
     * |        |          |6 = 1/7 Duty.
     * |        |          |7 = 1/8 Duty.
     * |[5]     |TYPE      |LCD Waveform Type Selection
     * |        |          |This bit is used to select the waveform type.
     * |        |          |0 = Type A.
     * |        |          |1 = Type B.
     * |[6]     |INV       |LCD Waveform Inverse
     * |        |          |This bit is used to set the inverse LCD waveform.
     * |        |          |0 = COM/SEG waveform is normal.
     * |        |          |1 = COM/SEG waveform is inverse.
     * |[17:8]  |FREQDIV   |LCD Operating Frequency (FLCD) Divider
     * |        |          |The field is used to divide CLKLCD to generate the LCD operating frequency.
     * |        |          |LCD Operating Frequency = (CLKLCD Frequency) / (FRRQDIV + 1).
     * |        |          |Note 1: FREQDIV can be set from 0 to 1023, therefore,
     * |        |          |the fastest LCD operating frequency is equal to CLKLCD frequency, and
     * |        |          |the lowest LCD operating frequency is equal to CLKLCD frequency divided by 1024.
     * |        |          |Note 2: LCD frame rate is (LCD Operating Frequency) x (Duty Ratio) x 1/2 for type A waveform, and
     * |        |          |(LCD Operating Frequency) x (Duty Ratio) for type B waveform.
     * |[21:18] |VSEL      |LCD Operating Voltage (VLCD) Select (For Charge Pump Only)
     * |        |          |This field is used to select the LCD operating voltage.
     * |        |          |0 = 3.0V
     * |        |          |1 = 3.2V
     * |        |          |2 = 3.4V
     * |        |          |3 = 3.6V
     * |        |          |4 = 3.8V
     * |        |          |5 = 4.0V
     * |        |          |6 = 4.2V
     * |        |          |7 = 4.4V
     * |        |          |8 = 4.6V
     * |        |          |9 = 4.8V
     * |        |          |10 = 5.0V
     * |        |          |11 = 5.2V
     * |        |          |Others =  (Reserved)
     * |        |          |Note: This field is meaningful only if the VLCD source is the charge pump. Otherwise, this field is ignored.
     * |[27:24] |VTUNE     |LCD Operating Voltage (VLCD) Fine Tuning (For Charge Pump Only)
     * |        |          |This field is used to fine tune the LCD operating voltage.
     * |        |          |0 = No tuning
     * |        |          |1 = decrease  by 1 unit of voltage
     * |        |          |2 = decrease  by 2 unit of voltage
     * |        |          |3 = decrease  by 3 unit of voltage
     * |        |          |4 = decrease   by 4 unit of voltage
     * |        |          |5 = decrease  by 5 unit of voltage
     * |        |          |6 = decrease  by 6 unit of voltage
     * |        |          |7 = decrease  by 7 unit of voltage
     * |        |          |8 = increase by 8 unit of voltage
     * |        |          |9 = increase by 7 unit of voltage
     * |        |          |10 = increase by 6 unit of voltage
     * |        |          |11 = increase by 5 unit of voltage
     * |        |          |12 = increase by 4 unit of voltage
     * |        |          |13 = increase by 3 unit of voltage
     * |        |          |14 = increase by 2 unit of voltage
     * |        |          |15 = increase by 1 unit of voltage
     * |        |          |Note 1: a unit of voltage is about 0.04 V.
     * |        |          |Note 2: This field is meaningful only if the VLCD source is the charge pump. Otherwise, this field is ignored.
     * @var LCD_T::FCTL
     * Offset: 0x08  LCD Frame Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BLINK     |LCD Blinking Enable Bit
     * |        |          |0 = LCD blinking function Disabled.
     * |        |          |1 = LCD blinking function Enabled.
     * |[17:8]  |FVC       |Frame Counting Value
     * |        |          |This field indicates the maximum value that the frame counter can reach.
     * |        |          |Note 1: The frame counter automatically increases by 1 at the end of every frame. When the counter reaches FCV,
     * |        |          |it will recounts from 0 at the end of the next frame. At this moment, the hardware sets a dedicated flag to 1, and triggers a dedicated interrupt if it is enabled.
     * |        |          |Note 2: For type B waveform, the frame counter increases at the end of odd frames, not even frames.
     * |[27:24] |NFTIME    |Null Frame Time
     * |        |          |This field is used to set up the length of a null frame.
     * |        |          |0 = No null frame inserted.
     * |        |          |N = (1 / FLCD) x N (N<=15)
     * |        |          |Note: All COM and SEG output voltages are 0 V during a null frame.
     * |[31:28] |NFNUM     |Number of Frames Inserted By One Null Frame
     * |        |          |This field is used to set up the number of continuous normal frames inserted by one null frame.
     * |        |          |N = N+1 Frames (N<=15)
     * @var LCD_T::DCTL
     * Offset: 0x0C  LCD Driving Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |VSRC      |LCD Operating Voltage (VLCD) Source
     * |        |          |0 = VLCD Power
     * |        |          |1 = AVDD Power
     * |        |          |2 = Built-In Charge Pump
     * |        |          |3 = (None)
     * |        |          |Note: Whenever the LCD controller is disabled, all VLCD sources are automatically cut off.
     * |[2]     |RESMODE   |Resistive Network Driving Mode
     * |        |          |0 = Low-Drive Mode
     * |        |          |1 = High-Drive Mode
     * |[3]     |BUFEN     |Voltage Buffer Enable Bit
     * |        |          |0 = Voltage Buffer Disabled
     * |        |          |1 = Voltage Buffer Enabled
     * |        |          |Note: When RES_MODE = 1, the voltage buffers are automatically disabled. The setting of BUF_EN bit is ignored.
     * |[4]     |PSVEN     |Power Saving Mode Enable Bit
     * |        |          |0 = Power Saving Mode Disabled
     * |        |          |1 = Power Saving Mode Enabled
     * |        |          |Note: when RES_MODE = 0 and BUF_EN = 0, the output drivers consumes the least driving current.
     * |        |          |In this case, the power saving mode is automatically disabled. The setting of PSV_EN bit is ignored.
     * |[5]     |PSVREV    |Power Saving Timing Reverse
     * |        |          |0 = Timing of power saving is normal
     * |        |          |1 = Timing of power saving is reversed
     * |        |          |When the timing is reversed,
     * |        |          |the original power-saving period becomes no-power-saving, and
     * |        |          |the original no-power-saving period becomes power-saving.
     * |[11:8]  |PSVT1     |Power Saving “Enable Time” Setting
     * |        |          |The “Enable Time” of the power saving mode is calculated as “Enable Time” = 15.26 us x (PSV_T1 + 1),
     * |        |          |where 15.26 us is the half-cycle time of CLKLCD, whose frequency is about 32 kHz.
     * |        |          |PSV_T1 can be set as 0, 1, 2, …, 15, so
     * |        |          |the minimum “Enable Time” is about 15.26 us, and
     * |        |          |the maximum “Enable Time” is about 15.26 x 16 = 244.14 us.
     * |        |          |Note: In the following two cases, the power saving mode is disabled. The setting of PSV_T1 bits is ignored.
     * |        |          |1. SV_EN = 0
     * |        |          |2. ES_MODE = 0 and BUF_EN = 0 (In this case, SV_EN is ignored)
     * |[15:12] |PSVT2     |Power Saving “On Time” Setting
     * |        |          |The “On Time” of the power saving mode is calculated as “On Time” = 15.26 us x (PSV_T2 + 1),
     * |        |          |where 15.26 us is the half-cycle time of CLKLCD, whose frequency is about 32 kHz.
     * |        |          |PSV_T2 can be set as 0, 1, 2, …, 15, so
     * |        |          |the minimum “Enable Time” is about 15.26 us, and
     * |        |          |the maximum “Enable Time” is about 15.26 x 16 = 244.14 us.
     * |        |          |Note: In the following two cases, the power saving mode is disabled. The setting of PSV_T2 bits is ignored.
     * |        |          |1. SV_EN = 0
     * |        |          |2. ES_MODE = 0 and BUF_EN = 0 (In this case, SV_EN is ignored)
     * |[28:16] |CTOUT     |Charging Timer TimeOut
     * |        |          |This field is used to specify the timeout value for the charging timer. When the charging timer reaches this timeout value, a status bit or an interrupt will occur.
     * |        |          |The timeout is calculated by the following formula:
     * |        |          |Timeout = 30.52 us x (CTOUT + 1),
     * |        |          |where 30.52 us is the cycle time of CLKLCD, whose frequency is about 32 kHz.
     * |        |          |CTOUT can be set as 0, 1, 2,…, 8191, so
     * |        |          |the minimum timeout is 30.52 us, and
     * |        |          |the maximum timeout is 30.52 x 8192 = 256 ms.
     * @var LCD_T::OCTL
     * Offset: 0x10  LCD Output Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SEL8      |LCD8 Output Select
     * |        |          |0 = LCD8 is SEG43.
     * |        |          |1 = LCD8 is COM4.
     * |[1]     |SEL9      |LCD9 Output Select
     * |        |          |0 = LCD9 is SEG42.
     * |        |          |1 = LCD9 is COM5.
     * |[2]     |SEL10     |LCD10 Output Select
     * |        |          |0 = LCD10 is SEG20.
     * |        |          |1 = LCD10 is COM0.
     * |[3]     |SEL11     |LCD11 Output Select
     * |        |          |0 = LCD11 is SEG19.
     * |        |          |1 = LCD11 is COM1.
     * |[4]     |SEL12     |LCD12 Output Select
     * |        |          |0 = LCD12 is SEG18.
     * |        |          |1 = LCD12 is COM2.
     * |[5]     |SEL13     |LCD13 Output Select
     * |        |          |0 = LCD13 is SEG17.
     * |        |          |1 = LCD13 is COM3.
     * |[6]     |SEL14     |LCD14 Output Select
     * |        |          |0 = LCD14 is SEG41.
     * |        |          |1 = LCD14 is COM6.
     * |[7]     |SEL15     |LCD15 Output Select
     * |        |          |0 = LCD15 is SEG40.
     * |        |          |1 = LCD15 is COM7.
     * |[8]     |SEL24     |LCD24 Output Select
     * |        |          |0 = LCD24 is SEG31.
     * |        |          |1 = LCD24 is COM4.
     * |[9]     |SEL25     |LCD25 Output Select
     * |        |          |0 = LCD25 is SEG30.
     * |        |          |1 = LCD25 is COM5.
     * |[10]    |SEL26     |LCD26 Output Select
     * |        |          |0 = LCD26 is SEG29.
     * |        |          |1 = LCD26 is COM6.
     * |[11]    |SEL27     |LCD27 Output Select
     * |        |          |0 = LCD27 is SEG28.
     * |        |          |1 = LCD27 is COM7.
     * |[12]    |SEL28     |LCD28 Output Select
     * |        |          |0 = LCD28 is SEG27.
     * |        |          |1 = LCD28 is COM2.
     * |[13]    |SEL29     |LCD29 Output Select
     * |        |          |0 = LCD29 is SEG26.
     * |        |          |1 = LCD29 is COM3.
     * |[15:14] |SEL35     |LCD35 Output Select
     * |        |          |00 = LCD35 is COM4.
     * |        |          |01 = LCD35 is SEG20.
     * |        |          |10 = LCD35 is SEG47.
     * |[17:16] |SEL36     |LCD36 Output Select
     * |        |          |00 = LCD36 is COM5.
     * |        |          |01 = LCD36 is SEG19.
     * |        |          |10 = LCD36 is SEG46.
     * |[19:18] |SEL37     |LCD37 Output Select
     * |        |          |00 = LCD37 is COM6.
     * |        |          |01 = LCD37 is SEG18.
     * |        |          |10 = LCD37 is SEG45.
     * |[21:20] |SEL38     |LCD38 Output Select
     * |        |          |00 = LCD38 is COM7.
     * |        |          |01 = LCD38 is SEG17.
     * |        |          |10 = LCD38 is SEG44.
     * |[22]    |SEL41     |LCD41 Output Select
     * |        |          |0 = LCD41 is SEG14.
     * |        |          |1 = LCD41 is COM0.
     * |[23]    |SEL42     |LCD42 Output Select
     * |        |          |0 = LCD42 is SEG13.
     * |        |          |1 = LCD42 is COM1.
     * |[24]    |SEL47     |LCD47 Output Select
     * |        |          |0 = LCD47 is SEG08.
     * |        |          |1 = LCD47 is LCD_V1.
     * |[25]    |SEL48     |LCD48 Output Select
     * |        |          |0 = LCD48 is SEG07.
     * |        |          |1 = LCD48 is LCD_V2.
     * |[26]    |SEL49     |LCD48 Output Select
     * |        |          |0 = LCD49 is SEG06.
     * |        |          |1 = LCD49 is LCD_V3.
     * @var LCD_T::STS
     * Offset: 0x14  LCD Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FCEND     |End of Frame-Counting Flag
     * |        |          |This flag is automatically set by hardware at the end of a frame, and the frame counter value must be equal to FCV (LCD_FSET[17:8], Frame Counting Value).
     * |        |          |0 = End of Frame-Counting did not occur.
     * |        |          |1 = End of Frame-Counting occurred.
     * |        |          |Note 1: Software can clear this bit by writing 1 to it.
     * |        |          |Note 2: For type B waveform, this flag is set only at the end of an odd frame.
     * |[1]     |FEND      |End of Frame Flag
     * |        |          |This flag is automatically set by hardware at the end of a frame.
     * |        |          |0 = End of Frame did not occur.
     * |        |          |1 = End of Frame occurred.
     * |        |          |Note 1: Software can clear this bit by writing 1 to it.
     * |        |          |Note 2: For type B waveform, this flag is set only at the end of an odd frame.
     * |[2]     |CTOUT     |Charging Timeout Flag
     * |        |          |This flag is automatically set by hardware when the charging timer reaches the timeout value.
     * |        |          |0 = Charging Timeout did not occur.
     * |        |          |1 = Charging Timeout occurred.
     * |        |          |Note: Software can clear this bit by writing 1 to it.
     * |[28:16] |CTIME     |Charging Timer Value (Read Only)
     * |        |          |The field contains the value of the charging timer. It records the charging time of the charge pump.
     * |        |          |The charging timer stops counting when the charge pump stops charging or a timeout occurs. At this moment, the hardware dumps the current charging timer value into this field.
     * |        |          |Charging Time = 30.52 us x (CTIME + 1), where 30.52 us is the cycle time of CLKLCD, whose frequency is about 32 kHz.
     * @var LCD_T::INTEN
     * Offset: 0x18  LCD Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FCEND     |End of Frame-Counting Interrupt Enable Bit
     * |        |          |An interrupt occurs at the end of a frame, and the frame counter value must be equal to FCV (LCD_FSET[17:8], Frame Counting Value).
     * |        |          |0 = End of Frame-Counting Interrupt Disabled
     * |        |          |1 = End of Frame-Counting Interrupt Enabled
     * |        |          |Note: For type B waveform, the interrupt occurs only at the end of an odd frame.
     * |[1]     |FEND      |End of Frame Interrupt Enable Bit
     * |        |          |An interrupt occurs at the end of a frame.
     * |        |          |0 = End of Frame Interrupt Disabled
     * |        |          |1 = End of Frame Interrupt Enabled
     * |        |          |Note: For type B waveform, the interrupt occurs only at the end of an odd frame.
     * |[3]     |CTOUT     |Charging Timeout Interrupt Enable Bit
     * |        |          |An interrupt occurs when the charging timer reaches the timeout value.
     * |        |          |0 = Charging Timeout Interrupt Disabled
     * |        |          |1 = Charging Timeout Interrupt Enabled
     * @var LCD_T::DATA[12]
     * Offset: 0x20~0x4C  LCD Segment Display Data Register 0~11
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DD0       |Display Data of Segments S, where S is (4 x N) + 0, and N is 0, 1, 2, …, 11
     * |        |          |Each bit specifies the brightness of each pixel in a segment.
     * |        |          |0 = the pixel is light.
     * |        |          |1 = the pixel is dark.
     * |        |          |Note 1: DD0 corresponds to SEG00, SEG04, SEG08, SEG12, SEG16, SEG20, SEG24, SEG28, SEG32, SEG36, SEG40, and SEG44.
     * |        |          |Note 2: Each bit, DD0[n], corresponds to COMn, n = 0 – 7.
     * |[15:8]  |DD1       |Display Data of Segments S, where S is (4 x N) + 1, and N is 0, 1, 2, …, 11
     * |        |          |Each bit specifies the brightness of each pixel in a segment.
     * |        |          |0 = the pixel is light.
     * |        |          |1 = the pixel is dark.
     * |        |          |Note 1: DD1 corresponds to SEG01, SEG05, SEG09, SEG13, SEG17, SEG21, SEG25, SEG29, SEG33, SEG37, SEG41, and SEG45.
     * |        |          |Note 2: Each bit, DD1[n], corresponds to COMn, n = 0 – 7.
     * |[23:16] |DD2       |Display Data of Segments S, where S is (4 x N) + 2, and N is 0, 1, 2, …, 11
     * |        |          |Each bit specifies the brightness of each pixel in a segment.
     * |        |          |0 = the pixel is light.
     * |        |          |1 = the pixel is dark.
     * |        |          |Note 1: DD2 corresponds to SEG02, SEG06, SEG10, SEG14, SEG18, SEG22, SEG26, SEG30, SEG34, SEG38, SEG42, and SEG46.
     * |        |          |Note 2: Each bit, DD2[n], corresponds to COMn, n = 0 – 7.
     * |[31:24] |DD3       |Display Data of Segments S, where S is (4 x N) + 3, and N is 0, 1, 2, …, 11
     * |        |          |Each bit specifies the brightness of each pixel in a segment.
     * |        |          |0 = the pixel is light.
     * |        |          |1 = the pixel is dark.
     * |        |          |Note 1: DD3 corresponds to SEG03, SEG07, SEG11, SEG15, SEG19, SEG23, SEG27, SEG31, SEG35, SEG39, SEG43, and SEG47.
     * |        |          |Note 2: Each bit, DD3[n], corresponds to COMn, n = 0 – 7.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] LCD Control Register                                            */
    __IO uint32_t PCTL;                  /*!< [0x0004] LCD Panel Control Register                                      */
    __IO uint32_t FCTL;                  /*!< [0x0008] LCD Frame Control Register                                      */
    __IO uint32_t DCTL;                  /*!< [0x000C] LCD Driving Control Register                                    */
    __IO uint32_t OCTL;                  /*!< [0x0010] LCD Output Control Register                                     */
    __IO uint32_t STS;                   /*!< [0x0014] LCD Status Register                                             */
    __IO uint32_t INTEN;                 /*!< [0x0018] LCD Interrupt Enable Register                                   */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0;
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t DATA[12];              /*!< [0x0020- 0x004C] LCD Segment Display Data Register 0-11                  */
} LCD_T;

/**
    @addtogroup LCD_CONST LCD Bit Field Definition
    Constant Definitions for LCD Controller
@{ */

#define LCD_CTL_EN_Pos                   (0)                                               /*!< LCD_T::CTL: EN Position                */
#define LCD_CTL_EN_Msk                   (0x1ul << LCD_CTL_EN_Pos)                         /*!< LCD_T::CTL: EN Mask                    */

#define LCD_CTL_SYNC_Pos                 (31)                                              /*!< LCD_T::CTL: SYNC Position              */
#define LCD_CTL_SYNC_Msk                 (0x1ul << LCD_CTL_SYNC_Pos)                       /*!< LCD_T::CTL: SYNC Mask                  */

#define LCD_PCTL_BIAS_Pos                (0)                                               /*!< LCD_T::PCTL: BIAS Position             */
#define LCD_PCTL_BIAS_Msk                (0x3ul << LCD_PCTL_BIAS_Pos)                      /*!< LCD_T::PCTL: BIAS Mask                 */

#define LCD_PCTL_DUTY_Pos                (2)                                               /*!< LCD_T::PCTL: DUTY Position             */
#define LCD_PCTL_DUTY_Msk                (0x7ul << LCD_PCTL_DUTY_Pos)                      /*!< LCD_T::PCTL: DUTY Mask                 */

#define LCD_PCTL_TYPE_Pos                (5)                                               /*!< LCD_T::PCTL: TYPE Position             */
#define LCD_PCTL_TYPE_Msk                (0x1ul << LCD_PCTL_TYPE_Pos)                      /*!< LCD_T::PCTL: TYPE Mask                 */

#define LCD_PCTL_INV_Pos                 (6)                                               /*!< LCD_T::PCTL: INV Position              */
#define LCD_PCTL_INV_Msk                 (0x1ul << LCD_PCTL_INV_Pos)                       /*!< LCD_T::PCTL: INV Mask                  */

#define LCD_PCTL_FREQDIV_Pos             (8)                                               /*!< LCD_T::PCTL: FREQDIV Position          */
#define LCD_PCTL_FREQDIV_Msk             (0x3fful << LCD_PCTL_FREQDIV_Pos)                 /*!< LCD_T::PCTL: FREQDIV Mask              */

#define LCD_PCTL_VSEL_Pos                (18)                                              /*!< LCD_T::PCTL: VSEL Position             */
#define LCD_PCTL_VSEL_Msk                (0xful << LCD_PCTL_VSEL_Pos)                      /*!< LCD_T::PCTL: VESL Mask                 */

#define LCD_PCTL_VTUNE_Pos               (24)                                              /*!< LCD_T::PCTL: VTUNE Position            */
#define LCD_PCTL_VTUNE_Msk               (0xful << LCD_PCTL_VTUNE_Pos)                     /*!< LCD_T::PCTL: VTUNE Mask                */

#define LCD_FCTL_BLINK_Pos               (0)                                               /*!< LCD_T::FCTL: BLINK Position            */
#define LCD_FCTL_BLINK_Msk               (0x1ul << LCD_FCTL_BLINK_Pos)                     /*!< LCD_T::FCTL: BLINK Mask                */

#define LCD_FCTL_FCV_Pos                 (8)                                               /*!< LCD_T::FCTL: FCV Position              */
#define LCD_FCTL_FCV_Msk                 (0x3fful << LCD_FCTL_FCV_Pos)                     /*!< LCD_T::FCTL: FCV Mask                  */

#define LCD_FCTL_NFTIME_Pos              (24)                                              /*!< LCD_T::FCTL: NFTIME Position           */
#define LCD_FCTL_NFTIME_Msk              (0xf << LCD_FCTL_NFTIME_Pos)                      /*!< LCD_T::FCTL: NFTIME Mask               */

#define LCD_FCTL_NFNUM_Pos               (28)                                              /*!< LCD_T::FCTL: NFNUM Position            */
#define LCD_FCTL_NFNUM_Msk               (0xf << LCD_FCTL_NFNUM_Pos)                       /*!< LCD_T::FCTL: NFNUM Mask                */

#define LCD_DCTL_VSRC_Pos                (0)                                               /*!< LCD_T::DCTL: VSRC Position             */
#define LCD_DCTL_VSRC_Msk                (0x3ul << LCD_DCTL_VSRC_Pos)                      /*!< LCD_T::DCTL: VSRC Mask                 */

#define LCD_DCTL_RESMODE_Pos             (2)                                               /*!< LCD_T::DCTL: RESMODE Position          */
#define LCD_DCTL_RESMODE_Msk             (0x1 << LCD_DCTL_RESMODE_Pos)                     /*!< LCD_T::DCTL: RESMODE Mask              */

#define LCD_DCTL_BUFEN_Pos               (3)                                               /*!< LCD_T::DCTL: BUFEN Position            */
#define LCD_DCTL_BUFEN_Msk               (0x1 << LCD_DCTL_BUFEN_Pos)                       /*!< LCD_T::DCTL: BUFEN Mask                */

#define LCD_DCTL_PSVEN_Pos               (4)                                               /*!< LCD_T::DCTL: PSVEN Position            */
#define LCD_DCTL_PSVEN_Msk               (0x1 << LCD_DCTL_PSVEN_Pos)                       /*!< LCD_T::DCTL: PSVEN Mask                */

#define LCD_DCTL_PSVREV_Pos              (5)                                               /*!< LCD_T::DCTL: PSVREV Position           */
#define LCD_DCTL_PSVREV_Msk              (0x1 << LCD_DCTL_PSVREV_Pos)                      /*!< LCD_T::DCTL: PSVREV Mask               */

#define LCD_DCTL_PSVT1_Pos               (8)                                               /*!< LCD_T::DCTL: PSVT1 Position            */
#define LCD_DCTL_PSVT1_Msk               (0xF << LCD_DCTL_PSVT1_Pos)                       /*!< LCD_T::DCTL: PSVT1 Mask                */

#define LCD_DCTL_PSVT2_Pos               (12)                                              /*!< LCD_T::DCTL: PSVT2 Position            */
#define LCD_DCTL_PSVT2_Msk               (0xF << LCD_DCTL_PSVT2_Pos)                       /*!< LCD_T::DCTL: PSVT2 Mask                */

#define LCD_DCTL_CTOUT_Pos               (16)                                              /*!< LCD_T::DCTL: CTOUT Position            */
#define LCD_DCTL_CTOUT_Msk               (0x1fff << LCD_DCTL_CTOUT_Pos)                    /*!< LCD_T::DCTL: CTOUT Mask                */

#define LCD_OCTL_SEL8_Pos                (0)                                               /*!< LCD_T::OCTL: SEL8 Position             */
#define LCD_OCTL_SEL8_Msk                (0x1ul << LCD_OCTL_SEL8_Pos)                      /*!< LCD_T::OCTL: SEL8 Mask                 */

#define LCD_OCTL_SEL9_Pos                (1)                                               /*!< LCD_T::OCTL: SEL9 Position             */
#define LCD_OCTL_SEL9_Msk                (0x1ul << LCD_OCTL_SEL9_Pos)                      /*!< LCD_T::OCTL: SEL9 Mask                 */

#define LCD_OCTL_SEL10_Pos               (2)                                               /*!< LCD_T::OCTL: SEL10 Position            */
#define LCD_OCTL_SEL10_Msk               (0x1ul << LCD_OCTL_SEL10_Pos)                     /*!< LCD_T::OCTL: SEL10 Mask                */

#define LCD_OCTL_SEL11_Pos               (3)                                               /*!< LCD_T::OCTL: SEL11 Position            */
#define LCD_OCTL_SEL11_Msk               (0x1ul << LCD_OCTL_SEL11_Pos)                     /*!< LCD_T::OCTL: SEL11 Mask                */

#define LCD_OCTL_SEL12_Pos               (4)                                               /*!< LCD_T::OCTL: SEL12 Position            */
#define LCD_OCTL_SEL12_Msk               (0x1ul << LCD_OCTL_SEL12_Pos)                     /*!< LCD_T::OCTL: SEL12 Mask                */

#define LCD_OCTL_SEL13_Pos               (5)                                               /*!< LCD_T::OCTL: SEL13 Position            */
#define LCD_OCTL_SEL13_Msk               (0x1ul << LCD_OCTL_SEL13_Pos)                     /*!< LCD_T::OCTL: SEL13 Mask                */

#define LCD_OCTL_SEL14_Pos               (6)                                               /*!< LCD_T::OCTL: SEL14 Position            */
#define LCD_OCTL_SEL14_Msk               (0x1ul << LCD_OCTL_SEL14_Pos)                     /*!< LCD_T::OCTL: SEL14 Mask                */

#define LCD_OCTL_SEL15_Pos               (7)                                               /*!< LCD_T::OCTL: SEL15 Position            */
#define LCD_OCTL_SEL15_Msk               (0x1ul << LCD_OCTL_SEL15_Pos)                     /*!< LCD_T::OCTL: SEL15 Mask                */

#define LCD_OCTL_SEL24_Pos               (8)                                               /*!< LCD_T::OCTL: SEL24 Position            */
#define LCD_OCTL_SEL24_Msk               (0x1ul << LCD_OCTL_SEL24_Pos)                     /*!< LCD_T::OCTL: SEL24 Mask                */

#define LCD_OCTL_SEL25_Pos               (9)                                               /*!< LCD_T::OCTL: SEL25 Position            */
#define LCD_OCTL_SEL25_Msk               (0x1ul << LCD_OCTL_SEL25_Pos)                     /*!< LCD_T::OCTL: SEL25 Mask                */

#define LCD_OCTL_SEL26_Pos               (10)                                              /*!< LCD_T::OCTL: SEL26 Position            */
#define LCD_OCTL_SEL26_Msk               (0x1ul << LCD_OCTL_SEL26_Pos)                     /*!< LCD_T::OCTL: SEL26 Mask                */

#define LCD_OCTL_SEL27_Pos               (11)                                              /*!< LCD_T::OCTL: SEL27 Position            */
#define LCD_OCTL_SEL27_Msk               (0x1ul << LCD_OCTL_SEL27_Pos)                     /*!< LCD_T::OCTL: SEL27 Mask                */

#define LCD_OCTL_SEL28_Pos               (12)                                              /*!< LCD_T::OCTL: SEL28 Position            */
#define LCD_OCTL_SEL28_Msk               (0x1ul << LCD_OCTL_SEL28_Pos)                     /*!< LCD_T::OCTL: SEL28 Mask                */

#define LCD_OCTL_SEL29_Pos               (13)                                              /*!< LCD_T::OCTL: SEL29 Position            */
#define LCD_OCTL_SEL29_Msk               (0x1ul << LCD_OCTL_SEL29_Pos)                     /*!< LCD_T::OCTL: SEL29 Mask                */

#define LCD_OCTL_SEL35_Pos               (14)                                              /*!< LCD_T::OCTL: SEL35 Position            */
#define LCD_OCTL_SEL35_Msk               (0x3ul << LCD_OCTL_SEL35_Pos)                     /*!< LCD_T::OCTL: SEL35 Mask                */

#define LCD_OCTL_SEL36_Pos               (16)                                              /*!< LCD_T::OCTL: SEL36 Position            */
#define LCD_OCTL_SEL36_Msk               (0x3ul << LCD_OCTL_SEL36_Pos)                     /*!< LCD_T::OCTL: SEL36 Mask                */

#define LCD_OCTL_SEL37_Pos               (18)                                              /*!< LCD_T::OCTL: SEL37 Position            */
#define LCD_OCTL_SEL37_Msk               (0x3ul << LCD_OCTL_SEL37_Pos)                     /*!< LCD_T::OCTL: SEL37 Mask                */

#define LCD_OCTL_SEL38_Pos               (20)                                              /*!< LCD_T::OCTL: SEL38 Position            */
#define LCD_OCTL_SEL38_Msk               (0x3ul << LCD_OCTL_SEL38_Pos)                     /*!< LCD_T::OCTL: SEL38 Mask                */

#define LCD_OCTL_SEL41_Pos               (22)                                              /*!< LCD_T::OCTL: SEL41 Position            */
#define LCD_OCTL_SEL41_Msk               (0x1ul << LCD_OCTL_SEL41_Pos)                     /*!< LCD_T::OCTL: SEL41 Mask                */

#define LCD_OCTL_SEL42_Pos               (23)                                              /*!< LCD_T::OCTL: SEL42 Position            */
#define LCD_OCTL_SEL42_Msk               (0x1ul << LCD_OCTL_SEL42_Pos)                     /*!< LCD_T::OCTL: SEL42 Mask                */

#define LCD_OCTL_SEL47_Pos               (24)                                              /*!< LCD_T::OCTL: SEL47 Position            */
#define LCD_OCTL_SEL47_Msk               (0x1ul << LCD_OCTL_SEL47_Pos)                     /*!< LCD_T::OCTL: SEL47 Mask                */

#define LCD_OCTL_SEL48_Pos               (25)                                              /*!< LCD_T::OCTL: SEL48 Position            */
#define LCD_OCTL_SEL48_Msk               (0x1ul << LCD_OCTL_SEL48_Pos)                     /*!< LCD_T::OCTL: SEL48 Mask                */

#define LCD_OCTL_SEL49_Pos               (26)                                              /*!< LCD_T::OCTL: SEL49 Position            */
#define LCD_OCTL_SEL49_Msk               (0x1ul << LCD_OCTL_SEL49_Pos)                     /*!< LCD_T::OCTL: SEL49 Mask                */

#define LCD_STS_FCEND_Pos                (0)                                               /*!< LCD_T::STS: FCEND Position             */
#define LCD_STS_FCEND_Msk                (0x1ul << LCD_STS_FCEND_Pos)                      /*!< LCD_T::STS: FCEND Mask                 */

#define LCD_STS_FEND_Pos                 (1)                                               /*!< LCD_T::STS: FEND Position              */
#define LCD_STS_FEND_Msk                 (0x1ul << LCD_STS_FEND_Pos)                       /*!< LCD_T::STS: FEND Mask                  */

#define LCD_STS_CTOUT_Pos                (2)                                               /*!< LCD_T::STS: CTOUT Position             */
#define LCD_STS_CTOUT_Msk                (0x1ul << LCD_STS_CTOUT_Pos)                      /*!< LCD_T::STS: CTOUT Mask                 */

#define LCD_STS_CTIME_Pos                (16)                                              /*!< LCD_T::STS: CTIME Position             */
#define LCD_STS_CTIME_Msk                (0x1ffful << LCD_STS_CTIME_Pos)                   /*!< LCD_T::STS: CTIME Mask                 */

#define LCD_INTEN_FCEND_Pos              (0)                                               /*!< LCD_T::INTEN: FCEMD Position           */
#define LCD_INTEN_FCEND_Msk              (0x1ul << LCD_INTEN_FCEND_Pos)                    /*!< LCD_T::INTEN: FCEMD Mask               */

#define LCD_INTEN_FEND_Pos               (1)                                               /*!< LCD_T::INTEN: FEND Position            */
#define LCD_INTEN_FEND_Msk               (0x1ul << LCD_INTEN_FEND_Pos)                     /*!< LCD_T::INTEN: FEND Mask                */

#define LCD_INTEN_CTOUT_Pos              (2)                                               /*!< LCD_T::INTEN: CTOUT Position           */
#define LCD_INTEN_CTOUT_Msk              (0x1ul << LCD_INTEN_CTOUT_Pos)                    /*!< LCD_T::INTEN: CTOUT Mask               */

#define LCD_DATA0n_DD0_Pos               (0)                                               /*!< LCD_T::DATA0n: DD0 Position            */
#define LCD_DATA0n_DD0_Msk               (0xfful << LCD_DATA0n_DD0_Pos)                    /*!< LCD_T::DATA0n: DD0 Mask                */

#define LCD_DATA0n_DD1_Pos               (8)                                               /*!< LCD_T::DATA0n: DD1 Position            */
#define LCD_DATA0n_DD1_Msk               (0xfful << LCD_DATA0n_DD1_Pos)                    /*!< LCD_T::DATA0n: DD1 Mask                */

#define LCD_DATA0n_DD2_Pos               (16)                                              /*!< LCD_T::DATA0n: DD2 Position            */
#define LCD_DATA0n_DD2_Msk               (0xfful << LCD_DATA0n_DD2_Pos)                    /*!< LCD_T::DATA0n: DD2 Mask                */

#define LCD_DATA0n_DD3_Pos               (24)                                              /*!< LCD_T::DATA0n: DD3 Position            */
#define LCD_DATA0n_DD3_Msk               (0xfful << LCD_DATA0n_DD3_Pos)                    /*!< LCD_T::DATA0n: DD3 Mask                */



/**@}*/ /* LCD_CONST */
/**@}*/ /* end of LCD register group */
/**@}*/ /* end of REGISTER group */

#endif /* __LCD_REG_H__ */
