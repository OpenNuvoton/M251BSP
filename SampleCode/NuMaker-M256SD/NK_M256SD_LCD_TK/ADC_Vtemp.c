/**************************************************************************//**
 * @file     ADC_Vtemp.c
 * @version  V1.00
 * @brief    Measure internal temperature sensor.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "stdlib.h"
#include "NuMicro.h"

/**
  *  When Temperature 25C, temperature voltage = 675mV, 1.83mV /per C
  *  TemperatureVoltage = ADC_result*VREF / 4096
  *  Temperature = 25 + ((673 - TemperatureVoltage)/1.83)
  *  so Temperature = 25 + (TTMPCAL - ADC_result*VREF)/VTEMPCOMDIV
  */

#define   TMPCAL        0x2A3000       /*    675*4096     */
#define   VTEMPCOMDIV   7694           /*    4096*1.83    */

uint32_t internal_Temperature(void)
{
    unsigned int  ADC_TEMP_Result, ADC_BG_Result;
    double  VREF_Voltage, RealTemperature;

    SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, 0);

    /* Set sample module 17/16 external sampling time to 0x1F */
    EADC_SetExtendSampleTime(EADC, 16, 0x1F);
    EADC_SetExtendSampleTime(EADC, 17, 0x1F);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 17 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT0);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT17);//Enable sample module 17 interrupt.

    /* Reset the ADC interrupt indicator and trigger sample module 16/17 to start A/D conversion */
    EADC_START_CONV(EADC, BIT16 | BIT17);

    /* Wait EADC conversion done */
    while (EADC_GET_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk) == 0);

    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, BIT0);

    /* Get the conversion result of the sample module 16 */
    ADC_BG_Result = EADC_GET_CONV_DATA(EADC, 16);
    ADC_TEMP_Result = EADC_GET_CONV_DATA(EADC, 17);

    /* Close the A/D converter */
    EADC_Close(EADC);

    /**
      *                  VDD  Now                             4096
      *    ------------------------------------- = ----------------------------------
      *    814mV(Storage value test condition)      NOW ADC Bandgap convert reuslt
      */

    VREF_Voltage = ((float)4096 / (float)ADC_BG_Result) * 814;

    RealTemperature = 25 + ((TMPCAL - (ADC_TEMP_Result * VREF_Voltage)) / VTEMPCOMDIV);
    RealTemperature = abs((int32_t)RealTemperature);

    return (uint32_t)(RealTemperature);
}
