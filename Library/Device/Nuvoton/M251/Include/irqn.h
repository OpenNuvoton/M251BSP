/**************************************************************************//**
 * @file     irqn.h
 * @version  V1.00
 * @brief    IRQ number definition for M251
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __IRQN_H__
#define __IRQN_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

/**
 * @details  Interrupt Number Definition.
 */
typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***********************************************/
    NonMaskableInt_IRQn       = -14,    /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn            = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                     */
    SVCall_IRQn               = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                       */
    PendSV_IRQn               = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                       */
    SysTick_IRQn              = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                   */

    /******  ARMIKMCU Swift specific Interrupt Numbers ********************************************/
    BOD_IRQn                  = 0,      /*!< Brown-Out Low Voltage Detected Interrupt             */
    IRCTRIM_IRQn              = 1,      /*!< Watch Dog Timer Interrupt                            */
    PWRWU_IRQn                = 2,      /*!< EINT0, EINT2 and EINT4 Interrupt                     */
    RESERVE0                  = 3,      /*!< Reserve 0                                            */
    CLKFAIL_IRQn              = 4,      /*!< Clock fail detected Interrupt                          */
    RESERVE1                  = 5,      /*!< Reserve 1                                            */
    RTC_IRQn                  = 6,      /*!< Real Time Clock Interrupt                            */
    TAMPER_IRQn               = 7,      /*!< Tamper detection Interrupt                           */
    WDT_IRQn                  = 8,      /*!< Watch Dog Timer Interrupt                            */
    WWDT_IRQn                 = 9,      /*!< Window Watch Dog Timer Interrupt                     */
    EINT0_IRQn                = 10,     /*!< External Input 0 Interrupt                           */
    EINT1_IRQn                = 11,     /*!< External Input 1 Interrupt                           */
    EINT2_IRQn                = 12,     /*!< External Input 2 Interrupt                           */
    EINT3_IRQn                = 13,     /*!< External Input 3 Interrupt                           */
    EINT4_IRQn                = 14,     /*!< External Input 4 Interrupt                           */
    EINT5_IRQn                = 15,     /*!< External Input 5 Interrupt                           */
    GPA_IRQn                  = 16,     /*!< GPIO PORT A Interrupt                                */
    GPB_IRQn                  = 17,     /*!< GPIO PORT B Interrupt                                */
    GPC_IRQn                  = 18,     /*!< GPIO PORT C Interrupt                                */
    GPD_IRQn                  = 19,     /*!< GPIO PORT D Interrupt                                */
    GPE_IRQn                  = 20,     /*!< GPIO PORT E Interrupt                                */
    GPF_IRQn                  = 21,     /*!< GPIO PORT F Interrupt                                */
    QSPI0_IRQn                = 22,     /*!< QSPI0 Interrupt                                      */
    SPI0_IRQn                 = 23,     /*!< SPI0 Interrupt                                       */
    BRAKE0_IRQn               = 24,     /*!< PWM Brake0 Interrupt                                 */
    PWM0_P0_IRQn              = 25,     /*!< PWM0 P0 Interrupt                                    */
    PWM0_P1_IRQn              = 26,     /*!< PWM0 P1 Interrupt                                    */
    PWM0_P2_IRQn              = 27,     /*!< PWM0 P2 Interrupt                                    */
    BRAKE1_IRQn               = 28,     /*!< PWM Brake1 Interrupt                                 */
    PWM1_P0_IRQn              = 29,     /*!< PWM1 P0 Interrupt                                    */
    PWM1_P1_IRQn              = 30,     /*!< PWM1 P1 Interrupt                                    */
    PWM1_P2_IRQn              = 31,     /*!< PWM1 P2 Interrupt                                    */
    TMR0_IRQn                 = 32,     /*!< TIMER0  Interrupt                                    */
    TMR1_IRQn                 = 33,     /*!< TIMER1  Interrupt                                    */
    TMR2_IRQn                 = 34,     /*!< TIMER2  Interrupt                                    */
    TMR3_IRQn                 = 35,     /*!< TIMER3  Interrupt                                    */
    UART0_IRQn                = 36,     /*!< UART0  Interrupt                                     */
    UART1_IRQn                = 37,     /*!< UART1  Interrupt                                     */
    I2C0_IRQn                 = 38,     /*!< I2C0  Interrupt                                      */
    I2C1_IRQn                 = 39,     /*!< I2C1  Interrupt                                      */
    PDMA_IRQn                 = 40,     /*!< Peripheral DMA Interrupt                             */
    DAC_IRQn                  = 41,     /*!< DAC Interrupt                                        */
    EADC_INT0_IRQn            = 42,     /*!< Enhance ADC Interrupt 0                              */
    EADC_INT1_IRQn            = 43,     /*!< Enhance ADC Interrupt 1                              */
    ACMP01_IRQn               = 44,     /*!< ACMP0 Interrupt                                      */
    BPWM0_IRQn                = 45,     /*!< BPWM0 Interrupt                                      */
    EADC_INT2_IRQn            = 46,     /*!< Enhance EADC Interrupt 2                             */
    EADC_INT3_IRQn            = 47,     /*!< Enhance EADC Interrupt 3                             */
    UART2_IRQn                = 48,     /*!< UART2 Interrupt                                      */
    UART3_IRQn                = 49,     /*!< UART3 Interrupt                                      */
    USCI0_IRQn                = 50,     /*!< USCI0 Interrupt                                      */
    SPI1_IRQn                 = 51,     /*!< SPI0 Interrupt                                       */
    USCI1_IRQn                = 52,     /*!< USCI1 Interrupt                                      */
    USBD_IRQn                 = 53,     /*!< USB Device Interrupt                                 */
    BPWM1_IRQn                = 54,     /*!< BPWM1 Interrupt                                      */
    PSIO_IRQn                 = 55,     /*!< PSIO Interrupt                                       */
    RESERVE4                  = 56,     /*!< Reserve 4                                            */
    CRPT_IRQn                 = 57,     /*!< Crypto interrupt                                     */
    SC0_IRQn                  = 58,     /*!< Smart Card0 Interrupt                                */
    RESERVE5                  = 59,     /*!< Reserve 5                                            */
    USCI2_IRQn                = 60,     /*!< USCI2 Interrupt                                      */
    LCD_IRQn                  = 61,     /*!< LCD Interrupt                                        */
    OPA_IRQn                  = 62,     /*!< OPA Interrupt                                        */
    TK_IRQn                   = 63,     /*!< Touch Key Interrupt                                  */
} IRQn_Type;

#endif /* __IRQN_H__ */