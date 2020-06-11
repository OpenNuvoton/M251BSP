/****************************************************************************//**
 * @file     UART_LIN.h
 * @brief    UART LIN header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __UART_LIN_H__
#define __UART_LIN_H__

/* CheckSum Method */

#define MODE_CLASSIC    2
#define MODE_ENHANCED   1

/* LIN Max Speed is 20K */

#define LIN_BAUD_RATE   20000

/*---------------------------------------------------------------------------------------------------------*/
/*                             Define functions prototype                                                  */
/*---------------------------------------------------------------------------------------------------------*/
extern void LIN_MasterTestItem(void);
extern void LIN_SlaveTestItem(void);
extern void LIN_MasterFunctionTest(void);
extern void LIN_MasterFunctionTestUsingLinCtlReg(void);
extern void LIN_MasterTest(uint32_t u32Id, uint32_t u32ModeSel);
extern void LIN_MasterTestUsingLinCtlReg(uint32_t u32Id, uint32_t u32ModeSel);
extern void LIN_SendHeader(uint32_t u32Id);
extern void LIN_SendHeaderUsingLinCtlReg(uint32_t u32Id, uint32_t u32HeaderSel);
extern void LIN_SendResponse(int32_t i32CheckSumOption, uint32_t *pu32TxBuf);
extern void LIN_RecvResponse(uint8_t *pu32RxBuf, uint8_t u8Len);
extern void LIN_SendResponseWithByteCnt(int32_t i32CheckSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt);
extern void LIN_MasterReceiveTest(uint32_t u32Id, uint8_t u8Len, uint32_t u32ModeSel);
extern void LIN_MasterReveiceTestUsingLinCtlReg(uint32_t u32Id, uint8_t u8Len, uint32_t u32ModeSel);
extern void LIN_SlaveFunctionTest(void);
extern void LIN_SlaveRecevieHeader(uint32_t u32Id);
extern void LIN_SlaveTransmitResponse(uint32_t u32Id, int32_t i32CheckSumOption);
extern void LIN_SlaveReceiveResponse(uint32_t u32Id, int32_t i32CheckSumOption, uint8_t u8Len);
extern uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel);
extern uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt);
extern uint8_t GetParityValue(uint32_t u32Id);

#endif  /* __UART_LIN_H_ */

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/