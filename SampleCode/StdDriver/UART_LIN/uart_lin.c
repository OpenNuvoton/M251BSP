/****************************************************************************//**
 * @file     hid_mouse.c
 * @brief    M251 series UART LIN sample file
 *
 * @note
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "NuMicro.h"
#include "uart_lin.h"


/* The Define Macro can to selecte LIN port */
#define LIN_PORT  UART0

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t g_i32Pointer;
uint8_t g_au8SendData[12] ;
uint8_t g_au8RecvData[12] ;


/*---------------------------------------------------------------------------------------------------------*/
/* LIN Master Sample Code Menu                                                                             */
/*---------------------------------------------------------------------------------------------------------*/

void LIN_MasterTestItem(void)
{
    printf("\n\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("|               LIN Master Function Test                              |\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("| [1] Master send header with ID = 0x30                               |\n");
    printf("| [2] Master send header and response with classic checksum           |\n");
    printf("| [3] Master send header and response with enhanced checksum          |\n");
    printf("| [4] Master send header and received response(from Salve)            |\n");
    printf("|     with classic checksum                                           |\n");
    printf("| [5] Master send header and received response(from Salve)            |\n");
    printf("|     with enhanced checksum                                          |\n");
    printf("| To measure UART0_TXD(PA.1)and UART0_RXD(PA.0) to check waveform ... |\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("| Quit                                                        - [ESC] |\n");
    printf("+---------------------------------------------------------------------+\n\n");

}


/*---------------------------------------------------------------------------------------------------------*/
/* LIN Slave Sample Code Menu                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

void LIN_SlaveTestItem(void)
{
    printf("\n\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("|                LIN Slave Function Test                              |\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("| [1] Received the header from Master                                 |\n");
    printf("| [2] Received the header from Master and Send response(classic       |\n");
    printf("|     checksum) to Master                                             |\n");
    printf("| [3] Received the header from Master and Send response(enhanced      |\n");
    printf("|     checksum) to Master                                             |\n");
    printf("| [4] Received the header and  response(from Master) with classic     |\n");
    printf("|      checksum                                                       |\n");
    printf("| [5] Received the header and  response(from Master) with enhanced    |\n");
    printf("|      checksum                                                       |\n");
    printf("| To measure UART0_TXD(PA.1)and UART0_RXD(PA.0) to check waveform ... |\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("| Quit                                                        - [ESC] |\n");
    printf("+---------------------------------------------------------------------+\n\n");

}


/*---------------------------------------------------------------------------------------------------------*/
/* LIN Master Function Test                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterFunctionTest(void)
{
    uint32_t u32Item;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLine_Config(LIN_PORT, LIN_BAUD_RATE, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* === CASE 1====
        The sample code will send a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 2====
        The sample code will send a LIN header with ID is 0x35 and response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 3====
        The sample code will send a LIN header with ID is 0x12 and response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 4====
        The sample code will send a LIN header with ID is 0x34 and Recieve response field.
        The recieved response with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx and Rx pin to check it.
    */

    /* === CASE 5====
        The sample code will send a LIN header with ID is 0x36 and Recieve response field.
        The recieved field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx and Rx pin to check it.
    */


    do
    {
        LIN_MasterTestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);

        switch (u32Item)
        {
            case '1':
                LIN_SendHeader(0x30);
                break;

            case '2':
                LIN_MasterTest(0x35, MODE_CLASSIC);
                break;

            case '3':
                LIN_MasterTest(0x12, MODE_ENHANCED);
                break;

            case '4':
                LIN_MasterReceiveTest(0x34, 8, MODE_CLASSIC);
                break;

            case '5':
                LIN_MasterReceiveTest(0x36, 8, MODE_ENHANCED);
                break;

            default:
                break;
        }
    } while (u32Item != 27);

    /* Clear header select setting */
    LIN_PORT->LINCTL &= ~UART_LINCTL_HSEL_Msk;
    /* Select UART function mode */
    LIN_PORT->FUNCSEL = UART_FUNCSEL_UART;

    printf("\nLIN Sample Code End.\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/* LIN Master Function Test(Using LIN control register)                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterFunctionTestUsingLinCtlReg(void)
{
    uint32_t u32Item;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLine_Config(LIN_PORT, LIN_BAUD_RATE, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* === CASE 1====
        The sample code will send a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 2====
        The sample code will send a LIN header with ID is 0x35 and response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 3====
        The sample code will send a LIN header with ID is 0x12 and response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx pin to check it.
    */
    /* === CASE 4====
        The sample code will send a LIN header with ID is 0x34 and recieve response field.
        The recieved response with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx and Rx pin to check it.
    */

    /* === CASE 5====
        The sample code will send a LIN header with ID is 0x36 and recieve response field.
        The recieved field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx and Rx pin to check it.
    */


    do
    {
        LIN_MasterTestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);

        switch (u32Item)
        {
            case '1':
                LIN_SendHeaderUsingLinCtlReg(0x30, UART_LINCTL_HSEL_BREAK_SYNC);
                break;

            case '2':
                LIN_MasterTestUsingLinCtlReg(0x35, MODE_CLASSIC);
                break;

            case '3':
                LIN_MasterTestUsingLinCtlReg(0x12, MODE_ENHANCED);
                break;

            case '4':
                LIN_MasterReveiceTestUsingLinCtlReg(0x34, 8, MODE_CLASSIC);
                break;

            case '5':
                LIN_MasterReveiceTestUsingLinCtlReg(0x36, 8, MODE_ENHANCED);
                break;

            default:
                break;
        }
    } while (u32Item != 27);

    /* Clear header select setting */
    LIN_PORT->LINCTL &= ~UART_LINCTL_HSEL_Msk;

    /* Select UART function mode */
    LIN_PORT->FUNCSEL = UART_FUNCSEL_UART;

    printf("\nLIN Sample Code End.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* Master send header and response                                                                         */
/* u32id = Frame ID                                                                                        */
/* u32ModeSel = Classic checksum /Enhanced checksum                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterTest(uint32_t u32Id, uint32_t u32ModeSel)
{
    uint32_t testPattern[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};

    /* Send ID=0x35 Header and Response TestPatten */
    LIN_SendHeader(u32Id);
    LIN_SendResponse(u32ModeSel, &testPattern[0]);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Master send header and response(Using LIN control register)                                             */
/* u32id = Frame ID                                                                                        */
/* u32ModeSel = Classic checksum /Enhanced checksum                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterTestUsingLinCtlReg(uint32_t u32Id, uint32_t u32ModeSel)
{
    uint8_t au8TestPattern[9] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x0}; // 8 data byte + 1 byte checksum
    uint32_t u32LenCnt;

    if (u32ModeSel == MODE_CLASSIC)
    {
        /* Send break+sync+ID */
        LIN_SendHeaderUsingLinCtlReg(u32Id, UART_LINCTL_HSEL_BREAK_SYNC_ID);

        /* Compute checksum without ID and fill checksum value to  au8TestPattern[8] */
        au8TestPattern[8] = ComputeChksumValue(&au8TestPattern[0], 8);

        for (u32LenCnt = 0; u32LenCnt < 9; u32LenCnt++)
        {

            while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));  /* Wait Tx empty */

            LIN_PORT->DAT = au8TestPattern[u32LenCnt]; /* Send UART Data from buffer */
        }

    }
    else if (u32ModeSel == MODE_ENHANCED)
    {

        /* Send break+sync+ID */
        LIN_SendHeaderUsingLinCtlReg(u32Id, UART_LINCTL_HSEL_BREAK_SYNC_ID);

        /* Send break+sync and fill ID value to g_u8SendData[0]*/
        //LIN_SendHeaderUsingLinCtlReg(u32id, UART_LINCTL_HSEL_BREAK_SYNC);

        /* Send break and fill sync,ID value to g_u8SendData[0],g_u8SendData[1]*/
        //LIN_SendHeaderUsingLinCtlReg(u32id, UART_LINCTL_HSEL_BREAK);

        if ((LIN_PORT->LINCTL & UART_LINCTL_HSEL_Msk) == UART_LINCTL_HSEL_BREAK)
        {

            /* Fill test pattern to g_u8SendData[2]~ g_u8SendData[9] */
            for (u32LenCnt = 0; u32LenCnt < 8; u32LenCnt++)
            {
                g_au8SendData[g_i32Pointer++] = au8TestPattern[u32LenCnt];
            }

            /* Compute checksum value with ID and fill checksum value to g_u8SendData[10] */
            g_au8SendData[g_i32Pointer++] = ComputeChksumValue(&g_au8SendData[1], 9) ;

            for (u32LenCnt = 0; u32LenCnt < 9; u32LenCnt++)
            {

                while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk)) {}; /* Wait Tx empty*/

                LIN_PORT->DAT = g_au8SendData[u32LenCnt + 2]; /* Send UART Data from buffer */
            }
        }
        else if (((LIN_PORT->LINCTL & UART_LINCTL_HSEL_Msk) == UART_LINCTL_HSEL_BREAK_SYNC))
        {

            /* Fill test pattern to g_u8SendData[1]~ g_u8SendData[8] */
            for (u32LenCnt = 0; u32LenCnt < 8; u32LenCnt++)
            {
                g_au8SendData[g_i32Pointer++] = au8TestPattern[u32LenCnt];
            }

            /* Compute checksum value with ID and fill checksum value to g_u8SendData[9] */
            g_au8SendData[g_i32Pointer++] = ComputeChksumValue(&g_au8SendData[0], 9) ;

            for (u32LenCnt = 0; u32LenCnt < 9; u32LenCnt++)
            {

                while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk)) {}; /* Wait Tx empty */

                LIN_PORT->DAT = g_au8SendData[u32LenCnt + 1]; /* Send UART Data from buffer */
            }
        }
        else
        {

            g_au8SendData[g_i32Pointer++] = GetParityValue(u32Id);

            /* Fill test pattern to g_u8SendData[1]~ g_u8SendData[8] */
            for (u32LenCnt = 0; u32LenCnt < 8; u32LenCnt++)
            {
                g_au8SendData[g_i32Pointer++] = au8TestPattern[u32LenCnt];
            }

            /* Compute checksum value with ID and fill checksum value to g_u8SendData[9] */
            g_au8SendData[g_i32Pointer++] = ComputeChksumValue(&g_au8SendData[0], 9) ;

            for (u32LenCnt = 0; u32LenCnt < 9; u32LenCnt++)
            {

                while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk)) {}; /* Wait Tx empty */

                LIN_PORT->DAT = g_au8SendData[u32LenCnt + 1]; /* Send UART Data from buffer */
            }
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Master send header and Receive Response(Using LIN control register)                                     */
/* u32id = Frame ID                                                                                        */
/* u8Len = Receive Response Length                                                                         */
/* u32ModeSel = Classic checksum /Enhanced checksum                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterReveiceTestUsingLinCtlReg(uint32_t u32Id, uint8_t u8Len, uint32_t u32ModeSel)
{
    uint8_t u8Loop;
    memset((uint8_t *)g_au8RecvData, 0, sizeof(g_au8RecvData));
    LIN_PORT->FIFO |= UART_FIFO_RXRST_Msk;
    LIN_PORT->LINCTL |= (UART_LINCTL_LINRXOFF_Msk);
    /* Send ID=0x34 or 0x36 Header and Receive Response Data */
    /* Send break+sync+ID */
    LIN_SendHeaderUsingLinCtlReg(u32Id, UART_LINCTL_HSEL_BREAK_SYNC_ID);

    /* Send break+sync and fill ID value to g_u8SendData[0]*/
    //LIN_SendHeaderUsingLinCtlReg(u32id, UART_LINCTL_HSEL_BREAK_SYNC);

    /* Send break and fill sync,ID value to g_u8SendData[0],g_u8SendData[1]*/
    //LIN_SendHeaderUsingLinCtlReg(u32id, UART_LINCTL_HSEL_BREAK);

    LIN_PORT->LINCTL &= ~(UART_LINCTL_LINRXOFF_Msk);
    LIN_RecvResponse(g_au8RecvData, u8Len + 3);

    /* Check Frame ID is 0xB4(0x34) or 0x76(0x36)          */
    (g_au8RecvData[1] == GetParityValue(u32Id)) ? printf("Frame ID : %X\n", g_au8RecvData[1]) : printf("LIN Frame ID is Error\n");

    /* Show the  Receive Response                          */
    for (u8Loop = 2 ; u8Loop < u8Len + 2; u8Loop++)
        printf("Data[%d] = %x\n", u8Loop - 2, g_au8RecvData[u8Loop]);

    if (u32ModeSel == MODE_ENHANCED)
        (ComputeChksumValue(&g_au8RecvData[1], 9) == g_au8RecvData[u8Len + 2]) ? printf("CheckSum(Enhanced): %X\n", g_au8RecvData[u8Len + 2]) : printf("CheckSun is Error(Enhanced)\n ");
    else
        (ComputeChksumValue(&g_au8RecvData[2], 8) == g_au8RecvData[u8Len + 2]) ? printf("CheckSum(Classic) : %X\n", g_au8RecvData[u8Len + 2]) : printf("CheckSun is Error(Classic)\n ");


}


/*---------------------------------------------------------------------------------------------------------*/
/* Master send header and Receive Response                                                                 */
/* u32id = Frame ID                                                                                        */
/* u8Len = Receive Response Length                                                                         */
/* u32ModeSel = Classic checksum /Enhanced checksum                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterReceiveTest(uint32_t u32Id, uint8_t u8Len, uint32_t u32ModeSel)
{
    uint8_t u8Loop;
    memset((uint8_t *)g_au8RecvData, 0, sizeof(g_au8RecvData));
    LIN_PORT->FIFO |= UART_FIFO_RXRST_Msk;
    LIN_PORT->LINCTL |= UART_LINCTL_LINRXOFF_Msk;
    /* Send ID=0x34 Header and Receive Response Data */
    LIN_SendHeader(u32Id);
    LIN_PORT->LINCTL &= ~UART_LINCTL_LINRXOFF_Msk;
    LIN_RecvResponse(g_au8RecvData, u8Len + 2);

    /* Check Frame ID is 0xB4(0x34) or 0x76(0x36)          */
    (g_au8RecvData[0] == GetParityValue(u32Id)) ? printf("Frame ID : %X\n", g_au8RecvData[0]) : printf("LIN Frame ID is Error\n");

    /* Show the  Receive Response                          */
    for (u8Loop = 1 ; u8Loop < u8Len + 1; u8Loop++)
        printf("Data[%d] = %x\n", u8Loop - 1, g_au8RecvData[u8Loop]);

    if (u32ModeSel == MODE_ENHANCED)
        (ComputeChksumValue(&g_au8RecvData[0], 9) == g_au8RecvData[u8Len + 1]) ? printf("CheckSum(Enhanced): %X\n", g_au8RecvData[u8Len + 1]) : printf("CheckSun is Error(Enhanced)\n ");
    else
        (ComputeChksumValue(&g_au8RecvData[1], 8) == g_au8RecvData[u8Len + 1]) ? printf("CheckSum(Classic) : %X\n", g_au8RecvData[u8Len + 1]) : printf("CheckSun is Error(Classic)\n ");

}




/*---------------------------------------------------------------------------------------------------------*/
/* Send LIN Header Field                                                                                   */
/* u32id = Frame ID                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendHeader(uint32_t u32Id)
{
    uint32_t u32Count;

    g_i32Pointer = 0;

    /* Select LIN function mode */
    LIN_PORT->FUNCSEL = UART_FUNCSEL_LIN;

    /* Set LIN operation mode, Tx mode and break field length is 12 bits */
    LIN_PORT->ALTCTL &= ~(UART_ALTCTL_LINTXEN_Msk | UART_ALTCTL_LINRXEN_Msk | UART_ALTCTL_BRKFL_Msk);
    LIN_PORT->ALTCTL |= (UART_ALTCTL_LINTXEN_Msk | (11 << UART_ALTCTL_BRKFL_Pos));

    g_au8SendData[g_i32Pointer++] = 0x55 ;                   // SYNC Field
    g_au8SendData[g_i32Pointer++] = GetParityValue(u32Id);   // ID+Parity Field

    for (u32Count = 0; u32Count < 2; u32Count++)
    {
        while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));  /* Wait Tx empty */

        LIN_PORT->DAT = g_au8SendData[u32Count]; /* Send UART Data from buffer */
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* Send LIN Header Field                                                                                   */
/* u32id = Frame ID                                                                                        */
/* u32HeaderSel =  UART_LINCTL_HSEL_BREAK/UART_LINCTL_HSEL_BREAK_SYNC/UART_LINCTL_HSEL_BREAK_SYNC_ID       */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendHeaderUsingLinCtlReg(uint32_t u32Id, uint32_t u32HeaderSel)
{
    g_i32Pointer = 0 ;
    /* Switch back to LIN Function */
    LIN_PORT->FUNCSEL = UART_FUNCSEL_LIN;

    /* Set LIN 1. PID as 0x30 [UART_LINCTL_PID(0x30)]
               2. Header select as includes "break field", "sync field" and "frame ID field".[UART_LINCTL_HSEL_BREAK_SYNC_ID]
               3. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               4. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
               5. ID Parity Enable. Hardware will calculate and fill P0/P1 automatically  [UART_LINCTL_IDPEN_Msk]
    */
    if (u32HeaderSel == UART_LINCTL_HSEL_BREAK_SYNC_ID)
    {
        LIN_PORT->LINCTL = UART_LINCTL_PID(u32Id) | UART_LINCTL_HSEL_BREAK_SYNC_ID |
                           UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12) | UART_LINCTL_IDPEN_Msk;
        /* LIN TX Send Header Enable */
        LIN_PORT->LINCTL |= UART_LINCTL_SENDH_Msk;

        /* Wait until break field, sync field and ID field transfer completed */
        while ((LIN_PORT->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk);
    }
    /* Set LIN 1. Header select as includes "break field" and "sync field".[UART_LINCTL_HSEL_BREAK_SYNC]
               2. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               3. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
    */
    else if (u32HeaderSel == UART_LINCTL_HSEL_BREAK_SYNC)
    {
        LIN_PORT->LINCTL = UART_LINCTL_HSEL_BREAK_SYNC | UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12);
        /* LIN TX Send Header Enable */
        LIN_PORT->LINCTL |= UART_LINCTL_SENDH_Msk;

        /* Wait until break field and sync field transfer completed */
        while ((LIN_PORT->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk);

        /*Send ID field, g_u8SendData[0] is ID+parity field */
        g_au8SendData[g_i32Pointer++] = GetParityValue(u32Id);   // ID+Parity Field
        LIN_PORT->DAT = g_au8SendData[0];

        while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));
    }
    /* Set LIN 1. Header select as includes "break field".[UART_LINCTL_HSEL_BREAK]
               2. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               3. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
    */
    else if (u32HeaderSel ==  UART_LINCTL_HSEL_BREAK)
    {
        LIN_PORT->LINCTL =  UART_LINCTL_HSEL_BREAK | UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12);
        /* LIN TX Send Header Enable */
        LIN_PORT->LINCTL |= UART_LINCTL_SENDH_Msk;

        /* Wait until break field transfer completed */
        while ((LIN_PORT->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk);

        /* Send sync field and ID field */
        g_au8SendData[g_i32Pointer++] = 0x55 ;                  // SYNC Field
        g_au8SendData[g_i32Pointer++] = GetParityValue(u32Id);   // ID+Parity Field
        LIN_PORT->DAT = g_au8SendData[0];

        while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));

        LIN_PORT->DAT = g_au8SendData[1];

        while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Send LIN Response Field                                                                                 */
/* checkSumOption = Classic checksum /Enhanced checksum                                                    */
/* pu32TxBuf = Tx Data Buffer pointer                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponse(int32_t i32CheckSumOption, uint32_t *pu32TxBuf)
{
    int32_t i32Len;

    for (i32Len = 0; i32Len < 8; i32Len++)
        g_au8SendData[g_i32Pointer++] = pu32TxBuf[i32Len] ;

    g_au8SendData[g_i32Pointer++] = GetCheckSumValue(g_au8SendData, i32CheckSumOption) ; //CheckSum Field

    for (i32Len = 0; i32Len < 9; i32Len++)
    {
        while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk)) {}; /* Wait Tx empty */

        LIN_PORT->DAT = g_au8SendData[i32Len + 2]; /* Send UART Data from buffer */
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* Received LIN Response Field                                                                             */
/* pu32RxBuf = Rx Data Buffer pointer                                                                      */
/* u8Len = Receive Response Length                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_RecvResponse(uint8_t *pu32RxBuf, uint8_t u8Len)
{
    uint8_t u8Loop;

    for (u8Loop = 0 ; u8Loop < u8Len; u8Loop++)
    {

        while ((LIN_PORT->INTSTS & UART_INTSTS_RDAIF_Msk) == 0) {};

        pu32RxBuf[u8Loop] = LIN_PORT->DAT;
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* Send LIN Response Field                                                                                 */
/* checkSumOption = Classic checksum /Enhanced checksum                                                    */
/* pu32TxBuf = Rx Data Buffer pointer                                                                      */
/* u32ByteCnt = CheckSum Field Lenght                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponseWithByteCnt(int32_t i32CheckSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt)
{
    int32_t u32Loop;

    /* Prepare data */
    for (u32Loop = 0; u32Loop < u32ByteCnt; u32Loop++)
        g_au8SendData[g_i32Pointer++] = pu32TxBuf[u32Loop] ;

    /* Prepare check sum */
    if (i32CheckSumOption == MODE_CLASSIC)
        g_au8SendData[g_i32Pointer++] = GetCheckSumValue(&g_au8SendData[2], u32ByteCnt) ;  //CheckSum Field
    else if (i32CheckSumOption == MODE_ENHANCED)
        g_au8SendData[g_i32Pointer++] = GetCheckSumValue(&g_au8SendData[1], (u32ByteCnt + 1)) ; //CheckSum Field

    /* Send data and check sum */
    for (u32Loop = 0; u32Loop < 9; u32Loop++)
    {
        while (!(LIN_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));  /* Wait Tx empty */

        LIN_PORT->DAT = g_au8SendData[u32Loop + 2]; /* Send UART Data from buffer */
    }

}
/*---------------------------------------------------------------------------------------------------------*/
/* LIN Slave Function Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SlaveFunctionTest(void)
{
    uint32_t u32Item;
    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLine_Config(LIN_PORT, LIN_BAUD_RATE, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* Select LIN function mode */
    LIN_PORT->FUNCSEL = UART_FUNCSEL_LIN;

    /* Set LIN is Slave mode */
    LIN_PORT->LINCTL |= UART_LINCTL_SLVEN_Msk;

    /* === CASE 1====
        The sample code will recived a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UART1 Rx pin to check it.
    */

    /* === CASE 2====
        The sample code will recived a LIN header with ID is 0x30 and transmit response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UART1 Tx and Rx pin to check it.
    */

    /* === CASE 3====
        The sample code will recived a LIN header with ID is 0x30 and transmit response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx and Rx pin to check it.
    */

    /* === CASE 4====
        The sample code will recived a LIN header with ID is 0x30 and response field.
        The recieved response with 8 data bytes and checksum with including ID.
        Measurement the UART1 Rx pin to check it.
    */

    /* === CASE 5====
        The sample code will recived a LIN header with ID is 0x36 and response field.
        The recieved field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Rx pin to check it.
    */


    do
    {
        LIN_SlaveTestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);

        switch (u32Item)
        {
            case '1':
                LIN_SlaveRecevieHeader(0x30);
                break;

            case '2':
                LIN_SlaveTransmitResponse(0x30, MODE_CLASSIC);
                break;

            case '3':
                LIN_SlaveTransmitResponse(0x30, MODE_ENHANCED);
                break;

            case '4':
                LIN_SlaveReceiveResponse(0x30, MODE_CLASSIC, 8);
                break;

            case '5':
                LIN_SlaveReceiveResponse(0x30, MODE_ENHANCED, 8);
                break;

            default:
                break;
        }
    } while (u32Item != 27);

    /*claer Salve mode register */
    LIN_PORT->LINCTL &= ~UART_LINCTL_SLVEN_Msk;
    /* Clear header select setting */
    LIN_PORT->LINCTL &= ~UART_LINCTL_HSEL_Msk;
    /* Select UART function mode */
    LIN_PORT->FUNCSEL = UART_FUNCSEL_UART;

    printf("\nLIN Sample Code End.\n");

}



/*---------------------------------------------------------------------------------------------------------*/
/* Recevie LIN Header Field                                                                                */
/* u32id = Frame ID                                                                                        */
/* pu32TxBuf = Tx Data Buffer pointer                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SlaveRecevieHeader(uint32_t u32Id)
{
    /* Reset Rx FIFO             */
    LIN_PORT->FIFO |= UART_FIFO_RXRST_Msk;

    /* Set LIN  1. PID as 0x30 [UART_LINCTL_PID(0x30)]
               2. Header select as includes "break field".[UART_LINCTL_HSEL_BREAK]
               3. Enabled LIN Slave Header Detection
               4. ID Parity Enable. Hardware will calculate and fill P0/P1 automatically  [UART_LINCTL_IDPEN_Msk]
               5. Enabled Mute mode
    */

    LIN_PORT->LINCTL |= (UART_LINCTL_PID(u32Id) | UART_LINCTL_HSEL_BREAK
                         | UART_LINCTL_SLVHDEN_Msk | UART_LINCTL_IDPEN_Msk | UART_LINCTL_MUTE_Msk);

    /* Wait to Received the Header     */
    while (!(LIN_PORT->LINSTS & UART_LINSTS_SLVHDETF_Msk)) {};

    /*Check the LIN Parity bits*/
    if (LIN_PORT->LINSTS & UART_LINSTS_SLVIDPEF_Msk)
    {
        /* Clear SLVIDPEF and SLVHDETF Flag                    */
        LIN_PORT->LINSTS |= (UART_LINSTS_SLVIDPEF_Msk | UART_LINSTS_SLVHDETF_Msk);
        printf("Lin Frame ID parity is Error\n");
    }
    else
    {
        /* Received the Sync+Frame ID from Rx FIFO*/
        LIN_RecvResponse(g_au8RecvData, 2);
        /* Check Frame ID                         */
        (g_au8RecvData[1] == GetParityValue(u32Id)) ? printf("Frame ID : %X\n", g_au8RecvData[1]) : printf("LIN Frame ID is Error\n");
        /* Clear SLVHDETF Flag                    */
        LIN_PORT->LINSTS |= UART_LINSTS_SLVHDETF_Msk;
    }

    /*   Disable MUTE mode           */
    LIN_PORT->LINCTL &= ~UART_LINCTL_MUTE_Msk;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Recevied LIN Header Field from Master And Send Response Field                                           */
/* u32id = Frame ID                                                                                        */
/* checkSumOption = Classic checksum /Enhanced checksum                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SlaveTransmitResponse(uint32_t u32Id, int32_t i32CheckSumOption)
{
    uint32_t au8TestPattern[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8}; // 8 data byte

    LIN_SlaveRecevieHeader(u32Id);

    g_i32Pointer = 0;
    g_au8SendData[g_i32Pointer++] = 0x55 ;// SYNC Field
    g_au8SendData[g_i32Pointer++] = GetParityValue(u32Id);// ID+Parity Field

    LIN_SendResponse(i32CheckSumOption, &au8TestPattern[0]);

}


/*---------------------------------------------------------------------------------------------------------*/
/* Recevied LIN Header And Response Field                                                                  */
/* u32id = Frame ID                                                                                        */
/* checkSumOption = Classic checksum /Enhanced checksum                                                    */
/* u8Len= Received Response Field Lenght                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

void LIN_SlaveReceiveResponse(uint32_t u32Id, int32_t i32CheckSumOption, uint8_t u8Len)
{
    uint8_t u8Loop;

    /* Check Frame ID is 0xB4(0x34) or 0x76(0x36)          */
    LIN_SlaveRecevieHeader(u32Id);
    LIN_RecvResponse(&g_au8RecvData[2], u8Len + 1);

    for (u8Loop = 2 ; u8Loop < u8Len + 3 ; u8Loop++)
        printf("Data[%d] = %x\n", u8Loop - 2, g_au8RecvData[u8Loop]);

    if (i32CheckSumOption == MODE_ENHANCED)
        (ComputeChksumValue(&g_au8RecvData[1], 9) == g_au8RecvData[u8Len + 2])
        ? printf("CheckSum(Enhanced): %X\n", g_au8RecvData[u8Len + 2]) : printf("CheckSun is Error(Enhanced)\n ");
    else
        (ComputeChksumValue(&g_au8RecvData[2], 8) == g_au8RecvData[u8Len + 2])
        ? printf("CheckSum(Classic) : %X\n", g_au8RecvData[u8Len + 2]) : printf("CheckSun is Error(Classic)\n ");



}


/*---------------------------------------------------------------------------------------------------------*/
/* Compute Parity Value                                                                                    */
/* u32id = Frame ID                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t GetParityValue(uint32_t u32Id)
{
    uint32_t u32Res = 0, au32ID[6], au32ParityBit[2], u32Mask = 0;

    for (u32Mask = 0; u32Mask < 6; u32Mask++)
        au32ID[u32Mask] = (u32Id & (1 << u32Mask)) >> u32Mask;

    au32ParityBit[0] = (au32ID[0] + au32ID[1] + au32ID[2] + au32ID[4]) % 2;
    au32ParityBit[1] = (!((au32ID[1] + au32ID[3] + au32ID[4] + au32ID[5]) % 2));

    u32Res = u32Id + (au32ParityBit[0] << 6) + (au32ParityBit[1] << 7);
    return u32Res;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value , MODE_CLASSIC:(Not Include ID)    MODE_ENHANCED:(Include ID)                    */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel)
{
    uint32_t u32Loop, u32CheckSum = 0;

    for (u32Loop = u32ModeSel; u32Loop <= 9; u32Loop++)
    {
        u32CheckSum += pu8Buf[u32Loop];

        if (u32CheckSum >= 256)
            u32CheckSum -= 255;
    }

    return (255 - u32CheckSum);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value                                                                                  */
/* u32ByteCnt = Checksum lenght                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt)
{
    uint32_t u32Loop, u32CheckSum = 0;

    for (u32Loop = 0 ; u32Loop < u32ByteCnt; u32Loop++)
    {
        u32CheckSum += pu8Buf[u32Loop];

        if (u32CheckSum >= 256)
            u32CheckSum -= 255;
    }

    return (uint8_t)(255 - u32CheckSum);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
