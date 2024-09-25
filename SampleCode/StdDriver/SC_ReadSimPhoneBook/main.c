/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to read phone book information in the SIM card.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sclib.h"

#define SC_INTF 0  // Smartcard interface 0
#define PLL_CLOCK       FREQ_48MHZ
#define SW_CARD_DETECT_EN 0 /*S/W card detection for chip without card detection pin*/

#if SW_CARD_DETECT_EN
    volatile uint32_t g_u32CardIsInserted = 0; /*Card dectection pin status, 1: inserted; 0: removed*/
#endif

/* The definition of commands used in this sample code and directory structures could
   be found in GSM 11.11 which is free for download from Internet.
   Different from the command defined in ISO 7816-4, CLS of SIM command is 0xA0,
   So the command defined below starting with 0xA0 */

// Select File
const uint8_t g_au8SelectMF[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
const uint8_t g_au8SelectDF_TELECOM[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x7F, 0x10};
const uint8_t g_au8SelectEF_ADN[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x6F, 0x3A};
//Get Response
uint8_t g_au8GetResp[] = {0xA0, 0xC0, 0x00, 0x00, 0x00};
//Read Record
uint8_t g_au8ReadRec[] = {0xA0, 0xB2, 0x01, 0x04, 0x00};
//Verify CHV, CHV = Card Holder Verification information
uint8_t g_au8VerifyChv[] = {0xA0, 0x20, 0x00, 0x01, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t g_au8Buf[300];
uint32_t g_u32Len;

/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @return None
  */
void SC0_IRQHandler(void)
{
    // Please don't remove any of the function calls below
#if !SW_CARD_DETECT_EN
    // Card insert/remove event occurred, no need to check other event...
    if (SCLIB_CheckCDEvent(SC_INTF))
        return;

#endif

    SCLIB_CheckTimeOutEvent(SC_INTF);
    SCLIB_CheckTxRxEvent(SC_INTF);
    SCLIB_CheckErrorEvent(SC_INTF);

    return;
}

#if SW_CARD_DETECT_EN
void GPB_IRQHandler(void)
{
    /* To check if PB.0 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT0))
    {
        SCLIB_Deactivate(SC_INTF);

        g_u32CardIsInserted = PB0; /*Assume level HIGH indicates card inserted*/
        GPIO_CLR_INT_FLAG(PB, BIT0);

        if (SCLIB_CheckCDEvent_ByVar(SC_INTF, &g_u32CardIsInserted))
        {
            // Card insert/remove event occurred, no need to check other event...
            return;
        }
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */

        printf("Un-expected interrupts.0x%08x\n", PB->INTSRC);
        PB->INTSRC = PB->INTSRC;
    }
}

#endif



/**
  * @brief  Ask user to input PIN from console
  * @param  None
  * @return None
  * @details Valid input characters (0~9) are echo to console and store in command buffer.
  *         Backspace key can delete previous input digit, ESC key delete all input digits.
  *         Valid PIN length is between 4~8digits. If PIN length is shorter than 8
  *         digits, an Enter key can terminate the input procedure.
  */
void get_pin(void)
{
    uint32_t u32Idx = 0;

    printf("Please input PIN number:");

    while (u32Idx < 8)
    {
        uint8_t u8Char = getchar();

        if (u8Char >= 0x30 && u8Char <= 0x39)     // Valid input characters (0~9)
        {
            g_au8VerifyChv[5 + u32Idx] = u8Char;
            printf("%c", u8Char);
            u32Idx++;
        }
        else if (u8Char == 0x7F)     // DEL (Back space)
        {
            u32Idx--;
            printf("%c", u8Char);
        }
        else if (u8Char == 0x0D)     // Enter
        {
            if (u32Idx >= 4) //Min CHV length is 4 digits
                break;
        }
        else if (u8Char == 0x1B)     //ESC
        {
            printf("\nPlease input PIN number:");
            u32Idx = 0;  // retry
        }
        else
        {
            continue;
        }

    }

    // Fill remaining digits with 0xFF
    for (; u32Idx < 8; u32Idx++)
    {
        g_au8VerifyChv[5 + u32Idx] = 0xFF;
    }

    printf("\n");

    return;
}

/**
  * @brief  Send verify command to verify CHV1
  * @param  Remaining retry count, valid values are between 3~1
  * @return Unlock SIM card success or not
  * @retval 0 Unlock success
  * @retval -1 Unlock failed
  */
int unlock_sim(uint32_t u32RetryCnt)
{
    while (u32RetryCnt > 0)
    {

        get_pin(); // Ask user input PIN

        if (SCLIB_StartTransmission(SC_INTF, g_au8VerifyChv, 13, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
        {
            printf("Command Verify CHV failed\n");
            break;
        }

        if (g_au8Buf[0] == 0x90 || g_au8Buf[1] == 0x00)
        {
            printf("Pass\n");
            return 0;
        }
        else
        {
            u32RetryCnt--;
            printf("Failed, remaining retry count: %u\n", u32RetryCnt);
        }
    }

    printf("Oops, SIM card locked\n");

    return -1;
}

/**
  * @brief  Read phone book and print on console
  * @param  Phone book record number
  * @return None
  */
void read_phoneBook(uint32_t u32Cnt)
{
    uint32_t u32IdxI, u32IdxJ, u32IdxK;

    /*
        EF_ADN structure looks like below:

        Byte            Description                         M/O Length
        1 to X          Alpha Identifier                    O   X bytes
        X+1             Length of BCD number/SSC contents   M   1 byte
        X+2             TON and NPI                         M   1 byte
        X+3 to X + 12   Dialling Number/SSC String          M   10 bytes
        X+13            Capability/Configuration Identifier M   1 byte
        X+14            Extension1 Record Identifier        M   1 byte
    */
    for (u32IdxI = 1; u32IdxI < u32Cnt + 1; u32IdxI++)
    {
        g_au8ReadRec[2] = (uint8_t)u32IdxI;

        if (SCLIB_StartTransmission(SC_INTF, g_au8ReadRec, 5, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
        {
            printf("Command Read Record failed\n");
            break;
        }

        if (g_au8Buf[0] == 0xFF) // This is an empty entry
            continue;

        printf("\n======== %u ========", u32IdxI);
        printf("\nName: ");

        for (u32IdxJ = 0; g_au8Buf[u32IdxJ] != 0xFF; u32IdxJ++)
        {
            printf("%c", g_au8Buf[u32IdxJ]);
        }

        while (g_au8Buf[u32IdxJ] == 0xFF)  // Skip reset of the Alpha Identifier bytes
            u32IdxJ++;

        printf("\nNumber: ");
        u32IdxJ += 2; // Skip Length of BCD and TNO/NPI

        for (u32IdxK = 0; u32IdxK < 10; u32IdxK++)
        {
            if ((g_au8Buf[u32IdxJ + u32IdxK] & 0xf) != 0xF)
                printf("%c", (g_au8Buf[u32IdxJ + u32IdxK] & 0xf) + 0x30);
            else
                break;

            if ((g_au8Buf[u32IdxJ + u32IdxK] >> 4) != 0xF)
                printf("%c", (g_au8Buf[u32IdxJ + u32IdxK] >> 4) + 0x30);
            else
                break;
        }
    }

    printf("\n");
    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable IP clock */
    CLK_EnableModuleClock(SC0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select SC0 clock source from HIRC and divide the clock to 3MHz*/
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(16));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk
                       | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SC0_CLK | SYS_GPA_MFPL_PA1MFP_SC0_DAT
                      | SYS_GPA_MFPL_PA2MFP_SC0_RST | SYS_GPA_MFPL_PA3MFP_SC0_PWR);

#if SW_CARD_DETECT_EN
    CLK_EnableModuleClock(GPB_MODULE);

    /*For Chip without CD pin*/
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB0MFP_Msk;
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_GPIO;
#else
    SYS->GPA_MFPL &= ~SYS_GPA_MFPL_PA4MFP_Msk;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA4MFP_SC0_nCD;
#endif

    /* Lock protected registers */
    SYS_LockReg();
}


/*
    Each SIM card contains a file system, below is a simplified example

                                MF (Master File)
                                       |
                          -------------+--------------
                          |            |             |
               EF (Elementary File)    EF   DF (Dedicated File)
                                                     |
                                            ---------+--------
                                            |        |       |
                                            EF       DF      EF
                                                     |
                                                     EF
    Each file has an two byte ID, where the first byte indicates the type of file
    '3F': Master File
    '7F': Dedicated File
    '2F': Elementary File under the Master File
    '6F': Elementary File under a Dedicated File

*/

int main(void)
{
    uint32_t u32RetVal;
    uint32_t u32Retry = 0, u32Cnt, u32Chv1Disbled = 0;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\nThis sample code reads phone book from SIM card\n");

    // Open smartcard interface 1. CD pin state low indicates card insert and PWR pin high raise VCC pin to card
#if SW_CARD_DETECT_EN
    /*
        implement S/W card detection for chip without card detection pin
    */
    SYS_UnlockReg();

    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 0, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(GPB_IRQn);
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);

    SC_Open(SC0, SC_PIN_STATE_IGNORE, SC_PIN_STATE_HIGH);
#else
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
#endif

    NVIC_EnableIRQ(SC0_IRQn);

    // Wait 'til card insert
#if SW_CARD_DETECT_EN

    while ((!g_u32CardIsInserted)) {};

#else
    while (SC_IsCardInserted(SC0) == FALSE);

#endif

    // Activate slot 1
    u32RetVal = SCLIB_Activate(SC_INTF, FALSE);

    if (u32RetVal != SCLIB_SUCCESS)
    {
        printf("SIM card activate failed\n");
        goto exit;
    }

    // Select master file.
    if (SCLIB_StartTransmission(SC_INTF, (uint8_t *)g_au8SelectMF, 7, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
    {
        printf("Command Select MF failed\n");
        goto exit;
    }

    // If there is no error during transmission, check the response from card
    if (g_u32Len == 2 && g_au8Buf[0] == 0x9F)
    {
        // Everything goes fine, SIM card response 0x9F following by the response data length
        g_au8GetResp[4] = g_au8Buf[1]; // response data length

        // Issue "get response" command to get the response from SIM card
        if (SCLIB_StartTransmission(SC_INTF, g_au8GetResp, 5, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
        {
            printf("Command Get response failed\n");
            goto exit;
        }
    }
    else
    {
        printf("Unknown response\n");
        goto exit;
    }

    // Response ends with 0x9000 means command success
    if (g_au8Buf[g_u32Len - 2] != 0x90 || g_au8Buf[g_u32Len - 1] != 0x00)
    {
        printf("Cannot select MF\n");
        goto exit;
    }

    /*
        Response of select MF, DF listed here:
        Byte    Description
        1~2     RFU
        3~4     Total amount of memory of the selected directory which is not
                allocated to any of the DFs or EFs under the selected directory
        5~6     File ID
        7       Type of File
        8~12    RFU
        13      Length of the following data
        14      File characteristics
        15      Number of DFs which are a direct child of the current directory
        16      Number of EFs which are a direct child of the current directory
        17      Number of CHVs, UNBLOCK CHVs and administrative codes
        18      RFU
        19      CHV1 status
                b8 0: secret code not initialized, 1: secret code initialized
                b7~b5 RFU
                b4~b1 Number of false presentations remaining, 0 means blocked
        20      UNBLOCK CHV1 status
        21      CHV2 status
        22      UNBLOCK CHV2 status
        23      RFU
        24~34   Reserved for the administrative management (optional)
    */

    // Read byte 19 listed in above table to check if SIM is locked
    if (g_au8Buf[18] & 0x80)
    {
        if ((u32Retry = (g_au8Buf[18] & 0xF)) == 0)   //=> Blocked!!
        {
            printf("SIM locked, and unlock retry count exceed\n");
            goto exit;
        }
    }

    // Some SIM cards has file protect by CHV1, but CHV1 disabled.
    if (g_au8Buf[13] & 0x80)
    {
        printf("CHV1 disabled\n");
        u32Chv1Disbled = 1;
    }

    // Select Dedicated File DFTELECOM which contains service related information
    if (SCLIB_StartTransmission(SC_INTF, (uint8_t *)g_au8SelectDF_TELECOM, 7, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
    {
        printf("Command Select DF failed\n");
        goto exit;
    }

    // Don't care about the response of g_au8SelectDF_TELECOM command here as long as there's no error.


    /* Select Elementary File ADN, where ADN stands for "Abbreviated dialling numbers",
       this is the file used to store phone book */
    if (SCLIB_StartTransmission(SC_INTF, (uint8_t *)g_au8SelectEF_ADN, 7, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
    {
        printf("Command Select EF failed\n");
        goto exit;
    }

    if (g_u32Len == 2 && g_au8Buf[0] == 0x9F)    // response data length
    {
        // Everything goes fine, SIM card response 0x9F following by the response data length
        g_au8GetResp[4] = g_au8Buf[1];

        if (SCLIB_StartTransmission(SC_INTF, g_au8GetResp, 5, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
        {
            printf("Command Get response failed\n");
            goto exit;
        }
    }
    else
    {
        printf("Unknown response\n");
        goto exit;
    }

    /*
        Response of select EF listed here:
        Byte    Description
        1~2     RFU
        3~4     File size
        5~6     File ID
        7       Type of File
        8       RFU
        9~11    Access conditions. 0: ALW, 1: CHV1, 2: CHV2, 3: RFU, 4: ADM...
                Byte 9 b8~b4 for read, seek, b3~b1 for update
                Byte 10 b8~b4 for increase, b3~b1 is RFU
                Byte 11 b8~b4 for rehabilitate, b3~b1 for invalidate
        12      File status
        13      Length of the following data (byte 14 to the end)
        14      Structure of EF
        15      Length of a record
    */

    g_au8ReadRec[4] = g_au8Buf[14]; // Phone book record length
    u32Cnt = ((g_au8Buf[2] << 8) + g_au8Buf[3]) / g_au8Buf[14];   // Phone book record number

    // Read or update EF_ADN can be protected by CHV1, so check if CHV1 is enabled
    if (((g_au8Buf[8] & 0x10) == 0x10) && (u32Chv1Disbled == 0))   //Protect by CHV1 ?
    {
        if (unlock_sim(u32Retry) < 0)
        {
            printf("Unlock SIM card failed\n");
            goto exit;
        }
    }

    read_phoneBook(u32Cnt);
    printf("Done\n");
exit:

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
