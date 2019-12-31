/******************************************************************************//**
 * @file     hid_transfer.c
 * @version  V0.10
 * @brief    M251 series USBD HID transfer sample file
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "hid_transfer.h"

uint8_t volatile g_u8EP2Ready   = 0;
uint8_t volatile g_u8Suspend    = 0;
uint8_t g_u8Idle = 0, g_u8Protocol = 0;

void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
            g_u8Suspend = 0;
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enter power down to wait USB attached */
            g_u8Suspend = 1;

            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
            g_u8Suspend = 0;
        }

#ifdef SUPPORT_LPM

        if (u32State & USBD_STATE_L1SUSPEND)
        {
            /*
               TODO: Implement LPM SUSPEND flag here.
                     Recommend implementing the power-saving function in main loop.
            */
        }

        if (u32State & USBD_STATE_L1RESUME)
        {
            /*
               TODO: Implement LPM RESUME flag here.
            */
        }

#endif


    }

    if (u32IntSts & USBD_INTSTS_NEVWKIF_Msk)
    {
        /*Clear no-event wake up interrupt */
        USBD_CLR_INT_FLAG(USBD_INTSTS_NEVWKIF_Msk);
        /*
           TODO: Implement the function that will be executed when device is woken by non-USB event.
        */
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Interrupt IN
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Interrupt OUT
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }

        if (u32IntSts & USBD_INTSTS_EP8)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP8);
        }

        if (u32IntSts & USBD_INTSTS_EP9)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP9);
        }

        if (u32IntSts & USBD_INTSTS_EP10)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP10);
        }

        if (u32IntSts & USBD_INTSTS_EP11)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP11);
        }
    }
}

void EP2_Handler(void)  /* Interrupt IN handler */
{
    HID_SetInReport();
}

void EP3_Handler(void)  /* Interrupt OUT handler */
{
    uint8_t *pu8Ptr;
    /* Interrupt OUT */
    pu8Ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));
    HID_GetOutReport(pu8Ptr, USBD_GET_PAYLOAD_LEN(EP3));
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}


/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void HID_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Interrupt IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Interrupt OUT endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | INT_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

}

void HID_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if (buf[0] & 0x80)   /* request data transfer direction */
    {
        /* Device to host */
        switch (buf[1])
        {
            case GET_IDLE:
            {
                USBD_SET_PAYLOAD_LEN(EP1, buf[6]);
                /* Data stage */
                USBD_PrepareCtrlIn(&g_u8Idle, buf[6]);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case GET_PROTOCOL:
            {
                USBD_SET_PAYLOAD_LEN(EP1, buf[6]);
                /* Data stage */
                USBD_PrepareCtrlIn(&g_u8Protocol, buf[6]);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case GET_REPORT:

            //             {
            //                 break;
            //             }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
    else
    {
        /* Host to device */
        switch (buf[1])
        {
            case SET_REPORT:
            {
                if (buf[3] == 3)
                {
                    /* Request Type = Feature */
                    USBD_SET_DATA1(EP1);
                    USBD_SET_PAYLOAD_LEN(EP1, 0);
                }

                break;
            }

            case SET_IDLE:
            {
                g_u8Idle = buf[3];
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_PROTOCOL:
            {
                g_u8Protocol = buf[2];
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            default:
            {
                /* Stall */
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
}

/***************************************************************/
#define HID_CMD_SIGNATURE   0x43444948

/* HID Transfer Commands */
#define HID_CMD_NONE     0x00
#define HID_CMD_ERASE    0x71
#define HID_CMD_READ     0xD2
#define HID_CMD_WRITE    0xC3
#define HID_CMD_TEST     0xB4

#define PAGE_SIZE        512
#define TEST_PAGES       4
#define SECTOR_SIZE      1024
#define START_SECTOR     0x10

#pragma pack(1)
typedef struct
{
    uint8_t u8Cmd;
    uint8_t u8Size;
    uint32_t u32Arg1;
    uint32_t u32Arg2;
    uint32_t u32Signature;
    uint32_t u32Checksum;
} CMD_T;

CMD_T g_sCmd;

static uint8_t  s_au8PageBuff[PAGE_SIZE] = {0};    /* Page buffer to upload/download through HID report */
static uint32_t s_u32BytesInPageBuf = 0;          /* The bytes of data in s_au8PageBuff */
static uint8_t  s_au8TestPages[TEST_PAGES * PAGE_SIZE] = {0};    /* Test pages to upload/download through HID report */

int32_t HID_CmdEraseSectors(CMD_T *psCmd)
{
    uint32_t u32StartSector;
    uint32_t u32Sectors;

    u32StartSector = psCmd->u32Arg1 - START_SECTOR;
    u32Sectors = psCmd->u32Arg2;

    printf("Erase command - Sector: %d   Sector Cnt: %d\n", u32StartSector, u32Sectors);

    /* TODO: To erase the sector of storage */
    memset(s_au8TestPages + u32StartSector * SECTOR_SIZE, 0xFF, sizeof(uint8_t) * u32Sectors * SECTOR_SIZE);

    /* To note the command has been done */
    psCmd->u8Cmd = HID_CMD_NONE;

    return 0;
}


int32_t HID_CmdReadPages(CMD_T *psCmd)
{
    uint32_t u32StartPage;
    uint32_t u32Pages;

    u32StartPage = psCmd->u32Arg1;
    u32Pages     = psCmd->u32Arg2;

    printf("Read command - Start page: %d    Pages Numbers: %d\n", u32StartPage, u32Pages);

    if (u32Pages)
    {
        /* Update data to page buffer to upload */
        /* TODO: We need to update the page data if got a page read command. (0xFF is used in this sample code) */
        memcpy(s_au8PageBuff, s_au8TestPages, sizeof(s_au8PageBuff));
        s_u32BytesInPageBuf = PAGE_SIZE;

        /* The signature word is used as page counter */
        psCmd->u32Signature = 1;

        /* Trigger HID IN */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (void *)s_au8PageBuff, EP2_MAX_PKT_SIZE);
        USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);
        s_u32BytesInPageBuf -= EP2_MAX_PKT_SIZE;
    }

    return 0;
}


int32_t HID_CmdWritePages(CMD_T *psCmd)
{
    uint32_t u32StartPage;
    uint32_t u32Pages;

    u32StartPage = psCmd->u32Arg1;
    u32Pages     = psCmd->u32Arg2;

    printf("Write command - Start page: %d    Pages Numbers: %d\n", u32StartPage, u32Pages);
    s_u32BytesInPageBuf = 0;

    /* The signature is used to page counter */
    psCmd->u32Signature = 0;

    return 0;
}


int32_t gi32CmdTestCnt = 0;
int32_t HID_CmdTest(CMD_T *psCmd)
{
    int32_t i;
    uint8_t *pu8Ptr;

    pu8Ptr = (uint8_t *)psCmd;
    printf("Get test command #%d (%d bytes)\n", gi32CmdTestCnt++, psCmd->u8Size);

    for (i = 0; i < psCmd->u8Size; i++)
    {
        if ((i & 0xF) == 0)
        {
            printf("\n");
        }

        printf(" %02x", pu8Ptr[i]);
    }

    printf("\n");


    /* To note the command has been done */
    psCmd->u8Cmd = HID_CMD_NONE;

    return 0;
}


uint32_t CalCheckSum(uint8_t *pu8Buf, uint32_t u32Size)
{
    uint32_t u32Sum;
    int32_t i;

    i = 0;
    u32Sum = 0;

    while (u32Size--)
    {
        u32Sum += pu8Buf[i++];
    }

    return u32Sum;

}


int32_t ProcessCommand(uint8_t *pu8Buffer, uint32_t u32BufferLen)
{
    uint32_t u32Sum;


    USBD_MemCopy((uint8_t *)&g_sCmd, pu8Buffer, u32BufferLen);

    /* Check size */
    if ((g_sCmd.u8Size > sizeof(g_sCmd)) || (g_sCmd.u8Size > u32BufferLen))
        return -1;

    /* Check signature */
    if (g_sCmd.u32Signature != HID_CMD_SIGNATURE)
        return -1;

    /* Calculate checksum & check it*/
    u32Sum = CalCheckSum((uint8_t *)&g_sCmd, g_sCmd.u8Size);

    if (u32Sum != g_sCmd.u32Checksum)
        return -1;

    switch (g_sCmd.u8Cmd)
    {
        case HID_CMD_ERASE:
        {
            HID_CmdEraseSectors(&g_sCmd);
            break;
        }

        case HID_CMD_READ:
        {
            HID_CmdReadPages(&g_sCmd);
            break;
        }

        case HID_CMD_WRITE:
        {
            HID_CmdWritePages(&g_sCmd);
            break;
        }

        case HID_CMD_TEST:
        {
            HID_CmdTest(&g_sCmd);
            break;
        }

        default:
            return -1;
    }

    return 0;
}


void HID_GetOutReport(uint8_t *pu8EpBuf, uint32_t u32Size)
{
    uint8_t  u8Cmd;
    uint32_t u32StartPage;
    uint32_t u32Pages;
    uint32_t u32PageCnt;

    /* Get command information */
    u8Cmd        = g_sCmd.u8Cmd;
    u32StartPage = g_sCmd.u32Arg1;
    u32Pages     = g_sCmd.u32Arg2;
    u32PageCnt   = g_sCmd.u32Signature; /* The signature word is used to count pages */


    /* Check if it is in the data phase of write command */
    if ((u8Cmd == HID_CMD_WRITE) && (u32PageCnt < u32Pages))
    {
        /* Process the data phase of write command */

        /* Get data from HID OUT */
        USBD_MemCopy(&s_au8PageBuff[s_u32BytesInPageBuf], pu8EpBuf, EP3_MAX_PKT_SIZE);
        s_u32BytesInPageBuf += EP3_MAX_PKT_SIZE;

        /* The HOST must make sure the data is PAGE_SIZE alignment */
        if (s_u32BytesInPageBuf >= PAGE_SIZE)
        {
            printf("Writing page %d\n", u32StartPage + u32PageCnt);
            /* TODO: We should program received data to storage here */
            memcpy(s_au8TestPages + u32PageCnt * PAGE_SIZE, s_au8PageBuff, sizeof(s_au8PageBuff));

            u32PageCnt++;

            /* Write command complete! */
            if (u32PageCnt >= u32Pages)
            {
                u8Cmd = HID_CMD_NONE;

                printf("Write command complete.\n");
            }

            s_u32BytesInPageBuf = 0;

        }

        /* Update command status */
        g_sCmd.u8Cmd        = u8Cmd;
        g_sCmd.u32Signature = u32PageCnt;
    }
    else
    {
        /* Check and process the command packet */
        if (ProcessCommand(pu8EpBuf, sizeof(g_sCmd)))
        {
            printf("Unknown HID command!\n");
        }
    }
}

void HID_SetInReport(void)
{
    uint32_t u32StartPage;
    uint32_t u32TotalPages;
    uint32_t u32PageCnt;
    uint8_t *ptr;
    uint8_t u8Cmd;

    u8Cmd        = g_sCmd.u8Cmd;
    u32StartPage = g_sCmd.u32Arg1;
    u32TotalPages = g_sCmd.u32Arg2;
    u32PageCnt   = g_sCmd.u32Signature;

    /* Check if it is in data phase of read command */
    if (u8Cmd == HID_CMD_READ)
    {
        /* Process the data phase of read command */
        if ((u32PageCnt >= u32TotalPages) && (s_u32BytesInPageBuf == 0))
        {
            /* The data transfer is complete. */
            u8Cmd = HID_CMD_NONE;
            printf("Read command complete!\n");
        }
        else
        {
            if (s_u32BytesInPageBuf == 0)
            {
                /* The previous page has sent out. Read new page to page buffer */
                /* TODO: We should update new page data here. (0xFF is used in this sample code) */
                printf("Reading page %d\n", u32StartPage + u32PageCnt);
                memcpy(s_au8PageBuff, s_au8TestPages + u32PageCnt * PAGE_SIZE, sizeof(s_au8PageBuff));

                s_u32BytesInPageBuf = PAGE_SIZE;

                /* Update the page counter */
                u32PageCnt++;
            }

            /* Prepare the data for next HID IN transfer */
            ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));
            USBD_MemCopy(ptr, (void *)&s_au8PageBuff[PAGE_SIZE - s_u32BytesInPageBuf], EP2_MAX_PKT_SIZE);
            USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);
            s_u32BytesInPageBuf -= EP2_MAX_PKT_SIZE;
        }
    }

    g_sCmd.u8Cmd        = u8Cmd;
    g_sCmd.u32Signature = u32PageCnt;

}



