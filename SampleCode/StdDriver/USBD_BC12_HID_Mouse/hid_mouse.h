/****************************************************************************//**
 * @file     hid_mouse.h
 * @brief    M258 series USB HID mouse header file
 *
 * @note
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_HID_H__
#define __USBD_HID_H__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0x0123

/*!<Define HID Class Specific Request */
#define GET_REPORT          0x01
#define GET_IDLE            0x02
#define GET_PROTOCOL        0x03
#define SET_REPORT          0x09
#define SET_IDLE            0x0A
#define SET_PROTOCOL        0x0B

/*!<USB HID Interface Class protocol */
#define HID_NONE            0x00
#define HID_KEYBOARD        0x01
#define HID_MOUSE           0x02

/*!<USB HID Class Report Type */
#define HID_RPT_TYPE_INPUT      0x01
#define HID_RPT_TYPE_OUTPUT     0x02
#define HID_RPT_TYPE_FEATURE    0x03

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    8
#define EP1_MAX_PKT_SIZE    EP0_MAX_PKT_SIZE
#define EP2_MAX_PKT_SIZE    8

#define SETUP_BUF_BASE  0
#define SETUP_BUF_LEN   8
#define EP0_BUF_BASE    (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN     EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE    (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN     EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE    (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN     EP2_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define INT_IN_EP_NUM   0x01

/* Define Descriptor information */
#define HID_DEFAULT_INT_IN_INTERVAL     10
#define USBD_SELF_POWERED               0
#define USBD_REMOTE_WAKEUP              0
#define USBD_MAX_POWER                  50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

#define LEN_CONFIG_AND_SUBORDINATE      (LEN_CONFIG+LEN_INTERFACE+LEN_HID+LEN_ENDPOINT)


#define USBD_BCDC_DETMOD_IDLE   (0x00 << USBD_BCDC_DETMOD_Pos)
#define USBD_BCDC_DETMOD_VBUS   (0x01 << USBD_BCDC_DETMOD_Pos)
#define USBD_BCDC_DETMOD_DCD    (0x02 << USBD_BCDC_DETMOD_Pos)
#define USBD_BCDC_DETMOD_PD     (0x03 << USBD_BCDC_DETMOD_Pos)
#define USBD_BCDC_DETMOD_SD     (0x04 << USBD_BCDC_DETMOD_Pos)

#define USBD_BCDC_DETSTS_VBUS_UNREACH       (0x0 << USBD_BCDC_DETSTS_Pos)
#define USBD_BCDC_DETSTS_VBUS_REACH         (0x1 << USBD_BCDC_DETSTS_Pos)

#define USBD_BCDC_DETSTS_DCD_DATA_UNCONTACT (0x0 << USBD_BCDC_DETSTS_Pos)
#define USBD_BCDC_DETSTS_DCD_DATA_CONTACT   (0x1 << USBD_BCDC_DETSTS_Pos)

#define USBD_BCDC_DETSTS_PD_SDP_NUSP        (0x0 << USBD_BCDC_DETSTS_Pos)
#define USBD_BCDC_DETSTS_PD_DCP_CDP         (0x1 << USBD_BCDC_DETSTS_Pos)

#define USBD_BCDC_DETSTS_SD_CDP             (0x0 << USBD_BCDC_DETSTS_Pos)
#define USBD_BCDC_DETSTS_SD_DCP             (0x1 << USBD_BCDC_DETSTS_Pos)

typedef enum _USBD_BC12_PD_STATUS
{
    USBD_BC12_VBUS_OFF = 0,
    USBD_BC12_CDP,
    USBD_BC12_DCP,
    USBD_BC12_SDP,
    USBD_BC12_ERROR
} S_USBD_BC12_PD_STATUS; /*!< Detection result of API USB_BC_Detection. */

/*-------------------------------------------------------------*/
void HID_Init(void);
void HID_ClassRequest(void);

void EP2_Handler(void);
void HID_UpdateMouseData(void);

extern uint8_t volatile g_u8Suspend;

#endif  /* __USBD_HID_H_ */
