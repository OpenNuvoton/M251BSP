/****************************************************************************//**
 * @file     descriptors.c
 * @version  V0.10
 * @brief    USBD descriptors
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include "NuMicro.h"
#include "cdc_serial.h"
#include "massstorage.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t gu8DeviceDescriptor[] =
{
    LEN_DEVICE,                 /* bLength */
    DESC_DEVICE,                /* bDescriptorType */
#ifdef SUPPORT_LPM
    0x01, 0x02,                 /* bcdUSB => 0x0201 to support LPM */
#else
    0x10, 0x01,                 /* bcdUSB */
#endif
    0xEF,                       /* bDeviceClass: IAD */
    0x02,                       /* bDeviceSubClass */
    0x01,                       /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,           /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    (USBD_VID & 0xFF00) >> 8,
                        /* idProduct */
                        USBD_PID & 0x00FF,
                        (USBD_PID & 0xFF00) >> 8,
                        0x00, 0x00,                 /* bcdDevice */
                        0x01,                       /* iManufacture */
                        0x02,                       /* iProduct */
                        0x03,                       /* iSerialNumber */
                        0x01                        /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
const uint8_t gu8ConfigDescriptor[] =
{
    LEN_CONFIG,         /* bLength              */
    DESC_CONFIG,        /* bDescriptorType      */
    0x62, 0x00,         /* wTotalLength         */
    0x03,               /* bNumInterfaces       */
    0x01,               /* bConfigurationValue  */
    0x00,               /* iConfiguration       */
    0xC0,               /* bmAttributes         */
    0x32,               /* MaxPower             */

    // IAD
    0x08,               // bLength: Interface Descriptor size
    0x0B,               // bDescriptorType: IAD
    0x00,               // bFirstInterface
    0x02,               // bInterfaceCount
    0x02,               // bFunctionClass: CDC
    0x02,               // bFunctionSubClass
    0x01,               // bFunctionProtocol
    0x00,               // iFunction

    /* VCOM */
    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x00,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x01,           /* bNumEndpoints        */
    0x02,           /* bInterfaceClass      */
    0x02,           /* bInterfaceSubClass   */
    0x01,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x00,           /* Header functional descriptor subtype */
    0x10, 0x01,     /* Communication device compliant to the communication spec. ver. 1.10 */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x01,           /* Call management functional descriptor */
    0x00,           /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a Data Class Interface 0 */
    0x01,           /* Interface number of data class interface optionally used for call management */

    /* Communication Class Specified INTERFACE descriptor */
    0x04,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x02,           /* Abstract control management functional descriptor subtype */
    0x02,           /* bmCapabilities       */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* bLength              */
    0x24,           /* bDescriptorType: CS_INTERFACE descriptor type */
    0x06,           /* bDescriptorSubType   */
    0x00,           /* bMasterInterface     */
    0x01,           /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM),     /* bEndpointAddress */
    EP_INT,                         /* bmAttributes     */
    EP4_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x01,                           /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,                  /* bLength              */
    DESC_INTERFACE,                 /* bDescriptorType      */
    0x01,                           /* bInterfaceNumber     */
    0x00,                           /* bAlternateSetting    */
    0x02,                           /* bNumEndpoints        */
    0x0A,                           /* bInterfaceClass      */
    0x00,                           /* bInterfaceSubClass   */
    0x00,                           /* bInterfaceProtocol   */
    0x00,                           /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM),    /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP2_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM),  /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP3_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* MSC class device */
    /* INTERFACE descriptor */
    LEN_INTERFACE,  // bLength
    DESC_INTERFACE, // bDescriptorType
    0x02,           // bInterfaceNumber
    0x00,           // bAlternateSetting
    0x02,           // bNumEndpoints
    0x08,           // bInterfaceClass
    0x06,           // bInterfaceSubClass
    0x50,           // bInterfaceProtocol
    0x00,           // iInterface

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    // bLength
    DESC_ENDPOINT,                   // bDescriptorType
    (EP_INPUT | BULK_IN_EP_NUM_1),   // bEndpointAddress
    EP_BULK,                         // bmAttributes
    EP7_MAX_PKT_SIZE, 0x00,          // wMaxPacketSize
    0x00,                            // bInterval

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    // bLength
    DESC_ENDPOINT,                   // bDescriptorType
    (EP_OUTPUT | BULK_OUT_EP_NUM_1), // bEndpointAddress
    EP_BULK,                         // bmAttributes
    EP6_MAX_PKT_SIZE, 0x00,          // wMaxPacketSize
    0x00                             // bInterval
};

/*!<USB BOS Descriptor */
const uint8_t gu8BosDescriptor[] =
{
    LEN_BOS,                         /* bLength         */
    DESC_BOS,                        /* bDescriptorType */
    ((LEN_BOS + LEN_DEVCAP) & 0xFF), /* wTotalLength    */
    ((LEN_BOS + LEN_DEVCAP) >> 8),   /* wTotalLength    */
    0x01,                            /* bNumDevcieCaps  */
    LEN_DEVCAP,                      /* bLength         */
    DESC_CAPABILITY,                 /* bDescriptorType */
    CAP_USB20_EXT,                   /* bDevCapabilityType, 0x02 is USB 2.0 Extension */
    0x06, 0x04, 0x00, 0x00           /* bmAttributs, 32 bits                          */
    /* bit 0     : Reserved. Must be 0.                                         */
    /* bit 1     : Set 1 to support LPM.                                        */
    /* bit 2     : Set 1 to support BSL & Alternative HIRD.                     */
    /* bit 3     : Set 1 to use Baseline BESL.                                  */
    /* bit 4     : Set 1 to use Deep BESL.                                      */
    /* bit 11:8  : Baseline BESL value. Ignore it if bit3 is zero.              */
    /* bit 15:12 : Deep BESL value. Ignore it if bit4 is zero.                  */
    /* bit 31:16 : Reserved. Must be 0.                                         */
};


/*!<USB Language String Descriptor */
const uint8_t gu8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
const uint8_t gu8VendorStringDesc[] =
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
const uint8_t gu8ProductStringDesc[] =
{
    22,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0
};

uint8_t gu8StringSerial[26] =
{
    26,             // bLength
    DESC_STRING,    // bDescriptorType
    'A', 0, '0', 0, '2', 0, '0', 0, '1', 0, '5', 0, '0', 0, '8', 0, '1', 0, '9', 0, '0', 0, '1', 0
};

const uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const S_USBD_INFO_T gsInfo =
{
    (uint8_t *) gu8DeviceDescriptor,
    (uint8_t *) gu8ConfigDescriptor,
    (uint8_t **)gpu8UsbString,
    (uint8_t **)NULL,
#ifdef SUPPORT_LPM
    (uint8_t *) gu8BosDescriptor,
#else
    0,
#endif
    (uint32_t *)NULL,
    (uint32_t *)NULL,
};

