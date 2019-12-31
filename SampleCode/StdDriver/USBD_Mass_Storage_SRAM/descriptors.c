/****************************************************************************//**
 * @file     descriptors.c
 * @brief    M251 series USBD driver source file
 * @version  1.0.0
 * @date     August, 2019
 *
 * @note
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "massstorage.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8DeviceDescriptor[] =
{
#else
uint8_t gu8DeviceDescriptor[] __attribute__((aligned(4))) =
{
#endif
    LEN_DEVICE,         /* bLength */
    DESC_DEVICE,        /* bDescriptorType */
    0x10, 0x01,         /* bcdUSB */
    0x00,               /* bDeviceClass */
    0x00,               /* bDeviceSubClass */
    0x00,               /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    (USBD_VID & 0xFF00) >> 8,
                        /* idProduct */
                        USBD_PID & 0x00FF,
                        (USBD_PID & 0xFF00) >> 8,
                        0x00, 0x00,        /* bcdDevice */
                        0x01,              /* iManufacture */
                        0x02,              /* iProduct */
                        0x03,              /* iSerialNumber -  is required for BOT device */
                        0x01               /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8ConfigDescriptor[] =
{
#else
uint8_t gu8ConfigDescriptor[] __attribute__((aligned(4))) =
{
#endif
    LEN_CONFIG,                                         // bLength
    DESC_CONFIG,                                        // bDescriptorType
    (LEN_CONFIG + LEN_INTERFACE + LEN_ENDPOINT * 2), 0x00, // wTotalLength
    0x01,                                               // bNumInterfaces
    0x01,                                               // bConfigurationValue
    0x00,                                               // iConfiguration
    0xC0,                                               // bmAttributes
    0x32,                                               // MaxPower

    /* const BYTE cbyInterfaceDescriptor[LEN_INTERFACE] = */
    LEN_INTERFACE,  // bLength
    DESC_INTERFACE, // bDescriptorType
    0x00,           // bInterfaceNumber
    0x00,           // bAlternateSetting
    0x02,           // bNumEndpoints
    0x08,           // bInterfaceClass
    0x05,           // bInterfaceSubClass
    0x50,           // bInterfaceProtocol
    0x00,           // iInterface

    /* const BYTE cbyEndpointDescriptor1[LEN_ENDPOINT] = */
    LEN_ENDPOINT,           // bLength
    DESC_ENDPOINT,          // bDescriptorType
    (BULK_IN_EP_NUM | EP_INPUT), // bEndpointAddress
    EP_BULK,                // bmAttributes
    EP2_MAX_PKT_SIZE, 0x00,  // wMaxPacketSize
    0x00,                   // bInterval

    /* const BYTE cbyEndpointDescriptor2[LEN_ENDPOINT] = */
    LEN_ENDPOINT,           // bLength
    DESC_ENDPOINT,          // bDescriptorType
    BULK_OUT_EP_NUM,                   // bEndpointAddress
    EP_BULK,                // bmAttributes
    EP3_MAX_PKT_SIZE, 0x00,  // wMaxPacketSize
    0x00                    // bInterval
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
    0x06, 0x04, 0x00, 0x00  /* bmAttributs, 32 bits     */
    /* bit 0 : Reserved. Must 0.                                         */
    /* bit 1 : 1 to support LPM.                                         */
    /* bit 2 : 1 to support BSL & Alternat HIRD                          */
    /* bit 3 : 1 to recommend Baseline BESL                              */
    /* bit 4 : 1 to recommand Deep BESL                                  */
    /* bit 11:8 : Recommend Baseline BESL value. Ignore by bit3 is zero. */
    /* bit 15:12 : Recommend Deep BESL value. Ignore by bit4 is zero.    */
    /* bit 31:16 : Reserved. Must 0.                                     */
};

/*!<USB Language String Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8StringLang[4] =
{
#else
uint8_t gu8StringLang[4] __attribute__((aligned(4))) =
{
#endif
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8VendorStringDesc[] =
{
#else
uint8_t gu8VendorStringDesc[] __attribute__((aligned(4))) =
{
#endif
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8ProductStringDesc[] =
{
#else
uint8_t gu8ProductStringDesc[] __attribute__((aligned(4))) =
{
#endif
    22,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0
};

#ifdef __ICCARM__
    #pragma data_alignment=4
    uint8_t gu8StringSerial[] =
#else
    uint8_t gu8StringSerial[] __attribute__((aligned(4))) =
#endif
{
    26,             // bLength
    DESC_STRING,    // bDescriptorType
    'A', 0, '0', 0, '0', 0, '0', 0, '0', 0, '8', 0, '0', 0, '4', 0, '0', 0, '1', 0, '1', 0, '5', 0

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
    (uint8_t *)gu8DeviceDescriptor,
    (uint8_t *)gu8ConfigDescriptor,
    (uint8_t **)gpu8UsbString,
    NULL,
    NULL,
    NULL,
    NULL
};

