Windows Test Tool - HIDTransferTest
=====================================

A Windows command-line tool for testing HID Transfer on the VCOM + HID composite device.

Location
--------
BSP/Tool/HIDTransferTest

Usage
-----
HIDTransferTest -pid 0xDC00 -pagesize 512

Options
-------
  -pid 0xDC00      Specify the USB Product ID of the HID interface (VCOM + HID composite)
  -pagesize 512   Set the page size to 512 bytes (matches M251 RRAM page size)

Description
-----------
This tool communicates with the M251 device running the USBD_VCOM_And_HID_Transfer
firmware. It performs read/write/erase operations over the HID interface of the
VCOM + HID composite device, using the specified PID to identify the correct
USB device and a page size of 512 bytes aligned to the M251 RRAM page size.
