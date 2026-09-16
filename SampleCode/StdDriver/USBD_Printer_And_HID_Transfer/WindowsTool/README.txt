Windows Test Tool - HIDTransferTest
=====================================

A Windows command-line tool for testing HID Transfer on the Printer + HID composite device.

Location
--------
BSP/Tool/HIDTransferTest

Usage
-----
HIDTransferTest -pid 0xAACC -pagesize 512

Options
-------
  -pid 0xAACC      Specify the USB Product ID of the HID interface (Printer + HID composite)
  -pagesize 512   Set the page size to 512 bytes (matches M251 RRAM page size)

Description
-----------
This tool communicates with the M251 device running the USBD_Printer_And_HID_Transfer
firmware. It performs read/write/erase operations over the HID interface of the
Printer + HID composite device, using the specified PID to identify the correct
USB device and a page size of 512 bytes aligned to the M251 RRAM page size.
