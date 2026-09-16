Windows Test Tool - HIDTransferTest
=====================================

A Windows command-line tool for testing HID Transfer with Control endpoint.

Location
--------
BSP/Tool/HIDTransferTest

Usage
-----
HIDTransferTest -pagesize 512 -ctrl

Options
-------
  -pagesize 512   Set the page size to 512 bytes (matches M251 RRAM page size)
  -ctrl            Use Control endpoint for HID transfer

Description
-----------
This tool communicates with the M251 device running the USBD_HID_Transfer_CTRL
firmware. It performs read/write/erase operations over HID using the Control
endpoint, with a page size of 512 bytes aligned to the M251 RRAM page size.
