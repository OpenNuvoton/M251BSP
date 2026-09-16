Overview
========

ISP_DFU is a sample code that implements a USB DFU (Device Firmware Upgrade)
in-system programmer (ISP).

When ISP_DFU firmware is running on the target device, the device enumerates
as a USB DFU device.

This allows the host PC to transfer application firmware over USB (download and
upload), without extra programmer hardware.

To perform firmware transfer, install the dfu-util command-line tool on the
host side.


DFU Utility Tool (dfu-util)
===========================

1. Download Tool
----------------

Official release page:
https://dfu-util.sourceforge.net/releases/


2. Download Firmware (Host -> Target)
-------------------------------------

Command:

	.\dfu-util.exe -a 0 -d 0x416:0xBDF0 -D [file]

Parameters:

	-a 0             Select alternate interface 0.
	-d 0x416:0xBDF0  Target USB Vendor ID (0x416) and Product ID (0xBDF0).
	-D [file]        Firmware binary file path to write to target.

Example:

	.\dfu-util.exe -a 0 -d 0x416:0xBDF0 -D TIMER_Periodic.bin


3. Upload Firmware (Target -> Host)
-----------------------------------

Command:

	.\dfu-util.exe -a 0 -d 0x416:0xBDF0 -U [file]

Parameters:

	-a 0             Select alternate interface 0.
	-d 0x416:0xBDF0  Target USB Vendor ID (0x416) and Product ID (0xBDF0).
	-U [file]        Output file path on host for uploaded data.

Example:

	.\dfu-util.exe -a 0 -d 0x416:0xBDF0 -U backup.bin
