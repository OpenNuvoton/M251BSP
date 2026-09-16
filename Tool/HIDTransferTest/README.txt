HID Transfer Test Tool - Command Line Arguments
================================================

VERSION
-------
Tool Version: 1.1.0
Copyright (C) 2010-2026 Nuvoton Technology Corp. All rights reserved.

SYNOPSIS
--------
HIDTransferTest.exe [-pid <product_id>] [-pagesize <size>] [-pages <count>] 
					[-base <address>] [-verify] [-ctrl] [-v|--version] [-h|--help|/?]

DESCRIPTION
-----------
HID Transfer Test Tool supports two transfer modes:
1. Interrupt Transfer Mode (default) - For flash memory programming/verification
2. Control Transfer Mode (-ctrl) - For HID control transfer testing

OPTIONS
-------
-pid <product_id>
	USB Product ID in hexadecimal (0x prefix) or decimal format.
	Examples: 0x5020, 0x5021, 20512
	Default: 0x5020

-pagesize <size>
	Page size in bytes. Typically 512 or 1024.
	Default: 512 (or 1024 if HID_MAX_PACKET_SIZE_EP == 1024)
	Note: Only applicable in Interrupt Transfer mode

-pages <count>
	Number of pages to test (decimal or hexadecimal with 0x prefix).
	Examples: 100, 0x64
	Default: 1 page
	Note: Only applicable in Interrupt Transfer mode

-base <address>
	Test base address in hexadecimal (0x prefix) or decimal format.
	Examples: 0x10000, 0x20000, 65536
	Default: 0x10000 (64KB)
	Note: Only applicable in Interrupt Transfer mode

-verify
	Enable blank check and data verification tests.
	Default: Disabled
	Note: Only applicable in Interrupt Transfer mode

-ctrl, -control
	Use HID Control Transfer mode (HidD_SetOutputReport/HidD_GetInputReport).
	In this mode, the tool performs a single control transfer test:
	  - Sends an Output Report with test pattern data
	  - Receives an Input Report and displays the data
	Default: Interrupt Transfer mode
	Note: When enabled, -pages, -base, -pagesize, and -verify are ignored

-v, -version, --version
	Display version information and exit.

-h, --help, /?
	Display this help message and exit.

TRANSFER MODES
--------------

1. INTERRUPT TRANSFER MODE (Default)
   - Used for flash memory programming and verification
   - Supports erase, read, write operations
   - Configurable test parameters: pages, base address, page size
   - Optional verification with blank check and data verify

2. CONTROL TRANSFER MODE (-ctrl)
   - Uses HidD_SetOutputReport and HidD_GetInputReport
   - Performs a single test cycle:
	 * Send Output Report with test pattern (0x00, 0x01, 0x02, ...)
	 * Wait 100ms for device processing
	 * Receive Input Report
	 * Display received data
   - No timer events - single execution
   - Suitable for testing HID control transfer functionality

EXAMPLES
--------

INTERRUPT TRANSFER MODE (Default):
-----------------------------------
1. Use default settings (PID=0x5020, 1 page, base=0x10000):
   HIDTransferTest.exe

2. Use custom USB PID (hexadecimal):
   HIDTransferTest.exe -pid 0x5021

3. Test 10 pages starting at address 0x20000:
   HIDTransferTest.exe -pages 10 -base 0x20000

4. Test with verification enabled:
   HIDTransferTest.exe -pages 100 -verify

5. Full configuration with all parameters:
   HIDTransferTest.exe -pid 0x5021 -pagesize 1024 -pages 200 -base 0x30000 -verify

6. Test large memory region:
   HIDTransferTest.exe -pages 0x100 -base 0x40000

CONTROL TRANSFER MODE:
----------------------
1. Basic control transfer test with default PID:
   HIDTransferTest.exe -ctrl

2. Control transfer test with custom PID:
   HIDTransferTest.exe -pid 0x5020 -ctrl

3. Control transfer test (alternative syntax):
   HIDTransferTest.exe -pid 0x5021 -control

Note: In Control Transfer mode, -pages, -base, -pagesize, and -verify 
	  parameters are ignored.

CONFIGURATION OUTPUT
--------------------
The tool displays the active configuration before running tests:
- USB VID: 0x0416 (fixed - Nuvoton Technology)
- USB PID: (configurable via -pid)
- Transfer Mode: Interrupt Transfer or Control Transfer
- Page Size: (in Interrupt mode only)
- Test Pages: (in Interrupt mode only)
- Test Base: (in Interrupt mode only)
- Test Size: (in Interrupt mode only)
- Verification: Enabled or Disabled (in Interrupt mode only)

In Control Transfer mode, additional information is displayed:
- Output Report Length (bytes)
- Input Report Length (bytes)

OUTPUT EXAMPLES
---------------

Interrupt Transfer Mode:
-------------------------
Configuration:
  USB VID: 0x0416
  USB PID: 0x5020
  Transfer Mode: Interrupt Transfer
  Page Size: 512 bytes
  Test Pages: 10
  Test Base: 0x00020000
  Test Size: 5120 bytes (0x1400)
  Verification: Enabled

>>> Erase sectors: 4 - 13
>>> Write pages: 64 - 73
>>> Read pages: 64 - 73
Blank check passed.
Data verify passed.

HID Transfer test ok! (All tests passed)

Control Transfer Mode:
-----------------------
Configuration:
  USB VID: 0x0416
  USB PID: 0x5020
  Transfer Mode: Control Transfer

*** Control Transfer Mode ***
  Output Report Length: 64 bytes
  Input Report Length: 64 bytes
Sending Output Report...
Output Report sent successfully.
Receiving Input Report...
Input Report received successfully.
Received Input Report Data:
00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F
20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F
30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F
Control Transfer test completed successfully!

RETURN VALUES
-------------
0  - Success
-1 - Error (device not found, communication error, verification failed, etc.)

NOTES
-----
- USB VID is fixed at 0x0416 (Nuvoton Technology Corp.)
- In Interrupt Transfer mode, page size should match your target device's 
  flash page size (typically 512 or 1024 bytes)
- Control Transfer mode performs a single test and exits immediately 
  (no periodic/timer-based transfers)
- For repeated Control Transfer tests, run the command multiple times or 
  use a script/batch file
- Test base address and pages should be within your device's valid memory range
- Verification adds additional read operations to confirm data integrity

TROUBLESHOOTING
---------------
1. "Can't Open HID Device"
   - Check that device is connected
   - Verify correct USB PID with -pid parameter
   - Ensure device drivers are installed

2. "Device not detected"
   - Check USB connection
   - Verify VID (0x0416) and PID match your device
   - Check if device is in use by another application

3. "ERROR: Failed to allocate buffer"
   - Reduce number of test pages (-pages parameter)
   - System may have insufficient memory

4. Verification failures in Interrupt mode
   - Ensure device supports erase/write operations
   - Check if memory region is writable
   - Try smaller page counts first
   - Use -verify only when device firmware supports verification

5. Control Transfer failures
   - Ensure device firmware implements HidD_GetInputReport/SetOutputReport
   - Check that device Usage Page is 0xFF (vendor-defined)
   - Verify report descriptors are configured correctly

TECHNICAL DETAILS
-----------------
Interrupt Transfer Mode:
- Uses HID Interrupt endpoints
- Supports custom command protocol with:
  * HID_CMD_ERASE (0x71) - Erase sectors
  * HID_CMD_READ (0xD2) - Read pages
  * HID_CMD_WRITE (0xC3) - Write pages
  * HID_CMD_TEST (0xB4) - Test command
- Command packet includes signature (0x43444948) and checksum
- Packet size: 512 or 1024 bytes (compile-time configuration)

Control Transfer Mode:
- Uses HidD_SetOutputReport and HidD_GetInputReport APIs
- Synchronous, single-shot operation
- Test pattern: Sequential bytes (0x00, 0x01, 0x02, ...)
- 100ms delay between send and receive
- Displays received data in hexadecimal format

COMPATIBILITY
-------------
- Windows 7 and later
- Requires HID.DLL and SETUPAPI.DLL
- Compatible with devices using VID 0x0416 (Nuvoton)
- Supports both 512-byte and 1024-byte HID packet sizes

VERSION HISTORY
---------------
1.1.0 - Control Transfer support and enhanced features (2026)
	  - Added -ctrl parameter for Control Transfer mode
	  - Control Transfer uses HidD_SetOutputReport/HidD_GetInputReport
	  - No timer events - single execution for scripting compatibility
	  - Added -pages parameter for configurable test size
	  - Added -base parameter for configurable test address
	  - Added -verify parameter for optional verification
	  - Added -version parameter
	  - Default test pages changed from 1024 to 1
	  - Improved help and usage messages
	  - Dynamic memory allocation for test buffers
	  - Better error handling and status messages
	  - Comprehensive documentation updates

1.0.0 - Initial release
	  - Basic Interrupt Transfer mode
	  - Fixed parameters (512/1024 pages, fixed base address)
	  - USB VID 0x0416, configurable PID

CONTACT
-------
For issues or questions, please contact Nuvoton Technology Corp.

