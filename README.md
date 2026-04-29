![Eclipse](https://github.com/OpenNuvoton/M251BSP/actions/workflows/Eclipse.yml/badge.svg)
![VSCode](https://github.com/OpenNuvoton/M251BSP/actions/workflows/VSCode.yml/badge.svg)

# M251/M252/M254/M256/M258 Series CMSIS BSP

To experience the powerful features of M251/M252/M254/M256/M258 series in few minutes, please select the sample code to download and execute on their NuMaker boards. Open the project files to build them with Keil® MDK, IAR, NuEclipse or VS Code, and then download and trace them on the NuMaker board to see how it works.


## .\Document\

- CMSIS.html<br>
	Document of CMSIS version 6.1.0.

- NuMicro M251_252_254_256_258 Series CMSIS BSP Driver Reference Guide.chm<br>
	This document describes the usage of drivers in M251/M252/M254/M256/M258 Series BSP.

- NuMicro M251_252_254_256_258 Series CMSIS BSP Revision History.pdf<br>
	This document shows the revision history of M251/M252/M254/M256/M258 Series BSP.

- VS Code Quick Start Guide
	This document guides to install, configure and use VS Code.


## .\Library\

- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V6.1.0 definitions by Arm® Corp.

- Device<br>
	CMSIS compliant device header files.

- LCDLib<br>
	Library for controlling LCD module.

- SmartcardLib<br>
	Smart card library binary and header file.

- StdDriver<br>
	All peripheral driver header and source files.

- TKLib<br>
	Library for controlling touch key module.


## .\Sample Code\

- CardReader<br>
	USB CCID smart card reader sample code.

- CortexM23<br>
	Cortex®-M23 sample code.

- FreeRTOS<br>
	Simple FreeRTOS™ demo code.
	
- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened.<p>
	The hard fault handler shows some information including program counter, which is the address where the processor is executed when the hard fault occurs. The listing file (or map file) can show what function and instruction that is.<p>
	It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample code for In-System-Programming.

- Level1_Training<br>
	Level 1 training sample code.

- NuMaker-M256SD<br>
	Sample code for NuMaker-M256SD board.

- NuMaker-M258KE<br>
	Sample code for NuMaker-M258KE board.

- NuMaker-M258KG<br>
	Sample code for NuMaker-M258KG board.

- PowerManagement<br>
	Sample code for power management.

- Semihost<br>
	Show how to print and get character through IDE console window.

- StdDriver<br>
	Sample code to demonstrate the usage of M251/M252/M254/M256/M258 series MCU peripheral driver APIs.

- Template<br>
	Project template for M251/M252/M254/M256/M258 series MCU.

- XOM<br>
	Demonstrate how to create XOM library and use it.


## .\ThirdParty\

- FreeRTOS<br>
	Real-time operating system for microcontrollers.


## .\Tool\

- TK<br>
	Touch key tool.


# License

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.<p>
M251/M252/M254/M256/M258 Series BSP files are provided under the Apache-2.0 license.
