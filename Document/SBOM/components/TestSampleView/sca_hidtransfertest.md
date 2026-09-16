# HIDTransferTest Component Description (for SCA / SBOM)

Component metadata for the **HID Transfer Test Tool** Windows host utility located under `Tool/HIDTransferTest`.

## 1) Component Identity

- Component name (`name`): `HIDTransferTest`
- Component type (`type`): `application` (Windows Win32 command-line executable)
- Supplier / author: `Nuvoton Technology Corporation`
- Version: `1.1.0`
- Copyright: `Copyright (C) 2010-2026 Nuvoton Technology Corp. All rights reserved.`
- Origin: **first-party**
- Integration: **vendored_source_and_binary**
- Evidence path: `Tool/HIDTransferTest`

### Key files

- Executable: `Tool/HIDTransferTest/Debug/HIDTransferTest.exe`
- Documentation: `README.txt`
- License: `LICENSE.md`

### Version evidence

- `Tool/HIDTransferTest/HIDTransferTest/version.h` &rarr; `TOOL_VERSION_STR = "1.1.0"`
- `Tool/HIDTransferTest/README.txt` &rarr; `Tool Version: 1.1.0`

## 2) License Information

- License file: `Tool/HIDTransferTest/LICENSE.md`
- License: **Apache License, Version 2.0** (SPDX: `Apache-2.0`)
- License URL: <https://www.apache.org/licenses/LICENSE-2.0>

## 3) Functional / Technical Scope

Windows Win32 command-line utility that talks to NuMicro USB HID Transfer sample devices via Microsoft's user-mode HID API. Supports two modes:

1. **Interrupt Transfer Mode (default)** &mdash; flash program / verify with configurable `-pagesize`, `-pages`, `-base`, `-verify`.
2. **Control Transfer Mode (`-ctrl`)** &mdash; `HidD_SetOutputReport` / `HidD_GetInputReport` round-trip test.

USB identifiers: VID `0x0416` (fixed), PID configurable via `-pid` (default `0x5020`).

Runtime: Windows 7+; depends on `HID.DLL` / `SETUPAPI.DLL`. Built with Microsoft Visual C++.

## 4) Suggested CycloneDX Field Mapping

- `type`: `application`
- `name`: `HIDTransferTest`
- `version`: `1.1.0`
- `scope`: `optional`
- `purl`: `pkg:generic/hidtransfertest@1.1.0`
- `licenses`: `Apache-2.0`
- `properties` (`bsp:` namespace): file paths, integration, origin, platform, toolchain, USB VID/PID, version evidence, copyright, and SHA-256 for the shipped executable and license file.

## 5) Evidence Files & SHA-256

| File | SHA-256 |
| --- | --- |
| `Tool/HIDTransferTest/Debug/HIDTransferTest.exe` | `26f2fda0e6389e4d2cc2fe98bf989a84f1689f040530fb97af0c293222e99406` |
| `Tool/HIDTransferTest/LICENSE.md` | `0f768967a2d912d44f609abdc0e01bd8d5771174a944da02a29b94d5cfedba77` |

## 6) Notes

- `scope` is `optional` because this is a host-side tool, not part of the target firmware.
- `Debug/HIDTransferTest.exe` is a debug build; regenerate SHA-256 if a release build is shipped instead.
