# TouchView Component Description (for SCA / SBOM)

Component metadata for the **TouchView** Windows host-side touch panel view utility located under `Tool/TK`.

## 1) Component Identity

- Component name (`name`): `TouchView - Tool/TK/TouchView.exe`
- Component type (`type`): `application` (Windows executable)
- Supplier / author: `Nuvoton Technology Corporation`
- Version: `1.0.0.1`
- Copyright: `Copyright (C) 2010-2026 Nuvoton Technology Corp. All rights reserved.`
- Origin: **first-party**
- Integration: **vendored_binary**
- Evidence path: `Tool/TK`

### Key files

- Executable: `Tool/TK/TouchView.exe`
- License: `Tool/TK/LICENSE.md`

### Version evidence

- `Tool/TK/TouchView.exe` PE version resource:
	- File description: `TouchView`
	- File version: `1.0.0.1`
	- Product name: `TouchView`
	- Product version: `1.0.0.1`

## 2) License Information

- License file: `Tool/TK/LICENSE.md`
- License: **Nuvoton Software License Agreement**
- Distribution form: proprietary binary software for use with Nuvoton products, subject to the license terms.

## 3) Functional / Technical Scope

TouchView is a Windows host-side utility for viewing touch panel data. The component is distributed as a prebuilt executable; source code is not included in this BSP path.

## 4) Suggested CycloneDX Field Mapping

- `type`: `application`
- `name`: `TouchView - Tool/TK/TouchView.exe`
- `version`: `1.0.0.1`
- `scope`: `required`
- `purl`: `pkg:generic/touchview@1.0.0.1?file_path=Tool%2FTK%2FTouchView.exe`
- `licenses`: `Nuvoton Software License Agreement`
- `properties` (`bsp:` namespace): file paths, integration, origin, version evidence, copyright, and SHA-256 for the shipped executable and license file.

## 5) Evidence Files & SHA-256

| File | Size (bytes) | SHA-256 |
| --- | ---: | --- |
| `Tool/TK/TouchView.exe` | 3062272 | `2ec72ad2e7ebd1345017df7a405fc36c4a22d114bc709d98ace3924ccc9e506b` |
| `Tool/TK/LICENSE.md` | 6022 | `aa736bbf56948b7e7b4cd667a1d277b828545cde427a27bc772c2598b8547f10` |

## 6) Notes

- The SHA-256 values bind this component record to the reviewed binary and license file.
- Any file change requires the hashes and related SCA/SBOM records to be regenerated and reviewed.

