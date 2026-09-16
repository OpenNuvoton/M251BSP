# FreeRTOS-Kernel Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored FreeRTOS component under `ThirdParty/FreeRTOS`.

## 1) Component Identity

- Component name (`name`): `FreeRTOS-Kernel`
- Component type (`type`): `library`
- Supplier / project: `FreeRTOS` (Amazon)
- Version: `v11.3.0`
- License: `MIT` (SPDX)
- Evidence path: `ThirdParty/FreeRTOS`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/FreeRTOS/Source/manifest.yml`
  - `name : "FreeRTOS-Kernel"`
  - `version: "v11.3.0"`
  - `license: "MIT"`
- `ThirdParty/FreeRTOS/Source/include/FreeRTOS.h`
  - Header shows `FreeRTOS Kernel V11.3.0`
  - Header includes `SPDX-License-Identifier: MIT`
- `ThirdParty/FreeRTOS/Source/tasks.c`
  - Header shows `FreeRTOS Kernel V11.3.0`
  - Header includes `SPDX-License-Identifier: MIT`
- `ThirdParty/FreeRTOS/Source/sbom.spdx`
  - `PackageName: FreeRTOS-Kernel`
  - `PackageVersion: v11.3.0`
  - `PackageLicenseConcluded: MIT`

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = MIT`

Also preserve upstream license headers in vendored source files.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `FreeRTOS-Kernel`
- `version`: `11.3.0`
- `scope`: `required`
- `author`: `Amazon.com, Inc. (FreeRTOS project)`
- `description`: `FreeRTOS Kernel vendored in ThirdParty/FreeRTOS.`
- `licenses[0].license.id`: `MIT`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/FreeRTOS`
  - `integration = vendored_source`
  - `upstream_manifest = ThirdParty/FreeRTOS/Source/manifest.yml`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/freertos-kernel@11.3.0?source=vendored&path=ThirdParty/FreeRTOS`
- `purl`: `pkg:generic/freertos-kernel@11.3.0`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/freertos-kernel@11.3.0?source=vendored&path=ThirdParty/FreeRTOS",
  "name": "FreeRTOS-Kernel",
  "version": "11.3.0",
  "scope": "required",
  "author": "Amazon.com, Inc. (FreeRTOS project)",
  "purl": "pkg:generic/freertos-kernel@11.3.0",
  "cpe": "cpe:2.3:a:amazon:amazon_web_services_freertos:11.3.0:*:*:*:*:*:*:*",
  "description": "FreeRTOS Kernel vendored in ThirdParty/FreeRTOS.",
  "licenses": [
    {
      "license": {
        "id": "MIT"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/FreeRTOS" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "upstream_manifest", "value": "ThirdParty/FreeRTOS/Source/manifest.yml" }
  ]
}
```

## 7) Compliance Notes

- Keep original upstream copyright/license notices.
- If producing a finer-grained SBOM (for example, app + RTOS + board support), represent each major unit as separate components and declare dependencies between them.
