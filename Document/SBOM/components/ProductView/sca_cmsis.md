# CMSIS Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Arm CMSIS subset under `Library/CMSIS`.
This CMSIS SBOM component covers only the `Core`, `Driver`, and `RTOS2` directories and `Documentation/index.html`. Other content under `Library/CMSIS` is outside this component's scope or is tracked by separate SBOM components.

## 1) Component Identity

- Component name (`name`): `CMSIS`
- Component type (`type`): `library`
- Supplier / project: `Arm Limited`
- Release version: `6.3.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `Library/CMSIS`
- Upstream repository: `https://github.com/ARM-software/CMSIS_6`
- Upstream tag: `v6.3.0`
- Upstream commit: `45dab712ad84f8cbbf2b7bfc089c19088507df6f`

## 2) Evidence for Version and License

Primary evidence in this repository:


- `Library/CMSIS/Core/Include/cmsis_version.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - `__CM_CMSIS_VERSION_MAIN` is `6U`
  - `__CM_CMSIS_VERSION_SUB` is `2U`
  - `__CA_CMSIS_VERSION_MAIN` is `6U`
  - `__CA_CMSIS_VERSION_SUB` is `2U`
- `Library/CMSIS/RTOS2/Include/cmsis_os2.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - Header shows `Project: CMSIS-RTOS2 API`
  - Header shows `Version 2.3.0`
- `Library/CMSIS/Driver/Include/Driver_Common.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - Header shows common driver definitions API `Version 2.0`

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = Apache-2.0`

Also preserve upstream copyright and license headers in vendored source files.

## 4) Suggested CycloneDX Field Mapping

- `bom-ref`: `pkg:github/ARM-software/CMSIS_6@6.3.0?source=vendored&path=Library/CMSIS`
- `purl`: `pkg:github/ARM-software/CMSIS_6@6.3.0`
- `licenses[0].license.id`: `Apache-2.0`
- `properties`: source path, upstream repository, tag, commit, version-evidence, and license evidence are recorded in `sca_cmsis.json`.

## 5) Compliance Notes

- Keep original upstream copyright/license notices.
- The tracked CMSIS subset contains Core, Driver, RTOS2 API headers, and templates. It excludes upstream CoreValidation and Documentation sources, except for `Documentation/index.html`.
- If producing a finer-grained SBOM, represent major CMSIS subcomponents separately when product policy requires component-level version tracking.
