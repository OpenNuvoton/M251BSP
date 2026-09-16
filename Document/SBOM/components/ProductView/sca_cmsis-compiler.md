# CMSIS-Compiler Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Arm CMSIS-Compiler component under `Library/CMSIS/Compiler`.

## 1) Component Identity

- Component name (`name`): `CMSIS-Compiler`
- Component type (`type`): `library`
- Supplier / project: `Arm Limited`
- Version: `2.2.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `Library/CMSIS/Compiler`
- Upstream repository: `https://github.com/ARM-software/CMSIS-Compiler`
- Upstream tag: `v2.2.0`
- Upstream commit: `218fffe5bcd4da3594d411437b1b773f52f6ca36`

## 2) Evidence for Version and License

- The vendored source is identified as upstream CMSIS-Compiler release `v2.2.0` at commit `218fffe5bcd4da3594d411437b1b773f52f6ca36`.
- `Library/CMSIS/Compiler/ARM.CMSIS-Compiler.pdsc` identifies the package as `ARM::CMSIS-Compiler`, references the upstream Git repository, and identifies `Apache-2.0` as its SPDX license.
- `Library/CMSIS/Compiler/LICENSE` contains the Apache License, Version 2.0.
- `Library/CMSIS/Compiler/README.md` states that CMSIS-Compiler is licensed under Apache License 2.0.

## 3) Functional / Technical Scope

CMSIS-Compiler provides portable software components for retargeting standard I/O and file operations, plus RTOS-related thread-safe interfaces, across Arm Compiler, GCC, Clang, and IAR Compiler.

## 4) Suggested CycloneDX Field Mapping

- `bom-ref`: `pkg:github/arm-software/CMSIS-Compiler@2.2.0?source=vendored&path=Library/CMSIS/Compiler`
- `purl`: `pkg:github/arm-software/CMSIS-Compiler@2.2.0`
- `licenses[0].license.id`: `Apache-2.0`
- `properties`: source path, upstream repository, tag, commit, and evidence paths are recorded in `sca_cmsis-compiler.json`.

## 5) Compliance Notes

- Preserve upstream copyright and SPDX notices in the vendored source files.
- Track CMSIS-Compiler independently from CMSIS-Core because it has its own upstream repository and release lifecycle.