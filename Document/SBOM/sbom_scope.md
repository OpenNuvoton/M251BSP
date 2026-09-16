# Introduction

The MCU BSP SBOM package consists of two CycloneDX SBOM files: a Product SBOM and a Test Sample SBOM. The Product SBOM includes components that are included in, linked with, deployed to, or reasonably expected to be integrated into the final product firmware/software. The Test Sample SBOM includes components used only for samples, demonstrations, validation, or testing and not included in the product runtime unless explicitly integrated by the user.

# Product SBOM

Used for CRA compliance, product vulnerability management, and customer product integration risk assessment.

## Scope
Drivers, middleware, libraries, startup code, boot code, binaries, and source components that are compiled, linked, flashed, deployed, or reasonably expected to be integrated into the product by customers.

## Evidence
.<br>
└── Library

# Test Sample SBOM

Used for transparent disclosure and internal/customer evaluation, but labeled as non-product runtime dependency.

## Scope
sample code、demo project、host-side test tool、sample validation script、test harness、third-party tool for sample code.

## Evidence
.<br>
├── SampleCode<br>
├── ThirdParty<br>
└── Tool
