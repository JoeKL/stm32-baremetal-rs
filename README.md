# stm32-baremetal-rs

Bare-metal Rust compass application for the STM32F3DISCOVERY board. The firmware reads magnetometer data, calculates a heading through a small C math routine exposed via Rust FFI, and displays the current direction on the board LEDs.

This repository is useful as a compact example of embedded Rust, STM32 HAL setup, C/Rust interoperability, and simple runtime checks around an unsafe boundary.

## Features

- `#![no_std]` / `#![no_main]` Rust firmware for a Cortex-M microcontroller
- STM32F3DISCOVERY / STM32F303VCTx target configuration
- I2C magnetometer sampling with the `lsm303agr` driver
- Eight-LED compass display using the STM32F3DISCOVERY LED ring
- Button-triggered heading calibration on PA0
- C heading calculation compiled with Cargo through `build.rs` and the `cc` crate
- Rust wrapper around the raw C FFI call with basic validation and error reporting
- `probe-rs` runner configuration for flashing/running from Cargo

## Hardware target

| Part | Usage |
| --- | --- |
| STM32F3DISCOVERY / STM32F303VCTx | Main target board and MCU |
| LSM303AGR-compatible magnetometer | Magnetic field readings over I2C |
| GPIOE LEDs | Direction indicator, mapped into 8 compass segments |
| PA0 user button | Stores the current heading as the calibration offset |
| ST-LINK/debug probe | Flashing and debugging with `probe-rs` |

## Repository layout

```text
.
├── .cargo/config.toml        # Cargo target, runner, panic/profile settings
├── clib/
│   ├── include/library.h     # C function declaration
│   └── src/library.c         # C heading calculation using atan2f
├── src/
│   ├── bin/bin.rs            # Firmware entry point and compass application
│   ├── ffi/raw.rs            # Unsafe extern C binding
│   ├── ffi/wrapper.rs        # Safe Rust wrapper around the C function
│   └── lib.rs                # no_std library module exports
├── build.rs                  # Compiles the C helper library
├── memory.x                  # STM32F303VCTx flash/RAM layout
└── Cargo.toml                # Rust dependencies and package metadata
```

## How it works

1. The firmware configures the STM32 HAL, clocks, GPIO, EXTI interrupt, and I2C1.
2. The magnetometer is initialized in high-resolution continuous mode.
3. New magnetometer samples are read as raw X/Y values.
4. Rust calls `safe_calc_heading_in_rad`, which wraps the C `calc_heading_in_rad` function.
5. The C function applies hard-iron offsets and calculates the heading using `atan2f`.
6. Rust converts the heading to degrees, applies the user calibration offset, normalizes it to `0..360`, and maps it to one of eight LEDs.
7. Pressing the PA0 user button stores the latest uncalibrated heading as the new north offset.

## Prerequisites

Install the usual embedded Rust tooling:

```sh
rustup target add thumbv7em-none-eabihf
```

Install `probe-rs` tools. The official installer is recommended:

```sh
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
```

Alternatively, install from Cargo:

```sh
cargo install probe-rs-tools --locked
```

Because this project compiles a C helper library, make sure a C compiler for the embedded target is available, for example `arm-none-eabi-gcc` on your `PATH`. If the `cc` crate cannot find it automatically, set the target-specific compiler environment variable before building:

```sh
export CC_thumbv7em_none_eabihf=arm-none-eabi-gcc
```

On Linux, you may also need udev rules or permissions for your ST-LINK/debug probe. See the probe-rs probe setup guide for your platform.

## Build

The default target is configured in `.cargo/config.toml`, so a normal Cargo build should cross-compile for the STM32 target:

```sh
cargo build
```

For an optimized firmware build:

```sh
cargo build --release
```

## Flash and run

Connect the STM32F3DISCOVERY board over USB/ST-LINK, then run:

```sh
cargo run --bin bin
```

The Cargo runner is configured as:

```toml
runner = "probe-rs run --chip STM32F303VCTx"
```

After flashing, rotate the board to change the compass heading. The active LED indicates the closest 45-degree compass segment.

## Calibration

The firmware has two calibration mechanisms:

- Hard-iron offsets are currently defined in `clib/src/library.c`.
- Runtime north calibration is triggered by pressing the PA0 user button. The current uncalibrated heading is stored as the north offset.

For a different board, sensor placement, or magnetic environment, update the C offsets and retest the heading output.

## Useful links

- [Embedded Rust Book](https://doc.rust-lang.org/embedded-book/)
- [probe-rs installation](https://probe.rs/docs/getting-started/installation/)
- [probe-rs probe setup](https://probe.rs/docs/getting-started/probe-setup/)
- [stm32f3xx-hal crate](https://crates.io/crates/stm32f3xx-hal)
