# DFPlayer-async

An async no_std Rust driver for the DFPlayer Mini MP3 module, designed for embedded systems.

## About

This driver is based on the excellent work from [dfplayer-serial](https://github.com/Laboratorios-Gensokyo/dfplayer-serial) by José Fernando Morales Vargas. Published with the original author's kind permission as the original repository is currently not maintained.

Based on the original driver, this version has been reworked. See tags for changes, in case You are interested.

## Features

- **Async/await API** for embedded systems
- **(Probaly) full command support** for DFPlayer Mini and compatible modules
- **Proper error handling** and timeout management
- **no_std compatible** for use in bare-metal environments
- **Robust initialization sequence** with fallback mechanisms
- Optional **logging via defmt** (enable the "defmt" feature)

## Hardware Compatibility

This driver has been tested with:

- Original DFPlayer mini by DFRobot
- AZDeliver DFPlayer mini clone

Despite using different chips, both modules apparently follow the same protocol and work with this driver. Other compatible modules should work as well, but your mileage may vary.

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
dfplayer-async = "0.2.0"
```

## Examples

The repository contains complete working examples:

- [`examples/src/basic.rs`](examples/src/basic.rs) - Shows how to initialize the driver and control playback, including:
  - Setting up the UART connection
  - Initializing the DFPlayer
  - Setting volume and equalizer
  - Playing tracks and handling the busy pin

- [`examples/src/query.rs`](examples/src/query.rs) - Demonstrates how to query information from the device:
  - Getting the number of tracks on the SD card
  - Reading the current equalizer setting
  - Checking the current volume

The examples assume a Raspberry Pi Pico with specific pin connections:

- UART0 TX/RX on GPIO 17/16
- Busy Pin on GPIO 18

## Hardware Setup

To use the DFPlayer Mini:

1. Connect power (VCC and GND)
2. Connect UART TX/RX lines (9600 baud, 8N1)
3. Optionally connect the busy pin
4. Connect a speaker to the output
5. Insert an SD card with MP3 files

## File Organization

The DFPlayer expects MP3 files to be organized in a specific way. For basic usage:

- Place files in the root directory of the SD card
- Name them with a numerical prefix: `0001-song.mp3`, `0002-song.mp3`, etc.

For more advanced organization, you can create folders and use the `play_from_folder` method.

## License

Licensed under the MIT License - see the LICENSE file for details.
