## Riskeyboard 70 Firmware

This repository contains the super duper alpha code for the Riskeyboard 70: An analog hall effect keyboard that uses 3D printed switches and stabilizers.  It also has the ability to use a MAX7219 display with up to 8 matrices, an infrared receiver (e.g. TSOP382), and relay boards to provide super duper clicky feedback.

https://user-images.githubusercontent.com/66987/122682531-f986f300-d1c7-11eb-8253-ecbc9fb8b75e.mp4

The Riskeyboard 70 also features an analog rotary encoder and is (currently) built using [Keyberon](https://github.com/TeXitoi/keyberon)

## Building the firmware

To build the Riskeyboard 70 firmware you'll need to use the Nightly build of Rust (so we can use `-Z` in `.cargo/config` to cut down on code size): `rustup override set nightly`

### Build it

`cargo build --release`

Alternatively, you can use the `cargo_embed.sh` script to run with an interactive debugging interface (note: Requires an ST-LINK):

`./cargo_embed.sh`

To make a firmware you can flash over USB:

`./make_dfu_bin.sh`

Then you can boot your Riskeyboard 70 (or Black Pill--same chip!) into DFU bootloader mode and flash it:

`./flash_firmware.sh riskey_firmware.bin`
