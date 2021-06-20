#!/bin/bash
#
# Description: Makes a .bin file suitable for flashing over USB via dfu-util

cargo objcopy --release -- -O binary riskey_firmware.bin
