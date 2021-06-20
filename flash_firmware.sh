#!/bin/bash
#
# Description: Flashes the given ($1) firmware via dfu-util over USB

FIRMWARE="$1"

dfu-util -a0 -s 0x08000000  -D ${FIRMWARE}
[[ "$?" -ne "0" ]] && echo "Did you forget to put it in DFU flash mode (BOOT0 held while resetting)?"
