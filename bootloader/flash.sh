#!/bin/sh
cargo objcopy --release -- -O binary rfc-bootloader.bin
sudo dfu-util -w -a 0 -s 0x08000000:leave -D rfc-bootloader.bin