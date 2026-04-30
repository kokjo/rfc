cargo objcopy --release -- -O binary rfc-flight-controller.bin
sudo dfu-util -w -a 0 -s 0x08007000:leave -D rfc-flight-controller.bin