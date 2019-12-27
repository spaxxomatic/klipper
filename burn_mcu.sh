avrdude -c linuxspi -p m328p -P /dev/spidev0.0 -U flash:w:./out/klipper.elf.hex
