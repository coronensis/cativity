#
# CatIVity - CAT protocol (Icom CI-V) based remote VFO tuner
#
# Makefile: MAKE(1) control file to build the software and download
#           it to the microcontroller
#
# Copyright (c) 2026 Helmut Sipos, YO6ASM <yo6asm@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# MCU clock frequency
F_CPU = 8000000

# Flags for C files
CFLAGS += -std=gnu11
CFLAGS += -ffreestanding -Wall -Wextra -Werror -Wundef -Wshadow \
	  -Wunused-parameter -Warray-bounds -Wstrict-prototypes -Wredundant-decls \
	  -Wbad-function-cast -Wunreachable-code
CFLAGS += -gdwarf-2
CFLAGS += -mmcu=atmega328p
CFLAGS += -O2 -mcall-prologues
CFLAGS += -DF_CPU=$(F_CPU)

# Linker flags
LDFLAGS += -Wl,-Map,cativity.map

# Source code files
CSRC = cativity.c

# Default target
all: cativity.hex g90sim

# Compile the application into the image to be used for flashing.
cativity.hex: $(CSRC)
	avr-gcc --version
	avr-gcc $(CFLAGS) $(CSRC) -o cativity.elf
	avr-objcopy -j .text -j .data -j .eeprom -j .fuse -O ihex cativity.elf cativity.hex
	avr-objcopy -j .text -j .data -O binary cativity.elf cativity.bin
	avr-objdump -h -S -C cativity.elf > cativity.lst
	avr-nm -n cativity.elf > cativity.sym
	avr-size -A --mcu=atmega328p cativity.elf

# Compile a simple CI-V protocol simulator used for testing
# It simulates the Xiegu G90 CAT/CI-V behavior
g90sim: g90sim.c
	gcc g90sim.c -o g90sim

# Remove build artefacts
clean:
	rm -f *.o *.a *.elf *.hex *.bin *.lst *.sym trace*.txt g90sim

# NOTE: The following pinctrl stuff only applies when running on a Raspberry Pi

# Download the firmware to the microcontroller
install: cativity.hex
	pinctrl set 21 op
	pinctrl set 21 dl
	avrdude -V -p m328p -P /dev/spidev0.0:/dev/gpiochip0 -c linuxspi -B 10000 -U flash:w:cativity.hex
	pinctrl set 21 dh

# Read the eeprom memory of the microcontroller
rdeeprom:
	pinctrl set 21 op
	pinctrl set 21 dl
	avrdude -V -p m328p -P /dev/spidev0.0:/dev/gpiochip0 -c linuxspi -B 10000 -U eeprom:r:eeprom.bin:r
	pinctrl set 21 dh

# Write the eeprom memory of the microcontroller
wreeprom:
	pinctrl set 21 op
	pinctrl set 21 dl
	avrdude -V -p m328p -P /dev/spidev0.0:/dev/gpiochip0 -c linuxspi -B 10000 -U eeprom:w:eeprom.bin:r
	pinctrl set 21 dh

# Read the fuse bits of the microcontroller
rdfuse:
	pinctrl set 21 op
	pinctrl set 21 dl
	avrdude -V -p m328p -P /dev/spidev0.0:/dev/gpiochip0 -c linuxspi -B 10000 -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h
	pinctrl set 21 dh

# Write the fuse bits of the microcontroller
wrfuse:
	pinctrl set 21 op
	pinctrl set 21 dl
	avrdude -p m328p -P /dev/spidev0.0:/dev/gpiochip0 -c linuxspi -B 10000 -U lfuse:w:0xE2:m -U hfuse:w:0xDD:m -U efuse:w:0xFF:m
	pinctrl set 21 dh

# HW reset the microcontroller
reset:
	pinctrl set 21 op
	pinctrl set 21 dl
	sleep 1
	pinctrl set 21 dh
