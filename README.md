CatIVity - CAT protocol (Icom CI-V) based remote VFO tuner

This is a new iteration of CatIVity.
It is WORK IN PROGRESS and not yet fully tested.

You can find the previous version (tested and working) under v1.

## Hardware

## Software

## Development environment

Developed under GNU/Linux - [Debian](https://www.debian.org/) (Desktop) and [Raspbian](https://www.raspberrypi.org/downloads/raspbian/) (Raspberry Pi)
distributions

Toolchain and other tools/utilities used:

Desktop

- avr-gcc

Raspberry Pi

- avr-gcc
- avrdude

The provided Makefile is meant to be used on a Raspberry Pi.
Note that the reset pin of the ATmega328P is connected to pin 21 of the Raspberry
and is pulled low before starting to flash the binary to the microcontroller.

## References

- Icom's CI-V protocol specification [Computer Interface, version V](https://www.icomeurope.com/wp-content/uploads/2020/08/IC-705_ENG_CI-V_1_20200721.pdf)
- Xiegu G90 CAT protocol specification [Xiegu G90, CAT and digital modes](https://radioddity.s3.amazonaws.com/Radioddity%20-%20Xiegu%20G90%2C%20CAT%20and%20Digital%20modes%20V1.0_20210623.pdf)

## Homepage And Source Code Repository

https://github.com/coronensis/cativity

## Contact

Feel free to reach out if you run into problems building this project or want to extend or port it.

Helmut Sipos, YO6ASM <yo6asm@gmail.com>
