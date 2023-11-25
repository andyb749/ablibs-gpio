# ablibs-gpio

## Introduction

Classes for interfacing to common digital and analogue IO expanders.

The following devices are supported:

* 74HCT595 8-bit parallel/serial shift register.
* PCF8574 8-bit I2C GPIO expander.
* PCF8575 16-bit I2C GPIO expander.
* MCP23S08 (SPI) 8-bit digital expander.
* MCP23017 (I2C) 16-bit digital expander.

## Coming soon

* Support for the PCF8574A 8 bit digital expander.
* Support for MCP23S17 (SPI) 16-bit digital expander.
* Support for HM1638 IO expander - maybe?
* Support for PCA9685 PWM - maybe.
* Support for MCP3204 and MCP3208 4 and 8 channel 12 bit analogue input expanders.
* Support ADS1113, ADS1114, ADS1115 2 and 4 channel 16 bit analogue input expanders.
* Support for AD7780 and AD7795 ADC.
* Support for HX711 ADC - maybe.
* Support for MCP4725 DAC.
* Support for MAX520 & MAX521 DAC.

## PCF8574

This board has a set of three jumpers to set the address from 0-7.  This can be
specified in the constructor

All 8 port pins are available on a Dupont connector along with an interrupt pin
which is activated when any input is changed.

There is no configuration as such, however it is important to set the output register
to the correct state before trying to read the inputs.  

## PCF8575

This expander board has three solder pads that can be linked out to provide a similar
addressing arrangement as the PCF8574 board.

All 16 port pins are available on two Dupont connectors, along with an interrupt which
is activated when any input changes state.

## MCP23008

TODO:

## MCP23S08

TODO:

## MCP23017

This expander board has three DIP switches that can select one of eight addresses that
will be added to the base address similar to the other boards.

All 16 ports are available on two Dupont connectors, along with two pin-change interrupt
pins, one for each port.

A limitation of the I2C version of this device is that bit 7 on each port is not available
as an input.  This limitation is not applicable to the SPI version - MCP23S17.

Currently writes are not working in 16 bit mode.

## SN74HCT595

TODO:

## SN74HCT165