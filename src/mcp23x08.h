//***************************************************************************
//
//  File Name :		Mcp23x08.h
//
//  Project :		Arduino style libraries
//
//  Purpose :		GPIO Libraries: MCP23x08
//
// The MIT License (MIT)
//
// Copyright (c) 2015-2023 Andy Burgess
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//  Revisions :
//
//      see rcs below
//
//***************************************************************************

#ifndef __MCP23X08_H__
#define __MCP23X08_H__

/*
 * Objects for interfacing with MCP23008 ad MCP23S08 8 bit I/O expanders with
 * serial interface (I2C/SPI).
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Gpio.h"

// registers
#define MCP23008_IODIR 0x00
#define MCP23008_IPOL 0x01
#define MCP23008_GPINTEN 0x02
#define MCP23008_DEFVAL 0x03
#define MCP23008_INTCON 0x04
#define MCP23008_IOCON 0x05
#define MCP23008_GPPU 0x06
#define MCP23008_INTF 0x07
#define MCP23008_INTCAP 0x08
#define MCP23008_GPIO 0x09
#define MCP23008_OLAT 0x0A

#define MCP23008_ADDR 0x40

//static const uint32_t spiClk = 1000000; // 1 MHz
static const uint32_t spiClk = 250000; // 250 kHz

/// @brief Base class for the MCP23008 & MCP23S08 8-bit
/// IO expanders.
class Mcp23x08 : public GpioExpander8_t
{
    // fields
    protected:
        const uint8_t _wrAddr;
        const uint8_t _rdAddr;

    // methods
    public:
        Mcp23x08 (uint8_t addr) : _wrAddr(MCP23008_ADDR + (addr<<1)), _rdAddr(MCP23008_ADDR + (addr<<1) + 1)
        {
        }

        //! \brief Writes the supplied value to the GPIO pins
        //! \details Writes to the GPIO register to set the pins
        //! to the supplied value
        //! \param val The value to write to the output pins
        inline
        void write8(uint8_t val, uint8_t p = 0)
        {
            writeRegister(MCP23008_GPIO, val);
        }

        //! \brief Reads the value of the GPIO pins
        //! \details Reads the GPIO register to get the value on the input pins
        //! \returns The value of the GPIO register
        inline
        uint8_t read8(uint8_t p = 0)
        {
            return readRegister(MCP23008_GPIO);
        }

        //! \brief Sets the direction of each IO pin
        //! \details Writes to the IODIR register to set the
        //! direction of each pin. Set a bit to 1 to enable the pin as
        //! an input
        //! \param dir The bits pattern to set direction
        inline
        void setDirection(uint8_t dir, uint8_t p = 0)
        {
            writeRegister(MCP23008_IODIR, dir);
        }

        //! \brief Sets the polarity of each IO pin
        //! \details Writes to the IPOL register to invert the
        //! polarity of the input when read. Set a bit to 1 to enable the
        //! pin inversion
        //! \param pol The bit pattern to set inversion	
        inline
        void setPolarity(uint8_t pol, uint8_t p = 0)
        {
            writeRegister(MCP23008_IPOL, pol);
        }

        //! \brief Sets the pullups of each IO pin
        //! \details Writes to the GPPU register to enable the weak
        //! pullup of 100K.  Set a bit to 1 to enable the pullup	
        //! \param pullup The bit pattern to set pullups
        inline
        void setPullups (uint8_t pullup,  uint8_t p = 0)
        {
            writeRegister(MCP23008_GPPU, pullup);
        }

private:
    void virtual writeRegister (uint8_t reg, uint8_t val) = 0;
    uint8_t virtual readRegister (uint8_t reg) = 0;
};


/// @brief Class to interface with a MCP23S08, 8-bit
/// SPI IO expander.
class Mcp23S08 : public Mcp23x08
{
    // fields
    public:
    protected:
    private:
        SPIClass& _spi;
        const uint8_t _ss;

    // methods
    public:
        /// @brief Initialises a new instance of the MCP23S08 object with the supplied hardware address.
        /// @param spi A reference to the SPI interface to be used.
	    /// @param addr An address that specifies the state of the two address pins A1..A0.
        /// @param spi The SPI bus to use.
        Mcp23S08(uint8_t ss, uint8_t addr = 0, SPIClass &spi = SPI) : _ss(ss), _spi(spi), Mcp23x08(addr)
        {
            pinMode(_ss, OUTPUT);
            digitalWrite(_ss, HIGH);
        }


    private:
        void writeRegister (uint8_t reg, uint8_t val)
        {
          	_spi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
            digitalWrite(_ss, LOW);
            _spi.transfer(_wrAddr);
            _spi.transfer(reg);
             _spi.transfer(val);
            digitalWrite(_ss, HIGH);
  	        _spi.endTransaction();
        }
        
        uint8_t readRegister (uint8_t reg)
        {
          	_spi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
            digitalWrite(_ss, LOW);
            _spi.transfer(_rdAddr);
            _spi.transfer(reg);
            uint8_t ret = _spi.transfer(0);
            digitalWrite(_ss, HIGH);
  	        _spi.endTransaction();
            return ret;
        }

        Mcp23S08( const Mcp23S08 &c );
        Mcp23S08& operator=( const Mcp23S08 &c );
};


class Mcp23008 : public Mcp23x08
{
    // fields
    private:
        TwoWire& _wire;

    // methods
    public:
        /// @brief Initialises a new instance of the MCP23S08 object with the supplied hardware address.
        /// @param wire A reference to the I2C interface to be used.
	    /// @param addr An address that specifies the state of the two address pins A1..A0.
        Mcp23008(uint8_t addr = 0, TwoWire &wire = Wire) : _wire(wire), Mcp23x08(addr)
        {
        }

    private:
        void writeRegister (uint8_t reg, uint8_t val)
        {
		    _wire.beginTransmission(_wrAddr);
		    _wire.write(reg);
		    _wire.write(val);
		    _wire.endTransmission();
        }
        
        uint8_t readRegister (uint8_t reg)
        {
		    _wire.beginTransmission(_rdAddr);
		    _wire.write(reg);
		    uint8_t ret = _wire.read();
		    _wire.endTransmission();
            return ret;
        }

        Mcp23008( const Mcp23008 &c );
        Mcp23008& operator=( const Mcp23008 &c );
};

#endif  // __MCP23x08_H__