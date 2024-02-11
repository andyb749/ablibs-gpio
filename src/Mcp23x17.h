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
// Copyright (c) 2015-2024 Andy Burgess
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

#ifndef __MCP23X17_H__
#define __MCP23X17_H__

#include <SPI.h>
#include <Wire.h>
#include <Commons.h>
#include "Gpio.h"

#pragma message "MCP23x17 - software under development"

// registers
#define MCP23017_16BIT_MODE
#ifndef MCP23017_16BIT_MODE
// registers - 8 bit mode IOCON.BANK = 1
#define MCP23017_IODIRA     0x00
#define MCP23017_IPOLA      0x01
#define MCP23017_GPINTENA   0x02
#define MCP23017_DEFVALA    0x03
#define MCP23017_INTCONA    0x04
#define MCP23017_IOCONA     0x05
#define MCP23017_GPPUA      0x06
#define MCP23017_INTFA      0x07
#define MCP23017_INTCAPA    0x08
#define MCP23017_GPIOA      0x09
#define MCP23017_OLATA      0x0A
#define MCP23017_IODIRB     0x10
#define MCP23017_IPOLB      0x11
#define MCP23017_GPINTENB   0x12
#define MCP23017_DEFVALB    0x13
#define MCP23017_INTCONB    0x14
#define MCP23017_IOCONB     0x15
#define MCP23017_GPPUB      0x16
#define MCP23017_INTFB      0x17
#define MCP23017_INTCAPB    0x18
#define MCP23017_GPIOB      0x19
#define MCP23017_OLATB      0x1A
#else
// registers - 16 bit mode IOCON.BANK = 0
#define MCP23017_IODIRA     0x00
#define MCP23017_IODIRB     0x01
#define MCP23017_IPOLA      0x02
#define MCP23017_IPOLB      0x03
#define MCP23017_GPINTENA   0x04
#define MCP23017_GPINTENB   0x05
#define MCP23017_DEFVALA    0x06
#define MCP23017_DEFVALB    0x07
#define MCP23017_INTCONA    0x08
#define MCP23017_INTCONB    0x09
#define MCP23017_IOCONA     0x0A
#define MCP23017_IOCONB     0x0B
#define MCP23017_GPPUA      0x0C
#define MCP23017_GPPUB      0x0D
#define MCP23017_INTFA      0x0E
#define MCP23017_INTFB      0x0F
#define MCP23017_INTCAPA    0x10
#define MCP23017_INTCAPB    0x11
#define MCP23017_GPIOA      0x12
#define MCP23017_GPIOB      0x13
#define MCP23017_OLATA      0x14
#define MCP23017_OLATB      0x15
#endif
#define MCP23017_IOCONA_16     0x0A
#define MCP23017_IOCONB_16     0x0B

#define MCP23017_ADDR 0x20

//static const uint32_t spiClk = 1000000; // 1 MHz
static const uint32_t spiClk = 250000; // 250 kHz

class Mcp23x17 : public gpioBase_t
{
    // fields
    private:
        static const uint8_t _portCnt = 2;
        port8_t _ports[_portCnt];

    protected:
        const uint8_t _wrAddr;
        const uint8_t _rdAddr;

    // methods
    public:
        Mcp23x17 (uint8_t addr) : _wrAddr(MCP23017_ADDR + (addr<<1)), _rdAddr(MCP23017_ADDR + (addr<<1) + 1), _ports{{0,this},{1,this}}
        {
        }

        /*
         * Implementation of the base class methods
         */

		/// @brief Gets the count of 8-bit ports.
		/// @return The number of ports.
		inline
        uint8_t getPortCount()
		{
			return _portCnt;
		}

        /// @brief Gets the specified 8-bit port.
        /// @param p The port number (0-1).
        /// @return The port.
        //pcf8575Port_t& 
        port8_t& getPort(uint8_t p=0)
		{
			if (p >= _portCnt)
			{
				Serial.print("Invalid port passed: ");
				Serial.println(p);
				p = 0;
			}

			return _ports[p];
		}

        inline
        void setDirection8(uint8_t dir, uint8_t p = 0)
        {
            switch (p)
            {
                case 0:
                    writeRegister(MCP23017_IODIRA, dir);
                    break;
                case 1:
                    writeRegister(MCP23017_IODIRB, dir);
                    break;
            }
        }



        inline
        void setPullups8(uint8_t pullup, uint8_t p = 0)
        {
            switch (p)
            {
                case 0:
                    writeRegister(MCP23017_GPPUA, pullup);
                    break;

                case 1:
                    writeRegister(MCP23017_GPPUB, pullup);
                    break;
            }
        }


        inline
        void write8(uint8_t val, uint8_t p = 0)
        {
            switch (p)
            {
                case 0:
                    writeRegister(MCP23017_GPIOA, val);
                    break;
                case 1:
                    writeRegister(MCP23017_GPIOB, val);
                    break;
            }
        }


        inline
        uint8_t read8(uint8_t p = 0)
        {
            switch (p)
            {
                case 0:
                    return readRegister(MCP23017_GPIOA);
                case 1:
                    return readRegister(MCP23017_GPIOB);
            }
            return 0x00;
        }

#if false
        void write16(uint16_t val)
        {
            write8(val && 0xff, 0);
            write8(val >> 8, 1);
        }

        uint16_t read16()
        {
            uint16_t ret = read8(1) << 8;
            ret += read8(0);
            return ret;
        }
#endif

    private:
        virtual void writeRegister(uint8_t reg, uint8_t val) = 0;
        virtual uint8_t readRegister(uint8_t reg) = 0;
};


class Mcp23S17 : public Mcp23x17
{
    // fields
    private:
        SPIClass& _spi;
        const uint8_t _ss;

    // methods
    public:
        Mcp23S17(uint8_t ss, uint8_t addr = 0, SPIClass &spi = SPI) : _ss(ss), _spi(spi), Mcp23x17(addr)
        {
            pinMode(_ss, OUTPUT);
            digitalWrite(_ss, HIGH);
        }

    private:
        void writeRegister(uint8_t reg, uint8_t val)
        {
            // TODO:
        }

        uint8_t readRegister(uint8_t reg)
        {
            // TODO:
            return 0;
        }

        Mcp23S17( const Mcp23S17 &c );
        Mcp23S17& operator=( const Mcp23S17 &c );
};


class Mcp23017 : public Mcp23x17
{
    // fields
    private:
        TwoWire& _wire;
        const uint8_t _addr;

    // methods
    public:
        Mcp23017(uint8_t addr = 0, TwoWire &wire = Wire) : _wire(wire), _addr(addr + MCP23017_ADDR), Mcp23x17(addr)
        {
            // we control the device in 8 bit mode and want to use the non-incrementing mode
            //writeRegister(MCP23017_IOCONA_16, 0xA0);
        }

    private:
        void writeRegister(uint8_t reg, uint8_t val)
        {
            #ifdef DEBUG_MCP23017
            Serial.print("WR (");
            Serial.print(_addr, HEX);
            Serial.print(") ");
            Serial.print(reg, HEX);
            Serial.print(" = ");
            Serial.println(val, HEX);
            #endif
		    _wire.beginTransmission(_addr);
		    _wire.write(reg);
		    _wire.write(val);
		    _wire.endTransmission();
        }

        uint8_t readRegister(uint8_t reg)
        {
		    _wire.beginTransmission(_addr);
		    _wire.write(reg);
		    uint8_t ret = _wire.read();
		    _wire.endTransmission();
            return ret;
        }

        Mcp23017( const Mcp23017 &c );
        Mcp23017& operator=( const Mcp23017 &c );
};

#endif // __MCP23X17_H__