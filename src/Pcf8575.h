//***************************************************************************
//
//  File Name :		pcf8575.h
//
//  Project :		Arduino style libraries
//
//  Purpose :		GPIO Libraries: PCF8574 & PCF8575
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


#ifndef __PCF8575_H__
#define __PCF8575_H__

#include <Wire.h>
#include <stdint.h>
#include <commons.h>
#include "Gpio.h"

/// @brief Definition of an object to represent a PCF8575 16 bit digital
/// IO expander.
class Pcf8575 : public gpioBase_t
{
	// fields
	private:
		static const uint8_t _base = 0x20;
        static const uint8_t _portCnt = 2;
        port8_t _ports[_portCnt];
		const uint8_t _addr;
		TwoWire& _wire;
		uint8_t _p0;
		uint8_t _p1;
		uint16_t _dir;

	// methods
	public:
		/// @brief Initialises a new instance of the Pcf8575 object for the specified I2C address and interface.
		/// @param addr The I2C address.
		/// @param wire The I2C interface.
		Pcf8575 (uint8_t addr=0x00, TwoWire& wire = Wire) : _addr(_base+addr), _wire(wire), _ports{{0,this}, {1,this}}
		{
		}

		inline
		void write16(uint16_t val)
		{
			write (val & 0xff, val >> 8);
		}

		uint16_t read16()
		{
            _wire.requestFrom(_addr, (size_t)2);
			uint8_t b0 = _wire.read();
            uint8_t b1 = _wire.read();
			#ifdef DEBUG_PCF8575
				strprintf(Serial, "read16() = %02X%02X\n", b1, b0);
			#endif
			return (b1 << 8) + b0;
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

		void setDirection8(uint8_t dir, uint8_t port=0)
		{
			switch (port)
			{
				case 0:
					_dir &= 0xff00;
					_dir |= dir;
					break;

				case 1:
					_dir &= 0x00ff;
					_dir |= (dir << 8);
					break;
			}
		}

		inline 
		void setPullups8(uint8_t pullup, uint8_t p=0)
		{
		    // The PCF8575 doesn't have a pullup register, but it still requires
			// the port to be set to the high state to be allowed to read.
			write8(pullup, p);
		}

		/// @brief Reads the 8-bit value from the specified port.
		/// @param p The port to read from.
		/// @return The value read from the port.
		uint8_t read8(uint8_t p=0)
		{
			uint16_t read = read16();
			switch (p)
			{
				case 0: return read & 0xff;
				case 1: return read >> 8;
			}
			return 0xff;
		}

		/// @brief Writes the supplied 8-bit value to the specified port.
		/// @param val The value to write.
		/// @param p The port to write to.
		void write8(uint8_t val, uint8_t p=0)
		{
			switch (p)
			{
				case 0:
					write(val, _p1);
					break;
				case 1:
					write(_p0, val);
					break;
			}
		}



	protected:
	private:
		void write(uint8_t p0, uint8_t p1)
		{
			#ifdef DEBUG_PCF8575
				strprintf(Serial, "write(%02X, %02X)\n", p0, p1);
			#endif
			_p0 = p0;
			_p1 = p1;
		    _wire.beginTransmission(_addr);
		    _wire.write(_p0);
		    _wire.write(_p1);
		    _wire.endTransmission();
		}

		Pcf8575( const Pcf8575 &c );
		Pcf8575& operator=( const Pcf8575 &c );
};


#endif //__PCF8575_H__
