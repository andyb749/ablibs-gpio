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


#ifndef __PCF8575_H__
#define __PCF8575_H__

#include <Wire.h>
#include <stdint.h>

/// @brief Implmentation of an object to interface with a PCF8574 8 bit digital
/// port expander.  Note: the PCF8574A has a different base address.
class Pcf8574
{
	// variables
	public:
	protected:
	private:
		static const uint8_t _base = 0x20;
		static const uint8_t _altBase = 0x38;	// check...
		uint8_t _addr;
		TwoWire& _wire;

	// functions
	public:
		/// @brief Initialises a new instance of the Pcf8574 object for the specified I2C address and interface.
		/// @param addr The I2C address.
		/// @param wire The I2C interface.
		Pcf8574 (uint8_t addr=0x00, TwoWire& wire = Wire) : _addr(_base+addr), _wire(wire)
		{
		}


		/// @brief Writes the 8 bit value to the port.
		/// @param val The value to write.
		void write(uint8_t val)
		{
		    _wire.beginTransmission(_addr);
		    _wire.write(val);
		    _wire.endTransmission();
		}

		/// @brief Reads the 8 bit value from the port.
		/// @return The value read from the port.
		uint8_t read()
		{
		    _wire.beginTransmission(_addr);
		    _wire.endTransmission(false);
            _wire.requestFrom(_addr, (size_t)1, true);
            return _wire.read();
		}

	protected:
	private:
		Pcf8574( const Pcf8574 &c );
		Pcf8574& operator=( const Pcf8574 &c );
};



/// @brief Definition of an object to represent a PCF8575 16 bit digital
/// IO expander.
class Pcf8575
{
	// variables
	public:
	protected:
	private:
		static const uint8_t _base = 0x20;
		uint8_t _addr;
		TwoWire& _wire;

	// methods
	public:
		/// @brief Initialises a new instance of the Pcf8575 object for the specified I2C address and interface.
		/// @param addr The I2C address.
		/// @param wire The I2C interface.
		Pcf8575 (uint8_t addr=0x00, TwoWire& wire = Wire) : _addr(_base+addr), _wire(wire)
		{
		}

		/// @brief Writes the 16 bit value to the port.
		/// @param val The value to write.
		void write(uint16_t val)
		{
		    _wire.beginTransmission(_addr);
		    _wire.write(val & 0xff);
		    _wire.write(val >> 8);
		    _wire.endTransmission();
		}

		/// @brief Reads the 16 bit value from the port.
		/// @return The value read from the port.
		uint16_t read()
		{
		    _wire.beginTransmission(_addr);
		    _wire.endTransmission(false);
            _wire.requestFrom(_addr, (size_t)2, true);
			uint16_t ret = _wire.read() << 8;
            return ret + _wire.read();
		}

	protected:
	private:
		Pcf8575( const Pcf8575 &c );
		Pcf8575& operator=( const Pcf8575 &c );
};


#endif //__PCF8575_H__
