//***************************************************************************
//
//  File Name :		Pcf8574.h
//
//  Project :		Arduino style libraries
//
//  Purpose :		GPIO Libraries: PCF8574
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


#ifndef __PCF8574_H__
#define __PCF8574_H__

#include <Wire.h>
#include <stdint.h>
#include "Gpio.h"


#if false
/// @brief A class to represent an 8 bit port.
class Pcf8574Port : public Port8_t
{
	// fields
	private:
		//uint8_t _addr;
		//TwoWire& _wire;
		void (*_wrFunc(uint8_t));
		const uint8_t p;

	public:
		/// @brief Initialises a new instance of the Pcf8574 class.
		/// @param addr The I2C address.
		/// @param wire The I2C bus to use.
		//Pcf8574Port (uint8_t addr, TwoWire& wire) : _addr(addr), _wire(wire)
		//{
		//}
		Pcf8574Port(void (*wrFunc(uint8_t)), uint8_t port) : p(port)
		{
			_wrFunc = wrFunc;
		}



		/// @brief Sets the direction of the port.
		/// @param dir A bit mask of the required direction bits.
        /// @details Set each bit to 1 for an input; 0 for an output.
		//inline void setDirection(uint8_t dir)
		//{
		//	// PCF8574 does not have any direction register
		//}

		/// @brief Sets the pullups for the port.
		/// @param pullups A bit mask of the required pullups.
        /// @details Set each bit to 1 to enable the pullup.  For the
        /// PCF8574 there is no pullup, however by writing to the port
        /// it is possible to simulate the pullup.
		//inline void setPullups(uint8_t pullups)
		//{
        //    write(pullups);
		//}

		/// @brief Writes the supplied value to the port.
		/// @param val The value to write.
		void write(uint8_t val)
		{
			_wr->write8(val, p);
		//    _wire.beginTransmission(_addr);
		//    _wire.write(val);
		//    _wire.endTransmission();
		}

		/// @brief Reads the value of the port.
		/// @return The value of the port.
		//uint8_t read8() 
		//{ 
		//    _wire.beginTransmission(_addr);
		//    _wire.endTransmission();
		//	_wire.requestFrom(_addr, 1);
		//	if (_wire.available() == 1)
		//		return _wire.read();
		//	return 0;
		//}
};
#endif

/// @brief Implmentation of an object to interface with a PCF8574 8 bit digital
/// port expander.  Note: the PCF8574A has a different base address.
class Pcf8574 : public GpioExpander8_t
{
	// fields
	private:
		static const uint8_t _base = 0x20;
		static const uint8_t _altBase = 0x38;	// check...
		const uint8_t _addr;
		uint8_t _pol;
		TwoWire& _wire;

	// methods
	public:
		/// @brief Initialises a new instance of the Pcf8574 object for the specified I2C address and interface.
		/// @param addr The I2C address.
		/// @param wire The I2C interface.
		Pcf8574 (uint8_t addr=0x00, TwoWire& wire = Wire) : _addr(_base+addr), _wire(wire)
		{
		}

		void write8(uint8_t val, uint8_t p = 0)
		{
		    _wire.beginTransmission(_addr);
		    _wire.write(val);
		    _wire.endTransmission();
		}

		uint8_t read8(uint8_t p = 0)
		{
		    _wire.beginTransmission(_addr);
		    _wire.endTransmission(false);
            _wire.requestFrom(_addr, (size_t)1, (uint8_t)true);
            return _wire.read() ^ _pol;
		}

		inline
		void setDirection(uint8_t dir, uint8_t p = 0)
		{
			// The PCF8574 does not have a direction reg.
		}

		inline
		void setPolarity(uint8_t pol, uint8_t p = 0)
		{
			// The PCF85874 does not have a polarity reg.
			_pol = pol;
		}

		inline 
		void setPullups(uint8_t pullup, uint8_t p = 0)
		{
			// The PCF8574 does not have a pullup reg...
			// But we can simulate
			write8(pullup);
		}
	protected:
	private:
		Pcf8574( const Pcf8574 &c );
		Pcf8574& operator=( const Pcf8574 &c );
};


#endif //__PCF8574_H__
