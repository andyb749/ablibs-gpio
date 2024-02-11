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


#ifndef __PCF8574_H__
#define __PCF8574_H__

#include <Wire.h>
#include <stdint.h>
#include <commons.h>
#include "Gpio.h"


/// @brief Implmentation of an object to interface with a PCF8574 8 bit digital
/// port expander.  Note: the PCF8574A has a different base address.
class Pcf8574 : public gpioBase_t
{
	// fields
	private:
		static const uint8_t _base = 0x20;
		static const uint8_t _altBase = 0x38;	// check...
        static const uint8_t _portCnt = 1;
        port8_t _ports[_portCnt];
		const uint8_t _addr;
		uint8_t _pol;
		TwoWire& _wire;

	// methods
	public:
		/// @brief Initialises a new instance of the Pcf8574 object for the specified I2C address and interface.
		/// @param addr The I2C address.
		/// @param wire The I2C interface.
		Pcf8574 (uint8_t addr=0x00, TwoWire& wire = Wire) : _addr(_base+addr), _wire(wire), _ports{{0,this}}
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
			// The PCF8574 does not have a direction reg.
		}

		inline 
		void setPullups8(uint8_t pullup, uint8_t p=0)
		{
			// The PCF8574 does not have a pullup reg...
			// But we can simulate
			write8(pullup, p);
		}

		/// @brief Reads the 8-bit value from the specified port.
		/// @param p The port to read from.
		/// @return The value read from the port.
		uint8_t read8(uint8_t p=0)
		{
            _wire.requestFrom(_addr, 1);
			while (0 == _wire.available());
            return _wire.read() ^ _pol;
		}

		/// @brief Writes the supplied 8-bit value to the specified port.
		/// @param val The value to write.
		/// @param p The port to write to.
		void write8(uint8_t val, uint8_t p=0)
		{
		    _wire.beginTransmission(_addr);
		    _wire.write(val);
		    _wire.endTransmission();
		}


	protected:
	private:
		Pcf8574( const Pcf8574 &c );
		Pcf8574& operator=( const Pcf8574 &c );
};


#endif //__PCF8574_H__
