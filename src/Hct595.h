//***************************************************************************
//
//  File Name :		Hct595.h
//
//  Project :		Arduino style libraries
//
//  Purpose :		GPIO Libraries: 74HCT595
//
// The MIT License (MIT)
//
// Copyright (c) 2023 Andy Burgess
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

#ifndef __HCT595_H__
#define __HCT595_H__

#include <stdint.h>
#include <SPI.h>
#include <commons.h>
#include "Gpio.h"

#pragma message "HCT595 - software under development"


class Hct595 : public gpioBase_t
{
    // fields
    private:
        static const uint32_t _spiClk = 1000000;
        static const uint8_t _portCnt = 1;
        port8_t _ports[_portCnt];
        const uint8_t _ss;
        SPIClass& _spi;
        const SPISettings _spiSettings;


    // methods
    public:
        Hct595 (uint8_t ss, SPIClass& spi = SPI) : _ss(ss), _spi(spi), _spiSettings(_spiClk, MSBFIRST, SPI_MODE3), _ports{{0,this}}
        {
            pinMode(ss, OUTPUT);
            digitalWrite(ss, HIGH);
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
        }

        inline
        void setPullups8(uint8_t pullup, uint8_t p = 0)
        {
        }

        inline
        uint8_t read8(uint8_t p = 0)
        {
            return 0;
        }

        void write8(uint8_t val, uint8_t = 0)
        {
            SPI.beginTransaction(_spiSettings);
            digitalWrite(_ss, LOW); //pull SS low to prep other end for transfer
            SPI.transfer(val);
            digitalWrite(_ss, HIGH); //pull ss high to signify end of data transfer
            SPI.endTransaction();
        }


    private:
        Hct595( const Hct595 &c );
        Hct595& operator=( const Hct595 &c );
};

#endif  // __HCT595_H__