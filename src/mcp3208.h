//***************************************************************************
//
//  File Name :		mcp3208.h
//
//  Project :		Arduino style libraries
//
//  Purpose :		GPIO Libraries: MCP3204/3208
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

#ifndef __MCP3208_H__
#define __MCP3208_H__

/*
 * Objects for interfacing with MCP3204 and MCP3208 12 bit analogue inputs with
 * serial interface (SPI).
 */

#include <Arduino.h>
#include <SPI.h>

// Connections:
// Pin	Func	Connect
// 1	CH0		analog source
// 2	CH1
// -
// 8	CH7
// 9	DGND	0V
// 10	!CS		SS
// 11	DIN		MOSI
// 12	DOUT	MISO
// 13	CLK		SCK
// 14	AGND	0V
// 15	VREF	VCC
// 16	VDD		VCC

//! \brief Class to encapsulate the functionality of
//! MCP3204 and MCP3208 analog input expanders.
class Mcp3208
{
	//variables
	public:
	protected:
	private:
        SPIClass& _spi;
        uint8_t _ss;
        // The datasheet says the clock can be 2MHz at 5V, but only 1MHz at 2.7V
        static const int spiClk = 250000; 
	
	//functions
	public:
        //! \brief Initialises a new instance of the Mcp3208 class.
        //! \param spi The SPI interface to use.
        //! \param ss The chip select pin number.
        Mcp3208(uint8_t ss = SS, SPIClass &spi = SPI) : _spi(spi), _ss(ss)
        {
            pinMode(_ss, OUTPUT);
        }


        //! \brief Reads the specified analogue input channel.
        //! \details Reads the analogue input channel from the IC.  The value returned
        //! is the 12 bit value returned from the IC.
        //! \param chan The channel to read.
        uint16_t read (uint8_t chan)
        {
            // The format for 8 bit transfers is as follows:
            // Word 0: 0000 01SD where S = SINGLE / !DIFFERENTIONAL, D = D2
            // Word 1: DDXX XXXX where DD is D1, D0
            // Word 2: XXXX XXXX where X are don't care
            // The received data is:
            // Word 0: Hi-Z
            // Word 1: XXX0 B11B10B9B8
            // Word 2: B7B6B5B4 B3B2B1B0

            uint8_t txBuf[3];
            uint8_t rxBuf[3];
            txBuf[0] = 0x06 | (chan >> 2);
            txBuf[1] = (chan << 6);
            txBuf[2] = 0;

            // Device needs mode 0 or mode 3
          	_spi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
            digitalWrite(_ss, LOW);
            rxBuf[0] = _spi.transfer(txBuf[0]);
            rxBuf[1] = _spi.transfer(txBuf[1]);
            rxBuf[2] = _spi.transfer(txBuf[2]);
            digitalWrite(_ss, HIGH);
  	        _spi.endTransaction();
            return ((rxBuf[1] << 8) + rxBuf[2]) & 0xfff;
        }
	protected:
	private:
        Mcp3208( const Mcp3208 &c );
        Mcp3208& operator=( const Mcp3208 &c );
}; // Mcp3208

#endif