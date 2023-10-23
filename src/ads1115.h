//***************************************************************************
//
//  File Name :		ads1115.h
//
//  Project :		Arduino style libraries
//
//  Purpose :		GPIO Libraries: ADS1115
//
// The MIT License (MIT)
//
// Copyright (c) 2019-2023 Andy Burgess
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


#ifndef __ADS1115_H__
#define __ADS1115_H__

/*
 * Objects for interfacing with ADS1115 16 bit analogue inputs with
 * serial interface (I2C).
 */

#include <Arduino.h>
#include <Wire.h>


#define ADS1115_CONVERSION 0    // Conversion register (16 bit) default 0x0000
#define ADS1115_CONFIG 1        // Configuration register (16 bit) default 0x8583
#define ADS1115_LOTHRES 2       // Low threshold register (16 bit) default 0x8000
#define ADS1115_HITHRES 3       // High threshold register (16 bit) default 0x7fff


/// \brief Enumeration of the various input multiplexor values ADS1115 only.
enum Ads1115Chan
{
    AIN0_AIN1 = 0,  // AINp = AIN0, AINn = AIN1 (default)
    AIN0_AIN3 = 1,  // AINp = AIN0, AINn = AIN3
    AIN1_AIN3 = 2,  // AINp = AIN1, AINn = AIN3
    AIN2_AIN3 = 3,  // AINp = AIN2, AINn = AIN3
    AIN0 = 4,       // AINp = AIN0, AINn = 0V
    AIN1 = 5,       // AINp = AIN1, AINn = 0V
    AIN2 = 6,       // AINp = AIN2, AINn = 0V
    AIN3 = 7        // AINp = AIN3, AINn = 0V
};

/// \brief Enumeration of the various programmable gains ADS1115 only.
enum Ads1115Gain
{
    fsr_6144 = 0,       // +/- 6.144V
    fsr_4096 = 1,       // +/- 4.096V
    fsr_2048 = 2,       // +/- 2.048V (default)
    fsr_1024 = 3,       // +/- 1.024V
    fsr_0512 = 4,       // +/- 0.512V
    fsr_0256 = 5,       // +/- 0.256V
    //fsr_0256 = 6,       // +/- 0.256V
    //fsr_0256 = 7        // +/- 0.256V
};

/// @brief Enumeration of the various data sample rates.
enum Ads1115DataRate
{
    sps8 = 0,       // 8 samples per second
    sps16 = 1,      // 16 SPS
    sps32 = 2,      // 32 SPS
    sps64 = 3,      // 64 SPS
    sps128 = 4,     // 128 SPS (default)
    sps250 = 5,     // 250 SPS
    sps475 = 6,     // 475 SPS
    sps860 = 7      // 860 SPS
};

//! \brief Class to encapsulate the functionality of
//! ADS1113, ADS1114 and ADS1115 16 bit analogue input expanders.
//! \details The ADS1113 and 1114 are two channel devices, the 1115 is
//! a four channel device.  The 1114 and 1115 have a programmable gain offering
//! +/-256mv to +/- 6.144V as well as an output comparator.  The 1115
//! also has an input multiplexor that allows two differential or four single
//! ended input measurements.
class Ads1115
{
    // variables
    public:
    protected:
    private:
        TwoWire& _wire;
        uint8_t _addr;
        static const uint8_t _base = 0x48;
        static const uint16_t _default = 0x8583; // Default configuration value
        uint16_t _config = _default;             // Copy of the configuration register
        static const uint16_t _osMask = 1<<15;   // One Shot mask
        static const uint16_t _muxMask = 7<<12;  // Mux mask
        static const uint16_t _pgaMask = 7<<9;   // PGA mask
        static const uint16_t _modeMask = 1<<8;  // Device mode mask
        static const uint16_t _drMask = 7<<5;    // Data Rate mask
        static const uint16_t _compMask = 1<<4;  // Comparator mode mask
        static const uint16_t _polMask = 1<<3;   // Comparator polarity mask
        static const uint16_t _latMask = 1<<2;   // Comparator latch mask
        static const uint16_t _queMask = 3;      // Comparator queue and disable mask

    // functions
    public:
        Ads1115(uint8_t addr = 0, TwoWire &wire = Wire) : _wire(Wire), _addr(addr+_base)
        {
        }

        //! \brief Reset the device to defaults
        //! \details Resets the device to the power on defaults, one shot = true, mux = AIN0_AIN1, 
        //! mode = single shot, data rate = 128SPS, comparator mode = traditional, comparator polarity
        //! = active low, comparator latch = non latching, comparator queue and disable = disabled.
        void reset()
        {
            _config = _default;
            writeReg(ADS1115_CONFIG, _config);
        }


        //! \brief Reads the specified analogue input channel.
        //! \details Reads the analogue input channel from the IC.  The value
        //! value returned is the 16 bit value returned from the IC.
        //! \param chan One of the channel enumeration values.
        int16_t read(Ads1115Chan chan = AIN0_AIN1)
        {
            _config &= ~_muxMask;
            _config |= (chan & 0x7) << 12;
            writeReg(ADS1115_CONFIG, _config);
            return readReg(ADS1115_CONVERSION);
        }


        //! \brief Sets the gain of the PGA.
        //! \details Sets the gain of the device to that specified.
        //! \param gain One of the gain enumeration values.
        void setGain(Ads1115Gain gain = fsr_2048)
        {
            _config &= ~_pgaMask;
            _config |= (gain << 9);
            // we'll write this to the device at the next read
        }

        //! \brief Sets the conversion mode.
        //! \param mode True to enable single shot/power down mode, false for continuous.
        void setMode(bool mode = true)
        {
            _config &= ~_modeMask;
            _config |= mode<< 8;
            writeReg(ADS1115_CONFIG, _config);
        }

        //! \brief Sets the data rate.
        //! \param rate One of the data rate enumeration values.
        void setDataRate(Ads1115DataRate rate)
        {
            _config &= ~_drMask;
            _config |= rate << 5;
            writeReg(ADS1115_CONFIG, _config);
        }

        // TODO:
        // setComparatorMode, setComparatorPolarity, setComparatorLatch, setComparatorQueue

    protected:
    private:
        Ads1115(const Ads1115 & c);
        Ads1115& operator=( const Ads1115 &c);

        void writeReg (uint8_t reg, uint16_t val)
        {
		    _wire.beginTransmission(_addr);
		    _wire.write(reg);
		    _wire.write(val>>8);
            _wire.write(val&0xff);
		    _wire.endTransmission();
        }

        uint16_t readReg(uint8_t reg)
        {
            _wire.beginTransmission(_addr);
            _wire.write(reg);
            _wire.endTransmission(false);
            _wire.requestFrom(_addr, (size_t)2, true);
            uint16_t ret = _wire.read() << 8;
            ret += _wire.read();
            return ret;
        }
};

#endif