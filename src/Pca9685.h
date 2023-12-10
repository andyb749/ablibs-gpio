//***************************************************************************
//
//  File Name :		Pca9685.h
//
//  Project :		General Purpose IO library for Arduino
//
// The MIT License (MIT)
//
// Copyright (c) 2013-2023 Andy Burgess
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

#ifndef __PCA9685_H__
#define __PCA9685_H__

#include <Wire.h>

#define PCA9685_REG_MODE1         0x00
#define PCA9685_REG_MODE2         0x01
#define PCA9685_REG_SUBADR1       0x02
#define PCA9685_REG_SUBADR2       0x03
#define PCA9685_REG_SUBADR3       0x04
#define PCA9686_REG_ALLCALLADR    0x05
#define PCA9685_REG_BASE_LED_ON_L 0x06
#define PCA9685_REG_BASE_LED_ON_H 0x07
#define PCA9685_REG_BASE_LED_OFF_L  0x08
#define PCA9685_REG_BASE_LED_OFF_H  0x09
#define PCA9685_REG_ALL_LED_ON_L  0xFA
#define PCA9685_REG_ALL_LED_ON_H  0xFB
#define PCA9685_REG_ALL_LED_OFF_L 0xFC
#define PCA9685_REG_ALL_LED_OFF_H 0xFD
#define PCA9685_REG_PRE_SCALE     0xFE
#define PCA9685_REG_TESTMODE      0xFF

// Bit definitions for MODE1 register
#define PCA9685_MODE1_RESTART 0x80
#define PCA9685_MODE1_EXTCLK  0x40
#define PCA9685_MODE1_AI      0x20
#define PCA9685_MODE1_SLEEP   0x10
#define PCA9685_MODE1_SUB1    0x08
#define PCA9685_MODE1_SUB2    0x04
#define PCA9685_MODE1_SUB3    0x02
#define PCA9685_MODE1_ALLCALL 0x01

// Bit definitions for MODE2 register
#define PCA9685_MODE2_INVRT   0x10
#define PCA9685_MODE2_OCH     0x08
#define PCA9685_MODE2_OUTDRV  0x04
#define PCA9685_MODE2_OUTNE1  0x02
#define PCA9685_MODE2_OUTNE0  0x01

/// @brief Class to encapsulate a PCA9685 device.
class Pca9685
{
  // fields
  private:
    static const uint8_t _base = 0x40;
    const uint8_t _addr;
		TwoWire& _wire;

  // methods
  public:
	  /// @brief Initialise a new instance of the \a Pca9685 class with the specified address and interface bus.
	  /// @param addr The address.
	  /// @param wire The I2C interface.
	  Pca9685 (uint8_t addr=0x00, TwoWire& wire = Wire) : _addr(_base+addr), _wire(wire)
		{
		}

    /// @brief Sets the PWM prescaler.
    /// @param value The value (minimum 0x03) of the prescaler.
    /// @remark The internal clock frequency is 25MHz. 0x03 = 1.526 kHz, 0x06 = 1kHz, 0x0C = 500Hz, 0x1E = 200Hz, 0x7A = 0x50Hz, 0xFF = 24Hz.
    /// The default setting is 0x1E results in approx. 200Hz.   Calculate the prescaler value with the formula prescale = (25000000 / (4096 * update_rate)) - 1.
    inline
    void setPrescaler(uint8_t value)
    {
      // FIXME: the prescaler can only be set when the device is in sleep.
      write(PCA9685_REG_PRE_SCALE, value);
    }

    /// @brief Set the device into sleep (low power mode).
    inline
    void sleep()
    {
      write(PCA9685_REG_MODE1, PCA9685_MODE1_SLEEP | PCA9685_MODE1_ALLCALL);
    }

    /// @brief Sets the device into normal operating mode.
    void powerOn()
    {
      if ((read(PCA9685_REG_MODE1) & PCA9685_MODE1_RESTART) == PCA9685_MODE1_RESTART)
      {
        write(PCA9685_REG_MODE1, PCA9685_MODE1_ALLCALL);
        delayMicroseconds(500);
        write(PCA9685_REG_MODE1, PCA9685_MODE1_RESTART | PCA9685_MODE1_ALLCALL);
      }
      else
      {
        write(PCA9685_REG_MODE1, PCA9685_MODE1_ALLCALL);
        delayMicroseconds(500);
      }
    }

    /// @brief Writes the off and on times for the specified channel.
    /// @param chan The channel (0-15).
    /// @param offTime The off time (0-FFF).
    /// @param onTime The on time (0-FFF).
    void writeChannel (uint8_t chan, uint16_t offTime, uint16_t onTime = 0)
    {
      if (chan > 0x0f) return;
      if (offTime > 0xfff) return;
      if (onTime > 0xfff) return;

      // calculate the base of the channel registers
      uint8_t base = PCA9685_REG_BASE_LED_ON_L + chan * 4;

      write(base, onTime & 0xff);
      write(base+1, onTime >> 8);

      write(base+2, offTime & 0xff);
      write(base+3, offTime >> 8);

      #ifdef DEBUG_PCA9685
        strprintf(Serial, "%d %04X %04X\n", chan, readStart(chan), readStop(chan));
      #endif
    }

  /// @brief Sets the output inversion.
  inline
  void invert()
  {
    setBit(PCA9685_REG_MODE2, PCA9685_MODE2_INVRT);
  }

  /// @brief clears any output inversion.
  inline
  void noInvert()
  {
    clearBit(PCA9685_REG_MODE2, PCA9685_MODE2_INVRT);
  }

  /// @brief Sets the outputs to TTL style output (default).
  inline
  void setTotemPole()
  {
    setBit(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
  }

  /// @brief Sets the outputs to open drain output.
  inline
  void setOpenDrain()
  {
    clearBit(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
  }

  /// @brief Sets the outputs as high impedance when !OE inactive.
  inline
  void setHighZOnOE()
  {
    setBit(PCA9685_REG_MODE2, PCA9685_MODE2_OUTNE1);
  }

  /// @brief Sets the outputs logic high when !OE inactive.
  inline
  void setHighOnOE()
  {
    clearBit(PCA9685_REG_MODE2, PCA9685_MODE2_OUTNE1);
    setBit(PCA9685_REG_MODE2, PCA9685_MODE2_OUTNE0);
  }

  /// @brief Sets the output logic low when !OE inactive.
  inline
  void setLowOnOE()
  {
    clearBit(PCA9685_REG_MODE2, PCA9685_MODE2_OUTNE1);
    clearBit(PCA9685_REG_MODE2, PCA9685_MODE2_OUTNE0);
  }

#ifdef DEBUG_PCA9685
    void dumpRegisters(Stream& stream = Serial)
    {
      strprintf(stream, "Reg Value\n");
      strprintf(stream, "=== ==\n");
      for (uint8_t u = 0; u < PCA9685_REG_BASE_LED_ON_L; u++)
        strprintf(stream, "%3d %02X\n", u, read(u));

      strprintf(stream, "Chn Start Stop\n");
      strprintf(stream, "=== ===== ====\n");
      for (uint8_t u = 0; u < 16; u++)
      {
        strprintf(stream, "%3d %04X  %04X\n", u, readStart(u), readStop(u));
      }

      uint16_t start = read(PCA9685_REG_ALL_LED_ON_L) + read(PCA9685_REG_ALL_LED_ON_H) << 8;
      uint16_t stop = read(PCA9685_REG_ALL_LED_OFF_L) + read(PCA9685_REG_ALL_LED_OFF_H) << 8;
      strprintf(stream, "ALL %04X  %04X\n", start, stop);

      strprintf(stream, "Reg Value\n");
      strprintf(stream, "=== ==\n");
      for (uint8_t u = 0; u < 2; u++)
        strprintf(stream, "%3d %02X\n", u + PCA9685_REG_PRE_SCALE, read(PCA9685_REG_PRE_SCALE + u));
    }
#endif

  private:
    void write(uint8_t reg, uint8_t value)
    {
		    _wire.beginTransmission(_addr);
		    _wire.write((uint8_t)reg);
		    _wire.write((uint8_t)value);
		    _wire.endTransmission();
        //strprintf(Serial, "WR: %02X %02X=%02X\n", _addr, reg, value);
		}

    uint8_t read(uint8_t reg)
    {
      _wire.beginTransmission(_addr);
      _wire.write((uint8_t)reg);
      _wire.endTransmission(false);
      _wire.requestFrom(_addr, 1);
      while (0 == _wire.available());
      uint8_t ret = _wire.read();

      //strprintf(Serial, "RD: %02X %02X=%02X\n", _addr, reg, ret);
      return ret;
    }

    uint16_t readStart(uint8_t chan)
    {
      uint8_t base = PCA9685_REG_BASE_LED_ON_L + chan * 4;
      return (read(base+1) << 8) + read(base);
    }

    uint16_t readStop(uint8_t chan)
    {
      // calculate the base of the channel registers
      uint8_t base = PCA9685_REG_BASE_LED_ON_L + chan * 4;
      return (read(base+3) << 8) + read(base+2);
    }

    void setBit (uint8_t reg, uint8_t mask)
    {
      auto val = read(reg);
      val |= mask;
      write(reg, val);
    }

    void clearBit (uint8_t reg, uint8_t mask)
    {
      auto val = read(reg);
      val &= ~mask;
      write(reg, val);
    }
    
		Pca9685( const Pca9685 &c );
		Pca9685& operator=( const Pca9685 &c );
};

#endif  // __PCA9685_H__