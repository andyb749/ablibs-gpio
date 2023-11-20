//***************************************************************************
//
//  File Name :		Gpio.h
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

#ifndef __GPIO_H__
#define __GPIO_H__

//#ifdef __AVR__
//#warning "AVR detected"
//#endif

/// @brief A base class for all GPIO pin classes.
/// @tparam pin The Arduino style pin number.
template <uint8_t pin>
class GpioPin
{
    // fields
    private:
        uint8_t _pin;

    // methods
    public:
    protected:
        /// @brief Initialises a new instance of the GpioPin class.
        GpioPin() { }
        ~GpioPin() { }
    private:
};


/// @brief A class to represent an input pin.
/// @tparam pin The Arduino style pin number.
template <uint8_t pin>
class InputPin : GpioPin<pin>
{
    // fields

    // methods
    public:
        /// @brief Initialises a new instance of the InputPin class with an option pullup
        /// enabled.
        /// @param pullup True to enable a pullup resistor; otherwise false.
        InputPin(bool pullup=true)
        {
            if (pullup)
                pinMode(pin, INPUT_PULLUP);
            else
                pinMode(pin, INPUT);
        }

        /// @brief Destructs this instance of the InputPin class.
        ~InputPin() {}

        /// @brief Reads the state of this input pin.
        /// @return True if the pin was high; otherwise false.
        inline bool read()
        {
            return digitalRead(pin);
        }

        /// @brief Reads the state of this input pin.        
        inline operator bool()
        {
            return read();
        }

    protected:
    private:
};


/// @brief A class to represent an output pin.
/// @tparam pin The Arudino style pin number.
template <uint8_t pin>
class OutputPin : GpioPin<pin>
{
    // fields

    // methods
    public:
        /// @brief Initialises a new instance of the OutputPin class with the supplied
        /// initial value.
        /// @param value The initial value: true for a high output, false for a low output.
        OutputPin(bool value = false)
        {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, value);
        }

        /// @brief Destructs this instance of the OutputPin class and makes it an input again.
        ~OutputPin() 
        {
            pinMode(pin, INPUT);
        }

        /// @brief Writes the supplied value to this output pin.
        /// @param value The value to write.
        inline void write(bool value)
        {
            digitalWrite(pin, value);
        }

        /// @brief Sets the output pin to high value.
        inline void set()
        {
            write(true);
        }

        /// @brief Clears the output pin to low value.
        inline void clear()
        {
            write(false);
        }

        inline void toggle()
        {
            digitalWrite(pin, !digitalRead(pin));
        }

    	/// @brief Sets the output pin value.
    	/// @param value The value to set the pin to.
    	/// @return A reference to this output pin.
    	inline OutputPin& operator =(bool value)
	    {
		    write(value);
		    return *this;
	    }

    protected:
    private:
};


#if false
/// @brief Abstract class for an 8 bit port
class Port8_t
{
    public:
        /// @brief Sets the direction of the port.
        /// @param dir A bit mask that specifies each pin's direction.
        /// @remark Set the corresponding bit to 1 for input; 0 for output.
        virtual void setDirection(uint8_t dir) = 0;

        /// @brief Sets any pullups for the port.
        /// @param pullups A bit mask that specifies each pin's direction.
        /// @remark Set the corresponding bit to 1 to enable the pullup.
        virtual void setPullups(uint8_t pullups) = 0;

        /// @brief Write the supplied value to the port.
        /// @param value The value to write to the port.
        virtual void write(uint8_t value) = 0;

        /// @brief Reads the value of the port.
        /// @return The value read from the port.
        virtual uint8_t read() = 0;
};
#endif


/// @brief Abstract class for an IO expander.
class GpioExpander8_t
{
    // methods
    public:
        /// @brief Write the supplied value to the port.
        /// @param val The value to write.
        /// @param p The port to write to.
        virtual void write8(uint8_t val, uint8_t p = 0) = 0;

        /// @brief Reads the value at the port.
        /// @param p The port to read from.
        /// @return The value of the port.
        virtual uint8_t read8(uint8_t p = 0) = 0;

        /// @brief Sets the direction of the port pins.
        /// @param dir A bit mask containing the pin directions.
        /// @param p The port to set.
        /// @remark Set a pin as input by writing a 1; 0 for an output.
        virtual void setDirection(uint8_t dir, uint8_t p = 0) = 0;

        /// @brief Sets the polarity of the port pins.
        /// @param pol A bit mask containing the pin polarity.
        /// @param p The port to set.
        /// @remark Set a pin as inverted by writing a 1; 0 for an output.
        virtual void setPolarity(uint8_t pol, uint8_t p = 0) = 0;

        /// @brief Sets the pullups resistors of the port pins.
        /// @param pullup A bit mask containing the pin pullup.
        /// @param p The port to set.
        /// @remark Set a pin pullup by writing a 1; 0 for no pullup.
        virtual void setPullups(uint8_t pullup, uint8_t p = 0) = 0;
};

#endif