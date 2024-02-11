//***************************************************************************
//
//  File Name :		Gpio.h
//
//  Project :		General Purpose IO library for Arduino
//
// The MIT License (MIT)
//
// Copyright (c) 2013-2024 Andy Burgess
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

#if false
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
#endif


class port8_t;

/// @brief Base class for GPIO port expanders.
class gpioBase_t
{
    // methods
    public:
        /// @brief Gets the count of the 8-bit ports.
        /// @return The count of ports.
        virtual uint8_t getPortCount() = 0;

        /// @brief Gets a reference to the specified port index.
        /// @param p The port index.
        /// @return A reference to the port object.
        virtual port8_t& getPort(uint8_t p) = 0;

        /// @brief Sets the direction mask for the specified port index.
        /// @param dir The direction mask.
        /// @param p The port index.
        virtual void setDirection8(uint8_t dir, uint8_t p) = 0;

        /// @brief Sets the pullups mask for the specified port index.
        /// @param pull The pullups mask.
        /// @param p The port index.
        virtual void setPullups8(uint8_t pull, uint8_t p) = 0;

        /// @brief Reads the 8-bit port of the specified port index.
        /// @param p The port index.
        /// @return The 8-bit port value.
        virtual uint8_t read8(uint8_t p) = 0;

        /// @brief Writes the 8-bit port of the specified port index.
        /// @param val The value to write.
        /// @param p The port index.
        virtual void write8(uint8_t val, uint8_t p) = 0;
};


/// @brief Base class for an 8-bit port of a GPIO expander.
class port8_t
{
    //fields
    private:
        const uint8_t _idx;
        gpioBase_t* _owner;
        uint8_t _val;

    protected:
        uint8_t _polarity = 0x00;

    // methods
    public:
        /// @brief Initialise a new instance of the port8_t class with the specified
        /// port index and owner.
        /// @param idx The port index.
        /// @param owner The port owner object.
        port8_t(uint8_t idx, gpioBase_t* owner) : _idx(idx), _owner(owner)
        {
        }

        /// @brief Gets the index number of this port.
        /// @return The port index.
        inline uint8_t getPortIndex() { return _idx; }

        /// @brief Sets the direction of each bit of the port.
        /// @param dir A mask where a 1 specifies an input.
        inline
        void setDirection(uint8_t dir)
        {
            _owner->setDirection8(dir, _idx);
        }

        /// @brief Sets the pullups (if any) for each bit of the port.
        /// @param pullups A mask where a 1 specifies that a pullup should be enabled.
        inline
        void setPullups(uint8_t pullups)
        {
            _owner->setPullups8(pullups, _idx);
        }

        /// @brief Sets the polarity of each bit of the port.
        /// @param pol A mask where a 1 specifies that that bit shall be inverted.
        inline
        void setPolarity(uint8_t pol)
        {
            _polarity = pol;
        }

        /// @brief Reads the value from the port.
        /// @return The 8-bit value of the port.
        /// @remark If any polarity bits have been defined, then these bits will be inverted
        /// before returning.
        inline
        uint8_t read()
        {
            return _owner->read8(_idx) ^ _polarity;
        }

        /// @brief Writes the supplied value to the port.
        /// @param val The 8-bit value to write to the port.
        inline
        void write(uint8_t val)
        {
            _val = val;
            _owner->write8(val ^ _polarity, _idx);
        }

        /// @brief Determines if the supplied bit is set.
        /// @param b The bit number.
        /// @return True if set; otherwise false.
        inline
        bool isSet(uint8_t b)
        {
            return (read() & (1<<b)) > 0;
        }

        /// @brief Determines if the supplied bit is clear.
        /// @param b The bit number.
        /// @return True if clear; otherwise false.
        inline
        bool isClear(uint8_t b)
        {
            return !isSet(b);
        }

        /// @brief Sets the specified bit in the port.
        /// @param b The bit number.
        inline void setBit(uint8_t b)
        {
            _val |= (1<<b);
            _owner->write8(_val, _idx);
        }

        /// @brief Clears the specified bit in the port.
        /// @param b The bit number.
        inline void clearBit(uint8_t b)
        {
            _val &= ~(1<<b);
            _owner->write8(_val, _idx);
        }

        /// @brief Writes the supplied value to the specified bit.
        /// @param b The bit number.
        /// @param val The value.
        inline void writeBit(uint8_t b, bool val)
        {
            if (val)
                setBit(b);
            else
                clearBit(b);
        }

        /// @brief Toggles the specified bit in the port.
        /// @param b The bit number.
        inline void toggleBit(uint8_t b)
        {
            writeBit(b, isSet(b));
        }

        /// @brief Operator overload that performs the same as the read method.
        inline operator uint8_t()
        {
            return read();
        }

    	/// @brief Write to the port.
    	/// @param value The value to set the port to.
    	/// @return A reference to this output pin.
    	inline port8_t& operator =(uint8_t value)
	    {
		    write(value);
		    return *this;
	    }

};


// --------------------------------------------------


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

class GpioExpander16_t : public GpioExpander8_t
{
    public:
        virtual void write16(uint16_t val) = 0;
        virtual uint16_t read16() = 0;
};
#endif