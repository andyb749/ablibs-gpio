/*
    TM1638
    There are three main types of boards available:
    Name        PCB Label               LEDs                    Pushbuttons
    Model 1     TM1638 LED & Key        8 Red                   8
    Model 2     TM1638 KEYS, QYF        0                       16
    Model 3     TM1638 V1.3 or LKM1638  8 bi-colour red/green   8

    All have 2 x 4 x 7 segment displays

    From the datasheet:
    TM1638 is an IC dedicated to LED drive control and equipped with a keypad scan interface. It integrates
    MCU digital interface, data latch, LED drive and keypad scanning circuit. 
    * 10 segments x 8 bits
    * Keypad scanning (8 x 3)
    * Brightness adjustment circuit (8-level adjustable duty ratio)
    * Built in power on reset
    
    In output mode, SEG1-SEG8 and SEG9/10? are the segment drives for a common cathod LED display, GRID1-GRID8 are used for the
    common.

    In input mode, SEG1-SEG8 (KS1-KS8) are used as keyboard scanning along with K1-K3.

    The TM1638 has three pins for communication: clock, strobe and data.  The protocol is very similar to SPI, however a common
    pin is used for both data input and output.

    The type 1 board I have has 8 x 7 segment LED displays using SEG1-SEG8 for the segments A-G and DP and GR1-GR8 driving the
    common with GR1 on the MSB.

    The 8 individual LEDs are driven from SEG9 and GR1-GR8.

    The 8 pushbuttons are read from K3 and SEG1-SEG8
*/

#include <Arduino.h>
#include <commons.h>

#define TM_DATACMD      0x40
#define TM_WRITEDATA    0x00
#define TM_READDATA     0x02
#define TM_AUTOADDR     0x00
#define TM_FIXADDR      0x04
#define TM_NORMMODE     0x00
#define TM_TESTMODE     0x08

#define TM_DISPCTRL     0x80
#define TM_INTENSITY    0x07
#define TM_DISPLAY_ON   0x08

#define TM_ADDRCMD      0xC0

        static const uint8_t hexLookup [] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71 };

class Tm1638
{
    // fields
    private:
        const uint8_t _strobe;
        const uint8_t _clock;
        const uint8_t _data;
        uint8_t _intensity = 0;

    // methods
    public:
        /// @brief Initialises a new instance of the \a Tm1638 class with the supplied
        /// strobe, clock and data pins.
        /// @param strobe The strobe pin.
        /// @param clock The clock pin.
        /// @param data The data pin.
        Tm1638 (uint8_t strobe, uint8_t clock, uint8_t data) : _strobe(strobe), _clock(clock), _data(data)
        {
            pinMode(_strobe, OUTPUT);
            pinMode(_clock, OUTPUT);
            pinMode(_data, OUTPUT);

            // Setup the device for writing data into display memory in fixed address mode
            command(TM_DATACMD | TM_FIXADDR);
            for (uint8_t u = 0; u < 16; u++)
                writeData(u, 0);

            // Send display control command to set minimum brightness
            command(TM_DISPCTRL | TM_DISPLAY_ON | 0x00);
        }


        /// @brief Writes the supplied command to the device.
        /// @param cmd The command byte.
        void command (uint8_t cmd)
        {
            #ifdef DEBUG
            strprintf(Serial, "CMD %02X\n", cmd);
            #endif
            digitalWrite(_strobe, LOW);
            shiftOut(_data, _clock, LSBFIRST, cmd);
            digitalWrite(_strobe, HIGH);
        }

        /// @brief Write the supplied command and argument to the device
        /// @param grid The grid location offset.
        /// @param val The value.
        void writeData(uint8_t grid, uint8_t val)
        {
            // registers 0, 2, 4, 6, 8, A, C, E are connected to segments 1-8
            // registers 1, 3, 5, 7, 9, B, D, F are connected to segments 9 & 10
            grid &= 0x0f;   // limit to 0-15
            uint8_t cmd = 0xC0 | grid;

            #ifdef DEBUG
            strprintf(Serial, "CMD %02X, %02X\n", cmd, val);
            #endif
            digitalWrite(_strobe, LOW);
            shiftOut(_data, _clock, LSBFIRST, cmd);
            shiftOut(_data, _clock, LSBFIRST, val);
            digitalWrite(_strobe, HIGH);
        }


        /// @brief Enables the display.
        void displayOn()
        {
            _intensity |= TM_DISPLAY_ON;
            command(TM_DISPCTRL | _intensity); 
        }

        /// @brief Disables the display.
        void displayOff()
        {
            _intensity &= ~TM_DISPLAY_ON;
            command(TM_DISPCTRL | _intensity); 
        }

        /// @brief Sets the display intensity.
        /// @param val The intensity (0-7).
        void setIntensity(uint8_t val)
        {
            val &= TM_INTENSITY;
            _intensity &= ~TM_INTENSITY;
            _intensity |= val;
            command(TM_DISPCTRL | _intensity);
        }

        /*
            There are 16 display registers, mapped to the LEDs as follows:
            Reg B7..B0
            0   Grid1 segs1-8
            1   Grid1 segs9-10
            2   Grid2 segs1-8
            3   Grid2 segs9-10
            4   Grid3 segs1-8
            5   Grid3 segs9-10
            6   Grid4 segs1-8
            7   Grid4 segs9-10
            8   Grid5 segs1-8
            9   Grid5 segs9-10
            A   Grid6 segs1-8
            B   Grid6 segs9-10
            C   Grid7 segs1-8
            D   Grid7 segs9-10
            E   Grid8 segs1-8
            F   Grid8 segs9-10

            On my board, segs1-8 are connected to a, b, c, d, e, f, g, DP resepectively and the CC connected to grids1-8 (L-R).
            Seg 1 is bit 0, seg 8 is bit 7
            Seg 9 is connected to the LEDs, grids 1-8, therefore, to switch on write 0x01 to register 1, 3, 5, 7, 9, B, D or F.

            Segments are:
             AAAAA
            F     B
            F     B
            F     B
             GGGGG
            E     C
            E     C
            E     C
             DDDDD DP

            Number  A   B   C   D   E   F   G   DP
            0       1   1   1   1   1   1   0   0
            1       0   1   1   0   0   0   0   0
            2       1   1   0   1   1   0   0   0
            3       1   1   1   0   0   0   1   0
            4       0   1   1   0   0   1   1   0
            5       1   0   1   1   0   1   1   0
            6       1   0   1   1   1   1   1   0
            7       1   1   1   0   0   0   0   0
            8       1   1   1   1   1   1   1   0
            9       1   1   1   0   0   1   1   0
            A       1   1   1   0   1   1   1   0
            B       0   0   1   1   1   1   1   0
            C       1   0   0   1   1   1   0   0
            D       0   1   1   1   1   0   1   0
            E       1   0   0   1   1   1   1   0
            F       1   0   0   0   1   1   1   0

        */


        /// @brief Reads the keypad
        uint8_t readData ()
        {
            // The TM1638 takes each column output low one at a time, reading
            // the row inputs on K1-K3 to determine which keys are on.
            // K1-K3 are on bits b2..b0
            // The resulting value are stored:
            // byte bit key seg
            // 0    7
            // 0    6   K1  KS2
            // 0    5   K2  KS2
            // 0    4   K3  KS2
            // 0    3
            // 0    2   K1  KS1
            // 0    1   K2  KS1
            // 0    0   K3  KS1
            // 1    7
            // 1    6   K1  KS4
            // 1    5   K2  KS4
            // 1    4   K3  KS4
            // 1    3
            // 1    2   K1  KS3
            // 1    1   K2  KS3
            // 1    0   K3  KS3
            // 2    7
            // 2    6   K1  KS6
            // 2    5   K2  KS6
            // 2    4   K3  KS6
            // 2    3
            // 2    2   K1  KS5
            // 2    1   K2  KS5
            // 2    0   K3  KS5
            // 3    7
            // 3    6   K1  KS8
            // 3    5   K2  KS8
            // 3    4   K3  KS8
            // 3    3
            // 3    2   K1  KS7
            // 3    1   K2  KS7
            // 3    0   K3  KS7
            uint8_t cmd = TM_DATACMD | TM_READDATA;
            digitalWrite(_strobe, LOW);
            shiftOut(_data, _clock, LSBFIRST, cmd);
            pinMode(_data, INPUT);
            delayMicroseconds(2);
            uint8_t bytes[4];
            for(uint8_t u = 0; u < 4; u++)
                bytes[u] = shiftIn(_data, _clock, LSBFIRST);
            digitalWrite(_strobe, HIGH);
            pinMode(_data, OUTPUT);

            for (uint8_t u = 0; u < 4; u++)
            {
                uint8_t key = decodeKey(bytes[u]);
                if (key != 0xff)
                {
                    uint8_t code = key + u;
                    #ifdef DEBUG
                    strprintf(Serial, "READ: u=%d key=%d code=%d\n", u, key, code);
                    #endif
                    return code;
                }
            }
            return 0xff;
        }

        /// @brief Encodes the value supplied and decimal point into the value required to display.
        /// @param val The value (0-F).
        /// @param dp True to enable the decimal place.
        /// @return The value to send to the display.
        uint8_t sevenSegEncode(uint8_t val, bool dp = false)
        {
            if (val > 0x0f) val = 0;

            uint8_t ret = hexLookup[val];
            if (dp) ret |= 0x80;
            return ret;
        }

    private:
        uint8_t decodeKey(uint8_t reg)
        {
            reg &= 0x77;

            // First nibble
            if (reg & 0x04) return 0;
            if (reg & 0x02) return 8;
            if (reg & 0x01) return 16;

            // Second nibble
            if (reg & 0x40) return 4;
            if (reg & 0x20) return 12;
            if (reg & 0x10) return 20;
            return 0xff;
        }    
};