// OLED SSD1306 using the I2C interface
// Written by Larry Bank (bitbank@pobox.com)
// Project started 1/15/2017
//
// The I2C writes (through a file handle) can be single or multiple bytes.
// The write mode stays in effect throughout each call to write()
// To write commands to the OLED controller, start a byte sequence with 0x00,
// to write data, start a byte sequence with 0x40,
// The OLED controller is set to "page mode". This divides the display
// into 8 128x8 "pages" or strips. Each data write advances the output
// automatically to the next address. The bytes are arranged such that the LSB
// is the topmost pixel and the MSB is the bottom.
// The font data comes from another source and must be rotated 90 degrees
// (at init time) to match the orientation of the bits on the display memory.
// A copy of the display memory is maintained by this code so that single pixel
// writes can occur without having to read from the display controller.

#include "oled.h"

#include <memory.h>

#include <esp_log.h>
#include <string>
#include <sstream>
#include <iomanip>

#include <esp_debug_helpers.h>

// when WTF is true, the 128x64 oled screen has I2C address of 0xbc instead of 0x7a or 0x78;
#define OLED_128x64_WTF false

extern unsigned char ucSmallFont[];

namespace exw
{
    OLED::OLED(const OLEDConfig& _config, uint32_t _frequency)
        : m_Config(_config)
    {
        m_Device_config.scl_speed_hz = _frequency;
        m_Device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        m_Tag = "Oled";

        switch (_config.type)
        {
            case OLEDType::OLED_128x32:
            {
                m_Tag = "Oled_128x32";
                m_Device_config.device_address = 0x3c;
                break;
            }
            case OLEDType::OLED_128x64:
            {
                m_Tag = "Oled_128x64";
                #if OLED_128x64_WTF
                m_Device_config.device_address = 0xbc;
                m_Device_config.dev_addr_length = I2C_ADDR_BIT_LEN_10;
                #else
                if (_config.alternativeaddr)
                    m_Device_config.device_address = 0x7a;
                else
                    m_Device_config.device_address = 0x78;
                #endif

                break;
            }
            default:
                ESP_LOGW(m_Tag, "Unsupport oled type: %d", (uint8_t)_config.type);
                return;
        }
    }

    bool OLED::init()
    {
        // according to https://cdn-shop.adafruit.com/datasheets/UG-2864HSWEG01.pdf, chapter 4.4, page 15;

        // reference: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf;
        // https://github.com/bitbank2/Multi_OLED/blob/master/src/Multi_OLED.cpp

        const uint8_t* initBuffer = nullptr;
        uint8_t bufferLength = 0U;

        switch (m_Config.type)
        {
            case OLEDType::OLED_128x32:
            {
                constexpr uint8_t oled32_initbuf[] = { 0x00,
                    0xae,       // Set Display Off
                    0xd5, 0x80, // Set Osc Freq and Clock Div Ratio, (0x80 = 0b 1000 0000, Freq = 0b1000 (means RESET), Clock Div Ratio = 0b0000 (means 1))
                    0xa8, 0x1f, // Set MUX Ratio to 0x1f (MUX 32)
                    0xd3, 0x00, // Set Display Offset (offs set to 0)
                    0x40,       // Set Display Start Line to 0 (0b01 000000)
                    0xa1,       // Set Segment Re-map: map column 127 to SEG0 (0b1010000 0)
                    0x8d, 0x14, // Set Charge Pump to Enable Charge Pump during display on (0b00010 1 00)
                    0xaf,       // Set Display On (to enable Charge Pump)
                    0xc8,       // Set COM scan direction to Scan from COM[N-1] to COM0
                    0xda, 0x02, // Set COM pin HW config to [Disable COM L/R Remap][Sequential COM pin config] (0b00 00 0010)
                    0x81, 0x7f, // Set Contrast Control to RESET (0x7f)
                    0xd9, 0xf1, // Set Pre-charge Period to [15 CLK in Phase 2][1 CLK in Phase 1] (0b 1111 0001)
                    0xdb, 0x40, // Set Vcomh Deselect Level to Unspecified value (0b0 100 0000)
                    0xa4,       // Set Entire Display OFF
                    0xa6,       // Set NORMAL display (means that bitwise 0 = pixel off, 1 = pixel on)
                };
                initBuffer = oled32_initbuf;
                bufferLength = sizeof(oled32_initbuf);
                break;
            }

            case OLEDType::OLED_128x64:
            {
                constexpr uint8_t oled64_initbuf[] = { 0x00,
                    0xae,       // Set Display Off
                    0xa8, 0x3f, // Set MUX Ratio to 0x3f (0b00111111, RESET, or MUX 64)
                    0xd3, 0x00, // Set Display Offset (offs set to 0)
                    0x40,       // Set Display Start Line to 0 (0b01 000000)
                    0xa1,       // Set Segment Re-map: map column 127 to SEG0 (0b1010000 0)
                    0xc8,       // Set COM scan direction to Scan from COM[N-1] to COM0
                    0xda, 0x12, // Set COM pin HW config to [Disable COM L/R Remap][Alternative COM pin config] (0b00 01 0010)
                    0x81, 0xff, // Set Contrast Control to 256 (0xff)
                    0xa4,       // Set Entire Display OFF
                    0xa6,       // Set NORMAL display (means that bitwise 0 = pixel off, 1 = pixel on)
                    0xd5, 0x80, // Set Osc Freq and Clock Div Ratio, (0x80 = 0b 1000 0000, Freq = 0b1000 (means RESET), Clock Div Ratio = 0b0000 (means 1))
                    0x8d, 0x14, // Set Charge Pump to Enable Charge Pump during display on (0b00010 1 00)
                    0xaf,       // Set Display On (to enable Charge Pump)
                    0x20, 0x02, // Set Memory Addressing Mode to Page Addressing Mode (0b000000 10)
                    //0xd9, 0xf1, // Set Pre-charge Period to [15 CLK in Phase 2][1 CLK in Phase 1] (0b 1111 0001)
                    //0xdb, 0x40, // Set Vcomh Deselect Level to Unspecified value (0b0 100 0000)
                };
                initBuffer = oled64_initbuf;
                bufferLength = sizeof(oled64_initbuf);
                break;
            }

            default:
                ESP_LOGW(m_Tag, "Unsupport oled type: %d", (uint8_t)m_Config.type);
                return false;
        }

        register_me();
        write(initBuffer, bufferLength);
        m_Initialized = true;

        if (m_Config.invertcolor)
        {
            write_command(0xa7); // invert color
        }

        if (m_Config.rotate180)
        {
            write_command(0xa0); // map column 0 to SEG0
            write_command(0xc0); // change scan mode to Scan COM0 To COM[N-1]
        }

        return true;
    }

    void OLED::set_cursor_pos(int _x, int _y)
    {
        if (!is_initialized())
            return;

        m_Screen_offset = (_y * 128) + _x;

        #if 0
        if (m_Oled_type == OLEDType::OLED_64x32) // visible display starts at column 32, row 4
        {
            _x += 32; // display is centered in VRAM, so this is always true
            if (!m_Is_rotated_180) // non-flipped display starts from line 4
                _y += 4;
        }
        else if (m_Oled_type == OLEDType::OLED_132x64) // SH1106 has 128 pixels centered in 132
        {
            _x += 2;
        }
        #endif

        write_command(0xb0 | _y);                   // go to page Y
        write_command(0x00 | (_x & 0x0f));          // lower col addr
        write_command(0x10 | ((_x >> 4) & 0x0f));   // upper col addr
    }

    void OLED::write_data_block(uint8_t* _data, uint16_t _length)
    {
        if (!is_initialized())
            return;

        uint8_t buffer[129] = { 0x40, 0x00 };
        memcpy(&buffer[1], _data, _length);

        write(buffer, _length + 1);

        // keep a copy in local buffer;
        memcpy(&m_Frame_buffer[m_Screen_offset], _data, _length);
        //m_Screen_offset += _length;
    }

    bool OLED::draw_pixel(int _x, int _y, uint8_t _color)
    {
        if (!is_initialized())
            return false;

        int pos = ((_y >> 3) * 128) + _x;
        if (pos < 0 || pos > 1023) // off the screen
            return false;

        uint8_t pixelOld = m_Frame_buffer[pos];

        uint8_t pixel = pixelOld;
        pixel &= ~(0x01 << (_y & 7));
        if (_color != 0)
        {
            pixel |= (0x01 << (_y << 7));
        }

        if (pixel != pixelOld)
        {
            // pixel changed
            set_cursor_pos(_x, _y >> 3);
            write_data_block(&pixel, 1);
        }

        return true;
    }

    bool OLED::draw_string(int _x, int _y, const std::string& _string)
    {
        if (!is_initialized())
            return false;

        int charCount = _string.length();

        set_cursor_pos(_x * 6, _y);

        if (charCount + _x > 21)
            charCount = 21 - _x;
        if (charCount < 0)
            return false;

        for (int i = 0; i < charCount; ++i)
        {
            uint8_t* str = &ucSmallFont[(unsigned char)_string[i] * 6];
            write_data_block(str, 6);
        }

        return true;
    }

    bool OLED::clear(uint8_t _color)
    {
        if (!is_initialized())
            return false;

        int rows = 8;
        int cols = 8;
        switch (m_Config.type)
        {
            case OLEDType::OLED_128x32:
                rows = 4;
                cols = 8;
                break;
            case OLEDType::OLED_64x32:
                rows = 4;
                cols = 4;
                break;
            default:
                break;
        }

        uint8_t buffer[128] = { _color };
        //memset(buffer, _color, 128);
        for (int y = 0; y < rows; ++y)
        {
            set_cursor_pos(0, y);
            write_data_block(buffer, cols * 16);
        }

        return true;
    }

    bool OLED::clear_line(uint8_t _line, uint8_t _color)
    {
        if (!is_initialized())
            return false;

        if (_line > 4)
            return -1;

        uint8_t buffer[128] = { _color };
        //memset(buffer, _color, 128);
        set_cursor_pos(0, _line);
        write_data_block(buffer, 128);

        return true;
    }


}



// below are legacy code that I keep as reference while porting
// I don't 




#if 0


// OLED SSD1306 using the I2C interface
// Written by Larry Bank (bitbank@pobox.com)
// Project started 1/15/2017
//
// The I2C writes (through a file handle) can be single or multiple bytes.
// The write mode stays in effect throughout each call to write()
// To write commands to the OLED controller, start a byte sequence with 0x00,
// to write data, start a byte sequence with 0x40,
// The OLED controller is set to "page mode". This divides the display
// into 8 128x8 "pages" or strips. Each data write advances the output
// automatically to the next address. The bytes are arranged such that the LSB
// is the topmost pixel and the MSB is the bottom.
// The font data comes from another source and must be rotated 90 degrees
// (at init time) to match the orientation of the bits on the display memory.
// A copy of the display memory is maintained by this code so that single pixel
// writes can occur without having to read from the display controller.

#include "i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "oled.h"

extern unsigned char ucSmallFont[];
static int iScreenOffset;            // current write offset of screen data
static unsigned char ucScreen[1024]; // local copy of the image buffer
static int oled_type, oled_flip;

static void write_command(unsigned char);
static esp_err_t write(uint8_t*, uint8_t);

static bool oled_active;

// Initialialize the OLED Screen
bool OLED_init(void)
{
    uint8_t oled32_initbuf[] = { 0x00,
                                0xae, // cmd: display off
                                0xd5, // cmd: set display clock
                                0x80,
                                0xa8, // cmd: set multiplex ratio
                                0x1f, // HEIGHT - 1 -> 31
                                0xd3, // cmd: set display offset
                                0x00,
                                0x40, // cmd: Set Display Start Line
                                0x8d,
                                0x14, // cmd: Set Higher Column Start Address for Page Addressing Mode
                                0xa1,
                                0xc8, // cmd: Set COM Output Scan Direction C0/C8
                                0xda, // cmd: Set COM Pins Hardware Configuration
                                0x02, //
                                0x81, // cmd: Set Contrast control
                                0x7f,
                                0xd9, // cmd: Set Pre-Charge Period
                                0xf1,
                                0xdb, // comd: Vcom regulator output
                                0x40,
                                0xa4,  // cmd: display on ram contents
                                0xa6,  // cmd: set normal
                                0xaf }; // cmd: display on
    uint8_t uc[4];

    uint8_t bFlip = nvs_config_get_u16(NVS_CONFIG_FLIP_SCREEN, 1);
    uint8_t bInvert = nvs_config_get_u16(NVS_CONFIG_INVERT_SCREEN, 0);
    oled_active = false;

    // //enable the module
    // GPIO_write(Board_OLED_DISP_ENABLE, 0);
    // DELAY_MS(50);
    // GPIO_write(Board_OLED_DISP_ENABLE, 1);
    // DELAY_MS(50);

    oled_type = OLED_128x32;
    oled_flip = bFlip;

    // /* Call driver init functions */
    // I2C_init();

    // /* Create I2C for usage */
    // I2C_Params_init(&oled_i2cParams);
    // oled_i2cParams.bitRate = I2C_100kHz;
    // oled_i2c = I2C_open(Board_I2C_SSD1306, &oled_i2cParams);

    // if (oled_i2c == NULL) {
    //     return false;
    // }
    oled_active = true;

    write(oled32_initbuf, sizeof(oled32_initbuf));

    if (bInvert)
    {
        uc[0] = 0;    // command
        uc[1] = 0xa7; // invert command
        write(uc, 2);
    }

    if (bFlip)
    {   // rotate display 180
        uc[0] = 0; // command
        uc[1] = 0xa0;
        write(uc, 2);
        uc[1] = 0xc0;
        write(uc, 2);
    }
    return true;
}

// Sends a command to turn off the OLED display
// Closes the I2C file handle
void OLED_shutdown()
{

    write_command(0xaE); // turn off OLED
    // I2C_close(oled_i2c);
    // GPIO_write(Board_OLED_DISP_ENABLE, 0); //turn off power
    oled_active = false;
}

// Send a single byte command to the OLED controller
static void write_command(uint8_t c)
{
    uint8_t buf[2];

    buf[0] = 0x00; // command introducer
    buf[1] = c;
    write(buf, 2);
}

static void oledWriteCommand2(uint8_t c, uint8_t d)
{
    uint8_t buf[3];

    buf[0] = 0x00;
    buf[1] = c;
    buf[2] = d;
    write(buf, 3);
}

bool OLED_setContrast(uint8_t ucContrast)
{

    oledWriteCommand2(0x81, ucContrast);
    return true;
}

// Send commands to position the "cursor" to the given
// row and column
static void oledSetPosition(int x, int y)
{
    iScreenOffset = (y * 128) + x;
    if (oled_type == OLED_64x32) // visible display starts at column 32, row 4
    {
        x += 32;            // display is centered in VRAM, so this is always true
        if (oled_flip == 0) // non-flipped display starts from line 4
            y += 4;
    }
    else if (oled_type == OLED_132x64) // SH1106 has 128 pixels centered in 132
    {
        x += 2;
    }

    write_command(0xb0 | y);                // go to page Y
    write_command(0x00 | (x & 0xf));        // // lower col addr
    write_command(0x10 | ((x >> 4) & 0xf)); // upper col addr
}

// Write a block of pixel data to the OLED
// Length can be anything from 1 to 1024 (whole display)
static void oledWriteDataBlock(uint8_t* ucBuf, int iLen)
{
    uint8_t ucTemp[129];

    ucTemp[0] = 0x40; // data command
    memcpy(&ucTemp[1], ucBuf, iLen);
    write(ucTemp, iLen + 1);
    // Keep a copy in local buffer
    memcpy(&ucScreen[iScreenOffset], ucBuf, iLen);
    iScreenOffset += iLen;
}

// Set (or clear) an individual pixel
// The local copy of the frame buffer is used to avoid
// reading data from the display controller
int OLED_setPixel(int x, int y, uint8_t ucColor)
{
    int i;
    uint8_t uc, ucOld;

    // if (oled_i2c == NULL)
    //     return -1;

    i = ((y >> 3) * 128) + x;
    if (i < 0 || i > 1023) // off the screen
        return -1;
    uc = ucOld = ucScreen[i];
    uc &= ~(0x1 << (y & 7));
    if (ucColor)
    {
        uc |= (0x1 << (y & 7));
    }
    if (uc != ucOld) // pixel changed
    {
        oledSetPosition(x, y >> 3);
        oledWriteDataBlock(&uc, 1);
    }
    return 0;
}

//
// Draw a string of small (8x8), large (16x24), or very small (6x8)  characters
// At the given col+row
// The X position is in character widths (8 or 16)
// The Y position is in memory pages (8 lines each)
//
int OLED_writeString(int x, int y, char* szMsg)
{
    int i, iLen;
    uint8_t* s;

    // if (oled_i2c == NULL) return -1; // not initialized

    iLen = strlen(szMsg);

    oledSetPosition(x * 6, y);
    if (iLen + x > 21)
        iLen = 21 - x;
    if (iLen < 0)
        return -1;
    for (i = 0; i < iLen; i++)
    {
        s = &ucSmallFont[(unsigned char)szMsg[i] * 6];
        oledWriteDataBlock(s, 6);
    }

    return 0;
}

// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
int OLED_fill(uint8_t ucData)
{
    int y;
    uint8_t temp[128];
    int iLines, iCols;

    // if (oled_i2c == NULL) return -1; // not initialized

    iLines = (oled_type == OLED_128x32 || oled_type == OLED_64x32) ? 4 : 8;
    iCols = (oled_type == OLED_64x32) ? 4 : 8;

    memset(temp, ucData, 128);
    for (y = 0; y < iLines; y++)
    {
        oledSetPosition(0, y);                // set to (0,Y)
        oledWriteDataBlock(temp, iCols * 16); // fill with data byte
    }                                         // for y
    return 0;
}

int OLED_clearLine(uint8_t line)
{
    uint8_t temp[128];

    // if (oled_i2c == NULL) return -1; // not initialized
    if (line > 4)
        return -1; // line number too big

    memset(temp, 0, 128);
    oledSetPosition(0, line);      // set to (0,Y)
    oledWriteDataBlock(temp, 128); // fill with data byte

    return 0;
}

bool OLED_status(void)
{
    return oled_active;
}

/**
 * @brief Write a byte to a I2C register
 */
static esp_err_t write(uint8_t* data, uint8_t len)
{
    int ret;

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, 0x3C, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}
#endif









#if 0


#include "i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "oled.h"

extern unsigned char ucSmallFont[];
static int iScreenOffset;            // current write offset of screen data
static unsigned char ucScreen[1024]; // local copy of the image buffer
static int oled_type, oled_flip;

static void write_command(unsigned char);
static esp_err_t write(uint8_t*, uint8_t);

static bool oled_active;

// Initialialize the OLED Screen
bool OLED_init(void)
{
    uint8_t oled32_initbuf[] = { 0x00,
                                0xae, // cmd: display off
                                0xd5, // cmd: set display clock
                                0x80,
                                0xa8, // cmd: set multiplex ratio
                                0x1f, // HEIGHT - 1 -> 31
                                0xd3, // cmd: set display offset
                                0x00,
                                0x40, // cmd: Set Display Start Line
                                0x8d,
                                0x14, // cmd: Set Higher Column Start Address for Page Addressing Mode
                                0xa1,
                                0xc8, // cmd: Set COM Output Scan Direction C0/C8
                                0xda, // cmd: Set COM Pins Hardware Configuration
                                0x02, //
                                0x81, // cmd: Set Contrast control
                                0x7f,
                                0xd9, // cmd: Set Pre-Charge Period
                                0xf1,
                                0xdb, // comd: Vcom regulator output
                                0x40,
                                0xa4,  // cmd: display on ram contents
                                0xa6,  // cmd: set normal
                                0xaf }; // cmd: display on
    uint8_t uc[4];

    uint8_t bFlip = nvs_config_get_u16(NVS_CONFIG_FLIP_SCREEN, 1);
    uint8_t bInvert = nvs_config_get_u16(NVS_CONFIG_INVERT_SCREEN, 0);
    oled_active = false;

    // //enable the module
    // GPIO_write(Board_OLED_DISP_ENABLE, 0);
    // DELAY_MS(50);
    // GPIO_write(Board_OLED_DISP_ENABLE, 1);
    // DELAY_MS(50);

    oled_type = OLED_128x32;
    oled_flip = bFlip;

    // /* Call driver init functions */
    // I2C_init();

    // /* Create I2C for usage */
    // I2C_Params_init(&oled_i2cParams);
    // oled_i2cParams.bitRate = I2C_100kHz;
    // oled_i2c = I2C_open(Board_I2C_SSD1306, &oled_i2cParams);

    // if (oled_i2c == NULL) {
    //     return false;
    // }
    oled_active = true;

    write(oled32_initbuf, sizeof(oled32_initbuf));

    if (bInvert)
    {
        uc[0] = 0;    // command
        uc[1] = 0xa7; // invert command
        write(uc, 2);
    }

    if (bFlip)
    {   // rotate display 180
        uc[0] = 0; // command
        uc[1] = 0xa0;
        write(uc, 2);
        uc[1] = 0xc0;
        write(uc, 2);
    }
    return true;
}

// Sends a command to turn off the OLED display
// Closes the I2C file handle
void OLED_shutdown()
{

    write_command(0xaE); // turn off OLED
    // I2C_close(oled_i2c);
    // GPIO_write(Board_OLED_DISP_ENABLE, 0); //turn off power
    oled_active = false;
}

// Send a single byte command to the OLED controller
static void write_command(uint8_t c)
{
    uint8_t buf[2];

    buf[0] = 0x00; // command introducer
    buf[1] = c;
    write(buf, 2);
}

static void oledWriteCommand2(uint8_t c, uint8_t d)
{
    uint8_t buf[3];

    buf[0] = 0x00;
    buf[1] = c;
    buf[2] = d;
    write(buf, 3);
}

bool OLED_setContrast(uint8_t ucContrast)
{

    oledWriteCommand2(0x81, ucContrast);
    return true;
}

// Send commands to position the "cursor" to the given
// row and column
static void oledSetPosition(int x, int y)
{
    iScreenOffset = (y * 128) + x;
    if (oled_type == OLED_64x32) // visible display starts at column 32, row 4
    {
        x += 32;            // display is centered in VRAM, so this is always true
        if (oled_flip == 0) // non-flipped display starts from line 4
            y += 4;
    }
    else if (oled_type == OLED_132x64) // SH1106 has 128 pixels centered in 132
    {
        x += 2;
    }

    write_command(0xb0 | y);                // go to page Y
    write_command(0x00 | (x & 0xf));        // // lower col addr
    write_command(0x10 | ((x >> 4) & 0xf)); // upper col addr
}

// Write a block of pixel data to the OLED
// Length can be anything from 1 to 1024 (whole display)
static void oledWriteDataBlock(uint8_t* ucBuf, int iLen)
{
    uint8_t ucTemp[129];

    ucTemp[0] = 0x40; // data command
    memcpy(&ucTemp[1], ucBuf, iLen);
    write(ucTemp, iLen + 1);
    // Keep a copy in local buffer
    memcpy(&ucScreen[iScreenOffset], ucBuf, iLen);
    iScreenOffset += iLen;
}

// Set (or clear) an individual pixel
// The local copy of the frame buffer is used to avoid
// reading data from the display controller
int OLED_setPixel(int x, int y, uint8_t ucColor)
{
    int i;
    uint8_t uc, ucOld;

    // if (oled_i2c == NULL)
    //     return -1;

    i = ((y >> 3) * 128) + x;
    if (i < 0 || i > 1023) // off the screen
        return -1;
    uc = ucOld = ucScreen[i];
    uc &= ~(0x1 << (y & 7));
    if (ucColor)
    {
        uc |= (0x1 << (y & 7));
    }
    if (uc != ucOld) // pixel changed
    {
        oledSetPosition(x, y >> 3);
        oledWriteDataBlock(&uc, 1);
    }
    return 0;
}

//
// Draw a string of small (8x8), large (16x24), or very small (6x8)  characters
// At the given col+row
// The X position is in character widths (8 or 16)
// The Y position is in memory pages (8 lines each)
//
int OLED_writeString(int x, int y, const char* szMsg)
{
    int i, iLen;
    uint8_t* s;

    // if (oled_i2c == NULL) return -1; // not initialized

    iLen = strlen(szMsg);

    oledSetPosition(x * 6, y);
    if (iLen + x > 21)
        iLen = 21 - x;
    if (iLen < 0)
        return -1;
    for (i = 0; i < iLen; i++)
    {
        s = &ucSmallFont[(unsigned char)szMsg[i] * 6];
        oledWriteDataBlock(s, 6);
    }

    return 0;
}

// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
int OLED_fill(uint8_t ucData)
{
    int y;
    uint8_t temp[128];
    int iLines, iCols;

    // if (oled_i2c == NULL) return -1; // not initialized

    iLines = (oled_type == OLED_128x32 || oled_type == OLED_64x32) ? 4 : 8;
    iCols = (oled_type == OLED_64x32) ? 4 : 8;

    memset(temp, ucData, 128);
    for (y = 0; y < iLines; y++)
    {
        oledSetPosition(0, y);                // set to (0,Y)
        oledWriteDataBlock(temp, iCols * 16); // fill with data byte
    }                                         // for y
    return 0;
}

int OLED_clearLine(uint8_t line)
{
    uint8_t temp[128];

    // if (oled_i2c == NULL) return -1; // not initialized
    if (line > 4)
        return -1; // line number too big

    memset(temp, 0, 128);
    oledSetPosition(0, line);      // set to (0,Y)
    oledWriteDataBlock(temp, 128); // fill with data byte

    return 0;
}

bool OLED_status(void)
{
    return oled_active;
}

/**
 * @brief Write a byte to a I2C register
 */
static esp_err_t write(uint8_t* data, uint8_t len)
{
    int ret;

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, 0x3C, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

#endif
