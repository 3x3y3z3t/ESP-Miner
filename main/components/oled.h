//
// OLED96
// Library for accessing the 0.96" SSD1306 128x64 OLED display
// Written by Larry Bank (bitbank@pobox.com)
// Copyright (c) 2017 BitBank Software, Inc.
// Project started 1/15/2017
//

// Ported to C++ in 2024/06/08

#pragma once

#include "system/i2c_device.h"

// OLED type for init function
enum class OLEDType
{
    Unknown = 0,
    OLED_128x32,
    OLED_128x64,
    OLED_132x64,
    OLED_64x32
};

enum class OLEDFontSize
{
    Normal = 0, // 8x8
    Big,        // 16x24
    Small       // 6x8
};

struct OLEDConfig
{
    OLEDType type = OLEDType::Unknown;
    OLEDFontSize fontsize = OLEDFontSize::Normal;
    bool invertcolor = false;
    bool rotate180 = false;
    bool alternativeaddr = false;
};

class OLED : public I2CDevice
{
public:
    //OLED(uint32_t _frequency, OLEDType _oledType, bool _invert = false, bool _rotate180 = false);
    OLED(const OLEDConfig& _config, uint32_t _frequency = 100000);


    virtual bool init() override;

    void turn_off() { write_command(0xae); }
    void set_contrast(uint8_t _contrast) { write_command(0x81, _contrast); }

    // Send commands to position the "cursor" to the given
// row and column
    void set_cursor_pos(int _x, int _y);

    // Write a block of pixel data to the OLED
// Length can be anything from 1 to 1024 (whole display)
    void write_data_block(uint8_t* _data, uint16_t _length);

    // Set (or clear) an individual pixel
// The local copy of the frame buffer is used to avoid
// reading data from the display controller
    bool draw_pixel(int _x, int _y, uint8_t _color);


    // Draw a string of small (8x8), large (16x24), or very small (6x8)  characters
    // At the given col+row
    // The X position is in character widths (8 or 16)
    // The Y position is in memory pages (8 lines each)
    bool draw_string(int _x, int _y, const std::string& _string);



    /*  Clear the whole screen with the specified clear color.
     *  @param _color: Use `0x00` for black screen and `0xff` for white screen.
     */
    bool clear(uint8_t _color);

    bool clear_line(uint8_t _line, uint8_t _color);

private:
    void write_command(uint8_t _commandByte) { uint8_t buffer[2] = { 0x00, _commandByte }; write(buffer, 2); }
    void write_command(uint8_t _commandByte0, uint8_t _commandByte1) { uint8_t buffer[3] = { 0x00, _commandByte0, _commandByte1 }; write(buffer, 3); }

private:
    OLEDConfig m_Config = {};
    //OLEDType m_Oled_type = OLEDType::Unknown;
    //bool m_Is_invert = false;
    //bool m_Is_rotated_180 = false;

    int m_Screen_offset = 0;
    uint8_t m_Frame_buffer[1024] = { 0 };
};






#ifndef OLED96_H
#define OLED96_H



// Initialize the OLED96 library for a specific I2C address
// Optionally enable inverted or flipped mode
// returns 0 for success, 1 for failure
//
bool OLED_init(void);

// Turns off the display and closes the I2C handle
void OLED_shutdown(void);

// Fills the display with the byte pattern
int OLED_fill(uint8_t ucPattern);

// Write a text string to the display at x (column 0-127) and y (row 0-7)
// bLarge = 0 - 8x8 font, bLarge = 1 - 16x24 font
int OLED_writeString(int x, int y, const char* szText);

// Sets a pixel to On (1) or Off (0)
// Coordinate system is pixels, not text rows (0-127, 0-63)
int OLED_setPixel(int x, int y, uint8_t ucPixel);

// Sets the contrast (brightness) level of the display
// Valid values are 0-255 where 0=off and 255=max brightness
bool OLED_setContrast(uint8_t ucContrast);
int OLED_clearLine(uint8_t);
bool OLED_status(void);

#endif // OLED96_H

