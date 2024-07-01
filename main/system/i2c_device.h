// ;
#pragma once

#include "i2c.h"

class I2CDevice
{
public:
    //I2CDevice()
    //{}

    I2CDevice(uint16_t _address)
    {
        m_Device_config.device_address = _address;
    }

    inline i2c_device_config_t get_config() const
    {
        return m_Device_config;
    }

    // void set_frequency(uint32_t _frequency) { m_Device_config.scl_speed_hz = _frequency; }
    inline const char* get_tag() const
    {
        return m_Tag;
    }

public:
    virtual bool init()
    {
        if (register_me())
        {
            m_Initialized = true;
            return true;
        }
        return false;
    }

    inline bool probe() const
    {
        return I2C::probe(*this);
    }

    inline bool write(const uint8_t* _data, size_t _length) const
    {
        return I2C::write(*this, _data, _length);
    }
    inline size_t read(uint8_t* _buffer, size_t _length) const
    {
        if (!is_registered())
            return false;
        return I2C::read(*this, _buffer, _length);
    }
    inline esp_err_t write_then_read(const uint8_t* _writeData, size_t _writeLength, uint8_t* _readBuffer, size_t _readLength) const;

    inline bool write_byte_to_register(uint8_t _registerAddress, uint8_t _data) const
    {
        uint8_t buffer[2] = { _registerAddress, _data };
        return write(buffer, sizeof(buffer));
    }

    inline esp_err_t read_from_register(uint8_t _registerAddress, uint8_t* _readBuffer, size_t _readLength) const
    {
        return write_then_read(&_registerAddress, 1, _readBuffer, _readLength);
    }

    inline virtual bool test() const { return probe(); }

protected:
    inline bool register_me()
    {
        return I2C::register_device(*this);
    }
    inline bool is_initialized() const
    {
        if (m_Initialized)
            return true;

        ESP_LOGW(m_Tag, "Device is not initialized, maybe you forget to call init()?");
        return false;
    }

private:
    inline bool is_registered() const
    {
        if (m_Registered)
            return true;

        ESP_LOGW(m_Tag, "Device %s is not registered to I2C, maybe you forget to call register_me()?", m_Tag);
        return false;
    }

protected:
    /* You are responsible to manage I2C device tags. Each device must have an unique tag. */
    const char* m_Tag = "I2CDevice";
    bool m_Initialized = false;
    bool m_Registered = false;

    // std::string m_Name = "";
    i2c_device_config_t m_Device_config;
};
