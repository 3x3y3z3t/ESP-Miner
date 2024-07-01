// ;
#pragma once

#include <string>
#include <unordered_map>

#include <driver/i2c_master.h>
#include <driver/i2c_types.h>
#include <esp_log.h>
#include <hal/gpio_types.h>

#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */

class I2CDevice;

struct I2CConfig
{
    gpio_num_t gpioSCL = GPIO_NUM_NC;
    gpio_num_t gpioSDA = GPIO_NUM_NC;

    i2c_port_num_t portNumber = 0;
    bool enableTxBuffer = false;
    bool enableRxBuffer = false;

    // int32_t frequency = -1;
    uint16_t timeoutMs = 1000;
};

struct I2CDeviceInfo
{
    i2c_addr_bit_len_t addressLength = I2C_ADDR_BIT_LEN_7;
    uint16_t address = 0x00;
    uint32_t frequency = 0;
};

class I2C {

  public:
    static bool init(const I2CConfig& _config);

    inline static bool probe(const I2CDevice& _device)
    {
        if (!is_initialized())
            return false;
        return s_Instance.probe_internal(_device);
    }
    inline static bool write(const I2CDevice& _device, const uint8_t* _data, size_t _length)
    {
        if (!is_initialized())
            return false;
        return s_Instance.write_internal(_device, _data, _length);
    }
    static size_t read(const I2CDevice& _device, uint8_t* _buffer, size_t _length)
    {
        if (!is_initialized())
            return false;
        return s_Instance.read_internal(_device, _buffer, _length);
    }
    static size_t write_then_read(const I2CDevice& _device, const uint8_t* _writeData, size_t _writeLength, uint8_t* _readBuffer, size_t _readLength)
    {
        if (!is_initialized())
            return false;
        return s_Instance.write_then_read_internal(_device, _writeData, _writeLength, _readBuffer, _readLength);
    }

    static bool activate_bus();
    static bool deactivate_bus();

    inline static bool register_device(const I2CDevice& _device)
    {
        if (!is_initialized())
            return false;
        return s_Instance.register_device_internal(_device);
    }
    inline static bool unregister_device(const I2CDevice& _device)
    {
        if (!is_initialized())
            return false;
        return s_Instance.unregister_device_internal(_device);
    }

    inline static bool is_initialized()
    {
        if (s_Bus_ready)
            return true;

        ESP_LOGW(s_Tag, "I2C Bus is not ready, maybe you forgot to call I2C::init()?");
        return false;
    }

  private:
    bool probe_internal(const I2CDevice& _device);
    bool write_internal(const I2CDevice& _device, const uint8_t* _data, size_t _length);
    size_t read_internal(const I2CDevice& _device, uint8_t* _buffer, size_t _length);
    size_t write_then_read_internal(const I2CDevice& _device, const uint8_t* _writeData, size_t _writeLength, uint8_t* _readBuffer, size_t _readLength);

    bool register_device_internal(const I2CDevice& _device);
    bool unregister_device_internal(const I2CDevice& _device);


  private:
    static const char* const s_Tag;

    static I2C s_Instance;
    static bool s_Bus_ready;

    i2c_master_bus_handle_t m_Master_bus_handle = nullptr;
    // i2c_master_dev_handle_t m_Active_device_handle = nullptr;

    I2CConfig m_Config;
    i2c_master_bus_config_t m_Master_bus_config;
    std::unordered_map<std::string, i2c_master_dev_handle_t> m_Slave_devices;
};
