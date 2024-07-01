// ;
#include "i2c.h"

#include "i2c_device.h"

#include <driver/i2c_master.h>
#include <esp_debug_helpers.h>
#include <esp_log.h>

const char* const I2C::s_Tag = "I2C";

I2C I2C::s_Instance;
bool I2C::s_Bus_ready = false;

bool I2C::init(const I2CConfig& _config)
{
    ESP_LOGI(s_Tag, "Initializing I2C..");
    if (s_Bus_ready)
    {
        ESP_LOGD(s_Tag, "Bus already activated.");
        return false;
    }

    s_Instance.m_Config = _config;

    s_Instance.m_Master_bus_config = {
        .i2c_port = _config.portNumber,
        .sda_io_num = _config.gpioSDA,
        .scl_io_num = _config.gpioSCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags =
            {
                .enable_internal_pullup = true,
            },
    };

    activate_bus();

    ESP_LOGD(s_Tag, "Done.");
    return true;
}

#if 0
bool I2C::probe(const std::string& _name)
{
    if (!is_initialized())
        return false;

    auto dvcInfo = s_Instance.m_Slave_devices.find(_name);
    if (dvcInfo == s_Instance.m_Slave_devices.end())
        return false;

    esp_err_t status = i2c_master_probe(s_Instance.m_Master_bus_handle, dvcInfo->second.device_address, -1);
    ESP_ERROR_CHECK(status);

    return status == ESP_OK;
}

bool I2C::write(const std::string& _name, const uint8_t* _data, size_t _length)
{
    if (!is_initialized())
        return false;

    auto dvcInfo = s_Instance.m_Slave_devices.find(_name);
    if (dvcInfo == s_Instance.m_Slave_devices.end())
        return false;

    // activate device;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_Instance.m_Master_bus_handle, &dvcInfo->second, &s_Instance.m_Active_device_handle));

    ESP_ERROR_CHECK(i2c_master_transmit(s_Instance.m_Active_device_handle, _data, _length, -1));

    // deactivate device;
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(s_Instance.m_Active_device_handle));

    return false;
}

size_t I2C::read(const std::string& _name, uint8_t* _buffer, size_t _length)
{
    if (!is_initialized())
        return 0;

    auto dvcInfo = s_Instance.m_Slave_devices.find(_name);
    if (dvcInfo == s_Instance.m_Slave_devices.end())
        return 0;

    // activate device;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_Instance.m_Master_bus_handle, &dvcInfo->second, &s_Instance.m_Active_device_handle));

    ESP_ERROR_CHECK(i2c_master_receive(s_Instance.m_Active_device_handle, _buffer, _length, -1));

    // deactivate device;
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(s_Instance.m_Active_device_handle));

    return _length;
}

size_t I2C::write_then_read(const std::string& _name, const uint8_t* _writeData, size_t _writeLength, uint8_t* _readBuffer, size_t _readLength)
{
    if (!is_initialized())
        return 0;

    auto dvcInfo = s_Instance.m_Slave_devices.find(_name);
    if (dvcInfo == s_Instance.m_Slave_devices.end())
        return 0;

    // activate device;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_Instance.m_Master_bus_handle, &dvcInfo->second, &s_Instance.m_Active_device_handle));

    ESP_ERROR_CHECK(i2c_master_transmit_receive(s_Instance.m_Active_device_handle, _writeData, _writeLength, _readBuffer, _readLength, -1));

    // deactivate device;
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(s_Instance.m_Active_device_handle));

    return _readLength;
}

bool I2C::write(const i2c_device_config_t& _deviceCfg, const uint8_t* _data, size_t _length)
{
    if (!is_initialized())
        return false;

    // activate device;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_Instance.m_Master_bus_handle, &_deviceCfg, &s_Instance.m_Active_device_handle));

    ESP_ERROR_CHECK(i2c_master_transmit(s_Instance.m_Active_device_handle, _data, _length, s_Instance.m_Config.timeoutMs));

    // deactivate device;
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(s_Instance.m_Active_device_handle));

    return false;
}
#endif

bool I2C::activate_bus()
{
    ESP_LOGI(s_Tag, "Activating I2C Master bus..");

    if (s_Bus_ready)
    {
        ESP_LOGD(s_Tag, "Bus already activated.");
        return false;
    }

    ESP_ERROR_CHECK(i2c_new_master_bus(&s_Instance.m_Master_bus_config, &s_Instance.m_Master_bus_handle));

    s_Bus_ready = true;
    ESP_LOGD(s_Tag, "Done.");
    return true;
}

bool I2C::deactivate_bus()
{
    if (!is_initialized())
        return false;

    ESP_ERROR_CHECK(i2c_del_master_bus(s_Instance.m_Master_bus_handle));

    s_Bus_ready = false;
    return true;
}

#if 0
bool I2C::activate_device(const std::string& _name)
{
    return false;
}

bool I2C::deactivate_device(const std::string& _name)
{
    return false;
}

bool I2C::register_device(const std::string& _name, const I2CDeviceInfo& _dvcInfo)
{
    if (!is_initialized())
        return false;

    if (s_Instance.m_Slave_devices.contains(_name))
        return false;

    i2c_device_config_t cfg = {
        .dev_addr_length = _dvcInfo.addressLength,
        .device_address = _dvcInfo.address,
        .scl_speed_hz = _dvcInfo.frequency,
    };

    s_Instance.m_Slave_devices.emplace(_name, cfg);
    return true;
}

bool I2C::unregister_device(const std::string& _name)
{
    if (!is_initialized())
        return false;

    if (!s_Instance.m_Slave_devices.contains(_name))
        return false;

    s_Instance.m_Slave_devices.erase(_name);
    return true;
}
#endif

bool I2C::probe_internal(const I2CDevice& _device)
{
    auto addr = _device.get_config().device_address;
    auto tag = _device.get_tag();

    esp_err_t status = i2c_master_probe(m_Master_bus_handle, addr, m_Config.timeoutMs);
    switch (status)
    {
        case ESP_OK:
            ESP_LOGD(s_Tag, "Probe device %s (0x%x): Device found.", tag, addr);
            return true;
        case ESP_ERR_NOT_FOUND:
            ESP_LOGI(s_Tag, "Probe device %s (0x%x): Device not found.", tag, addr);
            return false;
        case ESP_ERR_TIMEOUT:
            ESP_LOGI(s_Tag, "Probe device %s (0x%x): Timeout.", tag, addr);
            return false;
    }

    ESP_ERROR_CHECK(status);
    return false;
}

bool I2C::write_internal(const I2CDevice& _device, const uint8_t* _data, size_t _length)
{
    auto iter = m_Slave_devices.find(_device.get_tag());
    if (iter == m_Slave_devices.end())
        return false;

    i2c_master_dev_handle_t handle = iter->second;
    ESP_ERROR_CHECK(i2c_master_transmit(handle, _data, _length, m_Config.timeoutMs));

    return true;
}

size_t I2C::read_internal(const I2CDevice& _device, uint8_t* _buffer, size_t _length)
{
    auto iter = m_Slave_devices.find(_device.get_tag());
    if (iter == m_Slave_devices.end())
        return 0;

    i2c_master_dev_handle_t handle = iter->second;
    ESP_ERROR_CHECK(i2c_master_receive(handle, _buffer, _length, m_Config.timeoutMs));

    return _length;
}

size_t I2C::write_then_read_internal(const I2CDevice& _device, const uint8_t* _writeData, size_t _writeLength,
                                     uint8_t* _readBuffer, size_t _readLength)
{
    auto iter = m_Slave_devices.find(_device.get_tag());
    if (iter == m_Slave_devices.end())
        return 0;

    i2c_master_dev_handle_t handle = iter->second;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(handle, _writeData, _writeLength, _readBuffer, _readLength, m_Config.timeoutMs));

    return _readLength;
}

bool I2C::register_device_internal(const I2CDevice& _device)
{
    if (m_Slave_devices.contains(_device.get_tag()))
        return true;

    // activate device;
    i2c_device_config_t cfg = _device.get_config();
    i2c_master_dev_handle_t handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(m_Master_bus_handle, &cfg, &handle));

    m_Slave_devices.emplace(_device.get_tag(), handle);
    return true;
}

bool I2C::unregister_device_internal(const I2CDevice& _device)
{
    auto iter = m_Slave_devices.find(_device.get_tag());
    if (iter == m_Slave_devices.end())
        return true;

    // deactivate device;
    i2c_master_dev_handle_t handle = iter->second;
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(handle));

    m_Slave_devices.erase(_device.get_tag());
    return true;
}
