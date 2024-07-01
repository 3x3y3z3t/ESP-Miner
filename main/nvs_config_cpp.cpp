 #include "nvs_config_cpp.h"

#include "system/nvs.h"

#include <esp_log.h>

#include <string.h>

constexpr const char* NVS_CONFIG_WIFI_SSID = "wifissid";
constexpr const char* NVS_CONFIG_WIFI_PASS = "wifipass";
constexpr const char* NVS_CONFIG_HOSTNAME = "hostname";
//#define NVS_CONFIG_STRATUM_URL "stratumurl"
//#define NVS_CONFIG_STRATUM_PORT "stratumport"
//#define NVS_CONFIG_STRATUM_USER "stratumuser"
//#define NVS_CONFIG_STRATUM_PASS "stratumpass"
constexpr const char* NVS_CONFIG_ASIC_FREQ = "asicfrequency";
//#define NVS_CONFIG_ASIC_VOLTAGE "asicvoltage"
//#define NVS_CONFIG_ASIC_MODEL "asicmodel"
constexpr const char* NVS_CONFIG_DEVICE_MODEL = "devicemodel";
//#define NVS_CONFIG_BOARD_VERSION "boardversion"
//#define NVS_CONFIG_FLIP_SCREEN "flipscreen"
//#define NVS_CONFIG_INVERT_SCREEN "invertscreen"
//#define NVS_CONFIG_INVERT_FAN_POLARITY "invertfanpol"
constexpr const char* NVS_CONFIG_AUTO_FAN_SPEED = "autofanspeed";
//#define NVS_CONFIG_FAN_SPEED "fanspeed"
//#define NVS_CONFIG_BEST_DIFF "bestdiff"
//#define NVS_CONFIG_SELF_TEST "selftest"
//
//#define NVS_CONFIG_SWARM "swarmconfig"

constexpr const char* NVS_CONFIG_NAMESPACE = "main";
constexpr const char* TAG = "nvs_config";

Config::Config()
{
    // we don't need to check for nvs initialization, it should be init already;
    
    m_Wifi_ssid = get_string(NVS_CONFIG_WIFI_SSID, WIFI_SSID);
    m_Wifi_password = get_string(NVS_CONFIG_WIFI_PASS, WIFI_PASS);
    m_Hostname = get_string(NVS_CONFIG_HOSTNAME, HOSTNAME);

    m_Asic_frequency = get_u16(NVS_CONFIG_ASIC_FREQ, CONFIG_ASIC_FREQUENCY);

    m_Device_model = get_string(NVS_CONFIG_DEVICE_MODEL, "");

    m_Auto_fan_speed_enabled = get_u16(NVS_CONFIG_AUTO_FAN_SPEED, 1U);







}

void Config::copy_from(const Config& _other)
{

}

void Config::set_asic_freq(uint16_t _value)
{
    set_u16(NVS_CONFIG_ASIC_FREQ, _value);
    m_Asic_frequency = _value;
}


std::string get_string(const char* _key, const char* _defaultValue)
{
    std::string output;
    if (!NVS::try_get_string(NVS_CONFIG_NAMESPACE, _key, output))
    {
        ESP_LOGW(TAG, "Could not read nvs key: %s", _key);
        return _defaultValue;
    }

    return output;
}

uint16_t Config::get_u16(const char* _key, const uint16_t _defaultValue)
{
    uint16_t output;
    if (!NVS::try_get_u16(NVS_CONFIG_NAMESPACE, _key, output))
    {
        return _defaultValue;
    }

    return (uint16_t)output;
}

uint64_t get_u64(const char* _key, const uint64_t _defaultValue)
{
    uint64_t output;
    if (!NVS::try_get_u64(NVS_CONFIG_NAMESPACE, _key, output))
    {
        return _defaultValue;
    }

    return (uint64_t)output;
}

void set_string(const char* _key, const char* _value)
{
    if (!NVS::try_set_string(NVS_CONFIG_NAMESPACE, _key, _value))
    {
        ESP_LOGW(TAG, "Could not write nvs key: %s, value: %s", _key, _value);
    }
}

void Config::set_u16(const char* _key, const uint16_t _value)
{
    if (!NVS::try_set_u16(NVS_CONFIG_NAMESPACE, _key, _value))
    {
        ESP_LOGW(TAG, "Could not write nvs key: %s, value: %u", _key, _value);
    }
}

void set_u64(const char* _key, const uint64_t _value)
{
    if (!NVS::try_set_u64(NVS_CONFIG_NAMESPACE, _key, _value))
    {
        ESP_LOGW(TAG, "Could not write nvs key: %s, value: %ull", _key, _value);
    }
}
