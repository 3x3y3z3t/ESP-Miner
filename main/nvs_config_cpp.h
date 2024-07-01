#pragma once

#include <string>

namespace nvsconfig
{
    enum class ConfigKeyEnum
    {
        //WifiSSID = 0,
        //WifiPass,
        //Hostname,

        StratumUrl,
        StratumPort,
        StratumUser,
        StratumPass,

        //ASICFreq,
        ASICVoltage,
        ASICModel,

        //DeviceModel,
        BoardVersion,
        FlipScreen,
        InvertScreen,
        InvertFanPolarity,
        //AutoFanSpeed,
        FanSpeed,
        BestDiff,
        SelfTest,

        Swarm,
    };

    inline const char* const get_key(ConfigKeyEnum _keyEnum)
    {
        switch (_keyEnum)
        {
            //case ConfigKeyEnum::WifiSSID:           return "wifissid";
            //case ConfigKeyEnum::WifiPass:           return "wifipass";
            //case ConfigKeyEnum::Hostname:           return "hostname";
            case ConfigKeyEnum::StratumUrl:         return "stratumurl";
            case ConfigKeyEnum::StratumPort:        return "stratumport";
            case ConfigKeyEnum::StratumUser:        return "stratumuser";
            case ConfigKeyEnum::StratumPass:        return "stratumpass";
            //case ConfigKeyEnum::ASICFreq:           return "asicfrequency";
            case ConfigKeyEnum::ASICVoltage:        return "asicvoltage";
            case ConfigKeyEnum::ASICModel:          return "asicmodel";
            //case ConfigKeyEnum::DeviceModel:        return "devicemodel";
            case ConfigKeyEnum::BoardVersion:       return "boardversion";
            case ConfigKeyEnum::FlipScreen:         return "flipscreen";
            case ConfigKeyEnum::InvertScreen:       return "invertscreen";
            case ConfigKeyEnum::InvertFanPolarity:  return "invertfanpol";
            //case ConfigKeyEnum::AutoFanSpeed:       return "autofanspeed";
            case ConfigKeyEnum::FanSpeed:           return "fanspeed";
            case ConfigKeyEnum::BestDiff:           return "bestdiff";
            case ConfigKeyEnum::SelfTest:           return "selftest";
            case ConfigKeyEnum::Swarm:              return "swarmconfig";

            default:                                return "";
        }
    }

}

class Config
{
public:
    Config();


    void copy_from(const Config& _other);


    const std::string& get_wifi_ssid() const { return m_Wifi_ssid; }
    const std::string& get_wifi_pass() const { return m_Wifi_password; }
    const std::string& get_hostname() const { return m_Hostname; }

    uint16_t get_asic_freq() const { return m_Asic_frequency; }
    const std::string& get_device_model() const { return m_Device_model; }

    const uint16_t get_auto_fan_speed_enabled() const { return m_Auto_fan_speed_enabled; }

    void set_asic_freq(uint16_t _value);

private:
    std::string get_string(const char* _key, const char* _defaultValue);
    uint16_t get_u16(const char* _key, const uint16_t _defaultValue);
    uint64_t get_u64(const char* _key, const uint64_t _defaultValue);

    void set_string(const char* _key, const char* _value);
    void set_u16(const char* _key, const uint16_t _value);
    void set_u64(const char* _key, const uint64_t _value);



private:
    std::string m_Wifi_ssid;
    std::string m_Wifi_password;
    std::string m_Hostname;

    uint16_t m_Asic_frequency = 0U;

    std::string m_Device_model;

    uint16_t m_Auto_fan_speed_enabled = false;
};

//
//#ifndef MAIN_NVS_CONFIG_H
//#define MAIN_NVS_CONFIG_H
//
//#include <stdint.h>
//
//// Max length 15
//


//char * nvs_config_get_string(const char * key, const char * default_value);
//void nvs_config_set_string(const char* key, const char* default_value);
//uint16_t nvs_config_get_u16(const char* key, const uint16_t default_value);
//void nvs_config_set_u16(const char* key, const uint16_t value);
//uint64_t nvs_config_get_u64(const char* key, const uint64_t default_value);
//void nvs_config_set_u64(const char* key, const uint64_t value);

//#endif // MAIN_NVS_CONFIG_H
