#pragma once

#include <nvs_flash.h>
#include <esp_log.h>

#include <string>
#include <vector>

class NVS
{
    struct NVSHandle
    {
        nvs_handle_t handle = 0U;
        nvs_open_mode_t openMode = NVS_READONLY;
        int64_t lastAccess = 0U;
        bool isBusy = false;
    };


public:
    static bool init();

    static bool try_get_string(const char* _namespace, const char* _key, std::string& _output);
    static bool try_get_u16(const char* _namespace, const char* _key, uint16_t& _output);
    static bool try_get_u64(const char* _namespace, const char* _key, uint64_t& _output);


    static bool try_set_string(const char* _namespace, const char* _key, const char* _value);
    static bool try_set_u16(const char* _namespace, const char* _key, uint16_t _value);
    static bool try_set_u64(const char* _namespace, const char* _key, uint64_t _value);

    inline static bool is_initialized()
    {
        if (s_Is_initialized)
            return true;

        ESP_LOGW(s_Tag, "NVS is not not ready, maybe you forgot to call NVS::init() AT THE START OF THE PROGRAM?");
        return false;
    }


private:
    static NVSHandle* get_or_compute_handle(const char* _namespace, nvs_open_mode_t _openMode);
    static void cleanup_handle();
    //static nvs_handle_t open(const char* _namespace, nvs_open_mode_t _openMode);

private:
    static bool s_Is_initialized;


    static std::vector<NVSHandle> s_Opened_handle;


};










