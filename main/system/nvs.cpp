#include "nvs.h"


#include <esp_timer.h>
#include <nvs_config.h>


/** The time (in microsecond) for a handle to be keep open.
 *  When a handle is not accessed for this amount of time, it will be closed.
 */
constexpr int64_t c_Handle_expire_time = 60 * 1000 * 1000;

nvs_handle_t open(const char* _namespace, nvs_open_mode_t _openMode);
bool close(nvs_handle_t _handle);


constexpr const char* TAG = "nvs";
bool NVS::s_Is_initialized = false;

std::vector<NVS::NVSHandle> NVS::s_Opened_handle;

bool NVS::init()
{
    if (s_Is_initialized)
        return true; // prevent DDOS by calling init() in infinite loop lol;

    esp_err_t err = nvs_flash_init();

    if (err != ESP_OK)
    {
        ESP_ERROR_CHECK(err);
        return false;
    }

    // TODO: create cleanup task to cleanup unused handle;

    s_Is_initialized = true;
    return true;
}

bool NVS::try_get_string(const char* _namespace, const char* _key, std::string& _output)
{
    if (!is_initialized()) return false;
    if (_key == nullptr || strcmp(_key, "") == 0) return false;

    NVSHandle* handle = get_or_compute_handle(_namespace, NVS_READONLY);
    if (handle == nullptr) return false;

    size_t length = 0;
    // nvs string are currently limited to 4096b;
    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/nvs_flash.html#keys-and-values

    char buffer[4096] = { 0 };
    esp_err_t err = nvs_get_str(handle->handle, _key, buffer, &length);

    handle->isBusy = false;
    handle->lastAccess = esp_timer_get_time();

    if (err != ESP_OK)
    {
        return false;
    }

    _output = buffer;
    return true;
}

bool NVS::try_get_u16(const char* _namespace, const char* _key, uint16_t& _output)
{
    if (!is_initialized()) return false;
    if (_key == nullptr || strcmp(_key, "") == 0) return false;

    NVSHandle* handle = get_or_compute_handle(_namespace, NVS_READONLY);
    if (handle == nullptr) return false;

    uint16_t output;
    esp_err_t err = nvs_get_u16(handle->handle, _key, &output);

    handle->isBusy = false;
    handle->lastAccess = esp_timer_get_time();

    if (err != ESP_OK)
    {
        return false;
    }

    _output = output;
    return true;
}

bool NVS::try_get_u64(const char* _namespace, const char* _key, uint64_t& _output)
{
    if (!is_initialized()) return false;
    if (_key == nullptr || strcmp(_key, "") == 0) return false;

    NVSHandle* handle = get_or_compute_handle(_namespace, NVS_READONLY);
    if (handle == nullptr) return false;

    uint64_t output;
    esp_err_t err = nvs_get_u64(handle->handle, _key, &output);

    handle->isBusy = false;
    handle->lastAccess = esp_timer_get_time();

    if (err != ESP_OK)
    {
        return false;
    }

    _output = output;
    return true;
}

bool NVS::try_set_string(const char* _namespace, const char* _key, const char* _value)
{
    if (!is_initialized()) return false;
    if (_key == nullptr || strcmp(_key, "") == 0) return false;

    NVSHandle* handle = get_or_compute_handle(_namespace, NVS_READWRITE);
    if (handle == nullptr) return false;

    esp_err_t err = nvs_set_str(handle->handle, _key, _value);

    handle->isBusy = false;
    handle->lastAccess = esp_timer_get_time();

    if (err != ESP_OK)
    {
        return false;
    }

    return true;
}

bool NVS::try_set_u16(const char* _namespace, const char* _key, uint16_t _value)
{
    if (!is_initialized()) return false;
    if (_key == nullptr || strcmp(_key, "") == 0) return false;

    NVSHandle* handle = get_or_compute_handle(_namespace, NVS_READWRITE);
    if (handle == nullptr) return false;

    esp_err_t err = nvs_set_u16(handle->handle, _key, _value);

    handle->isBusy = false;
    handle->lastAccess = esp_timer_get_time();

    if (err != ESP_OK)
    {
        return false;
    }

    return true;
}

bool NVS::try_set_u64(const char* _namespace, const char* _key, uint64_t _value)
{
    if (!is_initialized()) return false;
    if (_key == nullptr || strcmp(_key, "") == 0) return false;

    NVSHandle* handle = get_or_compute_handle(_namespace, NVS_READWRITE);
    if (handle == nullptr) return false;

    esp_err_t err = nvs_set_u64(handle->handle, _key, _value);

    handle->isBusy = false;
    handle->lastAccess = esp_timer_get_time();

    if (err != ESP_OK)
    {
        return false;
    }

    return true;
}



NVS::NVSHandle* NVS::get_or_compute_handle(const char* _namespace, nvs_open_mode_t _openMode)
{
    int index = -1;
    for (size_t i = 0; i < s_Opened_handle.size(); ++i)
    {
        if (s_Opened_handle[i].isBusy) continue;
        if (_openMode == NVS_READWRITE && s_Opened_handle[i].openMode != NVS_READWRITE) continue;

        s_Opened_handle[i].isBusy = true;
        index = i;
        break;
    }

    if (index == -1)
    {
        nvs_handle_t h = open(_namespace, NVS_READONLY);
        if (h == 0) return nullptr;

        s_Opened_handle.emplace_back(h, NVS_READONLY, true, 0);
        index = s_Opened_handle.size() - 1;
    }

    return &s_Opened_handle[index];
}



void NVS::cleanup_handle()
{
    if (!s_Is_initialized) return;

    // TODO: lock the object;

    auto currentTime = esp_timer_get_time();
    for (size_t i = 0; i < s_Opened_handle.size(); ++i)
    {
        if (s_Opened_handle[i].isBusy) continue;

        if (currentTime - s_Opened_handle[i].lastAccess > c_Handle_expire_time)
        {
            if (!close(s_Opened_handle[i].handle))
                continue;

            s_Opened_handle.erase(s_Opened_handle.begin() + i);
            --i;
        }
    }
}



nvs_handle_t open(const char* _namespace, nvs_open_mode_t _openMode)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(_namespace, _openMode, &handle);

    if (err == ESP_OK)
        return handle;

    // TODO: log error when nvs open fail;
    ESP_LOGW(TAG, "Could not open NVS under namespace '%s' (mode %d)", _namespace, (int)_openMode);
    return 0;
}

bool close(nvs_handle_t _handle)
{
    nvs_close(_handle);
    return true;
}







