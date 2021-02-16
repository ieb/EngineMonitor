
#include "configstorage.h"

const int32_t config::ok = ESP_OK;

int32_t config::initStorage() {
    static bool storageInitDone = false;
    int32_t err = ESP_OK;
    if ( storageInitDone ) {
        return ESP_OK;
    } else {
        err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            
            err = nvs_flash_erase();
            if ( err == ESP_OK ) {
                err = nvs_flash_init();
            }
        }
        if (err == ESP_OK ) {
            storageInitDone = true;
        }
    }
    return err;
}

int32_t config::writeStorage(const char * storage_namespace, const char * key, void *data, size_t len) {
    nvs_handle my_handle;
    int32_t err;
    err = config::initStorage();
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_open(storage_namespace, NVS_READWRITE, &my_handle);
    if (err != ESP_OK ) {
        return err;
    }
    err = nvs_set_blob(my_handle, key, data, len);
    if (err != ESP_OK ) {
        return err;
    }
    err = nvs_commit(my_handle);
    if (err != ESP_OK ) {
        return err;
    }
    nvs_close(my_handle);
    return ESP_OK;
}

int32_t config::readStorage(const char * storage_namespace, const char * key, void *data, size_t len) {
    nvs_handle my_handle;
    esp_err_t err;
    err = config::initStorage();
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_open(storage_namespace, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t required_size = 0; 
    err = nvs_get_blob(my_handle, key, NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }
    // datafound of the correct size.
    if ( required_size == len) {
        err = nvs_get_blob(my_handle, key, data, &required_size);
        if (err != ESP_OK) {
            return err;
        }
    }
    nvs_close(my_handle);
    return ESP_OK;
}

