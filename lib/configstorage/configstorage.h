#ifndef _CONFIGSTORAGE_H
#define _CONFIGSTORAGE_H

#include <Arduino.h>


#ifdef UNIT_TEST
#include <iostream> 
#define unitout_ln(x) std::cout << x << std::endl
#define unitout(x) std::cout << x 
#define NVS_READWRITE 1
#define ESP_OK 1
#define ESP_ERR_NVS_NOT_FOUND 2
#define ESP_ERR_NVS_NO_FREE_PAGES 3
#define ESP_ERR_NVS_NEW_VERSION_FOUND 4
typedef int32_t nvs_handle;
typedef int32_t esp_err_t;
extern int32_t nvs_flash_init(void);
extern int32_t nvs_flash_erase(void);
extern int32_t nvs_open(const char * storage_namespace, int flags, nvs_handle *my_handle);
extern int32_t nvs_set_blob(nvs_handle my_handle, const char *key, void * data, size_t len);
extern int32_t nvs_commit(nvs_handle my_handle);
extern int32_t nvs_close(nvs_handle my_handle);
extern int32_t nvs_get_blob(nvs_handle my_handle, const char *key, void * data, size_t * len);

#else
#define unitout_ln(x)
#define unitout(x) 
#include <nvs_flash.h>
#include <nvs.h>
#endif


namespace config {
    extern int32_t initStorage(void);
    extern int32_t readStorage(const char * storage_namespace, const char * key, void *data, size_t len);
    extern int32_t writeStorage(const char * storage_namespace, const char * key, void *data, size_t len);
    extern const int32_t ok;
}


#endif