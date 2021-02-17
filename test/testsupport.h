
#pragma once

#include <Arduino.h>

#include <unity.h>
#include <iostream>







using namespace fakeit;


/**
 * When using this object make sure it stays in scope otherwise the lambdas will be called with 
 * references to memory that has gone and wierd things will happen.
 */ 
class MockStreamLoader {
    public:
       MockStreamLoader() {
        }
        void load(const char * _input) {
           input = _input;
           std::cout << "Input set to " << input  << "Length " << strlen(input) <<  std::endl;
           n = 0;
            When(Method(ArduinoFake(Stream), available)).AlwaysDo(
            [=]()->int{
                return this->available();
            });
            When(Method(ArduinoFake(Stream), read)).AlwaysDo(
            [=]()->char{
                return this->read();
            });
       }
       int available() {
           return input[n] != '\0';
       }
       char read() {
           char c = input[n];
           if ( c != '\0' ) {
               n++;
               return c;
           } else {
               return -1;
           }
       }
    private:
       const char * input;
       int n;

    
};






void setupStream() {

    When(OverloadedMethod(ArduinoFake(Stream), print, size_t(const char *))).AlwaysDo(
        [](const char * out)->int{ 
            std::cout << out;
            return 0;
        });
    When(OverloadedMethod(ArduinoFake(Stream), println, size_t(const char *))).AlwaysDo(
        [](const char * out)->int{ 
            std::cout << out << "[Serial]" << std::endl; 
            return 0;
        });
    When(OverloadedMethod(ArduinoFake(Stream), print, size_t(char))).AlwaysDo(
        [](char out)->int{ 
            std::cout << out; 
            return 0;
        });
    When(OverloadedMethod(ArduinoFake(Stream), println, size_t(char))).AlwaysDo(
        [](char out)->int{ 
            std::cout << out << "[Serial]" << std::endl; 
            return 0;
        });
    When(OverloadedMethod(ArduinoFake(Stream), println, size_t(int, int))).AlwaysDo(
        [](int out, int base)->int{ 
            std::cout << out << "[Serial]" << std::endl; 
            return 0;
        });
}

#define NVS_READWRITE 1
#define ESP_OK 1
#define ESP_ERR_NVS_NOT_FOUND 2
#define ESP_ERR_NVS_NO_FREE_PAGES 3
#define ESP_ERR_NVS_NEW_VERSION_FOUND 4
typedef int32_t nvs_handle_t;
typedef int32_t esp_err_t;


int32_t nvs_flash_init(void) {
    return ESP_OK;
}
int32_t nvs_flash_erase(void) {
    return ESP_OK;
}
int32_t nvs_open(const char * storage_namespace, int flags, nvs_handle_t *my_handle) {
    return ESP_OK;
}
int32_t nvs_set_blob(nvs_handle_t my_handle, const char *key, void * data, size_t len) {
    return ESP_OK;
}
int32_t nvs_commit(nvs_handle_t my_handle) {
    return ESP_OK;
}
int32_t nvs_close(nvs_handle_t my_handle) {
    return ESP_OK;
}
int32_t nvs_get_blob(nvs_handle_t my_handle, const char *key, void * data, size_t * len) {
    return ESP_OK;
}
