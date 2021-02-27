# Crash 1


Looks like the interupt handler was not registered correctly with ESP_INTR_FLAG_IRAM enabled, 

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/intr_alloc.html#iram-safe-interrupt-handlers

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/spi_flash.html#iram-safe-interrupt-handlers


Writing to Flash requires this. The pulse interuts were called while writing to the flash.


            Guru Meditation Error: Core  1 panic'ed (Cache disabled but cached memory region accessed)
            Core 1 register dump:
            PC      : 0x400d70d8  PS      : 0x00060034  A0      : 0x40084658  A1      : 0x3ffbe7b0  
            A2      : 0x0000001b  A3      : 0x3ffc0988  A4      : 0x00000000  A5      : 0x08000000  
            A6      : 0x3ff42000  A7      : 0x700000bb  A8      : 0x80080f18  A9      : 0xd0000070  
            A10     : 0x00000000  A11     : 0xb0000000  A12     : 0x00000000  A13     : 0x00000000  
            A14     : 0x3ff42000  A15     : 0x700000bb  SAR     : 0x00000011  EXCCAUSE: 0x00000007  
            EXCVADDR: 0x00000000  LBEG    : 0x4000c2e0  LEND    : 0x4000c2f6  LCOUNT  : 0x00000000  
            Core 1 was running in ISR context:
            EPC1    : 0x40062230  EPC2    : 0x00000000  EPC3    : 0x00000000  EPC4    : 0x400d70d8

            Backtrace: 0x400d70d8:0x3ffbe7b0 0x40084655:0x3ffbe7d0 0x4006222d:0x3ffb1be0 0x4008e2ef:0x3ffb1c00 0x4008e326:0x3ffb1c30 0x4008e67c:0x3ffb1c60 0x4008e8d1:0x3ffb1c80 0x400869dd:0x3ffb1cb0 0x40086c54:0x3ffb1cd0 0x400df945:0x3ffb1d20 0x400ddda6:0x3ffb1d40 0x400de121:0x3ffb1d60 0x400dd1f7:0x3ffb1de0 0x400dd796:0x3ffb1e60 0x400dcbbd:0x3ffb1ed0 0x400f38e5:0x3ffb1f10 0x400d74cc:0x3ffb1f40 0x400d75eb:0x3ffb1f70 0x400d1aef:0x3ffb1f90 0x400d9125:0x3ffb1fb0 0x40088a95:0x3ffb1fd0
            #0  0x400d70d8:0x3ffbe7b0 in edgeCountHandler0() at lib/enginemonitor/enginemonitor.cpp:499
            #1  0x40084655:0x3ffbe7d0 in _xt_lowint1 at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/freertos/xtensa_vectors.S:1154
            #2  0x4006222d:0x3ffb1be0 in ?? ??:0
            #3  0x4008e2ef:0x3ffb1c00 in esp_rom_spiflash_read_status at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/spi_flash/spi_flash_rom_patch.c:413
            #4  0x4008e326:0x3ffb1c30 in esp_rom_spiflash_wait_idle at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/spi_flash/spi_flash_rom_patch.c:413
            #5  0x4008e67c:0x3ffb1c60 in esp_rom_spiflash_program_page_internal at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/spi_flash/spi_flash_rom_patch.c:413
            #6  0x4008e8d1:0x3ffb1c80 in esp_rom_spiflash_write at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/spi_flash/spi_flash_rom_patch.c:476
            #7  0x400869dd:0x3ffb1cb0 in spi_flash_write_inner at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/spi_flash/flash_ops.c:266
            #8  0x40086c54:0x3ffb1cd0 in spi_flash_write at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/spi_flash/flash_ops.c:397
            #9  0x400df945:0x3ffb1d20 in nvs::nvs_flash_write(unsigned int, void const*, unsigned int) at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/nvs_flash/src/nvs_ops.cpp:66
            #10 0x400ddda6:0x3ffb1d40 in nvs::Page::writeEntry(nvs::Item const&) at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/nvs_flash/src/nvs_page.cpp:849
            #11 0x400de121:0x3ffb1d60 in nvs::Page::writeItem(unsigned char, nvs::ItemType, char const*, void const*, unsigned int, unsigned char) at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/nvs_flash/src/nvs_page.cpp:849
            #12 0x400dd1f7:0x3ffb1de0 in nvs::Storage::writeMultiPageBlob(unsigned char, char const*, void const*, unsigned int, nvs::VerOffset) at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/nvs_flash/src/nvs_storage.cpp:557
            #13 0x400dd796:0x3ffb1e60 in nvs::Storage::writeItem(unsigned char, nvs::ItemType, char const*, void const*, unsigned int) at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/nvs_flash/src/nvs_storage.cpp:557
            #14 0x400dcbbd:0x3ffb1ed0 in nvs_set_blob at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/nvs_flash/src/nvs_api.cpp:521
            #15 0x400f38e5:0x3ffb1f10 in config::writeStorage(char const*, char const*, void*, unsigned int) at lib/configstorage/configstorage.cpp:40
            #16 0x400d74cc:0x3ffb1f40 in EngineMonitor::saveEngineHours() at lib/enginemonitor/enginemonitor.cpp:499
            #17 0x400d75eb:0x3ffb1f70 in EngineMonitor::readSensors() at lib/enginemonitor/enginemonitor.cpp:499
            #18 0x400d1aef:0x3ffb1f90 in loop() at src/main.cpp:221
            #19 0x400d9125:0x3ffb1fb0 in loopTask(void*) at /Users/ieb/.platformio/packages/framework-arduinoespressif32/cores/esp32/main.cpp:19
            #20 0x40088a95:0x3ffb1fd0 in vPortTaskWrapper at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/freertos/port.c:355 (discriminator 1)

            Rebooting...


Fixed by adding IRAM_ATTR to the handlers and DRAM_ATTR to the variables accessed by the handler both in esp_attr.h