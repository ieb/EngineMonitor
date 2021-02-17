
#ifndef ENGINE_CONFIG_H
#define ENGINE_CONFIG_H

#include <Arduino.h>
#include <enginemonitor.h>

#ifdef UNIT_TEST
#include <iostream> 
#define unitout_ln(x) std::cout << x << std::endl
#define unitout(x) std::cout << x 
#else
#define unitout_ln(x)
#define unitout(x) 
#endif

#define READBUFFER_LEN 1024

union ConfigBlob {
    char blob[sizeof(EngineMonitorConfig)];    
    EngineMonitorConfig config;
};


class EngineConfig {
    public:
        EngineConfig(EngineMonitor * engineMonitor, Stream * _io = &Serial);
        void begin(void);
        void process();
        bool isOutputEnabled();
        ConfigBlob configBlob;
        EngineMonitorConfig * config;
    private:
        char * readLine();
        void docmd(const char * command);
        void dump();
        void save();
        void load();
        void activate();
        void help();
        void setEngineTempBridge(const char * data);
        void setEngineTempThermistor(const char * data);
        void setEngineRpmScale(const char * data);
        void set1WireConfig(const char * data);
        void setReadPeriodConfig(const char * data);
        void setOilPressureConfig(const char * data);
        void setFuelLevelCconfig(const char * data);
        int loadFloatTable(const char * data, int size, float * table);
        int loadLongTable(const char * data, int size, int16_t * table);
        int match(const char * command, const char ** commands, int ncommands, const char ** startData);
        void enableOutput(bool enable);
        Stream* io;
        EngineMonitor* engineMonitor;
        bool outputOn = false;
        int bufferPos = 0;
        char inputLine[READBUFFER_LEN];

};



#endif