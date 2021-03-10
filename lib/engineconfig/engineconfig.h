
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

#define CMD_HELP 0
#define CMD_DUMP 1
#define CMD_SAVE 2
#define CMD_LOAD 3
#define CMD_RESET 4
#define CMD_ACTIVATE 5
#define CMD_ENABLE_MON 6
#define CMD_DISABLE_MON 7
#define CMD_ETC_S 8
#define CMD_ETC_L 9
#define CMD_ETT_S 10
#define CMD_ETT_L 11

#define CMD_RPM_S 12
#define CMD_OWC_S 13
#define CMD_OWC_L 14
#define CMD_RPC_S 15
#define CMD_RPC_L 16
#define CMD_OPC_S 17
#define CMD_OPC_L 18
#define CMD_FLC_S 19
#define CMD_FLC_L 20
#define CMD_STATUS 21
#define CMD_UPD_S 22
#define CMD_UPD_L 23
#define NCOMMANDS 24


union ConfigBlob {
    char blob[sizeof(EngineMonitorConfig)];    
    EngineMonitorConfig config;
};



class EngineConfig {
    public:
        EngineConfig(EngineMonitor * engineMonitor, Stream * _io = &Serial);
        void begin(void);
        void dump();
        void process(void cb(int8_t));
        bool isMonitoringEnabled();
        ConfigBlob configBlob;
        EngineMonitorConfig * config;
    private:
        char * readLine();
        int8_t docmd(const char * command);
        void save();
        void load();
        void reset();
        void activate();
        void help();
        void setUpdatePeriod(const char * data);
        void setEngineTempBridge(const char * data);
        void setEngineTempThermistor(const char * data);
        void setEngineRpmScale(const char * data);
        void set1WireConfig(const char * data);
        void setReadPeriodConfig(const char * data);
        void setOilPressureConfig(const char * data);
        void setFuelLevelCconfig(const char * data);
        int loadFloatTable(const char * data, int size, float * table);
        int loadLongTable(const char * data, int size, int16_t * table);
        int loadUnsignedLongTable(const char * data, int size, unsigned long * table);
        int match(const char * command, const char ** commands, int ncommands, const char ** startData);
        void enableMonitoring(bool enable);
        Stream* io;
        EngineMonitor* engineMonitor;
        bool monitoringOn = false;
        int bufferPos = 0;
        char inputLine[READBUFFER_LEN];

};



#endif