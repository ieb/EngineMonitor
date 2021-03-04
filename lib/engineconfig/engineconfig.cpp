#include "engineconfig.h"
#include <configstorage.h>
#include <cstdlib>


#define CMD_HELP 0
#define CMD_DUMP 1
#define CMD_SAVE 2
#define CMD_LOAD 3
#define CMD_RESET 4
#define CMD_ACTIVATE 5
#define CMD_ENABLE_OUTPUT 6
#define CMD_DISABLE_OUTPUT 7
#define CMD_ENABLE_MON 8
#define CMD_DISABLE_MON 9
#define CMD_ETC_S 10
#define CMD_ETC_L 11
#define CMD_ETT_S 12
#define CMD_ETT_L 13

#define CMD_RPM_S 14
#define CMD_OWC_S 15
#define CMD_OWC_L 16
#define CMD_RPC_S 17
#define CMD_RPC_L 18
#define CMD_OPC_S 19
#define CMD_OPC_L 20
#define CMD_FLC_S 21
#define CMD_FLC_L 22
#define NCOMMANDS 23


static const char * engineconfig_commands[] = {
    "help",
    "dump",
    "save",
    "load",
    "reset",
    "activate",
    "on",
    "off",
    "mon",
    "moff",
    "etc ",
    "engine temp config ",
    "ett ",
    "engine temp therm ",
    "rpm scale ",
    "owc ",
    "1 wire ",
    "rpc ",
    "read period ",
    "opc ",
    "oil pressure ",
    "flc ",
    "fuel level "
};


EngineConfig::EngineConfig(EngineMonitor * _engineMonitor, Stream * _io) {
    io = _io;
    engineMonitor = _engineMonitor;
    config = &(configBlob.config);
}


void EngineConfig::begin() {
    reset();
    load();
}




void EngineConfig::help() {
    io->println("Lists of numbers should be comma seperated with no spaces.");
    io->println("Commands:");
    io->println("help|? - this help");
    io->println("dump                                 - Dumps the current configuration");
    io->println("save                                 - Saves the current configuration to non volatile storage.");
    io->println("load                                 - Loads configuration from no volatile storage");
    io->println("reset                                - Factory reset");
    io->println("actviate                             - Activates Current configuration");
    io->println("on                                   - Enables diagnostic output");
    io->println("off                                  - Disables diagnostic output");
    io->println("mon                                  - Enables sensor monitoring");
    io->println("moff                                 - Disables sensor monitoring");


    io->println("etc|engine temp config <v>,<r>      - set engine coolant top resistor and voltage , floatx2, default 5,545.5");
    io->println("ett|engine temp therm <n>,...<n>     - set engine coolant thermistor resistance values 0C-120C, floatx13, default as per manual");
    io->println("rpm scale <n>                        - set rpm per Hz scale, float, default 6.224463028");
    io->println("owc|1 wire <a>,..                  - set one wire index for alternator, exhaust, engine room, intx3, default 0,1,2");
    io->println("rpc|read period  <r>,..              - set read period in ms or rpm, engine, voltage, temp, intx4, default 2000,5000,10000,30000");
    io->println("opc|oil pressure  <o>, <s>           - set oil pressure offset and scale, floatx2, defult 0.5,50");
    io->println("flc|fuel level  <v>,<r1>,<re>,<rf>   - set fuel level r1, voltage, rempty, rfull, floatx4, defult 5,545,190,3");



}

void EngineConfig::docmd(const char * command) {
    const char *data;
    int cid = match(command, engineconfig_commands, NCOMMANDS, &data );
    unitout("Matched "); unitout_ln(cid); 

    switch(cid) {
        case CMD_ACTIVATE: 
            activate(); 
            break;
        case CMD_RESET: 
            reset(); 
            break;
        case CMD_LOAD: 
            load(); 
            break;
        case CMD_SAVE: 
            save(); 
            break;
        case CMD_DUMP: 
            dump(); 
            break;
        case CMD_HELP: 
            help(); 
            break;
        case CMD_ENABLE_OUTPUT: 
            enableOutput(true); 
            break;
        case CMD_DISABLE_OUTPUT: 
            enableOutput(false); 
            break;
        case CMD_ENABLE_MON:
            enableMonitoring(true);
            break;
        case CMD_DISABLE_MON:
            enableMonitoring(false);
            break;
        case CMD_ETC_S:
        case CMD_ETC_L:
            setEngineTempBridge(data);
            break;
        case CMD_ETT_S:
        case CMD_ETT_L:
            setEngineTempThermistor(data);
            break;
        case CMD_RPM_S :
            setEngineRpmScale(data);
            break;
        case CMD_OWC_S:
        case CMD_OWC_L:
            set1WireConfig(data);
            break;
        case CMD_RPC_S:
        case CMD_RPC_L:
            setReadPeriodConfig(data);
            break;
        case CMD_OPC_S:
        case CMD_OPC_L:
            setOilPressureConfig(data);
            break;
        case CMD_FLC_S:
        case CMD_FLC_L:
            setFuelLevelCconfig(data);
            break;

        default:
            io->print(F("Command Not recognised:"));
            io->println(command);
            break;

    }
}

void EngineConfig::save() {
    io->println("Do save to flash");
    int32_t err = config::writeStorage(STORAGE_NAMESPACE, STORAGE_KEY, configBlob.blob, sizeof(EngineMonitorConfig));
    if ( err != config::ok ) {
        io->print("Save Failed, err:");
        io->println(err);
    }
}

void EngineConfig::activate() {
    engineMonitor->calibrate(config);
}


void EngineConfig::reset() {
    memcpy(config, &defaultEngineMonitorConfig, sizeof(EngineMonitorConfig));
}

void EngineConfig::load() {
    // load from flash
    io->println("Do load");
    int32_t err = config::readStorage(STORAGE_NAMESPACE, STORAGE_KEY, configBlob.blob, sizeof(EngineMonitorConfig));
    if ( err != config::ok ) {
        io->print("Load Failed, err:");
        io->println(err);
    } else {
        io->println("Loaded");
    }
}

void EngineConfig::enableOutput(bool enable) {
    outputOn = enable;
}

bool EngineConfig::isOutputEnabled() {
    return outputOn;
}

void EngineConfig::enableMonitoring(bool enable) {
    monitoringOn = enable;
}

bool EngineConfig::isMonitoringEnabled() {
    return monitoringOn;
}


void EngineConfig::process() {
    char * command = readLine();
    if ( command != NULL ) {
        unitout("Processing"); unitout_ln(inputLine);
        docmd(command);
        io->print("$>");
    }
}


void EngineConfig::dump() { 
    char buffer[80];
    sprintf(buffer,"Alternator temperature sensor one wire index %d", config->alternatorTemperatureIDX);
    io->println(buffer);
    sprintf(buffer,"Exhaust temperature sensor one wire index %d", config->exhaustTemperatureIDX);
    io->println(buffer);
    sprintf(buffer,"Engine room temperature sensor one wire index %d", config->engineRoomTemperatureIDX);
    io->println(buffer);
    sprintf(buffer,"RPM read period %d", config->flywheelRPMReadPeriod);
    io->println(buffer);
    sprintf(buffer,"Engine temperature read period %d", config->engineTemperatureReadPeriod);
    io->println(buffer);
    sprintf(buffer,"Voltage read period %d", config->voltageReadPeriod);
    io->println(buffer);
    sprintf(buffer,"Temperature sensor read period %d", config->temperatureReadPeriod);
    io->println(buffer);
    sprintf(buffer,"Oil pressure scale %f", config->oilPressureScale);
    io->println(buffer);
    sprintf(buffer,"Oil pressure offset %f", config->oilPressureOffset);
    io->println(buffer);
    sprintf(buffer,"Fuel level V %f", config->fuelLevelVin);
    io->println(buffer);
    sprintf(buffer,"Fuel level R1 %f", config->fuelLevelR1);
    io->println(buffer);
    sprintf(buffer,"Fuel level R1 %f", config->fuelLevelEmptyR);
    io->println(buffer);
    sprintf(buffer,"Fuel level R1 %f", config->fuelLevelFullR);
    io->println(buffer);
    sprintf(buffer,"Engine RPM per Hz scale %f RPM/Hz", config->engineFlywheelRPMPerHz);
    io->println(buffer);
    sprintf(buffer,"Engine temperature bridge R1 %f R", config->coolantTempR1);
    io->println(buffer);
    sprintf(buffer,"Engine temperature bridge Voltage %f V ", config->coolantTempVin);
    io->println(buffer);
    io->println("Engine thermistor table ");
    io->println("   C,   R");
    for(int i = 0; i < MAX_ENGINE_TEMP; i++) {
        sprintf(buffer," %d, %f", i*10,  config->coolantTempR2[i]);
        io->println(buffer);
    }
}


void EngineConfig::setEngineTempBridge(const char * data){ 
    float fields[2];
    int nfields = loadFloatTable(data, 2, &fields[0]);
    if ( nfields == 2 ) {
        config->coolantTempVin = fields[0];
        config->coolantTempR1 = fields[1];
        io->println("Updated Coolant Thermistor config.");
    } else {
        io->println("Incorrect number of values supplied.");
    }
}

void EngineConfig::setEngineTempThermistor(const char * data) { 
    float fields[MAX_ENGINE_TEMP];
    int nfields = loadFloatTable(data, MAX_ENGINE_TEMP, &fields[0]);
    if ( nfields ==  MAX_ENGINE_TEMP) {
        for(int i = 0; i < MAX_ENGINE_TEMP; i++ ) {
            config->coolantTempR2[i] = fields[i];
        }
        io->println("Updated Thermistor resistance values.");
    } else {
        io->println("Incorrect number of resistance values supplied.");
    }
}

void EngineConfig::setEngineRpmScale(const char * data) {  
    config->engineFlywheelRPMPerHz = atof(data);
    io->println("Set Engine RPM Scale");
}

void EngineConfig::set1WireConfig(const char * data){ 
    int16_t fields[3];
    int nfields = loadLongTable(data, 3, &fields[0]);
    if ( nfields == 3) {
        config->alternatorTemperatureIDX = (int8_t)fields[0];
        config->exhaustTemperatureIDX = (int8_t)fields[1];
        config->engineRoomTemperatureIDX = (int8_t)fields[2];
        io->println("Updated One Wire Config.");
    } else {
        io->println("Incorrect number of values supplied.");
    }
}
void EngineConfig::setReadPeriodConfig(const char * data){ 
    int16_t fields[4];
    int nfields = loadLongTable(data, 4, &fields[0]);
    if ( nfields == 4) {
        config->flywheelRPMReadPeriod = fields[0];
        config->engineTemperatureReadPeriod = fields[1];
        config->voltageReadPeriod = fields[2];
        config->temperatureReadPeriod = fields[3];
        io->println("Updated Period Config.");
    } else {
        io->println("Incorrect number of values supplied.");
    }
}
void EngineConfig::setOilPressureConfig(const char * data){ 
    float fields[2];
    int nfields = loadFloatTable(data, 2, &fields[0]);
    if ( nfields == 2) {
        config->oilPressureOffset = fields[0];
        config->oilPressureScale = fields[1];
        io->println("Updated Oil Pressure Config.");
    } else {
        io->println("Incorrect number of values supplied.");
    }
}
void EngineConfig::setFuelLevelCconfig(const char * data){
    float fields[4];
    int nfields = loadFloatTable(data, 4, &fields[0]);
    if ( nfields == 4) {
        config->fuelLevelVin = fields[0];
        config->fuelLevelR1 = fields[1];
        config->fuelLevelEmptyR = fields[2];
        config->fuelLevelFullR = fields[3];
        io->println("Updated Fuel Level Config.");
    } else {
        io->println("Incorrect number of values supplied.");
    }
}





int EngineConfig::loadFloatTable(const char * data, int size, float * table) {
    char * pstart = (char *)data;
    char * pend = NULL;
    int n = 0;

    while(n < size && pstart != NULL && *(pstart) != '\0' ) {
        pend = NULL;
        float d = strtof(pstart, &(pend));
        while ( *pend == ',' || *pend == ' ') {
            pend++;
            if ( *pend == '\0' ) {
                break;
            }
        }
        if ( pend != NULL ) {
           table[n] = d;
           n++;
        }
        pstart = pend; 
    }
    return n;
}

int EngineConfig::loadLongTable(const char * data, int size, int16_t * table) {
    char * pstart = (char *)data;
    char * pend = NULL;
    int n = 0;

    while(n < size && pstart != NULL && *(pstart) != '\0' ) {
        pend = NULL;
        int16_t d = strtol(pstart, &(pend), 10);
        while ( *pend == ',' || *pend == ' ') {
            pend++;
            if ( *pend == '\0' ) {
                break;
            }
        }
        if ( pend != NULL ) {
            
           table[n] = d;
            n++;
        }
        pstart = pend; 
    }
    return n;
}



int EngineConfig::match(const char * command, const char ** commands, int ncommands, const char ** startData) {
    
    for( int i = 0; i < ncommands; i++) {
        int len = strlen(commands[i]);
        if ( strncmp(command, commands[i], len) == 0 ) {
            *startData = &command[len];
           return i;
        }
    }
    return -1;
}





char * EngineConfig::readLine() {
  // Read lines from serial into buffer for processing.
  // 
  while(io->available() > 0) {
    char b = io->read();

    if ( b == '\n') {
      inputLine[bufferPos] = '\0';
      if (bufferPos > 0 && inputLine[bufferPos-1] == '\r') {
          inputLine[bufferPos-1] = '\0';
      }
      bufferPos = 0;
      unitout("Got line "); unitout_ln(inputLine);
      return inputLine;
    } else if ( bufferPos < READBUFFER_LEN-1 ) {
      inputLine[bufferPos] = b;
      bufferPos++;
    } else {
      unitout("Too Long:"); unitout_ln(bufferPos);
      io->println("Error, input too long");
      bufferPos = 0;
      return NULL;
    }
  }
  unitout_ln("No more chars, end line not found");
  return NULL;
}

