#include "engineconfig.h"
#include <configstorage.h>
#include <cstdlib>




static const char * engineconfig_commands[] = {
    "help", //0
    "dump", //1
    "save", //2
    "load", //3
    "reset", //4
    "activate", //5
    "mon", //6
    "moff", //7
    "etc ", //8
    "engine temp config ", //9
    "ett ", //10
    "engine temp therm ", //11
    "rpm scale ", //12
    "owc ", //13
    "1 wire ", //14
    "rpc ", //14
    "read period ", //16
    "flc ", //17
    "fuel level ", //18
    "status", //19
    "upd", //20
    "update period ", //21
    "btoff", //22
    "son", //23
    "soff", //24
    "scool ", //25
    "salt ", //26
    "sbat ", //27
    "sfuel ", //28
    "srpm ", //29
    "alarm " //30
};


static const char * alarms[] = {
    "lowAlternatorVoltage ", // 0
    "lowEngineVoltage ", //1
    "maxRPM ", //2
    "highExhaustTemperature ", //3
    "highRoomTemperature ", //4
    "highAlternatorTemperature " //5
};




EngineConfig::EngineConfig(EngineMonitor * _engineMonitor, Stream * _io) {
    io = _io;
    engineMonitor = _engineMonitor;
    config = &(configBlob.config);
    engineMonitor->simulate(&simulation);
}


void EngineConfig::begin() {
    reset();
    load();
    activate();
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
    io->println("mon                                  - Enables sensor monitoring");
    io->println("moff                                 - Disables sensor monitoring");
    io->println("status                               - output current status.");
    io->println("btoff                                - switch bt off, will terminate connection.");    


    io->println("etc|engine temp config <v>,<r>       - set engine coolant top resistor and voltage , floatx2, default 5,545.5");
    io->println("ett|engine temp therm <n>,...<n>     - set engine coolant thermistor resistance values 0C-120C, floatx13, default as per manual");
    io->println("rpm scale <n>                        - set rpm per Hz scale, float, default 6.224463028");
    io->println("owc|1 wire <a>,..                    - set one wire index for service battery, exhaust,  alternator, engine room,  intx4, default 0,1,2,3");
    io->println("rpc|read period  <r>,..              - set read period in ms or rpm, engine, voltage, temp, intx4, default 2000,5000,10000,30000");
    io->println("flc|fuel level  <c>,...,<rf>         - set fuel level capacity, r1, voltage, rempty, rfull, floatx5, defult 60,5,220,0,190");
    io->println("upd|update period <t>,<ms>           - PGN Update periods t = PGN, ms = period in ms");
    io->println("                                     - 127488 = engine rapid update");
    io->println("                                     - 127489 = engine dynamic params");
    io->println("                                     - 130312 = temperature");
    io->println("                                     - 127508 = battery");
    io->println("                                     - 130311 = environment");
    io->println("alarm lowAlternatorVoltage <v>       - Low Alternator Voltage Alarm V");
    io->println("alarm lowEngineVoltage <v>           - Low Engine Voltage Alarm V");
    io->println("alarm maxRPM <v>                     - High RPM Alarm RPM");
    io->println("alarm highExhaustTemperature <v>     - High Exhaust Temperature Alarm C");
    io->println("alarm highRoomTemperature <v>        - High Engine Room Temperature Alarm C");
    io->println("alarm highAlternatorTemperature <v>  - High Alternator Temperature Alarm C");
    io->println("son                                  - enable simulation");
    io->println("soff                                 - disable simulation");
    io->println("scool <v>                            - set simulated coolant voltage");
    io->println("salt <v>                             - set simulated alternator voltage");
    io->println("sbat <v>                             - set simulated battery voltage");
    io->println("sfuel <v>                            - set simulated fuel voltage");
    io->println("srpm <n1>                            - set simulated edge counts");
}



int8_t EngineConfig::docmd(const char * command) {
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
        case CMD_FLC_S:
        case CMD_FLC_L:
            setFuelLevelCconfig(data);
            break;
        case CMD_UPD_S:
        case CMD_UPD_L:
            setUpdatePeriod(data);
            break;
        case CMD_STATUS:
        case CMD_BT_OFF:
            // handled by callback
            break;

        case CMD_SIMULATION_ON:
            simulation.enabled = true;
            break;
        case CMD_SIMULATION_OFF:
            simulation.enabled = false;
            break;
        case CMD_SIMULATION_COOL:
            simulation.adcRaw[COOLANT_TEMPERATURE_ADC] = atof(data);
            break;
        case CMD_SIMULATION_ALT:
            simulation.adcRaw[ALTERNATOR_VOLTAGE_ADC] = atof(data);
            break;
        case CMD_SIMULATION_BAT:
            simulation.adcRaw[SERVICE_BATTERY_VOLTAGE_ADC] = atof(data);
            break;
        case CMD_SIMULATION_FUEL:
            simulation.adcRaw[FUEL_LEVEL_ADC] = atof(data);
            break;
        case CMD_SIMULATION_RPMEDGE:
            simulation.rpmEdges = atof(data);
            break;
        case CMD_ALARM:
            setAlarms(data);
            break;
        default:
            io->print(F("Command Not recognised:"));
            io->println(command);
            break;

    }
    return cid;
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


void EngineConfig::enableMonitoring(bool enable) {
    monitoringOn = enable;
}

bool EngineConfig::isMonitoringEnabled() {
    return monitoringOn;
}



void EngineConfig::process(void cb(int8_t)) {
    char * command = readLine();
    if ( command != NULL ) {
        unitout("Processing"); unitout_ln(inputLine);
        cb(docmd(command));
        io->print("$>");
    }
}



void EngineConfig::dump() { 
    char buffer[80];
    sprintf(buffer,"Service Battery temperature sensor one wire index %d", config->serviceBatteryTemperatureIDX);
    io->println(buffer);
    sprintf(buffer,"Exhaust temperature sensor one wire index %d", config->exhaustTemperatureIDX);
    io->println(buffer);
    sprintf(buffer,"Alternator temperature sensor one wire index %d", config->alternatorTemperatureIDX);
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
    sprintf(buffer,"Fuel level V %f", config->fuelLevelVin);
    io->println(buffer);
    sprintf(buffer,"Fuel level R1 %f", config->fuelLevelR1);
    io->println(buffer);
    sprintf(buffer,"Fuel level R Empty %f", config->fuelLevelEmptyR);
    io->println(buffer);
    sprintf(buffer,"Fuel level R Full %f", config->fuelLevelFullR);
    io->println(buffer);
    sprintf(buffer,"Fuel level Capacity %f l", config->fuelTankCapacity);
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
    sprintf(buffer,"PGN 127488 Rapid Engine Params, update period %d ms", config->rapidEngineUpdatePeriod);
    io->println(buffer);
    sprintf(buffer,"PGN 127489 Dynamic Engine Params, update period %d ms", config->engineUpdatePeriod);
    io->println(buffer);
    sprintf(buffer,"PGN 130312 Temperatures, update period %d ms", config->temperatureUpdatePeriod);
    io->println(buffer);
    sprintf(buffer,"PGN 127508 Battery, update period %d ms", config->voltageUpdatePeriod);
    io->println(buffer);
    sprintf(buffer,"PGN 130311 Environment, update period %d ms", config->environmentUpdatePeriod);
    io->println(buffer);
    sprintf(buffer,"Low Alternator Voltage Alarm level %f v", config->alternatorVoltageAlarm);
    io->println(buffer);
    sprintf(buffer,"Low Engine Voltage Alarm level %f v", config->lowEngineVoltageAlarm);
    io->println(buffer);
    sprintf(buffer,"High Engne RPM Alarm level %f rpm", config->maxRPMAlarm);
    io->println(buffer);
    sprintf(buffer,"High Exhaust Temperature Alarm level %f C", config->exhaustTemperatureAlarm);
    io->println(buffer);
    sprintf(buffer,"High Engine Room Temperature Alarm level %f C", config->engineRoomTemperatureAlarm);
    io->println(buffer);
    sprintf(buffer,"High Alternator Temperature Alarm level %f C", config->alternatorTemperatureAlarm);
    io->println(buffer);
}


void EngineConfig::setUpdatePeriod(const char * data) {
    unsigned long fields[2];
    int nfields = loadUnsignedLongTable(data, 2, &fields[0]);
    if ( nfields == 2) {
        switch(fields[0]) {
            case 127488:
                config->rapidEngineUpdatePeriod = fields[1];
                break;
            case 127489:
                config->engineUpdatePeriod = fields[1];
                break;
            case 130312:
                config->temperatureUpdatePeriod = fields[1];
                break;
            case 127508:
                config->voltageUpdatePeriod = fields[1];
                break;
            case 130311:
                config->environmentUpdatePeriod = fields[1];
                break;
            default:
                io->println("PGN Not recognised");
                break;
        }
    } else {
        io->println("Incorrect number of values supplied.");
    }
}






void EngineConfig::setAlarms(const char * data) {
    const char *adata;
    int aid = match(data, alarms, NALARMS, &adata );
    switch(aid) {
        case ALARM_LOW_ALTERNATOR_V:
            config->alternatorVoltageAlarm = atof(data);
            break;
        case ALARM_LOW_ENGINE_V:
            config->lowEngineVoltageAlarm = atof(data);
            break;
        case ALARM_MAX_RPM:
            config->maxRPMAlarm = atof(data);
            break;
        case ALARM_HIGH_EXHAUST_T:
            config->exhaustTemperatureAlarm = atof(data);
            break;
        case ALARM_HIGH_ROOM_T:
            config->engineRoomTemperatureAlarm = atof(data);
            break;
        case ALARM_HIGH_ALTERNATOR_T:
            config->alternatorTemperatureAlarm = atof(data);
            break;
        default:
            io->println("Alarm name not recognised");
            return;

    }
    io->println("Alarm set");
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
    int16_t fields[4];
    int nfields = loadLongTable(data, 4, &fields[0]);
    if ( nfields == 4) {
        config->serviceBatteryTemperatureIDX = (int8_t)fields[0];
        config->exhaustTemperatureIDX = (int8_t)fields[1];
        config->alternatorTemperatureIDX = (int8_t)fields[2];
        config->engineRoomTemperatureIDX = (int8_t)fields[3];
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
void EngineConfig::setFuelLevelCconfig(const char * data){
    float fields[5];
    int nfields = loadFloatTable(data, 5, &fields[0]);
    if ( nfields == 5) {
        config->fuelTankCapacity = fields[0];
        config->fuelLevelVin = fields[1];
        config->fuelLevelR1 = fields[2];
        config->fuelLevelEmptyR = fields[3];
        config->fuelLevelFullR = fields[4];
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

int EngineConfig::loadUnsignedLongTable(const char * data, int size, unsigned long * table) {
    char * pstart = (char *)data;
    char * pend = NULL;
    int n = 0;

    while(n < size && pstart != NULL && *(pstart) != '\0' ) {
        pend = NULL;
        unsigned long d = strtoul(pstart, &(pend), 10);
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

