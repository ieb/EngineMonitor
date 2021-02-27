
#include "enginemonitor.h"
#include "configstorage.h"
#include "esp_attr.h"


#define EDGE_PIN_0 GPIO_NUM_27
#define EDGE_PIN_1 GPIO_NUM_26


#define RPM_COUNTER_CH 0
#define COOLANT_TEMPERATURE_ADC 0
#define ALTERNATOR_VOLTAGE_ADC 1
#define OIL_PRESSURE_ADC 2
#define FUEL_LEVEL_ADC 3


#define ADC_V_GAIN_FOUR 0.00003125 // 1.024
#define ADC_V_GAIN_TWO  0.0000625  // 2.048
#define ADC_V_GAIN_ONE  0.000125  // 4.096
#define VOLTAGE_SCALE  5.545454545 // (10+2.2)/(2.2)
#define COOLANT_TEMPERATURE_R3 10000
#define COOLANT_TEMPERATURE_R4 2200

volatile DRAM_ATTR unsigned long edgeCount0 = 0;
volatile DRAM_ATTR unsigned long edgeCount1 = 0;
bool interuptsActive = false;

void IRAM_ATTR edgeCountHandler0() {
  edgeCount0++;
}

void IRAM_ATTR edgeCountHandler1() {
  edgeCount1++;
}


unsigned long readEdges(uint8_t chno) {
  if (chno == 0) {
      return edgeCount0;
  } else {
    return edgeCount1;
  }
}




EngineMonitorConfig defaultEngineMonitorConfig {
    .alternatorTemperatureIDX = 0,
    .exhaustTemperatureIDX = 1,
    .engineRoomTemperatureIDX = 2,
    .flywheelRPMReadPeriod = 2000,
    .engineTemperatureReadPeriod = 5000,
    .voltageReadPeriod = 10000,
    .temperatureReadPeriod = 30000,
    .oilPressureScale = 50,  // 0.5V = 0PSI, 4.5V = 200, scale=200/(4.5-0.5)
    .oilPressureOffset = 0.5,
    .fuelLevelScale = 17.182130584, // see method readFuleLevel
    .fuelLevelOffset =  0.18, 
    .engineFlywheelRPMPerHz = 6.224463028,
    .coolantTempR1 = 545.5,
    .coolantTempVin = 5.0,
    //               0,    10,   20   30   40   50   60   70  80  90 100  110  120
    .coolantTempR2 = {1743, 1076, 677, 439, 291, 197, 134, 97, 70, 51,38  ,29, 22 },
    .rfDevices = {0,0,0,0,0,0,0,0,0,0}
};

/**
 * 
 * 
 * 
 * 
 */


EngineMonitor::EngineMonitor(OneWire * _onewire, Stream * _debug) {
    debugStream = _debug;
    tempSensors.setOneWire(_onewire); 
    calibrate(&defaultEngineMonitorConfig);
}

void EngineMonitor::begin() {
  loadEngineHours();
  engineRunning = false;
  if ( !interuptsActive) {
    pinMode(EDGE_PIN_0, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EDGE_PIN_0), edgeCountHandler0, RISING);
    pinMode(EDGE_PIN_1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EDGE_PIN_1), edgeCountHandler1, RISING);
    interuptsActive = true;
  }
  adc.begin();
  tempSensors.begin();
  for(int i = 0; i < MAX_ONE_WIRE_SENSORS; i++) {
    temperature[i] = 0;
    if (tempSensors.getAddress(tempDevices[i], i)) {
      maxActiveDevice = i+1;
    } 
  }
}


  /*
  Circuit is 
    5V |
       R1
       |______ Vt
       |     |
       |     R3
       |     |___ ADC1
       R2    |
       |     R4
   GND |_____|

   R1 = ~677R
   R2 = temperature sensor 1734R@0C to 22R@120C
   R3 = 2K2
   R4 = 4K7
   Max ADC = 3.1v when R2 is open circuit or 3.4v if R1 shorts to 5V.
  R3 and R4 will introduce an error in the default engine sensor reading, but its < 1R at operating 
  temperatures.

  Read the voltage.
  Calculate the resistance of R2
  Lookup 

  Vt = ADC1 * (R3+R4)/R4
  Rn = Vt*R1/(Vin-Vt)
  R2 = 1/(1/Rn - 1/(R3+R4))

  lookup temp using R2

   Voltages read by the ADC range from 0.03v for 393K to 120 to 0.657V at 0C
   20C is 0.47V, 80C is 0.098V with 10K and 2K2 bridge.

   120C = 0.03V
   80C = 0.98V
   20C = 0.47V
   0C = 0.657V

   the ADS1115 is 16 bit for the full scale however single ended only half the scale is available.
   Gains available are, however 000 and 001 are limited to 3.6V by the supply voltage.
   000 : FSR = ±6.144 V(1)
   001 : FSR = ±4.096 V(1)
   010 : FSR = ±2.048 V (default)
   011 : FSR = ±1.024 V
   100 : FSR = ±0.512 V
   101 : FSR = ±0.256 V
   110 : FSR = ±0.256 V
   111 : FSR = ±0.256 V


   011 gives a FSD of 1.024V at 15 bit resolution, this is probably the way to go. 
   That gives at worst a sensitivity of 0.1C at 120C and at 80C its 0.03C at the ADC
   Since the resistors temperature sensitive and the thermistor is not this accurate the ADC is insignificant.
   The track length between the ADC and resistor is short protected from noise by a 50nF capacitor.

Alternatives.
   There is a concern that these voltages are 2 low and will be more subject to noise.
   Could change the bridge to 10K/10K which will reduce impact of bridge and 
   raise the voltages.
   With 10K/10K 
   120C = 0.093V
   80C = 0.273V
   20C = 1.339V
   0C = 1.8468V

   Alternatively the ADC has diode protection provided the input current is below 10mA
   With 10K input resistors the maximum input voltage is 100V before the ADC is damaged
   The Lowest temperature that the thermistor will read is about 5C 
   So removing R4 is an option.


Thermistor temperature curve from Volvo Penta workshop manual is
C   K   R2
120	393	22
110	383	29
100	373	38
90	363	51
80	353	70
70	343	97
60	333	134
50	323	197
40	313	291
30	303	439
20	293	677
10	283	1076
0	273	1743


*/


void EngineMonitor::readCoolant() {
  // coolantPressure, needs sensor
  // engineCoolantTemperature
  adc.setGain(GAIN_FOUR);
  float adc1 = ADC_V_GAIN_FOUR*adc.readADC_SingleEnded(COOLANT_TEMPERATURE_ADC); // using ADC0 for 
  float vt = adc1*(COOLANT_TEMPERATURE_R3+COOLANT_TEMPERATURE_R4)/COOLANT_TEMPERATURE_R4;
  float rn = vt*config->coolantTempR1/(config->coolantTempVin-vt);
  float r2 = 1.0/((1/rn)-(1/(COOLANT_TEMPERATURE_R3+COOLANT_TEMPERATURE_R4)));
  if ( r2 > config->coolantTempR2[0] ) {
    // below 0, assume straight line extending below 0-10C
    engineCoolantTemperature = 10.0*((config->coolantTempR2[0]-r2)/
                 (config->coolantTempR2[0]-config->coolantTempR2[1]));
  } else {
    for (int i = 1; i < MAX_ENGINE_TEMP; i++) {
      if ( r2 > config->coolantTempR2[i] ) {
        // linear interpolate 10C steps, resistance is falling
        engineCoolantTemperature = (10.0*(i-1))+
           10.0*((config->coolantTempR2[i-1]-r2)/
                 (config->coolantTempR2[i-1]-config->coolantTempR2[i]));
        return;
      }
    }
    // above 120C assume straight line extending from 110-120C
    engineCoolantTemperature = (120.0)+
           10.0*((config->coolantTempR2[MAX_ENGINE_TEMP-1]-r2)/
                 (config->coolantTempR2[MAX_ENGINE_TEMP-2]-config->coolantTempR2[MAX_ENGINE_TEMP-1]));
  }
}


/**
 * The RPM of the engine is sensed fromthe W+ terminal of the alternator where edges are counted.
 * The frequency of the W+ terminal ranges from 80Hz to 1KHz (6K rpm flywheel)
 * Reading once every 2s results in an accuracy of 1/160 at worst which is perfectly good enougn
 *
 * The Flywheel RPM to W+ Hz conversion was measured at 0.1606564286 Hz/Flywheel RPM,
 * hence the the conversion factor to convert +W Hz to Flywheel Hz is 0.1037410505
 * And the RPM per W+ HZ is 6.224463028, this one is probably the easiest to understand.
 * 
 */ 

void EngineMonitor::readFlywheelRPM() {
  // engineHours (0 RPM == engine off)
  // load, when drive engaged, relative to RPM
  // torque, when drive engated, relative to RPM.
  // flyWheelRPM
  // RPM maintains a count of the number of pulses since last read.
  unsigned long readTime = millis();
  unsigned long edges = readEdges(RPM_COUNTER_CH);
  // for the edges rto wrap would require the engine running at 6K RPM for 2.3y
  unsigned long nedges = edges - lastEdges;
  // the time will only wrap every 2.3y from device start.
  unsigned long period = readTime - lastFlywheelRPMReadTime;
  lastEdges = edges;
  lastFlywheelRPMReadTime = readTime;
  // period is in ms
  float edgeFrequencyHz = (nedges)/(0.001*period);
  flyWheelRPM = config->engineFlywheelRPMPerHz * edgeFrequencyHz;

  load = 0; // not available.
  torque = 0; // not available.

  if ( !engineRunning && flyWheelRPM > 100) {
    loadEngineHours();
    engineStarted = millis();
    engineRunning = true;
  } else if ( engineRunning && flyWheelRPM < 150 ) {
    engineHoursPrevious = saveEngineHours();
    engineRunning = false;
  }
  debugStream->printf("RPM  nedges=%lu  period=%lu f=%f rpm=%f \n",nedges, period, edgeFrequencyHz, flyWheelRPM);

}

/**
 *   5V sensor -> 
 *   0.5V - 4.5V <- sensor signal
 *   oilPresureOffset = 0.5
 *   oilPressureScale = 200/(4.5-0.5) = 50
 */

void EngineMonitor::readOil() {
   // read an oil pressure sensor, most are linear 0.5 = 0PSI 4.5 = 200PSI, 5V supply
  // oilPressure, needs sensor
  adc.setGain(GAIN_ONE);
  oilPressure = config->oilPressureScale*((VOLTAGE_SCALE * (ADC_V_GAIN_ONE * adc.readADC_SingleEnded(OIL_PRESSURE_ADC)))-config->oilPressureOffset);

}

void EngineMonitor::readFuel() {
  //fuelRate, when the drive is engaged, this is relative to RPM, measuring fuel flow accurately is hard without a ECU.
  //fuelPressure, same as engine room pressure before injection
  // could measure post pump, pre-injection pressures.
  fuelRate = -1e9;
  fuelPressure = -1e9;
}

/**
 *   Circuit is 
    12V| from Ignition.
       R1  190R  (max 54mA, min 3mA)
       |______ Vt (min 0.18v(full) max 6v)
       |     |
       |     R3 (10K)
       |     |___ ADC (min 0.032v max 1.081)
 3-190  R2     |
       |     R4 (2K2)
   GND |_____|

   Level changes liearly with level.
   fuelLevelOffset = 0.18v
   fuelLevelScale = 100/(6-0.18) = 17.182130584
  */

void EngineMonitor::readFuelTank() {
    // fuelTankLevel, reads from a sensor, typically 3-190R, 3 being full 190 being empty, linear
  // would need R bridge, this doesnt need to be read actively
  adc.setGain(GAIN_TWO);
  oilPressure = config->fuelLevelScale*((VOLTAGE_SCALE * (ADC_V_GAIN_TWO * adc.readADC_SingleEnded(FUEL_LEVEL_ADC)))-config->fuelLevelOffset);
}


void EngineMonitor::readSensors() {
  // rpm
  unsigned long readTime = millis();
  if ( readTime > lastFlywheelRPMReadTime+config->flywheelRPMReadPeriod ) {
    debugStream->printf("Reading RPM  %ld  %lu \n",lastFlywheelRPMReadTime+config->flywheelRPMReadPeriod, readTime );

    readFlywheelRPM();
  }

  // coolant temperature
  if ( readTime >  lastEngineTemperatureReadTime+config->engineTemperatureReadPeriod ) {
    debugStream->printf("Reading Temperature  %ld  %lu\n",lastEngineTemperatureReadTime+config->engineTemperatureReadPeriod, readTime );

    readCoolant();
    readOil();
    readFuel();
    saveEngineHours();
    lastEngineTemperatureReadTime = readTime;
  }

  if ( readTime > lastVoltageReadTime+config->voltageReadPeriod  ) {
    debugStream->printf("Reading Voltage  %ld  %lu\n",lastVoltageReadTime+config->voltageReadPeriod, readTime );

    adc.setGain(GAIN_ONE);
    alternatorVoltage = VOLTAGE_SCALE * (ADC_V_GAIN_ONE * adc.readADC_SingleEnded(ALTERNATOR_VOLTAGE_ADC));
    lastVoltageReadTime = readTime;


  }
  if ( readTime > lastTemperatureReadTime+config->temperatureReadPeriod  ) {
    debugStream->printf("Reading External temps  %ld  %lu\n",lastTemperatureReadTime+config->temperatureReadPeriod, readTime );
    // slow changing values.
    readFuelTank();
    // 1 wire sensors
    for (int i = 0; i < maxActiveDevice; i++) {
      temperature[i] = tempSensors.getTempC(tempDevices[i]);
    }
    lastTemperatureReadTime = readTime;
  }
}


int8_t EngineMonitor::getLoad() {
  return load; 
}
int8_t EngineMonitor::getTorque() {
  return torque; 
}
uint16_t EngineMonitor::getStatus1() {
  return status1;
} /* tN2kEngineDiscreteStatus1 */
uint16_t EngineMonitor::getStatus2() {
  return status2;
} /* tN2kEngineDiscreteStatus2 */
float EngineMonitor::getFuelPressure() {
  return fuelPressure; 
}
float EngineMonitor::getCoolantPressure() {
  return coolantPressure; 
}



void EngineMonitor::loadEngineHours() {
    if (!engineRunning ) {
      // only load engine hours if the engine is not running
      // otherwise keep
      float engineHoursLoad = 0;
      int32_t err = config::readStorage(STORAGE_NAMESPACE, ENGINE_HOURS_KEY, &engineHoursLoad, sizeof(float));
      if ( err == config::ok ) {
        debugStream->printf("Loaded Engine Hours %f \n",engineHoursLoad);
        engineHoursPrevious = engineHoursLoad;
      } else {
        debugStream->print("Load Engine Hours Failed, err:");
        debugStream->println(err);
      }
    }
}
float EngineMonitor::saveEngineHours() {
    // get the engine hours and reset the engine
  float engineHoursSave = getEngineHours();
  // save engine hours.
  if ( engineRunning ) {
    debugStream->printf("Saving Engine Hours %f\n", engineHoursSave);
    int32_t err = config::writeStorage(STORAGE_NAMESPACE, ENGINE_HOURS_KEY, &engineHoursSave, sizeof(float));
    if ( err != config::ok ) {
      debugStream->print("Save Engine Hours Failed, err:");
      debugStream->println(err);
    }
  }
  return engineHoursSave;

}
float EngineMonitor::getEngineHours() {
  if ( engineRunning ) {
    return engineHoursPrevious+((millis()-engineStarted)/3600000);
  } else {
    return engineHoursPrevious;
  }
}
float EngineMonitor::getFuelRate() {
  return fuelRate; 
}
float EngineMonitor::getOilPressure() {
  return oilPressure; 
}
float EngineMonitor::getOilTemperature() {
  return engineCoolantTemperature; // not very accurate, but no sensor available.
}

float EngineMonitor::getAlternatorVoltage() {
  return alternatorVoltage;
}
float EngineMonitor::getFlyWheelRPM() {
  return flyWheelRPM;
}
float EngineMonitor::getCoolantTemperature() {
  return engineCoolantTemperature;
}
float EngineMonitor::getAlternatorTemperature() {
  return temperature[config->alternatorTemperatureIDX];
}
float EngineMonitor::getExhaustTemperature() {
    return temperature[config->exhaustTemperatureIDX];
}
float EngineMonitor::getEngineRoomTemperature() {
  return temperature[config->engineRoomTemperatureIDX];
}






void EngineMonitor::calibrate(EngineMonitorConfig * _config) {
  config = _config;
}



RFSensorMonitor::RFSensorMonitor(Jdy40 *_rf, Stream * _debug) {
  rf = _rf;
  debugStream = _debug;
}

void RFSensorMonitor::calibrate(EngineMonitorConfig * _config) {
  config = _config;
}

void RFSensorMonitor::begin() {

}

void RFSensorMonitor::readSensors() {
  unsigned long now = millis();
  if ( nextEvent < now ) {
    nextEvent = now + defaultPeriod;
    switch(state) {
      case SWITCHING_DEVICEID:
        if (rf->setDeviceID(config->rfDevices[currentDevice])) {
          state = SWITCHING_RFID;
        }
        nextEvent = now + 1000; // wait 1s before reading or retrying
        break;
      case SWITCHING_RFID:
        if (rf->setRFID(config->rfDevices[currentDevice])) {
          state = QUERY_DEVICE;
        }
        nextEvent = now + 1000; // wait 1s before reading or retrying
        break;
      case QUERY_DEVICE:
        if (queryDevice()) {
          state = READING_DEVICE;
        }
        nextEvent = now + 1000; // wait 1s before reading or retrying
        break;
      case READING_DEVICE:
        if (readResponse()) {
          state = SWITCHING_DEVICEID;
          currentDevice++;
          if ( config->rfDevices[currentDevice] == 0 || currentDevice == MAX_RF_DEVICES ) {
            currentDevice = 0;
          }
          nextEvent = now + 10000; // wait 10s to poll the next device
        }
        nextEvent = now + 1000; // wait 1s before retrying
        break;
    }
  }
}


bool RFSensorMonitor::queryDevice() {
  rf->writeLine("send\n");
  return true;
}


int RFSensorMonitor::csvParse(char * inputLine, uint16_t len, char * elements[]) {
  elements[0] = inputLine;
  int n = 1;
  for ( int i = 0; i < len-1 && n < 11; i++ ) {
    if ( inputLine[i] == ',') {
      inputLine[i] ='\0';
      elements[n] = &(inputLine[i+1]);
      n++;
    }
  }
  return n;
}

bool RFSensorMonitor::readResponse() {
  // line is an interna pointer so must be fully processed before it gets overwritten.
  char * line = rf->readLine();
  if ( line != NULL) {
    char * fields[10];
    // format of line is rfid,datatype,fields
    int nfields = csvParse(line, 10, fields);
    uint16_t rfid = strtoul(fields[0],NULL, 16);
    uint16_t datatype = strtoul(fields[0],NULL, 16);
    if ( rfid == config->rfDevices[currentDevice]) {
      // expected device, store the output
      saveResponse(rfid, datatype, fields,nfields);
      return true;
    } else {
      // not expected, re-send query
      queryDevice();
      // come back later to read.
    }
  } else {
    // no line yet, this is ok, come back later.
  }
  return false;
  
}

void RFSensorMonitor::saveResponse(uint16_t rfid, uint16_t datatype, char * fields[], int nfields ) {
    // format of line is rfid,datatype,nfields,fields
    // rfid has been checked and is expected.
    switch(datatype) {
      case DATATYPE_BATTERY_MONITOR:
        // rfid,datatype,fields,
        devices[currentDevice].type = DATATYPE_BATTERY_MONITOR;
        devices[currentDevice].battery.voltage = strtof(fields[2], NULL);
        devices[currentDevice].battery.current = strtof(fields[3], NULL);
        devices[currentDevice].battery.temperature = strtof(fields[4], NULL);
      break;
      case DATATYPE_ENVIRONMENT_MONITOR:
        devices[currentDevice].type = DATATYPE_ENVIRONMENT_MONITOR;
        devices[currentDevice].environment.temperature = strtof(fields[2], NULL);
        devices[currentDevice].environment.pressure = strtof(fields[3], NULL);
        devices[currentDevice].environment.humidity = strtof(fields[4], NULL);
        break;
    }
}

