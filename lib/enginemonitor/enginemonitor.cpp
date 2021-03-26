
#include "enginemonitor.h"
#include "configstorage.h"



#ifndef UNIT_TEST
#include "esp_attr.h"
#define debugf(args...) if ( debugOutput ) debugStream->printf(args)
#else
#define DRAM_ATTR
#define IRAM_ATTR
#define debugf(args...)
 
#endif


#define EDGE_PIN_0 GPIO_NUM_27
#define EDGE_PIN_1 GPIO_NUM_26




#define ADC_V_GAIN_EIGHT 0.000015625 // 0.512
#define ADC_V_GAIN_FOUR 0.00003125 // 1.024
#define ADC_V_GAIN_TWO  0.0000625  // 2.048
#define ADC_V_GAIN_ONE  0.000125  // 4.096
#define ALTERNATOR_VOLTAGE_SCALE  5.5272  // (9960+2200)/(2200)
#define SERVICE_BATTERY_VOLTAGE_SCALE  5.5479 // (9960+2190)/(2190)
// measured values on board
#define COOLANT_TEMPERATURE_R3 9940
#define COOLANT_TEMPERATURE_R4 2200


// 
// Powered from the MDI, which supplies it with 5V
// Will need to work out the resistance inside the MDI to 5V
// measured values onboard
#define FUEL_LEVEL_V1 5
#define FUEL_LEVEL_R1 2200
#define FUEL_LEVEL_R3 9970
#define FUEL_LEVEL_R4 10000000  // the ADC internal resistance.

volatile DRAM_ATTR unsigned long edgeCount0 = 0;
volatile DRAM_ATTR unsigned long edgeCount1 = 0;
bool interuptsActive = false;

void IRAM_ATTR edgeCountHandler0() {
  edgeCount0++;
}

void IRAM_ATTR edgeCountHandler1() {
  edgeCount1++;
}



#define RapidEngineUpdatePeriod 100
#define EngineUpdatePeriod 1000
#define TemperatureUpdatePeriod 10000
#define VoltageUpdatePeriod 5000
#define EnvironmentUpdatePeriod 60000

#define readADC(s,c) (simulation->enabled?simulation->adcRaw[c]:s*adc.readADC_SingleEnded(c))
#define readEdges(c) (c?edgeCount1:edgeCount0)



EngineMonitorConfig defaultEngineMonitorConfig {
    .alternatorTemperatureIDX = 0,
    .exhaustTemperatureIDX = 1,
    .engineRoomTemperatureIDX = 2,
    .serviceBatteryTemperatureIDX = 3,
    .flywheelRPMReadPeriod = 100,
    .engineTemperatureReadPeriod = 1000,
    .voltageReadPeriod = 1000,
    .temperatureReadPeriod = 10000,
    .rapidEngineUpdatePeriod = 100,
    .engineUpdatePeriod = 1000,
    .temperatureUpdatePeriod = 5000,
    .voltageUpdatePeriod = 5000,
    .environmentUpdatePeriod = 10000,
    .fuelLevelVin = 5.0,
    .fuelLevelR1 = 2200.0,
    .fuelLevelEmptyR = 0,
    .fuelLevelFullR = 190,
    .fuelTankCapacity = 60,
    .engineFlywheelRPMPerHz = 6.224463028,
    .coolantTempR1 = 545.5,
    .coolantTempVin = 5.0,

    // default alarm levels.
    .alternatorVoltageAlarm = 13.0,
    .lowEngineVoltageAlarm = 10.0,
    .maxRPMAlarm = 4500.0,
    .exhaustTemperatureAlarm = 60.0,
    .engineRoomTemperatureAlarm = 60.0,
    .alternatorTemperatureAlarm = 80.0,

    //               0,    10,   20   30   40   50   60   70  80  90 100  110  120
    .coolantTempR2 = {1743, 1076, 677, 439, 291, 197, 134, 97, 70, 51,38  ,29, 22 },
    .rfDevices = {0,0,0,0,0,0,0,0,0,0}
};


/**
 * 
 *     Vin |
       R1
       |______ Vt
       |     |
       |     R3
       |     |___ v
       R2    |
       |     R4
   GND |_____|

 * 
 * 
 */

float EngineMonitor::getBridgeSensorResistance(float v, float vin, float r1, float r3, float r4) {
    float vt = v*(r3+r4)/r4;
    float rn = vt*r1/(vin-vt);
    float r2 = 1.0/((1/rn)-(1/(r3+r4)));
    debugf("Reistor Bridge  vt=%f  rn=%f r2=%f  vin=%f r1=%f r3=%f r4=%f\n",vt, rn, r2, vin, r1, r3, r4);
    return r2;

}



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
  tempSensors.setWaitForConversion(false);
  for(int i = 0; i < MAX_ONE_WIRE_SENSORS; i++) {
    temperature[i] = 0;
    if (tempSensors.getAddress(tempDevices[i], i)) {
      debugStream->print("Got Temperature Sensor ");
      debugStream->print(i);
      debugStream->print(" ");
      for (int j = 0; j < 8; j++) {
        debugStream->print(tempDevices[i][j],HEX);
      }
      debugStream->println(" ");
      maxActiveDevice = i+1;
    } else {
      debugStream->print("No Temperature sensor ");
      debugStream->println(i);
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
   80C = 0.098V
   20C = 0.47V
   0C = 0.657V

   the ADS1115 is 16 bit for the full scale however single ended only half the scale is available.
   Gains available are, however 000 and 001 are limited to 3.6V by the supply voltage.
   000 : FSR = ±6.144 V(1)  
   001 : FSR = ±4.096 V(1)        GAIN_ONE
   010 : FSR = ±2.048 V (default) GAIN_TWO
   011 : FSR = ±1.024 V  GAIN_FOUR
   100 : FSR = ±0.512 V  GAIN_EIGHT
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
  coolantVoltage = readADC(ADC_V_GAIN_FOUR, COOLANT_TEMPERATURE_ADC);
  float r2 = getBridgeSensorResistance(coolantVoltage, config->coolantTempVin, config->coolantTempR1, COOLANT_TEMPERATURE_R3, COOLANT_TEMPERATURE_R4);
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
        debugf("Coolant  v=%f  r2=%f t=%f \n",coolantVoltage, r2,engineCoolantTemperature);
        return;
      }
    }
    // above 120C assume straight line extending from 110-120C
    engineCoolantTemperature = (120.0)+
           10.0*((config->coolantTempR2[MAX_ENGINE_TEMP-1]-r2)/
                 (config->coolantTempR2[MAX_ENGINE_TEMP-2]-config->coolantTempR2[MAX_ENGINE_TEMP-1]));
    if (engineCoolantTemperature > 150 ) {
      engineCoolantTemperature = 150;
    }
    if (engineCoolantTemperature < -50 ) {
      engineCoolantTemperature = -50;
    }
  }
  debugf("Coolant  v=%f  r2=%f t=%f \n",coolantVoltage, r2,engineCoolantTemperature);
}


/**
 * Pulse RPM/Hz is 6.224463028 based on measurements and means squared curve fitting. 
 * Measurements
 * RPM	Hz
 * 0	
 * 750	137
 * 1000	162
 * 1500	238
 * 2000	325
 * 2500	394
 * 3000	
 * 3500	
 * 4000	
 * 
 * Updates need to flow at 0.1s, hence to maintain accuracy there needs to be a buffer of 20
 * readings from which the RPM can be measured.
 */ 

void EngineMonitor::readFlywheelRPM() {
  // flyWheelRPM
  // this method is called frequently, so dont do too much in here.
  // store the current reading
  // since there is no guarentee that the method will be called precisely at the time
  // expected the time must also be stored.
  lastFlywheelRPMReadTime = millis();
  rpmReadTime[rpmBufferpos] = lastFlywheelRPMReadTime;
  rpmEdges[rpmBufferpos] = readEdges(RPM_COUNTER_CH);
  int8_t pnow = rpmBufferpos;
  // pos now points to the next reading which is the oldest reading.
  rpmBufferpos=(rpmBufferpos+1)%rpmSamples;
  // calculate the difference from the start of the circular buffer to the end of the buffer.
  // for the edges rto wrap would require the engine running at 6K RPM for 2.3y
  // the time will only wrap every 2.3y from device start
  unsigned long nedges = rpmEdges[pnow] - rpmEdges[rpmBufferpos];
  unsigned long period = rpmReadTime[pnow] - rpmReadTime[rpmBufferpos];
  if ( simulation->enabled ) {
    nedges = simulation->rpmEdges;
    period = config->flywheelRPMReadPeriod;
  }

  // period is in ms
  float edgeFrequencyHz = (nedges)/(0.001*period);
  flyWheelRPM = config->engineFlywheelRPMPerHz * edgeFrequencyHz;
  debugf("RPM  nedges=%lu  period=%lu f=%f rpm=%f \n",nedges, period, edgeFrequencyHz, flyWheelRPM);
}

void EngineMonitor::updateEngineStatus() {

  // if the alternator voltage or the coolantVoltage 
  // has a non zero voltage the engine is powered up.
  // need to check that the alternatorVoltage is not on all the time since
  // that is connected via the A2B charger.
  if ( coolantVoltage > 0.1 || alternatorVoltage > 5.0 ) {
    debugf("Engine on coolantV=%f > 0.1 || alternatorV=%f > 5.0 \n",coolantVoltage, alternatorVoltage);
    engineOn = true;
  } else {
    debugf("Engine off coolantV=%f < 0.1 && alternatorV=%f < 5.0 \n",coolantVoltage, alternatorVoltage);
    engineOn = false;
  }

  // if the rpm is non zero the engine is running.
  if ( !engineRunning && flyWheelRPM > 100) {
    loadEngineHours();
    engineStarted = millis();
    engineRunning = true;
  } else if ( engineRunning && flyWheelRPM < 150 ) {
    engineHoursPrevious = saveEngineHours();
    engineRunning = false;
  }
  // alarms, p113 of the LightHouse II manual has a list of Raymarine supported alarms.
  // Unsupported alarms can be added but need to be configured.
  //
  // Over Temperature   flagOverTemp
  // Low system voltage  flagLowSystemVoltage
  // Water  ow (Exhaust temperature) flagWaterFlow
  // Not charging (low alternator voltage) flagChargeIndicator
  // Rev limit exceeded flagRevLimitExceeded
  // Others  flagWarning1 flagWarning2

  status1.Bits.OverTemperature=false;
  status1.Bits.LowSystemVoltage=false;
  status1.Bits.WaterFlow=false;
  status1.Bits.ChargeIndicator=false;
  status1.Bits.RevLimitExceeded=false;
  status2.Bits.WarningLevel1=false;
  status2.Bits.WarningLevel2=false;

  if ( engineOn ) {
    if ( coolantVoltage > 0.8 ) {
       status2.Bits.WarningLevel1 = true;
      status1.Bits.CheckEngine=true;
    } else if ( coolantVoltage < 0.01 ) {
       status2.Bits.WarningLevel1 = true;
      status1.Bits.CheckEngine=true;
      // coolant sensor short circuit
    } 
    if ( engineRunning && alternatorVoltage < config->alternatorVoltageAlarm ) {
      status1.Bits.ChargeIndicator = true;
    }
    if ( alternatorVoltage < config->lowEngineVoltageAlarm ) {
      status1.Bits.LowSystemVoltage = true;
    }
    if ( flyWheelRPM > config->maxRPMAlarm ) {
      status1.Bits.RevLimitExceeded=true;
    }
  }
  if ( getExhaustTemperature() > config->exhaustTemperatureAlarm ) {
    status1.Bits.WaterFlow=true;
    status1.Bits.CheckEngine=true;
  }
  if ( getEngineRoomTemperature() > config->engineRoomTemperatureAlarm ) {
    status1.Bits.CheckEngine=true;
  }
  if ( getAlternatorTemperature() > config->alternatorTemperatureAlarm ) {
    status2.Bits.WarningLevel1 = true;
    status1.Bits.ChargeIndicator = true;
  }
  if ( fuelTankLevel < 10 ) { // less than 10% fuel left.
    status2.Bits.WarningLevel2 = true;
  }

}


tN2kEngineDiscreteStatus1 EngineMonitor::getEngineStatus1() {
  return status1;
}
tN2kEngineDiscreteStatus2 EngineMonitor::getEngineStatus2() {
  return status2;
}


/**
 *   
 * The curcuit used by the MDI some form  of bridge with the voltage of both terminals of the fuel sensor at about 2.5v
 * and the difference between them 0.25v, this cant be easilly measured from ground with 1 ADC so an alternative circuit
 * is needed.
 * 
 * Option 1.
 * Use the Alternator Voltage to power the Fuel Level sensor. This is only on when the engine is turned on and is 
 * measured so we know what it is when we measure the Fuel level. It does vary from about 11-15v.
 * 
 * 
 *   Circuit is 
    15V |
        R1
        |______ Vt
        |     |
        |     R3
        |     |___ ADC1
        R2    |
        |     R4
   GND  |_____|

 * R1 1000
 * R2 0-190
 * R3 10K
 * R4 2K2
 * Vt 15*190/(1000+190) = 2.39v full, 0v empty at 11v  11*190/(1000+190)=1.756
 * ADC1 (15*190/(1000+190))*2.2/12.2, 0.43 full, 0v empty (11*190/(1000+190))*2.2/12.2 = 0.316
 * I1 = 15/1000 = 15mA P1 = 0.015*0.015*1000 = 225mW
 * 15/(1000+190) = 12.6mA, P1 0.0126*0.0126*1000 = 158mW 0.0126*0.0126*190=30mW
 * 
 * Vt 15*190/(680+190) = 3.27v full, 0v empty at 11v  11*190/(680+190)=2.40
 * ADC1 (15*190/(680+190))*2.2/12.2, 0.59 full, 0v empty (11*190/(680+190))*2.2/12.2 = 0.433
 * I1 = 15/680 = 22mA P1 = 0.022*0.022*680 = 329mW
 * 15/(680+190) = 17.2mA, P1 0.0172*0.0172*680 = 201mW 0.0172*0.0172*190=56mW
 * 
 * Use 1K as lower voltage going into the tank, and fits better with the ADC ranges.
 * 
 * 
 * Option 2
 * Power from the ESP32 5v supply however this will need some changes.
 * Circuit is 
    3.3V from rail 
       R1  2200  (max 7mA, min 5mA)
       |______ Vt (empty 0v, full 0.95v )
       |     |
       |     R3 (10K)
       |    ` |___ ADC (0v empty, full 0.95v)
 0-190  R2     
       |      No resistor only a capacitor.
   GND |_____

  R2 resistence changes liearly with level.
  
  190 = 0%
  3 = 100%

  R1 = 2K2
  R2 = 190-0
  Vin = 3v
  ADCV = 3.3*190/(2200+190) = 0.26v
  ADCVmin = 0v
  I2 min 3.3/(2200+190) = 1.3mA
  I2 max 3.3/(2200) = 1.5mA


1.5mA is ok for this so going with option 2.
  */




void EngineMonitor::readFuelTank() {
  adc.setGain(GAIN_EIGHT);
  float v = readADC(ADC_V_GAIN_EIGHT, FUEL_LEVEL_ADC);
  adc.setGain(GAIN_ONE); // just in case a read happens without resetting gain.
  float r2 = getBridgeSensorResistance(v, config->fuelLevelVin, config->fuelLevelR1 ,FUEL_LEVEL_R3,FUEL_LEVEL_R4 );
  float rawFuelTankLevel = (r2-config->fuelLevelEmptyR)*100.0/(config->fuelLevelFullR-config->fuelLevelEmptyR);
  fuelTankLevel = rawFuelTankLevel;
  if ( fuelTankLevel < 0.0) {
    fuelTankLevel = 0.0;
  } else if ( fuelTankLevel > 100.0 ) {
    fuelTankLevel = 100.0;
  }
  debugf("Fuel Tank v=%f r2=%f rfl=%f fl=%f \n",v, r2, rawFuelTankLevel, fuelTankLevel);
}


void EngineMonitor::readSensors(bool debug) {
  // rpm
  debugOutput = debug;
  unsigned long readTime = millis();
  if ( readTime > lastFlywheelRPMReadTime+config->flywheelRPMReadPeriod ) {
    debugf("Reading RPM  %ld  %lu \n",lastFlywheelRPMReadTime+config->flywheelRPMReadPeriod, readTime );
    readFlywheelRPM();
  }

  // coolant temperature
  if ( readTime >  lastEngineTemperatureReadTime+config->engineTemperatureReadPeriod ) {
    debugf("Reading Temperature  %ld  %lu\n",lastEngineTemperatureReadTime+config->engineTemperatureReadPeriod, readTime );
    debugf("Will Read Voltage  %ld  \n",lastVoltageReadTime+config->voltageReadPeriod-readTime );
    debugf("Will Reading External temps  %ld \n",lastTemperatureReadTime+config->temperatureReadPeriod-readTime );
    updateEngineStatus();
    readCoolant();
    readFuelTank();
    saveEngineHours();
    lastEngineTemperatureReadTime = readTime;
  }

  if ( readTime > lastVoltageReadTime+config->voltageReadPeriod  ) {
    debugf("Reading Voltage  %ld  %lu\n",lastVoltageReadTime+config->voltageReadPeriod, readTime );
    adc.setGain(GAIN_ONE);
    float v =  readADC(ADC_V_GAIN_ONE, ALTERNATOR_VOLTAGE_ADC);
    alternatorVoltage = ALTERNATOR_VOLTAGE_SCALE * (v);
    debugf("Alternator Voltage v=%f va=%f \n",v, alternatorVoltage);

    v =  readADC(ADC_V_GAIN_ONE, SERVICE_BATTERY_VOLTAGE_ADC);
    serviceBatteryVoltage = SERVICE_BATTERY_VOLTAGE_SCALE * (v);
    debugf("Service Battery Voltage v=%f va=%f \n",v, serviceBatteryVoltage);

    lastVoltageReadTime = readTime;



  }
  
  if ( requestTemperaturesRequired && readTime > lastTemperatureReadTime+config->temperatureReadPeriod-5000) {
    for (int i = 0; i < maxActiveDevice; i++) {
      tempSensors.requestTemperaturesByAddress(tempDevices[i]);
    }   
    requestTemperaturesRequired = false;
  }
  if ( readTime > lastTemperatureReadTime+config->temperatureReadPeriod  ) {
    debugf("Reading External temps  %ld  %lu\n",lastTemperatureReadTime+config->temperatureReadPeriod, readTime );
    // 1 wire sensors
    for (int i = 0; i < maxActiveDevice; i++) {
      temperature[i] = tempSensors.getTempC(tempDevices[i]);
      debugf("Temperature i=%d t=%f \n",i,temperature[i]);
    }
    requestTemperaturesRequired = true;
    lastTemperatureReadTime = readTime;
  }
}



float EngineMonitor::getFuelTankLevel() {
  return fuelTankLevel;
}

float EngineMonitor::getFuelTankCapacity() {
  return config->fuelTankCapacity;
}



void EngineMonitor::loadEngineHours() {
    if (!engineRunning ) {
      // only load engine hours if the engine is not running
      // otherwise keep
      float engineHoursLoad = 0;
      int32_t err = config::readStorage(STORAGE_NAMESPACE, ENGINE_HOURS_KEY, &engineHoursLoad, sizeof(float));
      if ( err == config::ok ) {
        debugf("Loaded Engine Hours %f \n",engineHoursLoad);
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
    debugf("Saving Engine Hours %f\n", engineHoursSave);
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
float EngineMonitor::getServiceBatteryVoltage() {
  return serviceBatteryVoltage;
}

float EngineMonitor::getAlternatorVoltage() {
  if ( engineOn ) {
    return alternatorVoltage;
  } else {
    return 0;
  }
}
float EngineMonitor::getFlyWheelRPM() {
  return flyWheelRPM;
}
float EngineMonitor::getCoolantTemperature() {
  if ( engineOn ) {
    return engineCoolantTemperature;
  } else {
    return 0;
  }
}
float EngineMonitor::getServiceBatteryTemperature() {
  return temperature[config->serviceBatteryTemperatureIDX];
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

bool EngineMonitor::isEngineOn() {
  return engineOn;
}

bool EngineMonitor::isEngineRunning() {
  return engineRunning;
}


unsigned long EngineMonitor::getRapidEngineUpdatePeriod() {
  return config->rapidEngineUpdatePeriod;
}
unsigned long EngineMonitor::getEngineUpdatePeriod() {
  return config->engineUpdatePeriod;
}
unsigned long EngineMonitor::getTemperatureUpdatePeriod() {
  return config->temperatureUpdatePeriod;
}
unsigned long EngineMonitor::getVoltageUpdatePeriod() {
  return config->voltageUpdatePeriod;
}
unsigned long EngineMonitor::getEnvironmentUpdatePeriod() {
  return config->environmentUpdatePeriod;
}





void EngineMonitor::calibrate(EngineMonitorConfig * _config) {
  config = _config;
  rpmSamples = 2000/config->flywheelRPMReadPeriod;
  if ( rpmSamples > MAX_RPM_SAMPLES ) {
    rpmSamples = MAX_RPM_SAMPLES;
  }
}

void EngineMonitor::simulate(SensorSimulation * _simulation) {
  simulation = _simulation;
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

