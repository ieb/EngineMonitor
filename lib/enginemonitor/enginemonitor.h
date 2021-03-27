
#ifndef ENGINEMONITOR_H
#define ENGINEMONITOR_H

#include <Arduino.h>
#include <N2kTypes.h>


#define RPM_COUNTER_CH 0
#define COOLANT_TEMPERATURE_ADC 0
#define ALTERNATOR_VOLTAGE_ADC 1
#define SERVICE_BATTERY_VOLTAGE_ADC 2
#define FUEL_LEVEL_ADC 3


#ifndef UNIT_TEST
#include <Adafruit_ADS1015.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define unitout_ln(x)
#define unitout(x) 


#else
// UNIT Test fakes
#include <iostream> 
#define unitout_ln(x) std::cout << x << std::endl
#define unitout(x) std::cout << x 
#define GPIO_NUM_27 27
#define GPIO_NUM_26 26
#define GAIN_FOUR 4
#define GAIN_ONE 1
#define GAIN_TWO 2

class OneWire {
  
};

class DeviceAddress {

};

class Adafruit_ADS1115 {
  public:
  void begin() {

  }
  void setGain(int gain) {

  }
  int16_t readADC_SingleEnded(int c) {
    return 10;
  }
};

class DallasTemperature {
  public:
  void begin() {

  }
  void setOneWire(OneWire * onewire) {

  }
  bool getAddress(DeviceAddress address, int i) {
    return true;
  }
  float getTempC(DeviceAddress address) {
    return 20;
  }
};



#endif

#define MAX_ENGINE_TEMP 13
#define MAX_ONE_WIRE_SENSORS 4
#define MAX_RF_DEVICES 10
#define MAX_RPM_SAMPLES 20
#define STORAGE_NAMESPACE  "EngineConfig"
#define STORAGE_KEY "cf11" // config version 10, keynames need to be:
#define ENGINE_HOURS_KEY "eh"


struct EngineMonitorConfig { // 4+9*2+8*4+6*4+13*4+10*2=150 bytes config. 
    int8_t alternatorTemperatureIDX; // index fo the alternator 1Wire sensor
    int8_t exhaustTemperatureIDX; // etc
    int8_t engineRoomTemperatureIDX;
    int8_t serviceBatteryTemperatureIDX;
    int16_t flywheelRPMReadPeriod;
    int16_t engineTemperatureReadPeriod;
    int16_t voltageReadPeriod;
    int16_t temperatureReadPeriod;
    uint16_t rapidEngineUpdatePeriod;
    uint16_t engineUpdatePeriod;
    uint16_t temperatureUpdatePeriod;
    uint16_t voltageUpdatePeriod;
    uint16_t environmentUpdatePeriod;
    float fuelLevelVin;
    float fuelLevelR1;
    float fuelLevelEmptyR;
    float fuelLevelFullR;
    float fuelTankCapacity;
    float engineFlywheelRPMPerHz;
    float coolantTempR1; // value of R1 in engine temp coolant bridge
    float coolantTempVin; // Open circuit voltage

    float alternatorVoltageAlarm;
    float lowEngineVoltageAlarm;
    float maxRPMAlarm;
    float exhaustTemperatureAlarm;
    float engineRoomTemperatureAlarm;
    float alternatorTemperatureAlarm;



    float coolantTempR2[MAX_ENGINE_TEMP]; // 0 - 120C in 10C steps values for the resistnace of the temperature sensor
    uint16_t rfDevices[MAX_RF_DEVICES];
};

struct SensorSimulation {
  bool enabled;
  float adcRaw[4];
  unsigned long rpmEdges;
};


extern EngineMonitorConfig defaultEngineMonitorConfig;

class EngineMonitor {
    public:
      EngineMonitor(OneWire * _oneWire, Stream * _debug = &Serial);
      void calibrate(EngineMonitorConfig * config);
      void simulate(SensorSimulation * simulation);
      void readSensors(bool debug);
      void begin();
      bool isEngineOn();
      bool isEngineRunning();

      float getFuelTankLevel();
      float getFuelTankCapacity();
      void setStoredEngineHours(float storedEngineHours);
      float getEngineHours();
      float getAlternatorVoltage();
      float getServiceBatteryVoltage();
      float getFlyWheelRPM(); 
      float getCoolantTemperature(); 
      float getAlternatorTemperature();
      float getServiceBatteryTemperature();
      float getExhaustTemperature();
      float getEngineRoomTemperature();
      unsigned long getRapidEngineUpdatePeriod();
      unsigned long getEngineUpdatePeriod();
      unsigned long getTemperatureUpdatePeriod();
      unsigned long getVoltageUpdatePeriod();
      unsigned long getEnvironmentUpdatePeriod();
      tN2kEngineDiscreteStatus1 getEngineStatus1();
      tN2kEngineDiscreteStatus2 getEngineStatus2();

    private:
      void readCoolant();
      void readFuelTank();
      void readFlywheelRPM();
      float saveEngineHours();
      void loadEngineHours();
      void updateEngineStatus();
      float getBridgeSensorResistance(float v, float vin, float r1, float r3, float r4);
      int maxActiveDevice = 0;
      int8_t rpmSamples = MAX_RPM_SAMPLES;
      int8_t rpmBufferpos = 0;
      unsigned long lastEngineTemperatureReadTime = 0;
      unsigned long lastFlywheelRPMReadTime = 0;
      unsigned long lastVoltageReadTime = 0;
      unsigned long lastTemperatureReadTime = 0;
      bool engineRunning = false;
      bool debugOutput = false;
      bool engineOn = false;
      bool requestTemperaturesRequired = true;
      unsigned long engineStarted = 0;
      float engineHoursPrevious = 0.0;
      float alternatorVoltage = 0.0;
      float serviceBatteryVoltage = 0.0;
      float coolantVoltage = 0.0;
      float engineCoolantTemperature = 0.0;
      float flyWheelRPM = 0.0;
      float fuelTankLevel = -1e9; // N2K NA
      tN2kEngineDiscreteStatus1 status1 = 0;
      tN2kEngineDiscreteStatus2 status2 = 0;
      float temperature[MAX_ONE_WIRE_SENSORS];
      DeviceAddress tempDevices[MAX_ONE_WIRE_SENSORS];
      unsigned long rpmEdges[MAX_RPM_SAMPLES];
      unsigned long rpmReadTime[MAX_RPM_SAMPLES];

      Adafruit_ADS1115 adc;
      DallasTemperature tempSensors;
      EngineMonitorConfig *config;
      Stream * debugStream;
      SensorSimulation * simulation;
};





#endif