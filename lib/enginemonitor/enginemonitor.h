
#ifndef ENGINEMONITOR_H
#define ENGINEMONITOR_H

#include <Arduino.h>


#ifdef UNIT_TEST
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

#else
#include <Adafruit_ADS1015.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define unitout_ln(x)
#define unitout(x) 
#endif

#define MAX_ENGINE_TEMP 13
#define MAX_ONE_WIRE_SENSORS 4

struct EngineMonitorConfig { // 3+4*2+8*4+13*4=95 bytes config.
    int8_t alternatorTemperatureIDX; // index fo the alternator 1Wire sensor
    int8_t exhaustTemperatureIDX; // etc
    int8_t engineRoomTemperatureIDX;

    int16_t flywheelRPMReadPeriod;
    int16_t engineTemperatureReadPeriod;
    int16_t voltageReadPeriod;
    int16_t temperatureReadPeriod;
    float oilPressureScale;
    float oilPressureOffset;
    float fuelLevelScale;
    float fuelLevelOffset;
    float engineFlywheelRPMPerHz;
    float coolantTempR1; // value of R1 in engine temp coolant bridge
    float coolantTempVin; // Open circuit voltage
    float coolantTempR2[MAX_ENGINE_TEMP]; // 0 - 120C in 10C steps values for the resistnace of the temperature sensor
};

extern EngineMonitorConfig defaultEngineMonitorConfig;

class EngineMonitor {
    public:
      EngineMonitor(OneWire * _oneWire, Stream * _debug = &Serial);
      void calibrate(EngineMonitorConfig * config);
      void readSensors();
      void begin();

      int8_t getLoad(); 
      int8_t getTorque();
      uint16_t getStatus1(); /* tN2kEngineDiscreteStatus1 */
      uint16_t getStatus2(); /* tN2kEngineDiscreteStatus2 */
      float getFuelPressure();
      float getCoolantPressure();
      void setStoredEngineHours(float storedEngineHours);
      float getEngineHours();
      float getFuelRate();
      float getOilPressure();
      float getOilTemperature();
      float getAlternatorVoltage(); 
      float getFlyWheelRPM(); 
      float getCoolantTemperature(); 
      float getAlternatorTemperature();
      float getExhaustTemperature();
      float getEngineRoomTemperature();
    private:
      void readCoolant();
      void readOil();
      void readFuel();
      void readFuelTank();
      void readFlywheelRPM();
      int maxActiveDevice = 0;
      unsigned long lastEdges = 0;
      unsigned long lastFlywheelRPMReadTime = 0;
      unsigned long lastEngineTemperatureReadTime = 0;
      unsigned long lastVoltageReadTime = 0;
      unsigned long lastTemperatureReadTime = 0;
      bool engineRunning = false;
      unsigned long engineStarted = 0;
      float engineHoursPrevious = 0.0;
      uint16_t status1 = 0;
      uint16_t status2 = 0;
      int8_t load = 0;
      int8_t torque = 0;
      float alternatorVoltage = 0.0;
      float engineCoolantTemperature = 0.0;
      float flyWheelRPM = 0.0;
      float fuelPressure = -1e9; // N2K NA
      float coolantPressure = -1e9; // N2K NA
      float fuelRate = -1e9; // N2K NA
      float oilPressure = -1e9; // N2K NA
      float temperature[MAX_ONE_WIRE_SENSORS];
      DeviceAddress tempDevices[MAX_ONE_WIRE_SENSORS];
      Adafruit_ADS1115 adc;
      DallasTemperature tempSensors;
      EngineMonitorConfig *config;
      Stream * debugStream;
};



#endif