#include <Arduino.h>

#include <Adafruit_BMP280.h>


//#define SERIAL_DEBUG_DISABLED
// #define DEBUG_NMEA2000 1

#define ESP32_CAN_TX_PIN GPIO_NUM_19
#define ESP32_CAN_RX_PIN GPIO_NUM_18

#define ONEWIRE_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define SDA_PIN GPIO_NUM_23
#define DATAEN_PIN GPIO_NUM_4 
#define RF_RX  GPIO_NUM_16
#define RF_TX  GPIO_NUM_17
#define BT_INDICATOR_PIN GPIO_NUM_2
#define BT_ENABLE_PIN GPIO_NUM_25 // needs checking this can be used.


#include <NMEA2000_esp32.h>
#include <NMEA2000_CAN.h> 
#include <N2kMessages.h>


#include <enginemonitor.h>
#include <engineconfig.h>

#define BLUETOOTHCLASSIC 1
#ifdef BLUETOOTHCLASSIC
#include <BlueToothSerial.h>
BluetoothSerial SerialBT;
#define SerialIO SerialBT
#else
#define SerialIO Serial
#endif

const unsigned long TransmitMessagesEngine[] PROGMEM={
  127488L, // Rapid engine ideally 0.1s
  127489L, // Dynamic engine 0.5s
  127505L, // Battery status 2.5s
  130316L, // Extended Temperature 2.5s
  0};
const unsigned long TransmitMessagesTemperature[] PROGMEM={
  130312L, // temperature
  0};
const unsigned long TransmitMessagesAtmospheric[] PROGMEM={
  130311L, // environment
  0};
const unsigned long TransmitMessagesBatteries[] PROGMEM={
  127508L, // Battery status 5s
  0};




OneWire oneWire(ONEWIRE_PIN);
EngineMonitor engineMonitor(&oneWire,  &SerialIO);
EngineConfig engineConfig(&engineMonitor, &SerialIO);
Adafruit_BMP280 bmp; // I2C
bool hasBMP280 = true;

#define INPUT_BUFFER_SIZE 1024
char inputBuffer[INPUT_BUFFER_SIZE];


#define NMEA2000_DEV_ENGINE      0
#define NMEA2000_DEV_TEMPERATURE 1
#define NMEA2000_DEV_ATMOSPHERIC 2
#define NMEA2000_DEV_BATTERIES   3


int blueToothRunning = false;

void StopBluetooth() {
#ifdef BLUETOOTHCLASSIC      
  if ( blueToothRunning ) {
    SerialBT.end();
    digitalWrite(BT_INDICATOR_PIN, LOW); 
    blueToothRunning = false;
  }
#endif
}

void CheckBluetooth() {
#ifdef BLUETOOTHCLASSIC      
  if ( !blueToothRunning ) {
    if ( digitalRead(BT_ENABLE_PIN) == LOW ) {
      SerialBT.begin("EngineMonitor"); 
      blueToothRunning = true;
      digitalWrite(BT_INDICATOR_PIN, HIGH); 
    }
  }
#endif
}

void setup() {

  
#ifdef BLUETOOTHCLASSIC  
  pinMode(BT_ENABLE_PIN, INPUT_PULLUP); 
  pinMode(BT_INDICATOR_PIN, OUTPUT);
  digitalWrite(BT_INDICATOR_PIN, LOW); 
  CheckBluetooth();
#else 
#endif
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Starting sensors");
  engineConfig.begin();
  engineConfig.dump();
  engineMonitor.begin();  


  hasBMP280 = true;
  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring. true 0x76 and 0x77"));
      hasBMP280 = false;
    }
  }
  if ( hasBMP280 ) {
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  }



  Serial.println("Starting NMEA2000");

  NMEA2000.SetDeviceCount(4);

  // Setup NMEA2000
  // Set Product information
  NMEA2000.SetProductInformation("00000003", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Engine Monitor",  // Manufacturer's Model ID
                                 "1.1.0.22 (2021-02-10)",  // Manufacturer's Software version code
                                 "1.1.0.0 (2021-02-10)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 NMEA2000_DEV_ENGINE
                                 );
  NMEA2000.SetProductInformation("00000004", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Temperature Monitor",  // Manufacturer's Model ID
                                 "1.1.0.22 (2021-02-10)",  // Manufacturer's Software version code
                                 "1.1.0.0 (2021-02-10)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 NMEA2000_DEV_TEMPERATURE
                                 );
  NMEA2000.SetProductInformation("00000005", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Environment Monitor",  // Manufacturer's Model ID
                                 "1.1.0.22 (2021-02-10)",  // Manufacturer's Software version code
                                 "1.1.0.0 (2021-02-10)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 NMEA2000_DEV_ATMOSPHERIC
                                 );
  NMEA2000.SetProductInformation("00000006", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Battery Monitor",  // Manufacturer's Model ID
                                 "1.1.0.22 (2021-02-10)",  // Manufacturer's Software version code
                                 "1.1.0.0 (2021-02-10)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 NMEA2000_DEV_BATTERIES
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(3, // Unique number. Use e.g. Serial number.
                                160, // Device function=Engine Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                50, // Device class=Propulsion. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2085, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, // Marine
                                NMEA2000_DEV_ENGINE
                                );
  NMEA2000.SetDeviceInformation(4, // Unique number. Use e.g. Serial number.
                                130, // Device function=Engine Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75, // Device class=Propulsion. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2085, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, // Marine
                                NMEA2000_DEV_TEMPERATURE
                                );
  NMEA2000.SetDeviceInformation(5, // Unique number. Use e.g. Serial number.
                                130, // Device function=Engine Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=Propulsion. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2085, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, // Marine
                                NMEA2000_DEV_ATMOSPHERIC
                                );
  NMEA2000.SetDeviceInformation(5, // Unique number. Use e.g. Serial number.
                                170, // Device function=Engine Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                35, // Device class=Propulsion. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2085, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, // Marine
                                NMEA2000_DEV_BATTERIES
                                );

  // debugging with no chips connected.
#ifdef DEBUG_NMEA2000
  NMEA2000.SetForwardStream(&SerialIO);
  NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); 
  NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
#endif

  // this is a node, we are not that interested in other traffic on the bus.
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,23);
  NMEA2000.EnableForward(false);
  NMEA2000.ExtendTransmitMessages(TransmitMessagesEngine,NMEA2000_DEV_ENGINE);
  NMEA2000.ExtendTransmitMessages(TransmitMessagesTemperature,NMEA2000_DEV_TEMPERATURE);
  NMEA2000.ExtendTransmitMessages(TransmitMessagesAtmospheric,NMEA2000_DEV_ATMOSPHERIC);
  NMEA2000.ExtendTransmitMessages(TransmitMessagesBatteries,NMEA2000_DEV_BATTERIES);
  NMEA2000.Open();

  

  Serial.println("running, take pin 25 low to enable bluetooth");
  
}

// todo, configuration.


void SendRapidEngineData() {
  static unsigned long RapidEngineUpdated=millis();
  tN2kMsg N2kMsg;
  unsigned long period = engineMonitor.getRapidEngineUpdatePeriod();
  if ( period != 0 && RapidEngineUpdated+period<millis() ) {
    double rpm = engineMonitor.getFlyWheelRPM();
    if ( engineConfig.isMonitoringEnabled() ) {
      SerialIO.printf("RPM %f\n",rpm);
    }
    // PGN127488
    SetN2kEngineParamRapid(N2kMsg, 0, rpm);
    NMEA2000.SendMsg(N2kMsg, NMEA2000_DEV_ENGINE);
    RapidEngineUpdated = millis();
  }
}

void SendEngineData() {
  static unsigned long EngineUpdated=millis();
  tN2kMsg N2kMsg;

  unsigned long period = engineMonitor.getEngineUpdatePeriod();
  if ( period != 0 && EngineUpdated+period<millis() ) {
    // PGN127489
    double temperature = engineMonitor.getCoolantTemperature();
    double alternatorVoltage = engineMonitor.getAlternatorVoltage();
    double engineHours = engineMonitor.getEngineHours();

    if ( engineConfig.isMonitoringEnabled() ) {
      SerialIO.printf("Engine Params1 t=%f, av=%f, eh=%f\n",
        temperature, alternatorVoltage, engineHours);
    }

    /*

inline void SetN2kEngineDynamicParam(tN2kMsg &N2kMsg, unsigned char EngineInstance, double EngineOilPress, double EngineOilTemp, double EngineCoolantTemp, double AltenatorVoltage,
                       double FuelRate, double EngineHours, double EngineCoolantPress=N2kDoubleNA, double EngineFuelPress=N2kDoubleNA,
                       int8_t EngineLoad=N2kInt8NA, int8_t EngineTorque=N2kInt8NA,
                       tN2kEngineDiscreteStatus1 Status1=0, tN2kEngineDiscreteStatus2 Status2=0) {
                         */
    SetN2kEngineDynamicParam(N2kMsg, 0, N2kDoubleNA, N2kDoubleNA, CToKelvin(temperature), alternatorVoltage,
                       N2kDoubleNA, engineHours, N2kDoubleNA, N2kDoubleNA, N2kInt8NA, N2kInt8NA,0,0);

    NMEA2000.SendMsg(N2kMsg, NMEA2000_DEV_ENGINE);
    EngineUpdated=millis();

  }
}



void SendTemperatureData() {
  static unsigned long TemperatureUpdated=millis();
  tN2kMsg N2kMsg;

  unsigned long period = engineMonitor.getTemperatureUpdatePeriod();
  if ( period != 0 && TemperatureUpdated+period<millis() ) {
    // PGN130312
    if ( engineConfig.isMonitoringEnabled() ) {
      SerialIO.printf("Temperature er=%f, eg=%f, at=%f\n",
        engineMonitor.getEngineRoomTemperature(), engineMonitor.getExhaustTemperature(), engineMonitor.getAlternatorTemperature());
    }

    SetN2kTemperature(N2kMsg, 0, 0, N2kts_EngineRoomTemperature, CToKelvin(engineMonitor.getEngineRoomTemperature()));
    NMEA2000.SendMsg(N2kMsg, NMEA2000_DEV_TEMPERATURE);
    SetN2kTemperature(N2kMsg, 1, 1, N2kts_ExhaustGasTemperature, CToKelvin(engineMonitor.getExhaustTemperature()));
    NMEA2000.SendMsg(N2kMsg, NMEA2000_DEV_TEMPERATURE);
    SetN2kTemperature(N2kMsg, 2, 2, N2kts_LiveWellTemperature, CToKelvin(engineMonitor.getAlternatorTemperature()));
    NMEA2000.SendMsg(N2kMsg, NMEA2000_DEV_TEMPERATURE);
    TemperatureUpdated=millis();
  }
}


void SendVoltages() {
  static unsigned long VoltageUpdated=millis();
  tN2kMsg N2kMsg;

  unsigned long period = engineMonitor.getVoltageUpdatePeriod();
  if ( period != 0 && VoltageUpdated+period<millis() ) {

    // Battery Status PGN127508
    if ( engineConfig.isMonitoringEnabled() ) {
      SerialIO.printf("Voltages sb=%f, av=%f\n",
        engineMonitor.getServiceBatteryVoltage(), engineMonitor.getAlternatorVoltage());
    }
    
    SetN2kDCBatStatus(N2kMsg,0, engineMonitor.getServiceBatteryVoltage(), N2kDoubleNA, CToKelvin(engineMonitor.getServiceBatteryTemperature()), 0);
    NMEA2000.SendMsg(N2kMsg, NMEA2000_DEV_BATTERIES);
    // send the Alternator information as if it was a 3rd battery, becuase there is no other way.
    SetN2kDCBatStatus(N2kMsg,1, engineMonitor.getAlternatorVoltage(), N2kDoubleNA, CToKelvin(engineMonitor.getAlternatorTemperature()), 1);
    NMEA2000.SendMsg(N2kMsg, NMEA2000_DEV_BATTERIES);

    // Battery Configuration PGN127513
//    SetN2kBatConf(N2kMsg,1,N2kDCbt_Flooded,N2kDCES_Unavailable,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(55),53,1.251,75);
//    NMEA2000.SendMsg(N2kMsg);
//    SetN2kBatConf(N2kMsg,2,N2kDCbt_AGM,N2kDCES_Unavailable,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(330),53,1.251,75);
//    NMEA2000.SendMsg(N2kMsg);
    VoltageUpdated=millis();
  }
}


void SendEnvironment() {
  static unsigned long EnvironmentUpdated=millis();
  if ( hasBMP280 ) {

    tN2kMsg N2kMsg;

    unsigned long period = engineMonitor.getEnvironmentUpdatePeriod();
    if ( period != 0 && EnvironmentUpdated+period<millis() ) {

        double temperature = bmp.readTemperature();
        double pressure =  bmp.readPressure();

      if ( engineConfig.isMonitoringEnabled() ) {
        SerialIO.printf("Environment t=%f, p=%f\n",
          temperature, pressure);
      }

      // PGN130311
      SetN2kEnvironmentalParameters(N2kMsg,4,
        tN2kTempSource::N2kts_InsideTemperature,
        CToKelvin(temperature),N2khs_Undef,N2kDoubleNA,pressure);
      NMEA2000.SendMsg(N2kMsg, NMEA2000_DEV_ATMOSPHERIC);

      EnvironmentUpdated=millis();
        
    }
  }

}

void DumpStatus() {
    SerialIO.printf("Has BMP280              = %d\n", hasBMP280);
    SerialIO.printf("Engine Powered up       = %d\n", engineMonitor.isEngineOn());
    SerialIO.printf("Engine Running          = %d\n", engineMonitor.isEngineRunning());
    SerialIO.printf("RPM                     = %f\n", engineMonitor.getFlyWheelRPM());
    SerialIO.printf("Coolant Temperature     = %f\n", engineMonitor.getCoolantTemperature());
    SerialIO.printf("Fuel Tank Level     = %f\n", engineMonitor.getFuelTankLevel());
    SerialIO.printf("Alternator Voltage      = %f\n", engineMonitor.getAlternatorVoltage());
    SerialIO.printf("Service Battery Voltage = %f\n", engineMonitor.getServiceBatteryVoltage());
    SerialIO.printf("Engine Hours            = %f\n", engineMonitor.getEngineHours());
    SerialIO.printf("Engine Room Temperature = %f\n", engineMonitor.getEngineRoomTemperature());
    SerialIO.printf("Exhaust Temperature     = %f\n", engineMonitor.getExhaustTemperature());
    SerialIO.printf("Alternator Temperature  = %f\n", engineMonitor.getAlternatorTemperature());
    SerialIO.printf("Service Battery Temp    = %f\n", engineMonitor.getServiceBatteryTemperature());
    SerialIO.printf("Inside Temperature      = %f\n", bmp.readTemperature());
    SerialIO.printf("Inside Pressure         = %f\n", bmp.readPressure());

}

void CallBack(int8_t cmd) {
  switch(cmd) {
    case CMD_STATUS:
      DumpStatus();
      break;
    case CMD_BT_OFF:
      StopBluetooth();
      break;
  }
}





void loop() {
  // read the serial data and process.
  CheckBluetooth();
  engineConfig.process(CallBack);
  engineMonitor.readSensors(engineConfig.isMonitoringEnabled());
  // Send to N2K Bus
  if ( engineMonitor.isEngineOn() ) {
    SendRapidEngineData();
    SendEngineData();
  }
  SendTemperatureData();
  SendVoltages();
  SendEnvironment();
  NMEA2000.ParseMessages();
}