#include <Arduino.h>

//#define SERIAL_DEBUG_DISABLED

#define ESP32_CAN_TX_PIN GPIO_NUM_19
#define ESP32_CAN_RX_PIN GPIO_NUM_18

#define ONEWIRE_PIN GPIO_NUM_21
#define DATAEN_PIN GPIO_NUM_4 
#define RF_RX  GPIO_NUM_16
#define RF_TX  GPIO_NUM_17


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


const unsigned long TransmitMessages[] PROGMEM={130306L,0};


OneWire oneWire(ONEWIRE_PIN);
EngineMonitor engineMonitor(&oneWire,  &SerialIO);
EngineConfig engineConfig(&engineMonitor, &SerialIO);

#define INPUT_BUFFER_SIZE 1024
char inputBuffer[INPUT_BUFFER_SIZE];


void setup() {

  
#ifdef BLUETOOTHCLASSIC                               
  SerialBT.begin("EngineMonitor"); 
#else 
  Serial.begin(115200);
#endif

  engineMonitor.begin();  

  // Setup NMEA2000
  // Set Product information
  NMEA2000.SetProductInformation("00000003", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Engine Monitor",  // Manufacturer's Model ID
                                 "1.1.0.22 (2021-02-10)",  // Manufacturer's Software version code
                                 "1.1.0.0 (2021-02-10)" // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Device function=Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=External Environment. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  // debugging with no chips connected.
  NMEA2000.SetForwardStream(&SerialIO);
  NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); 
  NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega

  // this is a node, we are not that interested in other traffic on the bus.
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,23);
  NMEA2000.EnableForward(false);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.Open();

  

  SerialIO.println("Starting");
  
}

// todo, configuration.

#define RapidEngineUpdatePeriod 1000
#define EngineUpdatePeriod 3000
#define TemperatureUpdatePeriod 30000
#define VoltageUpdatePeriod 5000

void SendRapidEnginData() {
  static unsigned long RapidEngineUpdated=millis();
  tN2kMsg N2kMsg;

  if ( RapidEngineUpdated+RapidEngineUpdatePeriod<millis() ) {
    double rpm = engineMonitor.getFlyWheelRPM();
    if (rpm > 0) {
      // PGN127488
      SetN2kEngineParamRapid(N2kMsg, 1, rpm);
      NMEA2000.SendMsg(N2kMsg);
    }
    RapidEngineUpdated = millis();
  }
}

void SendEnginData() {
  static unsigned long EngineUpdated=millis();
  tN2kMsg N2kMsg;

  if ( EngineUpdated+EngineUpdatePeriod<millis() ) {
    // PGN127489
    double temperature = engineMonitor.getCoolantTemperature();
    double alternatorVoltage = engineMonitor.getAlternatorVoltage();
    double oilPressure = engineMonitor.getOilPressure();
    double oilTemperature = engineMonitor.getOilTemperature();
    double fuelRate = engineMonitor.getFuelRate();
    double engineHours = engineMonitor.getEngineHours();
    double coolantPressure = engineMonitor.getCoolantPressure();
    double fuelPressure = engineMonitor.getFuelPressure();
    int8_t load = engineMonitor.getLoad(); 
    int8_t torque = engineMonitor.getTorque();
    int8_t status1 = engineMonitor.getStatus1(); /* tN2kEngineDiscreteStatus1 */
    int8_t status2 = engineMonitor.getStatus2(); /* tN2kEngineDiscreteStatus2 */
    
    /*

inline void SetN2kEngineDynamicParam(tN2kMsg &N2kMsg, unsigned char EngineInstance, double EngineOilPress, double EngineOilTemp, double EngineCoolantTemp, double AltenatorVoltage,
                       double FuelRate, double EngineHours, double EngineCoolantPress=N2kDoubleNA, double EngineFuelPress=N2kDoubleNA,
                       int8_t EngineLoad=N2kInt8NA, int8_t EngineTorque=N2kInt8NA,
                       tN2kEngineDiscreteStatus1 Status1=0, tN2kEngineDiscreteStatus2 Status2=0) {
                         */
    SetN2kEngineDynamicParam(N2kMsg, 1, oilPressure, oilTemperature, temperature, alternatorVoltage,
                       fuelRate, engineHours, coolantPressure, fuelPressure, load, torque, status1, status2);

    NMEA2000.SendMsg(N2kMsg);


  }
}

void SendTemperatureData() {
  static unsigned long TemperatureUpdated=millis();
  tN2kMsg N2kMsg;

  if ( TemperatureUpdated+TemperatureUpdatePeriod<millis() ) {
    // PGN130312
    SetN2kTemperature(N2kMsg, 1, 1, N2kts_EngineRoomTemperature, engineMonitor.getEngineRoomTemperature());
    NMEA2000.SendMsg(N2kMsg);
    SetN2kTemperature(N2kMsg, 2, 2, N2kts_ExhaustGasTemperature, engineMonitor.getExhaustTemperature());
    NMEA2000.SendMsg(N2kMsg);
    SetN2kTemperature(N2kMsg, 2, 2, N2kts_LiveWellTemperature, engineMonitor.getAlternatorTemperature());
    NMEA2000.SendMsg(N2kMsg);
    TemperatureUpdated=millis();
  }
}


void SendVoltages() {
  static unsigned long VoltageUpdated=millis();
  tN2kMsg N2kMsg;

  if ( VoltageUpdated+VoltageUpdatePeriod<millis() ) {

    // Battery Status PGN127508
    
    SetN2kDCBatStatus(N2kMsg,1, 12.6, N2kDoubleNA, N2kDoubleNA, 1);
    NMEA2000.SendMsg(N2kMsg);
    SetN2kDCBatStatus(N2kMsg,2, 12.6, N2kDoubleNA, N2kDoubleNA, 2);
    NMEA2000.SendMsg(N2kMsg);
    // send the Alternator information as if it was a 3rd battery, becuase there is no other way.
    SetN2kDCBatStatus(N2kMsg,3, engineMonitor.getAlternatorVoltage(), N2kDoubleNA, CToKelvin(engineMonitor.getAlternatorTemperature()), 3);
    NMEA2000.SendMsg(N2kMsg);

    // Battery Configuration PGN127513
    SetN2kBatConf(N2kMsg,1,N2kDCbt_Flooded,N2kDCES_Unavailable,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(55),53,1.251,75);
    NMEA2000.SendMsg(N2kMsg);
    SetN2kBatConf(N2kMsg,2,N2kDCbt_AGM,N2kDCES_Unavailable,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(330),53,1.251,75);
    NMEA2000.SendMsg(N2kMsg);
    VoltageUpdated=millis();
  }
}





void loop() {
  // read the serial data and process.
  engineConfig.process();
  engineMonitor.readSensors();
  if ( engineConfig.isOutputEnabled() ) {
    NMEA2000.SetForwardStream(&SerialIO);
  } else {
    NMEA2000.SetForwardStream(0);
  }
  // Send to N2K Bus
  SendEnginData();
  SendTemperatureData();
  SendVoltages();
  NMEA2000.ParseMessages();
}