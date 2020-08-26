#include <Arduino.h>

//#define SERIAL_DEBUG_DISABLED

#define USE_LIB_WEBSOCKET true

#include "sensesp_app.h"
#include "sensors/onewire_temperature.h"
#include "sensors/bmp280.h"
#include "sensors/analog_input.h"
#include "sensors/digital_input.h"
#include "signalk/signalk_output.h"
#include "wiring_helpers.h"
#include "transforms/change_filter.h"
#include "transforms/linear.h"
#include "transforms/analogvoltage.h"
#include "transforms/voltagedividerR2.h"
#include "transforms/curveinterpolator.h"
#include "transforms/frequency.h"


/*
Based almost entirely on cut and paste from examples in the SenseESP project.
 */

/*
  Illustrates a custom transform that takes a resistance value in ohms and returns the estimated
  temperature in Kelvin. Sample data in this example were taken from a Westerbeke generator
  temperature sender and gauge.
  Note that you will never instantiate CurveInterpolator in your own SensESP
  project - you will always need to create a descendant class of it, like
  TemperatureInterpreter in this example. The constructor needs to do only two things:
  call clearSamples(), then call addSample() at least twice. (This class can't function
  without at least two samples.) You can call addSample() as many times as you like,
  and the more samples you have, the more accurately this transform will emulate the
  analog sensor. Note, however, that since all transforms inherit from class
  Configurable, you can make its data configurable in the Config UI, and that means
  not only that you can edit (at runtime) whatever samples you add in the constructor,
  but also that you can add more samples at runtime.
  
  One of your samples should be at the lowest value for the input,and one should be at
  the highest value, so the input to CurveInterpolator will always be between two values.
   
  This example is a bit complex because of the need for the VoltageDivider2 transform,
  but that also makes it an excellent "advanced" example. 
*/


/**
 *  Based on the Volvo Penta D2-40 workshop maual.
 * 5V supply to sensor.
 * 2-3V at 20C, will need an acurage measurement here to get the resistance to +5V
 * Sensor resistances vs temp, from p26 of the manual, test procedure.
*	C	R	K
*	120	22	393
*	110	29	383
*	100	38	373
*	90	51	363
*	80	70	353
*	70	97	343
*	60	134	333
*	50	197	323
*	40	291	313
*	30	439	303
*	20	677	293
*	10	1076	283
*	0	1743	273

 */
class TemperatureInterpreter : public CurveInterpolator {

    public:
        TemperatureInterpreter(String config_path="") :
           CurveInterpolator(NULL, config_path ) {

          // Populate a lookup table tp translate the ohm values returned by
          // our temperature sender to degrees Kelvin
          clearSamples();
          // addSample(CurveInterpolator::Sample(knownOhmValue, knownKelvin));
          addSample(CurveInterpolator::Sample(22,	393));
          addSample(CurveInterpolator::Sample(29,	383));
          addSample(CurveInterpolator::Sample(38,	373));
          addSample(CurveInterpolator::Sample(51,	363));
          addSample(CurveInterpolator::Sample(70,	353));
          addSample(CurveInterpolator::Sample(97,	343));
          addSample(CurveInterpolator::Sample(134, 333));
          addSample(CurveInterpolator::Sample(197, 323));
          addSample(CurveInterpolator::Sample(291,	313));
          addSample(CurveInterpolator::Sample(439,	303));
          addSample(CurveInterpolator::Sample(677,	293));
          addSample(CurveInterpolator::Sample(1076,	283));
          addSample(CurveInterpolator::Sample(1743,	273));

        }
};

ReactESP app([] () {
  #ifndef SERIAL_DEBUG_DISABLED
  Serial.begin(115200);

  // A small delay and one debugI() are required so that
  // the serial output displays everything
  delay(100);
  Debug.setSerialEnabled(true);
  #endif
  delay(100);
  debugI("Serial debug enabled");

  sensesp_app = new SensESPApp(noStdSensors);

  /* Find all the sensors and their unique addresses. Then, each new instance
     of OneWireTemperature will use one of those addresses. You can't specify
     which address will initially be assigned to a particular sensor, so if you
     have more than one sensor, you may have to swap the addresses around on
     the configuration page for the device. (You get to the configuration page
     by entering the IP address of the device into a browser.)
     ESP8266 pins are specified as DX
     ESP32 pins are specified as just the X in GPIOX
  */
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(15);

  // temp updates every 5s is fast enough.
  // the sensors have low noise and move slowly.
  uint engine_read_delay = 5000;

  auto* pOneWire1 = new OneWireTemperature(dts, engine_read_delay, "/oneWire-1/sensor");

    pOneWire1->connectTo(new Linear(1.0, 0.0, "/oneWire-1/linear"))
              ->connectTo(new ChangeFilter(0.1,50,10,"/oneWire-1/filter"))
                ->connectTo(new SKOutputNumber("propulsion.mainEngine.exhaustTemperature", "/oneWire-1/sk"));

  auto* pOneWire2 = new OneWireTemperature(dts, engine_read_delay, "/oneWire-2/sensor");
    
    pOneWire2->connectTo(new Linear(1.0, 0.0, "/oneWire-2/linear"))
              ->connectTo(new ChangeFilter(0.1,50,10,"/oneWire-2/filter"))
                ->connectTo(new SKOutputNumber("electrical.alternators.12V.temperature", "/oneWire-2/sk"));
  
  auto* pOneWire3 = new OneWireTemperature(dts, engine_read_delay, "/oneWire-3/sensor");
      
      pOneWire3->connectTo(new Linear(1.0, 0.0, "/oneWire-3/linear"))
              ->connectTo(new ChangeFilter(0.1,50,10,"/oneWire-3/filter"))
              ->connectTo(new SKOutputNumber("electrical.chargers.12V.temperature", "/oneWire-3/sk"));

  auto* pOneWire4 = new OneWireTemperature(dts, engine_read_delay, "/oneWire-4/sensor");
      
      pOneWire4->connectTo(new Linear(1.0, 0.0, "/oneWire-4/linear"))
              ->connectTo(new ChangeFilter(0.1,50,10,"/oneWire-4/filter"))
              ->connectTo(new SKOutputNumber("electrical.fridge.main.temperature", "/oneWire-4/sk"));      

// Environment

  // Create a BMP280, which represents the physical sensor.
  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  auto* pBMP280 = new BMP280(0x76);

  // If you want to change any of the settings that are set by Adafruit_BMP280::setSampling(), do
  // that here, like this:
  // pBMP280->pAdafruitBMP280->setSampling(); // pass in the parameters you want

  // Define the read_delays you're going to use:
  const uint env_read_delay = 5000; // once per 5second
  const uint pressure_read_delay = 60000; // once per minute

  // Create a BMP280value, which is used to read a specific value from the BMP280, and send its output
  // to SignalK as a number (float). This one is for the temperature reading.
  auto* pBMPtemperature = new BMP280value(pBMP280, temperature, env_read_delay, "/bmp280Temperature/sensor");
      
      pBMPtemperature->connectTo(new ChangeFilter(0.1,50,10,"/bmp280Temperature/filter"))
                     ->connectTo(new SKOutputNumber("environment.outside.temperature","/bmp280Temperature/sk"));


  // Do the same for the barometric pressure value. Its read_delay is longer, since barometric pressure can't
  // change all that quickly. It could be much longer for that reason.
  auto* pBMPpressure = new BMP280value(pBMP280, pressure,  pressure_read_delay, "/bmp280Pressure/sensor");
      
      pBMPpressure->connectTo(new ChangeFilter(1,10,999,"/bmp280Pressure/filter"))
                  ->connectTo(new SKOutputNumber("environment.outside.pressure","/bmp280Pressure/sk"));


/*
 ESP32s are 0-4095 ADCs, the libraries assume 1023, hence 
 the libraries use max_voltage to represent the voltage at 1023, 
 hence max_voltage = 3.299999952*1023/4095 =  0.8243955924

 13.5/12.87 = 
 

 1.048951049*14.62/14.82651 = 1.0343408082

 1.0343408082*13.50/13.31316 = 1.0487569889
 13.60/13.43784
 13.50/13.42399

2742, 12.91
2705, 
3122, 14.45
2950, 13.70
2917, 13.64
2897, 13.56
2890, 13.49




 */
// Engine Coolant 
  /*
  Circuit is 
    5V |
       R1
       |______ Vt
       |     |
       |     R3
       |     |___ ADC
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
    
    Process for reading
    Read ADC
    Convert to Voltage convert the value from the AnalogIn pin into an AnalogVoltage()
    Scale Voltage to Vt Linear()
    Calculate R2 VoltageDividerR2()
    Calibrate with curve TemperatureInterpreter()
    Scale ito calibrate if required Linear()
  */

   // Voltage sent into the voltage divider circuit that includes the analog sender
  const float Vin = 5.0; 
  // The resistance, in ohms, of the fixed resistor (R1) in the voltage divider circuit
  const float R1 = 545.5;  
  // TBD by measurement. 
  // R2 at stop (about 20C) 654
  // V across R2 2.726V
  // R1 = (654*(5-2.726))/2.726V = 545.5

  // 

  const uint coolant_read_delay = 5000; // once per second


// 33 == Pin A on the input header
  auto* pAnalogInputA = new AnalogInput(33, coolant_read_delay, "/analog-a/adc");

  pAnalogInputA->connectTo(new AnalogVoltage(0.8243955924F, 
              1.4680851064F, // using simple 22K/47K divider
              0.0F, // zero offset
               "/analog-a/voltage")) ->
              //  connectTo(new Linear(r3R4scale, 0.0, "/analog-a/adcscale")) -> 
                connectTo(new VoltageDividerR2(R1, Vin, "/analog-a/sender")) -> 
                connectTo(new TemperatureInterpreter("/analog-a/curve")) -> 
                connectTo(new Linear(1.0, 0.0, "/analog-a/calibrate")) -> 
                connectTo(new ChangeFilter(0.5,10,50,"/analog-a/filter")) ->
                connectTo(new SKOutputNumber("propulsion.mainEngine.temperature", 
                     "/analog-a/sk")); 

/**
 * Based on measurement with a 10K and 2K2 resistor
 * Voltage at 1023 0.8243955924
 * Multiplier 4.8458898894
 * Offset 2.2048799
 
 * 33,32,35,34,VN,VP
 */
  auto* pAnalogInputB = new AnalogInput(32, coolant_read_delay,  "/analog-b/adc");

  pAnalogInputB->connectTo(new AnalogVoltage(0.8243955924F, 
              4.8458898894F, 2.2048799F, "/analog-b/calibrate")) ->  
                connectTo(new ChangeFilter(0.1,10,10,"/analog-b/filter")) ->
                connectTo(new SKOutputNumber("propulsion.mainEngine.alternatorVoltage", 
                     "/analog-b/sk")); 


  auto* pAnalogInputC = new AnalogInput(35, coolant_read_delay,  "/analog-c/adc");

  pAnalogInputC->connectTo(new AnalogVoltage(0.8243955924F, 
              4.8458898894F, 2.2048799F, "/analog-c/calibrate")) -> 
                connectTo(new ChangeFilter(0.1,10,10,"/analog-c/filter")) ->
                connectTo(new SKOutputNumber("electrical.batteries.engine.voltage", 
                     "/analog-c/sk")); 

  auto* pAnalogInputD = new AnalogInput(34, coolant_read_delay,  "/analog-d/adc");

  pAnalogInputD->connectTo(new AnalogVoltage(0.8243955924F, 
              4.8458898894F, 2.2048799F, "/analog-d/calibrate")) ->  
                connectTo(new ChangeFilter(0.1,10,10,"/analog-d/filter")) ->
                connectTo(new SKOutputNumber("electrical.batteries.service.voltage", 
                     "/analog-d/sk")); 



/*
  Engine RPM

  Using the W+ terminal on the alternator. This is wired drectly to one of the alternator windings
  which produces a sine wave at a multiple of the alternator RPM, which is itself a multiple
  of the engine RPM.

  Circuit

  W+ alternator -|         ______________________
                 R1        |                     \
                 |_________| Schmit               \___ D19/Pin 25
                 |   |     | Trigger 74HC14 1/6   /
                 R2  C1    |_____________________/
            GND  |___|

   R1 = 10K
   R2 = 2K2
   C1 = 0.1uF
   */


  const float multiplier = 0.1037410505; // measured on D2-40 standard alternator
  const uint read_delay = 2000; // need a rapid check to deal with acceleration

// pin 1 hotwired to D4 
// TODO fix PCB
// 750 137
// 1000 162
// 1500 238
// 2000 325
// 2500 394
// 15.76V to -.885

  auto* pRPMSensor = new DigitalInputCounter(19, INPUT_PULLUP, RISING, read_delay, "/digital-1/pin4");

  pRPMSensor->connectTo(new Frequency(multiplier, "/digital-1/calibrate"))  // connect the output of pSensor to the input of Frequency()
        ->connectTo(new ChangeFilter(0.2,20,10,"/digital-1/filter")) // 0.2 Hz is about 10RPM. 
        ->connectTo(new SKOutputNumber("propulsion.mainEngine.revolutions", 
            "/digital-1/sk"));   // connect the output of Frequency() to a SignalK Output as a number


  sensesp_app->enable();
});