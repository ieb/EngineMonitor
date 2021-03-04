#include "../testsupport.h"
#include <engineconfig.h>



using namespace fakeit;

Stream * io = ArduinoFakeMock(Stream);
OneWire oneWire;
EngineMonitor engineMonitor(&oneWire, io);
EngineConfig * engineConfig;

void setUp(void) {
    ArduinoFakeReset();
    setupStream();
    engineConfig = new EngineConfig(&engineMonitor, io);
    engineConfig->begin();
}

void tearDown(void) {
    delete engineConfig;
}


void test_help() {
    MockStreamLoader loader;
    loader.load("help\n");
    engineConfig->process();
}

void test_dump() {
    MockStreamLoader loader;
    loader.load("dump\n");
    engineConfig->process();
}

// etc|engine temp config <r1>,<v>      - set engine coolant top resistor and voltage , floatx2, default 545.5,5[Serial]

void test_etc() {
    TEST_ASSERT_EQUAL_FLOAT(545.5,engineConfig->config->coolantTempR1);
    TEST_ASSERT_EQUAL_FLOAT(5.0,engineConfig->config->coolantTempVin);
    MockStreamLoader loader;
    loader.load("engine temp config 4.9,562.3\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_FLOAT(562.3,engineConfig->config->coolantTempR1);
    TEST_ASSERT_EQUAL_FLOAT(4.9,engineConfig->config->coolantTempVin);
    loader.load("etc 4.7,568.3\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_FLOAT(568.3,engineConfig->config->coolantTempR1);
    TEST_ASSERT_EQUAL_FLOAT(4.7,engineConfig->config->coolantTempVin);
}
//ett|engine temp therm <n>,...<n>     - set engine coolant thermistor resistance values 0C-120C, floatx13, default as per manual[Serial]

void test_ett() {
    float defaultValues[] = {1743, 1076, 677, 439, 291, 197, 134, 97, 70, 51,38  ,29, 22 };
    float changedValues[] = {1743, 1076, 677, 465, 291, 197, 134, 97, 62, 51,38  ,29, 22 };
    for (int i = 0; i < 13; i++ ) {
        TEST_ASSERT_EQUAL_FLOAT(defaultValues[i],engineConfig->config->coolantTempR2[i]);
    }
    MockStreamLoader loader;
    loader.load("engine temp therm 1743, 1076, 677, 465, 291, 197, 134, 97, 62, 51,38  ,29, 22 \n");
    engineConfig->process();
    for (int i = 0; i < 13; i++ ) {
        TEST_ASSERT_EQUAL_FLOAT(changedValues[i],engineConfig->config->coolantTempR2[i]);
    }
    loader.load("engine temp therm 1743, 1076, 677, 439, 291, 197, 134, 97, 70, 51,38  ,29, 22 \n");
    engineConfig->process();
    for (int i = 0; i < 13; i++ ) {
        TEST_ASSERT_EQUAL_FLOAT(defaultValues[i],engineConfig->config->coolantTempR2[i]);
    }
    loader.load("ett 1743, 1076, 677, 465, 291, 197, 134, 97, 62, 51,38  ,29, 22 \n");
    engineConfig->process();
    for (int i = 0; i < 13; i++ ) {
        TEST_ASSERT_EQUAL_FLOAT(changedValues[i],engineConfig->config->coolantTempR2[i]);
    }
    loader.load("ett 1743, 1076, 677, 439, 291, 197, 134, 97, 70, 51,38  ,29, 22 \n");
    engineConfig->process();
    for (int i = 0; i < 13; i++ ) {
        TEST_ASSERT_EQUAL_FLOAT(defaultValues[i],engineConfig->config->coolantTempR2[i]);
    }
    // check for no change
    loader.load("ett 1743, 1076, 677, 465, 291, 197, 134, 97, 62, 51,38  ,29\n");
    engineConfig->process();
    for (int i = 0; i < 13; i++ ) {
        TEST_ASSERT_EQUAL_FLOAT(defaultValues[i],engineConfig->config->coolantTempR2[i]);
    }
    // check too many are simply accepted
    loader.load("ett 1743, 1076, 677, 465, 291, 197, 134, 97, 62, 51,38  ,29, 22,24\n");
    engineConfig->process();
    for (int i = 0; i < 13; i++ ) {
        TEST_ASSERT_EQUAL_FLOAT(changedValues[i],engineConfig->config->coolantTempR2[i]);
    }
}

// rpm scale <n>                        - set rpm per Hz scale, float, default 6.224463028[Serial]

void test_rpm_scale() {
    TEST_ASSERT_EQUAL_FLOAT(6.224463028,engineConfig->config->engineFlywheelRPMPerHz);
    MockStreamLoader loader;
    loader.load("rpm scale 8.224463028\n");
    std::cout << " Processing " <<  std::endl;
    engineConfig->process();
    std::cout << 8.224463028 << " == " << engineConfig->config->engineFlywheelRPMPerHz << std::endl;
    TEST_ASSERT_EQUAL_FLOAT(8.224463028,engineConfig->config->engineFlywheelRPMPerHz);
}

// owc|one wire <a>,..           - set one wire index for alternator, exhaust, engine room, intx3, default 0,1,2[Serial]
void test_owc() {
    TEST_ASSERT_EQUAL_INT(0,engineConfig->config->alternatorTemperatureIDX);
    TEST_ASSERT_EQUAL_INT(1,engineConfig->config->exhaustTemperatureIDX);
    TEST_ASSERT_EQUAL_INT(2,engineConfig->config->engineRoomTemperatureIDX);
    MockStreamLoader loader;
    loader.load("1 wire 1,2,0\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_INT(1,engineConfig->config->alternatorTemperatureIDX);
    TEST_ASSERT_EQUAL_INT(2,engineConfig->config->exhaustTemperatureIDX);
    TEST_ASSERT_EQUAL_INT(0,engineConfig->config->engineRoomTemperatureIDX);
    loader.load("owc 2,0,1\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_INT(2,engineConfig->config->alternatorTemperatureIDX);
    TEST_ASSERT_EQUAL_INT(0,engineConfig->config->exhaustTemperatureIDX);
    TEST_ASSERT_EQUAL_INT(1,engineConfig->config->engineRoomTemperatureIDX);
}

// rpc|read period  <r>,..        - set read period in ms or rpm, engine, voltage, temp, intx4, default 2000,5000,10000,30000[Serial]

void test_rpc() {
    TEST_ASSERT_EQUAL_INT(2000,engineConfig->config->flywheelRPMReadPeriod);
    TEST_ASSERT_EQUAL_INT(5000,engineConfig->config->engineTemperatureReadPeriod);
    TEST_ASSERT_EQUAL_INT(10000,engineConfig->config->voltageReadPeriod);
    TEST_ASSERT_EQUAL_INT(30000,engineConfig->config->temperatureReadPeriod);
    MockStreamLoader loader;
    loader.load("read period  2001,5001,10001,30001\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_INT(2001,engineConfig->config->flywheelRPMReadPeriod);
    TEST_ASSERT_EQUAL_INT(5001,engineConfig->config->engineTemperatureReadPeriod);
    TEST_ASSERT_EQUAL_INT(10001,engineConfig->config->voltageReadPeriod);
    TEST_ASSERT_EQUAL_INT(30001,engineConfig->config->temperatureReadPeriod);
    loader.load("rpc  2002,5002,10002,30002\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_INT(2002,engineConfig->config->flywheelRPMReadPeriod);
    TEST_ASSERT_EQUAL_INT(5002,engineConfig->config->engineTemperatureReadPeriod);
    TEST_ASSERT_EQUAL_INT(10002,engineConfig->config->voltageReadPeriod);
    TEST_ASSERT_EQUAL_INT(30002,engineConfig->config->temperatureReadPeriod);
}

// opc|oil pressure  <o>, <s>    - set oil pressure offset and scale, floatx2, defult 0.5,50[Serial]
// .oilPressureScale = 50,  // 0.5V = 0PSI, 4.5V = 200, scale=200/(4.5-0.5)
// .oilPresureOffset = 0.5,
void test_opc() {
    TEST_ASSERT_EQUAL_FLOAT(0.5,engineConfig->config->oilPressureOffset);
    TEST_ASSERT_EQUAL_FLOAT(50,engineConfig->config->oilPressureScale);
    MockStreamLoader loader;
    loader.load("oil pressure 0.7,84.3\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_FLOAT(0.7,engineConfig->config->oilPressureOffset);
    TEST_ASSERT_EQUAL_FLOAT(84.3,engineConfig->config->oilPressureScale);
    loader.load("opc 0.21,30.2\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_FLOAT(0.21,engineConfig->config->oilPressureOffset);
    TEST_ASSERT_EQUAL_FLOAT(30.2,engineConfig->config->oilPressureScale);
}

// flc|fuel level  <o>, <s>             - set fuel level offset and scale, floatx2, defult 0.18,17.182130584
//    .fuelLevelScale = 0.18, // see method readFuleLevel
//    .fuelLevelOffset = 17.182130584,
void test_flc() {
    TEST_ASSERT_EQUAL_FLOAT(5.0,engineConfig->config->fuelLevelVin);
    TEST_ASSERT_EQUAL_FLOAT(545.5,engineConfig->config->fuelLevelR1);
    TEST_ASSERT_EQUAL_FLOAT(190.0,engineConfig->config->fuelLevelEmptyR);
    TEST_ASSERT_EQUAL_FLOAT(3.0,engineConfig->config->fuelLevelFullR);
    MockStreamLoader loader;
    loader.load("fuel level 5.2,560,180,4\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_FLOAT(5.2,engineConfig->config->fuelLevelVin);
    TEST_ASSERT_EQUAL_FLOAT(560.0,engineConfig->config->fuelLevelR1);
    TEST_ASSERT_EQUAL_FLOAT(180.0,engineConfig->config->fuelLevelEmptyR);
    TEST_ASSERT_EQUAL_FLOAT(4.0,engineConfig->config->fuelLevelFullR);

    loader.load("flc 5.3,562,173,8\n");
    engineConfig->process();
    TEST_ASSERT_EQUAL_FLOAT(5.3,engineConfig->config->fuelLevelVin);
    TEST_ASSERT_EQUAL_FLOAT(562.0,engineConfig->config->fuelLevelR1);
    TEST_ASSERT_EQUAL_FLOAT(173.0,engineConfig->config->fuelLevelEmptyR);
    TEST_ASSERT_EQUAL_FLOAT(8.0,engineConfig->config->fuelLevelFullR);
}


/*
void test_angle_correction() {
    TEST_ASSERT_EQUAL_INT(0,windConfig->config->angleCorrection);
    MockStreamLoader loader;
    loader.load("angle correction 15\n");
    windConfig->process();
    TEST_ASSERT_EQUAL_INT(15,windConfig->config->angleCorrection);
    loader.load("ac -2596\n");
    windConfig->process();
    TEST_ASSERT_EQUAL_INT(-2596,windConfig->config->angleCorrection);

}
void test_angle_direction() {
    TEST_ASSERT_EQUAL_INT(1,windConfig->config->signCorrection);
    MockStreamLoader loader;
    loader.load("angle dir 0\n");
    windConfig->process();
    TEST_ASSERT_EQUAL_INT(-1,windConfig->config->signCorrection);
    loader.load("angle dir 1\n");
    windConfig->process();
}

void test_angle_configuration() {
    MockStreamLoader loader;
    loader.load("angle size 36\n");
    windConfig->process(); 
    loader.load("angles 0,100,1,101,2,102,3,103,4,104,5,105,6,106,7,107,8,108,9,109\n");
    windConfig->process(); 
    loader.load("angles 10,110,11,111,12,112,13,113,14,114,15,115,16,116,17,117,18,118,19,119\n");
    windConfig->process(); 
    loader.load("a 20,120,21,121,22,122,23,123,24,124,25,125,26,126,27,127,28,128,29,129\n");
    windConfig->process(); 
    loader.load("angles 30,130,31,131,32,132,33,133,34,134,35,135\n");
    windConfig->process(); 
    TEST_ASSERT_EQUAL_INT(36,windConfig->config->angleTableSize);
    for (int i = 0; i < 36; i++) {
        TEST_ASSERT_EQUAL_INT16(i+100,windConfig->config->angleTable[i]);
    }
}

void test_speed_configuration() {
    MockStreamLoader loader;
    loader.load("speed size 5\n");
    windConfig->process(); 
    loader.load("speed table 0,5.5,20.5,40.6,45.6\n");
    windConfig->process(); 
    loader.load("speed values 0,1.5,2.5,4.6,6.7\n");
    windConfig->process(); 
    TEST_ASSERT_EQUAL_INT(5,windConfig->config->speedTableSize);
    double speedTable[] = {0,5.5,20.5,40.6,45.6};
    double speed[] = {0,1.5,2.5,4.6,6.7};
    for (int i = 0; i < 5; i++ ) {
        if ( fabs(speedTable[i]-windConfig->config->speedTable[i]) > 0.001 || fabs(speed[i]-windConfig->config->speed[i]) > 0.001 ) {
        std::cout << "FAIL" << i << "," << speedTable[i]  << "," << speed[i] << std::endl;
        std::cout << "FAIL" << i << "," << windConfig->config->speedTable[i]  << "," << windConfig->config->speed[i] << std::endl;
        std::cout << "FAIL" << i << "," << speedTable[i]-windConfig->config->speedTable[i]  << "," << speed[i]-windConfig->config->speed[i] << std::endl;

        }
        TEST_ASSERT_TRUE(fabs(speedTable[i]-windConfig->config->speedTable[i]) < 0.001);
        TEST_ASSERT_TRUE(fabs(speed[i]-windConfig->config->speed[i]) < 0.001);
    }
}

/ 
*
 io->println("Commands:");
        io->println("help|? - this help");
        io->println("as|angle size <n>         - set the number of angle caibrations.");
        io->println("a|angle <n>,<v>,<n1>,<v1> - set the angle calibrations, where n and v are ints. n = slot (0-slot), v = offset (+-)");
        io->println("st|speed table <n>,<n>    - Set the speed table up, where n is the puse frequency in Hz ");
        io->println("sv|speed values <n>,<n>   - Set the speed values up matching the speed table where n is the speed in m/s");
        io->println("dump                      - Dumps the current configuration");
        io->println("save                      - Saves the current configuration and makes it active.");
        io->print("$>")
* /
void test_config() {
    MockStreamLoader loader;

    loader.load("help\n");
    windConfig->process(); 
    loader.load("angle size 36\n");
    windConfig->process(); 
    loader.load("angles 0,100,1,101,2,102,3,103,4,104,5,105,6,106,7,107,8,108,9,109\n");
    windConfig->process(); 
    loader.load("angles 10,110,11,111,12,112,13,113,14,114,15,115,16,116,17,117,18,118,19,119\n");
    windConfig->process(); 
    loader.load("a 20,120,21,121,22,122,23,123,24,124,25,125,26,126,27,127,28,128,29,129\n");
    windConfig->process(); 
    loader.load("angles 30,130,31,131,32,132,33,133,34,134,35,135\n");
    windConfig->process(); 
    loader.load("speed size 5\n");
    windConfig->process(); 
    loader.load("speed table 0,5.5,20.5,40.6,45.6\n");
    windConfig->process(); 
    loader.load("speed values 0,1.5,2.5,4.6,6.7\n");
    windConfig->process(); 
    loader.load("dump\n");
    windConfig->process(); 
    loader.load("help\n");
    windConfig->process(); 
}

*/

int main(int argc, char **argv) { 
    try {
        UNITY_BEGIN();
        RUN_TEST(test_help);
        RUN_TEST(test_dump);
        RUN_TEST(test_etc);
        RUN_TEST(test_ett);
        RUN_TEST(test_rpm_scale);
        RUN_TEST(test_owc);
        RUN_TEST(test_rpc);
        RUN_TEST(test_opc);
        RUN_TEST(test_flc);
        return UNITY_END();
    } catch( UnexpectedMethodCallException e) {
            std::cout << "Exception:" << e << std::endl;

    }
}