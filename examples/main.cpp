
#include <Arduino.h>
#include <SmallNMEA2000.h>

#define RAPID_ENGINE_UPDATE_PERIOD 500
#define ENGINE_UPDATE_PERIOD 1000
#define VOLTAGE_UPDATE_PERIOD 9996
#define FUEL_UPDATE_PERIOD 10000
#define TEMPERATURE_UPDATE_PERIOD 4850

#define ENGINE_INSTANCE 0
#define SERVICE_BATTERY_INSTANCE 2
#define ENGINE_BATTERY_INSTANCE 3
#define FUEL_LEVEL_INSTANCE 0
#define FUEL_TYPE 0   // diesel


const SNMEA2000ProductInfo productInfomation PROGMEM={
                                       1300,                        // N2kVersion
                                       46,                         // Manufacturer's product code
                                       "TestCode",    // Manufacturer's Model ID
                                       "1.2.3.4 (2017-06-11)",     // Manufacturer's Software version code
                                       "55.6.7.8 (2017-06-11)",      // Manufacturer's Model version
                                       "0000001",                  // Manufacturer's Model serial code
                                       0,                           // SertificationLevel
                                       1                            // LoadEquivalency
};
const SNMEA2000ConfigInfo configInfo PROGMEM={
      "TestCode",
      "SmallNMEA2000",
      "https://github.com/ieb/SmallNMEA2000"
};


const unsigned long txPGN[] PROGMEM = { 
    127488L, // Rapid engine ideally 0.1s
    127489L, // Dynamic engine 0.5s
    127505L, // Tank Level 2.5s
    130316L, // Extended Temperature 2.5s
    127508L,
  SNMEA200_DEFAULT_TX_PGN
};
const unsigned long rxPGN[] PROGMEM = { 
  SNMEA200_DEFAULT_RX_PGN
};

SNMEA2000DeviceInfo devInfo = SNMEA2000DeviceInfo(
  222,   // device serial number
  140,  // Engine
  50 // Propulsion
);


#define DEVICE_ADDRESS 24
#define SNMEA_SPI_CS_PIN 10

EngineMonitor engineMonitor = EngineMonitor(DEVICE_ADDRESS,
  &devInfo,
  &productInfomation, 
  &configInfo, 
  &txPGN[0], 
  &rxPGN[0],
  SNMEA_SPI_CS_PIN);


void sendRapidEngineData() {
  static unsigned long lastRapidEngineUpdate=0;
  unsigned long now = millis();
  if ( now > lastRapidEngineUpdate+RAPID_ENGINE_UPDATE_PERIOD ) {
    lastRapidEngineUpdate = now;
    engineMonitor.sendRapidEngineDataMessage(ENGINE_INSTANCE, 100);
  }
}


void sendEngineData() {
  static unsigned long lastEngineUpdate=0;
  unsigned long now = millis();
  if ( now > lastEngineUpdate+ENGINE_UPDATE_PERIOD ) {
    lastEngineUpdate = now;
    engineMonitor.sendEngineDynamicParamMessage(ENGINE_INSTANCE,
        1234,
        350.0,
        12.2);
  }
}

void sendVoltages() {
  static unsigned long lastVoltageUpdate=0;
  static byte sid = 0;
  unsigned long now = millis();
  if ( now > lastVoltageUpdate+VOLTAGE_UPDATE_PERIOD ) {
    lastVoltageUpdate = now;
    // because the engine monitor is not on all the time, the sid and instance ids of these messages has been shifted
    // to make space for sensors that are on all the time, and would be used by default
    engineMonitor.sendDCBatterStatusMessage(SERVICE_BATTERY_INSTANCE, sid, 14.3);
    engineMonitor.sendDCBatterStatusMessage(ENGINE_BATTERY_INSTANCE, sid, 14.1);
    sid++;
  }
}

void sendFuel() {
  static unsigned long lastFuelUpdate=0;
  unsigned long now = millis();
  if ( now > lastFuelUpdate+FUEL_UPDATE_PERIOD ) {
    lastFuelUpdate = now;
    engineMonitor.sendFluidLevelMessage(FUEL_TYPE, FUEL_LEVEL_INSTANCE, 80, 60);

  }
}

void sendTemperatures() {
  static unsigned long lastTempUpdate=0;
  static byte sid = 0;
  unsigned long now = millis();
  if ( now > lastTempUpdate+TEMPERATURE_UPDATE_PERIOD ) {
    lastTempUpdate = now;
    engineMonitor.sendTemperatureMessage(sid, 0, 14,440);
    engineMonitor.sendTemperatureMessage(sid, 1, 3, 350);
    engineMonitor.sendTemperatureMessage(sid, 2, 15, 350);
    sid++;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Example engine monitor start"));
  Serial.println(F("Opening CAN"));
  while (!engineMonitor.open() ) {
     Serial.println(F("Failed to start NMEA2000, retry in 5s"));
    delay(5000);
  }
  Serial.println(F("Opened, MCP2515 Operational"));
  Serial.println(F("Running..."));;
}

void loop() {
  sendRapidEngineData();
  sendEngineData();
  sendVoltages();
  sendTemperatures();
  sendFuel();
  engineMonitor.processMessages();
}


