// VERSION 1.0.0

#include <ArduinoBLE.h>
#include <EEPROM.h>
#include <SFE_MicroOLED.h>
#include <Wire.h>

#include "MAX30105.h"

// -- Constant Values --
#define FIRMWARE_REVISION_STRING "v0.3"

#define PIN_RESET 9
#define DC_JUMPER 1

#define BLE_UUID_ROAST_METER_SERVICE "875A0EE0-03DD-4225-AE06-35E8AE92B84C"
#define BLE_UUID_PARTICLE_SENSOR "C32AFDBA-E9F2-453E-9612-85FBF4108AB2"
#define BLE_UUID_AGTRON "CE216811-0AD9-4AFF-AE29-8B171093A95F"
#define BLE_UUID_METER_STATE "8ACE2828-996F-48E4-8E9C-8284678B4B57"

#define BLE_UUID_DEVICE_INFOMATION_SERVICE "180A"
#define BLE_UUID_FIRMWARE_REVISION "2A26"

#define BLE_UUID_SETTING_SERVICE "59021473-DFC6-425A-9729-09310EBE535E"
#define BLE_UUID_LED_BRIGHTNESS "8313695F-3EA1-458B-BD2A-DF4AEE218514"
#define BLE_UUID_INTERSECTION_POINT "69548C4B-87D0-4E3E-AC6C-B143C7B2AB30"
#define BLE_UUID_DEVIATION "D17234FA-0F48-429A-9E9B-F5DB774EF682"
#define BLE_UUID_COEFFICIENT_0 "C2F27E34-93AD-4445-B008-3B93497E1E3D"
#define BLE_UUID_COEFFICIENT_1 "4B22319F-307F-440B-99B0-A1D6A485262C"
#define BLE_UUID_COEFFICIENT_2 "6FAD0CA7-AA9A-4747-B140-B7C0D14B1110"
#define BLE_UUID_COEFFICIENT_3 "54A41301-4278-4F2B-A42E-9A5576298DA3"
#define BLE_UUID_IR_OFFSET "15DFB217-B7B8-41B3-97F7-8FD154021F29"
#define BLE_UUID_AUTO_CALIBRATION "86B7E111-4D13-448E-91C6-428ED0734CD1"

#define BLE_UUID_BLE_NAME "CDE44FD7-4C1E-42A0-8368-531DC87F6B56"
#define BLE_UUID_UNBLOCK_LEVEL "B8BEFA0C-FFDD-4096-9ACD-208657B4B73C"

#define STATE_SETUP 0
#define STATE_WARMUP 1
#define STATE_READY 2
#define STATE_MEASURED 3

// -- End Constant Values --

// -- EEPROM constants --

#define EEPROM_MAX_LENGTH 256                  // 1024 bytes
#define EEPROM_VALID_IDX 0                     // 1 byte
#define EEPROM_VALID_CODE (0xAA)               // uint8
#define EEPROM_LED_BRIGHTNESS_IDX 1            // 1 byte
#define EEPROM_LED_BRIGHTNESS_DEFAULT 140      // uint8
#define EEPROM_INTERSECTION_POINT_IDX 2        // 1 byte
#define EEPROM_INTERSECTION_POINT_DEFAULT 117  // uint8
#define EEPROM_DEVIATION_IDX 3                 // 1 byte
#define EEPROM_DEVIATION_DEFAULT 0.165f        // float 32 bit 4 bytes
#define EEPROM_COEFFICIENT_0_IDX 7             // 1 byte
#define EEPROM_COEFFICIENT_0_DEFAULT -8.66f    // float 32 bit 4 bytes
#define EEPROM_COEFFICIENT_1_IDX 11            // 1 byte
#define EEPROM_COEFFICIENT_1_DEFAULT 0.773f    // float 32 bit 4 bytes
#define EEPROM_COEFFICIENT_2_IDX 15            // 1 byte
#define EEPROM_COEFFICIENT_2_DEFAULT 0.00284f  // float 32 bit 4 bytes
#define EEPROM_COEFFICIENT_3_IDX 19            // 1 byte
#define EEPROM_COEFFICIENT_3_DEFAULT 0         // float 32 bit 4 bytes
#define EEPROM_IR_OFFSET_IDX 23                // 1 byte
#define EEPROM_IR_OFFSET_DEFAULT 0             // float 32 bit 4 bytes
#define EEPROM_BLE_NAME_IDX 128                // 64 byte - 1 byte length + 63 ASCII

// -- End EEPROM constants

// -- Global Variables --

uint32_t unblockedValue = 30000;  // Average IR at power up

MAX30105 particleSensor;
MicroOLED oled(PIN_RESET, DC_JUMPER);

// -- End Global Variables --

// -- Global Setting --

// The variable below calibrates the LED output on your hardware.
byte ledBrightness;      // !EEPROM setup
byte sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, --32--
byte ledMode = 2;        // Options: 1 = Red only, --2 = Red + IR--, 3 = Red + IR + Green
int sampleRate = 50;     // Options: 50, 100, 200, 400, 800, 1000, 1600, --3200--
int pulseWidth = 411;    // Options: 69, 118, 215, --411--
int adcRange = 16384;    // Options: 2048, 4096, 8192, --16384--

// The variable below use to calculate Agtron from IR
int intersectionPoint = 117;  // !EEPROM setup
float deviation = 0.165;      // !EEPROM setup
float coefficient_0 = -8.66;
float coefficient_1 = 0.773;
float coefficient_2 = 0.00284;
float coefficient_3 = 0;
float irOffset;

// BLE
String bleName;  // !EEPROM setup

// -- End Global Setting --

// -- Setup Headers --

void setupEEPROM();
void setupBLE();
void setupParticleSensor();

// -- Setup Headers --

// -- Sub Routine Headers --

void displayStartUp();
void warmUpLED(int duration);
void measureSampleJob();
void displayPleaseLoadSample();
void displaySensorDirty();
void displayMeasurement(float agtronLevel);

// -- End Sub Routine Headers --

// -- BLE Function Headers --
BLEService roastMeterService(BLE_UUID_ROAST_METER_SERVICE);

BLEUnsignedIntCharacteristic particleSensorCharacteristic(BLE_UUID_PARTICLE_SENSOR, BLERead | BLENotify);
BLEByteCharacteristic agtronCharacteristic(BLE_UUID_AGTRON, BLERead | BLENotify);
BLEByteCharacteristic meterStateCharacteristic(BLE_UUID_METER_STATE, BLERead | BLENotify);

BLEService settingService(BLE_UUID_SETTING_SERVICE);

BLEByteCharacteristic ledBrightnessLevelCharacteristic(BLE_UUID_LED_BRIGHTNESS, BLERead | BLEWrite);
BLEByteCharacteristic intersectionPointCharacteristic(BLE_UUID_INTERSECTION_POINT, BLERead | BLEWrite);
BLEFloatCharacteristic deviationCharacteristic(BLE_UUID_DEVIATION, BLERead | BLEWrite);
BLEFloatCharacteristic coefficient0Characteristic(BLE_UUID_COEFFICIENT_0, BLERead | BLEWrite);
BLEFloatCharacteristic coefficient1Characteristic(BLE_UUID_COEFFICIENT_1, BLERead | BLEWrite);
BLEFloatCharacteristic coefficient2Characteristic(BLE_UUID_COEFFICIENT_2, BLERead | BLEWrite);
BLEFloatCharacteristic coefficient3Characteristic(BLE_UUID_COEFFICIENT_3, BLERead | BLEWrite);
BLEFloatCharacteristic irOffsetCharacteristic(BLE_UUID_IR_OFFSET, BLERead | BLEWrite);
BLEBooleanCharacteristic autoCalibrationCharacteristic(BLE_UUID_AUTO_CALIBRATION, BLERead | BLEWrite);
BLEStringCharacteristic bleNameCharacteristic(BLE_UUID_BLE_NAME, BLERead | BLEWrite, 64);

BLEService deviceInfomationService(BLE_UUID_DEVICE_INFOMATION_SERVICE);

BLEStringCharacteristic firmwareRevisionCharacteristic(BLE_UUID_FIRMWARE_REVISION, BLERead | BLEWrite, 64);

// -- End BLE Function Headers --

// -- BLE Handler Headers --

void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);

void bleLEDBrightnessLevelWritten(BLEDevice central, BLECharacteristic characteristic);
void bleIntersectionPointWritten(BLEDevice central, BLECharacteristic characteristic);
void bleDeviationWritten(BLEDevice central, BLECharacteristic characteristic);
void bleCoefficient0Written(BLEDevice central, BLECharacteristic characteristic);
void bleCoefficient1Written(BLEDevice central, BLECharacteristic characteristic);
void bleCoefficient2Written(BLEDevice central, BLECharacteristic characteristic);
void bleCoefficient3Written(BLEDevice central, BLECharacteristic characteristic);
void bleIROffsetWritten(BLEDevice central, BLECharacteristic characteristic);
void bleAutoCalibrationWritten(BLEDevice central, BLECharacteristic characteristic);
void bleBLENameWritten(BLEDevice central, BLECharacteristic characteristic);

// -- End BLE Handler Headers --

// -- Utility Function Headers --

String multiplyChar(char c, int n);
String stringLastN(String input, int n);
float mapIRToAgtron(int rawIR);
void writeStringToEEPROM(int addrOffset, const String &strToWrite);
String readStringFromEEPROM(int addrOffset);

// -- End Utillity Function Headers --

// -- Main Process --
void setup() {
  Serial.begin(9600);

  BLE.begin();

  Wire.begin();

  oled.begin();      // Initialize the OLED
  oled.clear(ALL);   // Clear the display's internal memory
  oled.clear(PAGE);  // Clear the buffer.

  setupEEPROM();
  setupBLE();

  // Initialize sensor
  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false)  // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }

  setupParticleSensor();

  displayStartUp();
  warmUpLED(0);
}

void loop() {
  measureSampleJob();
  BLE.poll();
}

// -- End Main Process --

// -- Setups --

void setupEEPROM() {
#ifdef ESP32
  // Flash Wrapper using EEPROM API
  EEPROM.begin(EEPROM_MAX_LENGTH);
#endif

#ifdef ARDUINO_ARCH_APOLLO3
  // Actual EEPROM
  EEPROM.init();
  EEPROM.setLength(EEPROM_MAX_LENGTH);
#endif

  // use EEPROM.get(int index, T type) to retrieve
  // an arbitrary type from flash memory
  uint8_t eeprom_valid;
  EEPROM.get(EEPROM_VALID_IDX, eeprom_valid);

  if (eeprom_valid != EEPROM_VALID_CODE) {
    Serial.println("EEPROM was invalid");

    // use EEPROM.put(int index, T type) to store
    // a variable in psuedo-eeprom flash memory
    // use a variable with a type so that EEPROM
    // knows how much memory to use
    uint8_t code_to_store = EEPROM_VALID_CODE;
    EEPROM.put(EEPROM_VALID_IDX, code_to_store);

    // store default LED brightness value in EEPROM
    uint8_t led_brightness_to_store = EEPROM_LED_BRIGHTNESS_DEFAULT;
    EEPROM.put(EEPROM_LED_BRIGHTNESS_IDX, led_brightness_to_store);

    // store default intersection point value in EEPROM
    uint8_t intersection_point_to_store = EEPROM_INTERSECTION_POINT_DEFAULT;
    EEPROM.put(EEPROM_INTERSECTION_POINT_IDX, intersection_point_to_store);

    // store default deviation value in EEPROM
    float deviation_to_store = EEPROM_DEVIATION_DEFAULT;
    EEPROM.put(EEPROM_DEVIATION_IDX, deviation_to_store);

    float coefficient_0_to_store = EEPROM_COEFFICIENT_0_DEFAULT;
    EEPROM.put(EEPROM_COEFFICIENT_0_IDX, coefficient_0_to_store);

    float coefficient_1_to_store = EEPROM_COEFFICIENT_1_DEFAULT;
    EEPROM.put(EEPROM_COEFFICIENT_1_IDX, coefficient_1_to_store);

    float coefficient_2_to_store = EEPROM_COEFFICIENT_2_DEFAULT;
    EEPROM.put(EEPROM_COEFFICIENT_2_IDX, coefficient_2_to_store);

    float coefficient_3_to_store = EEPROM_COEFFICIENT_3_DEFAULT;
    EEPROM.put(EEPROM_COEFFICIENT_3_IDX, coefficient_3_to_store);

    float ir_offset_to_store = EEPROM_IR_OFFSET_DEFAULT;
    EEPROM.put(EEPROM_IR_OFFSET_IDX, ir_offset_to_store);

#ifdef ESP32
    EEPROM.commit();
#endif

    // store default BLE name in EEPROM
    BLE.begin();
    String ble_name_to_store = "Roast Meter " + stringLastN(BLE.address(), 5);
    writeStringToEEPROM(EEPROM_BLE_NAME_IDX, ble_name_to_store);

    Serial.println("EEPROM initialized");
  }

  EEPROM.get(EEPROM_VALID_IDX, eeprom_valid);
  if (eeprom_valid != EEPROM_VALID_CODE) {
    Serial.println("EEPROM CAN NOT initialized");
    while (1) {
    };
  }

  Serial.println("EEPROM is valid");

  // Load setting from EEPROM

  uint8_t eeprom_led_brightness;
  EEPROM.get(EEPROM_LED_BRIGHTNESS_IDX, eeprom_led_brightness);
  ledBrightness = eeprom_led_brightness;
  Serial.println("Set ledBrightness to " + String(ledBrightness));

  uint8_t eeprom_intersection_point;
  EEPROM.get(EEPROM_INTERSECTION_POINT_IDX, eeprom_intersection_point);
  intersectionPoint = eeprom_intersection_point;
  Serial.println("Set intersection point to " + String(intersectionPoint));

  float eeprom_deviation;
  EEPROM.get(EEPROM_DEVIATION_IDX, eeprom_deviation);
  deviation = eeprom_deviation;
  Serial.print("Set deviation to ");
  Serial.print(deviation, 4);
  Serial.println();

  EEPROM.get(EEPROM_COEFFICIENT_0_IDX, coefficient_0);
  Serial.print("Set coefficient_0 to ");
  Serial.println(coefficient_0, 8);

  EEPROM.get(EEPROM_COEFFICIENT_1_IDX, coefficient_1);
  Serial.print("Set coefficient_1 to ");
  Serial.println(coefficient_1, 8);

  EEPROM.get(EEPROM_COEFFICIENT_2_IDX, coefficient_2);
  Serial.print("Set coefficient_2 to ");
  Serial.println(coefficient_2, 8);

  EEPROM.get(EEPROM_COEFFICIENT_3_IDX, coefficient_3);
  Serial.print("Set coefficient_3 to ");
  Serial.println(coefficient_3, 8);

  EEPROM.get(EEPROM_IR_OFFSET_IDX, irOffset);
  Serial.print("Set IR Offset to ");
  Serial.println(irOffset, 3);

  bleName = readStringFromEEPROM(EEPROM_BLE_NAME_IDX);
  Serial.println("Set BLE name to " + String(bleName));
}

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
  }

  // set the local name peripheral advertises
  BLE.setLocalName(bleName.c_str());
  BLE.setDeviceName(bleName.c_str());
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(roastMeterService);

  // add the characteristic to the service
  roastMeterService.addCharacteristic(particleSensorCharacteristic);
  roastMeterService.addCharacteristic(agtronCharacteristic);
  roastMeterService.addCharacteristic(meterStateCharacteristic);

  settingService.addCharacteristic(ledBrightnessLevelCharacteristic);
  settingService.addCharacteristic(intersectionPointCharacteristic);
  settingService.addCharacteristic(deviationCharacteristic);
  settingService.addCharacteristic(coefficient0Characteristic);
  settingService.addCharacteristic(coefficient1Characteristic);
  settingService.addCharacteristic(coefficient2Characteristic);
  settingService.addCharacteristic(coefficient3Characteristic);
  settingService.addCharacteristic(irOffsetCharacteristic);
  settingService.addCharacteristic(autoCalibrationCharacteristic);
  settingService.addCharacteristic(bleNameCharacteristic);

  deviceInfomationService.addCharacteristic(firmwareRevisionCharacteristic);

  // add service
  BLE.addService(roastMeterService);
  BLE.addService(settingService);
  BLE.addService(deviceInfomationService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  ledBrightnessLevelCharacteristic.setEventHandler(BLEWritten, bleLEDBrightnessLevelWritten);

  intersectionPointCharacteristic.setEventHandler(BLEWritten, bleIntersectionPointWritten);

  deviationCharacteristic.setEventHandler(BLEWritten, bleDeviationWritten);

  coefficient0Characteristic.setEventHandler(BLEWritten, bleCoefficient0Written);
  coefficient1Characteristic.setEventHandler(BLEWritten, bleCoefficient1Written);
  coefficient2Characteristic.setEventHandler(BLEWritten, bleCoefficient2Written);
  coefficient3Characteristic.setEventHandler(BLEWritten, bleCoefficient3Written);

  irOffsetCharacteristic.setEventHandler(BLEWritten, bleIROffsetWritten);

  autoCalibrationCharacteristic.setEventHandler(BLEWritten, bleAutoCalibrationWritten);

  bleNameCharacteristic.setEventHandler(BLEWritten, bleBLENameWritten);

  // Assign current value and setting for BLE Characteristic
  particleSensorCharacteristic.setValue(0);
  agtronCharacteristic.setValue(0);
  meterStateCharacteristic.setValue(STATE_SETUP);

  ledBrightnessLevelCharacteristic.setValue(ledBrightness);
  intersectionPointCharacteristic.setValue(intersectionPoint);
  deviationCharacteristic.setValue(deviation);
  coefficient0Characteristic.setValue(coefficient_0);
  coefficient0Characteristic.setValue(coefficient_1);
  coefficient0Characteristic.setValue(coefficient_2);
  coefficient0Characteristic.setValue(coefficient_3);
  irOffsetCharacteristic.setValue(irOffset);

  autoCalibrationCharacteristic.setValue(false);

  bleNameCharacteristic.setValue(bleName);

  firmwareRevisionCharacteristic.writeValue(FIRMWARE_REVISION_STRING);

  // start advertising
  BLE.advertise();

  Serial.println(("Bluetooth® device active, waiting for connections..."));
}

void setupParticleSensor() {
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);  // Configure sensor with these settings

  particleSensor.setPulseAmplitudeRed(34);
  particleSensor.setPulseAmplitudeGreen(0);

  particleSensor.disableSlots();
  particleSensor.enableSlot(2, 0x02);  // Enable only SLOT_IR_LED = 0x02
}

// -- End Setups --

// Sub Routines

void displayStartUp() {
  oled.clear(PAGE);
  oled.setCursor(0, 0);
  oled.setFontType(1);
  oled.println("Roast");
  oled.println("Meter");
  oled.print(FIRMWARE_REVISION_STRING);
  oled.display();

  delay(2000);

  oled.clear(PAGE);
  oled.setCursor(0, 0);
  oled.setFontType(1);
  oled.print(bleName);
  oled.display();

  delay(2000);
}

void warmUpLED(int duration) {
  if (duration <= 0)
    return;

  meterStateCharacteristic.writeValue(STATE_WARMUP);

  int countDownSeconds = duration;
  unsigned long jobTimerStart = millis();
  unsigned long jobTimer = jobTimerStart;

  while (millis() - jobTimerStart <= duration * 1000) {
    unsigned long elapsed = millis() - jobTimer;

    if (elapsed > 100) {
      oled.clear(PAGE);
      oled.setCursor(0, 0);
      oled.setFontType(1);

      countDownSeconds = duration - ((millis() - jobTimerStart) / 1000);

      oled.print("Warm Up " + String(countDownSeconds) + "s");
      oled.display();

      jobTimer = millis();
    }

    BLE.poll();
  }
}

int irLevelAccumulated;
unsigned long measureSampleJobTimer = millis();
unsigned long bleUpdateAndNotifyTimer = millis();
void measureSampleJob() {
  if (millis() - measureSampleJobTimer > 100) {
    int irLevel = particleSensor.getIR();
    long currentDelta = irLevel - unblockedValue;
    irLevelAccumulated = (irLevelAccumulated * 0.5) + (irLevel * 0.5);

    if (currentDelta > 0) {
      float calibratedAgtronLevel = mapIRToAgtron(irLevelAccumulated);

      if (millis() - bleUpdateAndNotifyTimer > 1000) {
        bleUpdateAndNotifyTimer = millis();

        agtronCharacteristic.writeValue(calibratedAgtronLevel);
        particleSensorCharacteristic.writeValue((u_int32_t)irLevel);
        meterStateCharacteristic.writeValue(STATE_MEASURED);
      }

      displayMeasurement(calibratedAgtronLevel);

      Serial.println("real: " + String(irLevelAccumulated));
      Serial.print("agtron: ");
      Serial.println(calibratedAgtronLevel);
      Serial.println("===========================");
    } else {
      agtronCharacteristic.writeValue(0);
      particleSensorCharacteristic.writeValue(0);
      meterStateCharacteristic.writeValue(STATE_READY);

      if (irLevel > 7000) {
        displaySensorDirty();
      } else {
        displayPleaseLoadSample();
      }
    }

    measureSampleJobTimer = millis();
  }
}

void displayPleaseLoadSample() {
  oled.clear(PAGE);
  oled.setCursor(0, 0);
  oled.setFontType(1);
  oled.print("Please load   sample!");
  oled.display();
}

void displaySensorDirty() {
  oled.clear(PAGE);
  oled.setCursor(0, 0);
  oled.setFontType(1);
  oled.println("Please");
  oled.println("Clean");
  oled.println("Sensor!");
  oled.display();
}

String agtronDescription(float agtronLevel) {
  if (agtronLevel <= 20) return String("Over   Develop");
  if (agtronLevel <= 30) return String("Very   Dark");
  if (agtronLevel < 40) return String("Dark");
  if (agtronLevel < 50) return String("Medium Dark");
  if (agtronLevel < 60) return String("Medium");
  if (agtronLevel < 70) return String("Medium Light");
  if (agtronLevel < 80) return String("Light");
  if (agtronLevel < 90) return String("Very   Light");
  if (agtronLevel < 100) return String("Extremely Light");

  return String("Under  Develop");
}

void displayMeasurement(float agtronLevel) {
  oled.clear(PAGE);
  oled.setCursor(0, 0);

  oled.setFontType(2);
  oled.println(agtronLevel, 1);

  oled.setCursor(0, 20);
  oled.setFontType(1);
  oled.println(agtronDescription(agtronLevel));

  oled.display();
}

// -- End Sub Routines --

// -- BLE Handler --

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("BLE Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("BLE Disconnected event, central: ");
  Serial.println(central.address());
}

void bleLEDBrightnessLevelWritten(BLEDevice central, BLECharacteristic characteristic) {
  ledBrightness = ledBrightnessLevelCharacteristic.value();

  Serial.print("bleLEDBrightnessLevelWritten event, written: ");
  Serial.println(ledBrightness);

  EEPROM.put(EEPROM_LED_BRIGHTNESS_IDX, ledBrightness);

#ifdef ESP32
  EEPROM.commit();
#endif

  setupParticleSensor();
}

void bleIntersectionPointWritten(BLEDevice central, BLECharacteristic characteristic) {
  intersectionPoint = intersectionPointCharacteristic.value();

  Serial.print("bleIntersectionPointWritten event, written: ");
  Serial.println(intersectionPoint);

  EEPROM.put(EEPROM_INTERSECTION_POINT_IDX, intersectionPoint);

#ifdef ESP32
  EEPROM.commit();
#endif
}

void bleDeviationWritten(BLEDevice central, BLECharacteristic characteristic) {
  deviation = deviationCharacteristic.value();

  Serial.print("bleDeviationWritten event, written: ");
  Serial.println(deviation);

  EEPROM.put(EEPROM_DEVIATION_IDX, deviation);

#ifdef ESP32
  EEPROM.commit();
#endif
}

void bleCoefficient0Written(BLEDevice central, BLECharacteristic characteristic) {
  coefficient_0 = coefficient0Characteristic.value();

  Serial.print("bleCoefficient0Written event, written: ");
  Serial.println(coefficient_0);

  EEPROM.put(EEPROM_COEFFICIENT_0_IDX, coefficient_0);

#ifdef ESP32
  EEPROM.commit();
#endif
}

void bleCoefficient1Written(BLEDevice central, BLECharacteristic characteristic) {
  coefficient_1 = coefficient0Characteristic.value();

  Serial.print("bleCoefficient1Written event, written: ");
  Serial.println(coefficient_1);

  EEPROM.put(EEPROM_COEFFICIENT_1_IDX, coefficient_1);

#ifdef ESP32
  EEPROM.commit();
#endif
}

void bleCoefficient2Written(BLEDevice central, BLECharacteristic characteristic) {
  coefficient_2 = coefficient2Characteristic.value();

  Serial.print("bleCoefficient0Written event, written: ");
  Serial.println(coefficient_2);

  EEPROM.put(EEPROM_COEFFICIENT_2_IDX, coefficient_2);

#ifdef ESP32
  EEPROM.commit();
#endif
}

void bleCoefficient3Written(BLEDevice central, BLECharacteristic characteristic) {
  coefficient_3 = coefficient3Characteristic.value();

  Serial.print("bleCoefficient3Written event, written: ");
  Serial.println(coefficient_3);

  EEPROM.put(EEPROM_COEFFICIENT_3_IDX, coefficient_3);

#ifdef ESP32
  EEPROM.commit();
#endif
}

void bleIROffsetWritten(BLEDevice central, BLECharacteristic characteristic) {
  irOffset = irOffsetCharacteristic.value();

  Serial.print("bleIROffsetWritten event, written: ");
  Serial.println(irOffset);

  EEPROM.put(EEPROM_IR_OFFSET_IDX, irOffset);

#ifdef ESP32
  EEPROM.commit();
#endif
}

void bleAutoCalibrationWritten(BLEDevice central, BLECharacteristic characteristic) {
  irOffset = irOffsetCharacteristic.value();

  Serial.print("bleAutoCalibrationWritten event, perform Auto Calibration");
}

void bleBLENameWritten(BLEDevice central, BLECharacteristic characteristic) {
  String newBLEName = bleNameCharacteristic.value();

  if (newBLEName.length() > 63) {
    Serial.println("bleBLENameWritten event, written rejected!. String length exceed 63.");
    bleNameCharacteristic.setValue(bleName.c_str());

    return;
  }

  bleName = newBLEName;
  Serial.print("bleBLENameWritten event, written: ");
  Serial.println(bleName);

  BLE.setLocalName(bleName.c_str());
  BLE.setDeviceName(bleName.c_str());

  writeStringToEEPROM(EEPROM_BLE_NAME_IDX, bleName);
}

// -- End BLE Handler --

// -- Utillity Functions --

String multiplyChar(char c, int n) {
  String result = "";
  for (int i = 0; i < n; i++) {
    result += c;
  }
  return result;
}

String stringLastN(String input, int n) {
  int inputSize = input.length();

  return (n > 0 && inputSize > n) ? input.substring(inputSize - n) : "";
}

float mapIRToAgtron(int rawIR) {
  float x = (float)rawIR / 1000;
  float agtron = coefficient_0;

  if (coefficient_1 != 0) agtron += coefficient_1 * x;
  if (coefficient_2 != 0) agtron += coefficient_2 * x * x;
  if (coefficient_3 != 0) agtron += coefficient_3 * x * x * x;

  return agtron;
}

// https://roboticsbackend.com/arduino-write-string-in-eeprom/
void writeStringToEEPROM(int addrOffset, const String &strToWrite) {
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++) {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }

#ifdef ESP32
  EEPROM.commit();
#endif
}

String readStringFromEEPROM(int addrOffset) {
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];

  for (int i = 0; i < newStrLen; i++) {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0';

  return String(data);
}

// -- End Utillity Functions --