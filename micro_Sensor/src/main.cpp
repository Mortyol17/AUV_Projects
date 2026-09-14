#include "DFRobot_BNO055.h"
#include "Wire.h"
#include <TinyGPS++.h>
#include <Adafruit_NeoPixel.h>
#include <driver/twai.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>

#define RGB_LED_PIN 48

Adafruit_NeoPixel statusLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

void showStatusColor(uint32_t color)
{
  statusLed.setPixelColor(0, color);
  statusLed.show();
}

// =====================================================
// TWAI CAN BUS (MCP2551 Transceiver)
// =====================================================
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

bool twaiConnected = false;

// =====================================================
// WATCHDOG TIMER (3 Second Timeout)
// =====================================================
#define WDT_TIMEOUT_SEC 3

void setupTWAI()
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 15; // Increased queue capacity for 7+ frame bursts

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI driver installed");
  } else {
    Serial.println("Failed to install TWAI driver");
    return;
  }

  if (twai_start() == ESP_OK) {
    twaiConnected = true;
    Serial.println("TWAI CAN Driver initialized at 500 kbps");
  } else {
    Serial.println("Failed to start TWAI driver");
  }
}

void setupWatchdog()
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WDT_TIMEOUT_SEC * 1000,
      .idle_core_mask = 0,
      .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&twdt_config);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);
#endif
  Serial.println("Watchdog Timer initialized (3s timeout)");
}

bool sendCanFrame(uint32_t identifier, const uint8_t *data, uint8_t length)
{
  if (!twaiConnected) return false;

  // Auto-recover if TWAI enters BUS_OFF state
  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK) {
    if (status.state == TWAI_STATE_BUS_OFF) {
      Serial.println("[TWAI] BUS_OFF detected! Initiating recovery...");
      twai_initiate_recovery();
      vTaskDelay(pdMS_TO_TICKS(10));
      twai_start();
      return false;
    }
  }

  twai_message_t message;
  message.identifier = identifier;
  message.extd = 0; // Standard 11-bit CAN ID
  message.rtr = 0;  // Data frame
  message.data_length_code = length;
  memcpy(message.data, data, length);

  esp_err_t res = twai_transmit(&message, pdMS_TO_TICKS(10));
  return (res == ESP_OK);
}

// =====================================================
// BNO055 I2C
// =====================================================

#define SDA_PIN 38
#define SCL_PIN 39

typedef DFRobot_BNO055_IIC BNO;

BNO bno(&Wire, 0x28);
bool bnoConnected = false;


// =====================================================
// GPS ATGM336H
// =====================================================

// GPS TX -> ESP32 GPIO 17 (RX)
// GPS RX -> ESP32 GPIO 18 (TX)

#define GPS_RX_PIN 18
#define GPS_TX_PIN 17

HardwareSerial GPS(1);

TinyGPSPlus gps;

// =====================================================
// UNDERWATER ULTRASONIC SENSOR (UART)
// =====================================================
// UW Sonar TX -> ESP32 GPIO 16 (RX2)
// UW Sonar RX -> ESP32 GPIO 15 (TX2)

#define UW_SONAR_RX_PIN 16
#define UW_SONAR_TX_PIN 15

HardwareSerial SonarSerial(2);
float uwDistanceMm = 0.0f;
bool uwValid = false;
unsigned long lastUwReadTime = 0;

void readUnderwaterSonar()
{
  static uint8_t buffer[4];
  static uint8_t bufIndex = 0;

  while (SonarSerial.available()) {
    uint8_t b = SonarSerial.read();
    if (bufIndex == 0) {
      if (b == 0xFF) {
        buffer[0] = b;
        bufIndex = 1;
      }
    } else {
      buffer[bufIndex++] = b;
      if (bufIndex == 4) {
        bufIndex = 0;
        uint8_t checksum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;
        if (checksum == buffer[3]) {
          uint16_t dist = (buffer[1] << 8) | buffer[2];
          uwDistanceMm = static_cast<float>(dist);
          uwValid = true;
          lastUwReadTime = millis();
        }
      }
    }
  }

  if (millis() - lastUwReadTime > 2000) {
    uwValid = false;
  }
}

float readAnalogDepthMm();

struct SensorPacket {
  uint32_t sequence;
  float depthMm;
  float distanceMm;
  float latitude;
  float longitude;
  float altitudeM;
  float speedKmph;
  float hdop;
  float pitch;
  float roll;
  float yaw;
  uint32_t satellites;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t gpsValid;
  uint8_t bnoValid;
  uint8_t uwValid;
};

SensorPacket sensorPacket = {};

void sendCanSensorData()
{
  if (!twaiConnected) return;

  // ---------------------------------------------------
  // CAN ID 0x100: System Monitoring & Health
  // ---------------------------------------------------
  uint8_t sysFrame[8] = {0};
  sysFrame[0] = static_cast<uint8_t>(sensorPacket.sequence & 0xFF);
  
  // Health Bitfield:
  // Bit 0: WDT active (1)
  // Bit 1: GPS valid
  // Bit 2: BNO valid
  // Bit 3: TWAI OK
  // Bit 4: ESP-NOW status
  // Bit 5: Underwater Sonar valid
  uint8_t healthFlags = 0x01; // WDT running
  if (sensorPacket.gpsValid) healthFlags |= (1 << 1);
  if (sensorPacket.bnoValid) healthFlags |= (1 << 2);
  if (twaiConnected)         healthFlags |= (1 << 3);
  if (sensorPacket.uwValid)  healthFlags |= (1 << 5);
  sysFrame[1] = healthFlags;

  // Free Heap in KB
  sysFrame[2] = static_cast<uint8_t>(ESP.getFreeHeap() / 1024);

  // TWAI TX Error Counter
  twai_status_info_t twaiStatus;
  if (twai_get_status_info(&twaiStatus) == ESP_OK) {
    sysFrame[3] = static_cast<uint8_t>(twaiStatus.tx_error_counter & 0xFF);
  } else {
    sysFrame[3] = 0;
  }

  // System Uptime in ms (4 bytes)
  uint32_t uptimeMs = millis();
  memcpy(&sysFrame[4], &uptimeMs, sizeof(uptimeMs));

  sendCanFrame(0x100, sysFrame, 8);

  // ---------------------------------------------------
  // CAN ID 0x101: Voltage-based Depth Sensor Data
  // ---------------------------------------------------
  uint8_t depthFrame[8] = {0};
  memcpy(&depthFrame[0], &sensorPacket.depthMm, sizeof(sensorPacket.depthMm));
  sendCanFrame(0x101, depthFrame, 8);

  // ---------------------------------------------------
  // CAN ID 0x102: GPS Position Data (Lat/Lng)
  // ---------------------------------------------------
  uint8_t gpsPosFrame[8] = {0};
  memcpy(&gpsPosFrame[0], &sensorPacket.latitude, sizeof(sensorPacket.latitude));
  memcpy(&gpsPosFrame[4], &sensorPacket.longitude, sizeof(sensorPacket.longitude));
  sendCanFrame(0x102, gpsPosFrame, 8);

  // ---------------------------------------------------
  // CAN ID 0x103: GPS Telemetry & Time
  // ---------------------------------------------------
  uint8_t gpsTelemFrame[8] = {0};
  uint16_t speed10 = static_cast<uint16_t>(sensorPacket.speedKmph * 10.0f);
  int16_t altM = static_cast<int16_t>(sensorPacket.altitudeM);
  uint8_t sats = static_cast<uint8_t>(sensorPacket.satellites);
  uint8_t hdop10 = static_cast<uint8_t>(sensorPacket.hdop * 10.0f);
  
  memcpy(&gpsTelemFrame[0], &speed10, sizeof(speed10));
  memcpy(&gpsTelemFrame[2], &altM, sizeof(altM));
  gpsTelemFrame[4] = sats;
  gpsTelemFrame[5] = hdop10;
  gpsTelemFrame[6] = sensorPacket.hour;
  gpsTelemFrame[7] = sensorPacket.minute;
  sendCanFrame(0x103, gpsTelemFrame, 8);

  // ---------------------------------------------------
  // CAN ID 0x104: BNO055 IMU Orientation
  // ---------------------------------------------------
  uint8_t imuFrame[8] = {0};
  int16_t pitch100 = static_cast<int16_t>(sensorPacket.pitch * 100.0f);
  int16_t roll100 = static_cast<int16_t>(sensorPacket.roll * 100.0f);
  int16_t yaw100 = static_cast<int16_t>(sensorPacket.yaw * 100.0f);

  memcpy(&imuFrame[0], &pitch100, sizeof(pitch100));
  memcpy(&imuFrame[2], &roll100, sizeof(roll100));
  memcpy(&imuFrame[4], &yaw100, sizeof(yaw100));
  sendCanFrame(0x104, imuFrame, 8);

  // ---------------------------------------------------
  // CAN ID 0x105: Underwater Ultrasonic Distance Data
  // ---------------------------------------------------
  uint8_t uwFrame[8] = {0};
  memcpy(&uwFrame[0], &sensorPacket.distanceMm, sizeof(sensorPacket.distanceMm));
  sendCanFrame(0x105, uwFrame, 8);

  // ---------------------------------------------------
  // CAN ID 0x106: Reserved Future Sensor ID
  // ---------------------------------------------------
  uint8_t reservedFrame[8] = {0};
  sendCanFrame(0x106, reservedFrame, 8);

  Serial.println("CAN Bus: Broadcasted frames 0x100 - 0x106");
}

void sendSensorData()
{
  sensorPacket.sequence++;
  sensorPacket.depthMm = readAnalogDepthMm();
  sensorPacket.distanceMm = uwDistanceMm;
  sensorPacket.uwValid = uwValid ? 1 : 0;
  sensorPacket.gpsValid = gps.location.isValid();
  sensorPacket.latitude = gps.location.lat();
  sensorPacket.longitude = gps.location.lng();
  sensorPacket.altitudeM = gps.altitude.meters();
  sensorPacket.speedKmph = gps.speed.kmph();
  sensorPacket.hdop = gps.hdop.hdop();
  sensorPacket.satellites = gps.satellites.value();
  sensorPacket.year = gps.date.year();
  sensorPacket.month = gps.date.month();
  sensorPacket.day = gps.date.day();
  sensorPacket.hour = gps.time.hour();
  sensorPacket.minute = gps.time.minute();
  sensorPacket.second = gps.time.second();
  sensorPacket.bnoValid = bnoConnected;

  if (bnoConnected) {
    BNO::sEulAnalog_t eul = bno.getEul();
    sensorPacket.pitch = eul.pitch;
    sensorPacket.roll = eul.roll;
    sensorPacket.yaw = eul.head;
  }

  sendCanSensorData();
}


// =====================================================
// 4-20 mA VOLTAGE-BASED DEPTH SENSOR
// =====================================================

#define SENSOR_PIN 13

constexpr float SENSOR_MAX_VOLTAGE = 3.3f;
constexpr float SENSOR_MAX_DISTANCE_MM = 5000.0f;
constexpr uint8_t SENSOR_SAMPLE_COUNT = 8;

float readAnalogDepthMm()
{
  uint32_t rawTotal = 0;

  for (uint8_t sample = 0; sample < SENSOR_SAMPLE_COUNT; sample++) {
    rawTotal += analogRead(SENSOR_PIN);
  }

  float rawAverage = static_cast<float>(rawTotal) / SENSOR_SAMPLE_COUNT;
  float voltage = rawAverage * SENSOR_MAX_VOLTAGE / 4095.0f;
  voltage = constrain(voltage, 0.0f, SENSOR_MAX_VOLTAGE);

  return voltage * SENSOR_MAX_DISTANCE_MM / SENSOR_MAX_VOLTAGE;
}


// =====================================================
// BNO055 STATUS
// =====================================================

void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch(eStatus) {

    case BNO::eStatusOK:
      Serial.println("OK");
      break;

    case BNO::eStatusErr:
      Serial.println("Unknown error");
      break;

    case BNO::eStatusErrDeviceNotDetect:
      Serial.println("Device not detected");
      break;

    case BNO::eStatusErrDeviceReadyTimeOut:
      Serial.println("Device ready timeout");
      break;

    case BNO::eStatusErrDeviceStatus:
      Serial.println("Device internal status error");
      break;

    default:
      Serial.println("Unknown status");
      break;
  }
}


// =====================================================
// PRINT GPS DATA
// =====================================================

void printGPSData()
{
  Serial.println();
  Serial.println("========== GPS ==========");

  // GPS FIX
  Serial.print("Fix: ");

  if (gps.location.isValid()) {
    Serial.println("YES");
  } 
  else {
    Serial.println("NO");
  }


  // LATITUDE
  Serial.print("Latitude: ");

  if (gps.location.isValid()) {
    Serial.println(gps.location.lat(), 6);
  } 
  else {
    Serial.println("INVALID");
  }


  // LONGITUDE
  Serial.print("Longitude: ");

  if (gps.location.isValid()) {
    Serial.println(gps.location.lng(), 6);
  } 
  else {
    Serial.println("INVALID");
  }


  // ALTITUDE
  Serial.print("Altitude: ");

  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters(), 2);
    Serial.println(" m");
  } 
  else {
    Serial.println("INVALID");
  }


  // SPEED
  Serial.print("Speed: ");

  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph(), 2);
    Serial.println(" km/h");
  } 
  else {
    Serial.println("INVALID");
  }


  // SATELLITES
  Serial.print("Satellites: ");

  if (gps.satellites.isValid()) {
    Serial.println(gps.satellites.value());
  } 
  else {
    Serial.println("INVALID");
  }


  // HDOP
  Serial.print("HDOP: ");

  if (gps.hdop.isValid()) {
    Serial.println(gps.hdop.hdop(), 2);
  } 
  else {
    Serial.println("INVALID");
  }


  // DATE
  Serial.print("Date: ");

  if (gps.date.isValid()) {

    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());

  } 
  else {
    Serial.println("INVALID");
  }


  // TIME
  Serial.print("UTC Time: ");

  if (gps.time.isValid()) {

    if (gps.time.hour() < 10) Serial.print("0");
    Serial.print(gps.time.hour());
    Serial.print(":");

    if (gps.time.minute() < 10) Serial.print("0");
    Serial.print(gps.time.minute());
    Serial.print(":");

    if (gps.time.second() < 10) Serial.print("0");
    Serial.println(gps.time.second());

  } 
  else {
    Serial.println("INVALID");
  }

  Serial.println("=========================");
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(2000);

  statusLed.begin();
  statusLed.clear();
  statusLed.show();

  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);

  Serial.println();
  Serial.println("==============================");
  Serial.println("BNO055 + ATGM336H TEST");
  Serial.println("==============================");


  // ===================================================
  // BNO055
  // ===================================================

  Wire.begin(SDA_PIN, SCL_PIN, 100000);

  Serial.println("I2C initialized");

  Serial.println("Starting BNO055...");

  BNO::eStatus_t status = bno.begin();

  if (status != BNO::eStatusOK) {

    Serial.print("BNO055 ERROR: ");
    printLastOperateStatus(status);
    Serial.println("Continuing without BNO055...");
  }
  else {
    bnoConnected = true;
    Serial.println("BNO055 connected!");
  }


  // ===================================================
  // GPS
  // ===================================================

  Serial.println("Starting GPS...");

  GPS.begin(
    9600,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );

  Serial.println("GPS serial initialized");
  Serial.println("Waiting for GPS fix...");
  Serial.println();

  // ===================================================
  // UNDERWATER ULTRASONIC SENSOR
  // ===================================================
  Serial.println("Starting Underwater Ultrasonic Sonar...");
  SonarSerial.begin(
    9600,
    SERIAL_8N1,
    UW_SONAR_RX_PIN,
    UW_SONAR_TX_PIN
  );
  Serial.println("Underwater Ultrasonic serial initialized");
  Serial.println();

  // Initialize TWAI CAN Bus (MCP2551 Transceiver)
  setupTWAI();

  // Initialize 1-Second Task Watchdog Timer
  setupWatchdog();
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // Reset Watchdog Timer on every loop iteration
  esp_task_wdt_reset();

  // Read Underwater Ultrasonic UART stream continuously
  readUnderwaterSonar();

  // ===================================================
  // READ GPS
  // ===================================================

  while (GPS.available()) {

    char c = GPS.read();

    gps.encode(c);
  }


  // ===================================================
  // PRINT SENSOR DATA EVERY 1 SECOND
  // ===================================================

  static unsigned long lastGPSPrint = 0;

  if (millis() - lastGPSPrint >= 1000) {

    lastGPSPrint = millis();

    printGPSData();

    float analogDepthMm = readAnalogDepthMm();

    Serial.println();
    Serial.println("========== ANALOG DEPTH SENSOR ==========");
    Serial.print("Depth: ");
    Serial.print(analogDepthMm, 1);
    Serial.println(" mm");
    Serial.println("=========================================");

    Serial.println();
    Serial.println("========== UNDERWATER ULTRASONIC ==========");
    Serial.print("Distance: ");
    if (uwValid) {
      Serial.print(uwDistanceMm, 1);
      Serial.println(" mm");
    } else {
      Serial.println("NO DATA / TIMEOUT");
    }
    Serial.println("===========================================");

    sendSensorData();
  }


  // ===================================================
  // READ BNO055
  // ===================================================

  if (bnoConnected) {
    BNO::sEulAnalog_t eul = bno.getEul();

    Serial.println();
    Serial.println("========== BNO055 ==========");

    Serial.print("Pitch: ");
    Serial.print(eul.pitch, 2);

    Serial.print(" | Roll: ");
    Serial.print(eul.roll, 2);

    Serial.print(" | Yaw: ");
    Serial.println(eul.head, 2);

    Serial.println("============================");
  }

  delay(100);
}