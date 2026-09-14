#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <WiFi.h>
#include <esp_now.h>
#include <driver/twai.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include <USB.h>
#include <USBMSC.h>
#include <Adafruit_NeoPixel.h>

// =====================================================
// PIN DEFINITIONS FOR ESP32-S3
// =====================================================
#define SD_CS_PIN   6
#define SD_SCK_PIN  7
#define SD_MOSI_PIN 15
#define SD_MISO_PIN 16

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

#define RGB_LED_PIN 48

#define WDT_TIMEOUT_SEC     5
#define CAN_TIMEOUT_MS      3000

// =====================================================
// SENSOR DATA PACKET STRUCTURE (Matches micro_Sensor)
// =====================================================
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

// Queue Item for Asynchronous FreeRTOS Logging
struct LogMessage {
  SensorPacket packet;
  char source[8];
};

// =====================================================
// BRIDGE COMMUNICATION PACKET STRUCTURES
// These are shared between micro_Supervisory (TX) and
// the Bridge ESP32 (RX) via ESP-NOW.
// =====================================================
#define BPKT_SENSOR   0x01  // Supervisory -> Bridge: Sensor telemetry
#define BPKT_ACTUATOR 0x02  // Supervisory -> Bridge: Actuator status
#define BPKT_COMMAND  0x03  // Bridge      -> Supervisory: Control command

// Command sub-types
#define CMD_STEPPER    1
#define CMD_BTS        2
#define CMD_HOME_YAW   3
#define CMD_HOME_PITCH 4
#define CMD_MODE       5

// Packed sensor broadcast sent via ESP-NOW to Bridge
struct __attribute__((packed)) BridgeSensorPkt {
  uint8_t  type;         // = BPKT_SENSOR
  uint32_t sequence;
  float    depthMm;
  float    distanceMm;
  float    latitude;
  float    longitude;
  float    pitch;
  float    roll;
  float    yaw;
  uint8_t  imuValid;
  uint8_t  gpsValid;
  uint8_t  uwValid;
};

// Packed actuator status sent via ESP-NOW to Bridge
struct __attribute__((packed)) BridgeActuatorPkt {
  uint8_t  type;    // = BPKT_ACTUATOR
  int32_t  pos1;    // Stepper 1 (Yaw) current step position
  int32_t  pos2;    // Stepper 2 (Pitch) current step position
  int8_t   btsSpeed;
};

// Packed command received via ESP-NOW from Bridge
struct __attribute__((packed)) BridgeCommandPkt {
  uint8_t  type;     // = BPKT_COMMAND
  uint8_t  cmdType;  // CMD_STEPPER, CMD_BTS, CMD_HOME_*, CMD_MODE
  int32_t  s1;       // Stepper 1 target step (for CMD_STEPPER)
  int32_t  s2;       // Stepper 2 target step (for CMD_STEPPER)
  int8_t   btsSpeed; // BTS speed -100..100 (for CMD_BTS)
  uint8_t  mode;     // 0=AUTO, 1=MANUAL (for CMD_MODE)
};

// =====================================================
// GLOBAL PERIPHERALS & FREERTOS HANDLES
// =====================================================
SPIClass sdSpi(FSPI);
USBMSC msc;
Adafruit_NeoPixel statusLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

QueueHandle_t sdLogQueue = NULL;
SemaphoreHandle_t sdMutex = NULL;

// Colors
const uint32_t COLOR_OFF   = statusLed.Color(0, 0, 0);
const uint32_t COLOR_RED   = statusLed.Color(120, 0, 0);
const uint32_t COLOR_GREEN = statusLed.Color(0, 120, 0);
const uint32_t COLOR_BLUE  = statusLed.Color(0, 0, 120);
const uint32_t COLOR_WHITE = statusLed.Color(120, 120, 120);

// Global States
bool sdInitialized   = false;
bool twaiInitialized = false;
bool canOnline       = false;
bool espNowReady     = false;

uint32_t lastCanRxTime = 0;

// Sensor data (updated by canTask and espNowTask)
SensorPacket latestSensorPkt = {};

// Actuator status (updated by canTask from CAN 0x202)
BridgeActuatorPkt latestActuatorPkt = {BPKT_ACTUATOR, 0, 0, 0};

// ESP-NOW fallback from micro_Sensor
SensorPacket espNowBuffer;
volatile bool newEspNowData = false;

// Pending command from Bridge ESP32
volatile BridgeCommandPkt pendingCommand = {};
volatile bool newBridgeCommand = false;

// Broadcast peer (FF:FF:FF:FF:FF:FF)
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// SD Log file
const char* LOG_FILENAME = "/datalog.csv";

// System Status Enum for LED
enum SystemLEDState {
    LED_STATE_OK_CAN,
    LED_STATE_OK_ESPNOW,
    LED_STATE_ERR_CAN_TIMEOUT,
    LED_STATE_ERR_SD_FAIL
};

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================
void setupSD();
void setupUSBMSC();
void setupTWAI();
void setupESPNow();
void setupWatchdog();
void updateLEDStatus();
void logDataToSD(const SensorPacket& pkt, const char* source);
void forwardCommandToCAN(const BridgeCommandPkt& cmd);

void canTask(void *pvParameters);
void sdLogTask(void *pvParameters);
void espNowTask(void *pvParameters);
void systemMonitorTask(void *pvParameters);
void broadcastTask(void *pvParameters);

// =====================================================
// NON-BLOCKING LED STATUS SYSTEM
// =====================================================
void updateLEDStatus()
{
    static unsigned long lastPatternTime = 0;
    static int patternStep = 0;
    unsigned long now = millis();

    SystemLEDState currentState;
    if (!sdInitialized) {
        currentState = LED_STATE_ERR_SD_FAIL;
    } else if (!canOnline) {
        currentState = espNowReady ? LED_STATE_OK_ESPNOW : LED_STATE_ERR_CAN_TIMEOUT;
    } else {
        currentState = LED_STATE_OK_CAN;
    }

    switch (currentState) {
        case LED_STATE_ERR_CAN_TIMEOUT:
            if (patternStep < 10) {
                if (now - lastPatternTime >= 100) {
                    lastPatternTime = now; patternStep++;
                    statusLed.setPixelColor(0, (patternStep % 2 != 0) ? COLOR_RED : COLOR_OFF);
                    statusLed.show();
                }
            } else {
                if (now - lastPatternTime >= 1000) { lastPatternTime = now; patternStep = 0; }
            }
            break;
        case LED_STATE_ERR_SD_FAIL:
            if (now - lastPatternTime >= 500) {
                lastPatternTime = now;
                patternStep = (patternStep + 1) % 2;
                statusLed.setPixelColor(0, patternStep ? COLOR_WHITE : COLOR_OFF);
                statusLed.show();
            }
            break;
        case LED_STATE_OK_CAN:
            if (now - lastPatternTime >= 1000) {
                lastPatternTime = now; patternStep = 1;
                statusLed.setPixelColor(0, COLOR_GREEN); statusLed.show();
            } else if (patternStep == 1 && (now - lastPatternTime >= 50)) {
                patternStep = 0;
                statusLed.setPixelColor(0, COLOR_OFF); statusLed.show();
            }
            break;
        case LED_STATE_OK_ESPNOW:
            if (now - lastPatternTime >= 1000) {
                lastPatternTime = now; patternStep = 1;
                statusLed.setPixelColor(0, COLOR_BLUE); statusLed.show();
            } else if (patternStep == 1 && (now - lastPatternTime >= 50)) {
                patternStep = 0;
                statusLed.setPixelColor(0, COLOR_OFF); statusLed.show();
            }
            break;
    }
}

// =====================================================
// USB MASS STORAGE CALLBACKS
// =====================================================
static int32_t onMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
    if (!sdInitialized) return -1;
    return SD.readRAW((uint8_t*)buffer, lba) ? bufsize : -1;
}
static int32_t onMscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
    if (!sdInitialized) return -1;
    return SD.writeRAW(buffer, lba) ? bufsize : -1;
}
static bool onMscStartStop(uint8_t power_condition, bool start, bool load_eject) { return true; }

void setupUSBMSC()
{
    if (!sdInitialized) return;
    msc.vendorID("ESP32-S3");
    msc.productID("AUV Logger");
    msc.productRevision("2.0");
    msc.onRead(onMscRead);
    msc.onWrite(onMscWrite);
    msc.onStartStop(onMscStartStop);
    msc.begin(SD.numSectors(), SD.sectorSize());
    USB.begin();
    Serial.println("[OK] USB Mass Storage Ready.");
}

// =====================================================
// ESP-NOW RECEIVE CALLBACK
// Handles: SensorPacket from micro_Sensor (fallback)
//          BridgeCommandPkt from Bridge ESP32
// =====================================================
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len)
#else
void onDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len)
#endif
{
    if (len <= 0) return;

    if (len == sizeof(SensorPacket)) {
        // Packet from micro_Sensor via ESP-NOW (CAN fallback, no type prefix)
        memcpy(&espNowBuffer, incomingData, sizeof(SensorPacket));
        newEspNowData = true;

    } else if (len == sizeof(BridgeCommandPkt) && incomingData[0] == BPKT_COMMAND) {
        // Command packet from Bridge ESP32
        memcpy((void*)&pendingCommand, incomingData, sizeof(BridgeCommandPkt));
        newBridgeCommand = true;
    }
}

// =====================================================
// SETUP ESP-NOW (always-on for bridge communication)
// =====================================================
void setupESPNow()
{
    Serial.println("Initializing ESP-NOW...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR] ESP-NOW initialization failed.");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    // Register broadcast peer for TX to bridge
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        espNowReady = true;
        Serial.println("[OK] ESP-NOW ready. Broadcast peer registered.");
        Serial.printf("[INFO] Supervisory MAC: %s\n", WiFi.macAddress().c_str());
    } else {
        Serial.println("[ERROR] Failed to register broadcast peer.");
    }
}

// =====================================================
// SETUP SD CARD
// =====================================================
void setupSD()
{
    Serial.println("Initializing MicroSD Card...");
    pinMode(SD_MISO_PIN, INPUT_PULLUP);
    sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    if (!SD.begin(SD_CS_PIN, sdSpi, 1000000)) {
        Serial.println("[ERROR] MicroSD Card Initialization Failed!");
        sdInitialized = false;
        return;
    }
    sdInitialized = true;
    Serial.println("[OK] MicroSD Card Ready.");

    if (!SD.exists(LOG_FILENAME)) {
        File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
        if (logFile) {
            logFile.println("uptime_ms,source,sequence,depth_mm,uw_distance_mm,lat,lng,alt_m,speed_kmh,pitch,roll,yaw,sats,gps_valid,bno_valid,uw_valid");
            logFile.close();
        }
    }
}

// =====================================================
// SETUP TWAI CAN BUS
// =====================================================
void setupTWAI()
{
    Serial.println("Initializing TWAI CAN Bus...");
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        twaiInitialized = true;
        lastCanRxTime   = millis();
        Serial.println("[OK] TWAI CAN Bus Initialized at 500 kbps.");
    } else {
        Serial.println("[ERROR] Failed to initialize TWAI CAN Bus.");
    }
}

// =====================================================
// LOG DATA TO SD CARD (THREAD-SAFE WITH MUTEX)
// =====================================================
void logDataToSD(const SensorPacket& pkt, const char* source)
{
    if (!sdInitialized) return;
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        File logFile = SD.open(LOG_FILENAME, FILE_APPEND);
        if (logFile) {
            logFile.printf("%lu,%s,%lu,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%u,%u,%u\n",
                millis(), source, pkt.sequence,
                pkt.depthMm, pkt.distanceMm,
                pkt.latitude, pkt.longitude, pkt.altitudeM, pkt.speedKmph,
                pkt.pitch, pkt.roll, pkt.yaw,
                pkt.satellites, pkt.gpsValid, pkt.bnoValid, pkt.uwValid);
            logFile.close();
        }
        xSemaphoreGive(sdMutex);
    }
}

// =====================================================
// FORWARD COMMAND TO CAN BUS (from Bridge ESP32)
// =====================================================
void forwardCommandToCAN(const BridgeCommandPkt& cmd)
{
    if (!twaiInitialized) {
        Serial.println("[WARN] CAN not initialized, cannot forward command.");
        return;
    }

    twai_message_t txMsg = {};
    txMsg.extd = 0;
    txMsg.rtr  = 0;

    switch (cmd.cmdType) {
        case CMD_STEPPER: {
            txMsg.identifier       = 0x200;
            txMsg.data_length_code = 8;
            memcpy(&txMsg.data[0], &cmd.s1, sizeof(int32_t));
            memcpy(&txMsg.data[4], &cmd.s2, sizeof(int32_t));
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
            Serial.printf("[CAN TX] STEPPER -> S1=%ld S2=%ld\n", (long)cmd.s1, (long)cmd.s2);
            break;
        }
        case CMD_BTS: {
            txMsg.identifier       = 0x201;
            txMsg.data_length_code = 1;
            txMsg.data[0]          = (uint8_t)cmd.btsSpeed;
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
            Serial.printf("[CAN TX] BTS -> Speed=%d%%\n", cmd.btsSpeed);
            break;
        }
        case CMD_HOME_YAW: {
            // CAN 0x203: HOME command, data[0]=axis (0x01=Yaw)
            txMsg.identifier       = 0x203;
            txMsg.data_length_code = 1;
            txMsg.data[0]          = 0x01;
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
            latestActuatorPkt.pos1 = 0; // Reset local tracking
            Serial.println("[CAN TX] HOME -> Stepper 1 (Yaw) reset to 0");
            break;
        }
        case CMD_HOME_PITCH: {
            // CAN 0x203: HOME command, data[0]=axis (0x02=Pitch)
            txMsg.identifier       = 0x203;
            txMsg.data_length_code = 1;
            txMsg.data[0]          = 0x02;
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
            latestActuatorPkt.pos2 = 0; // Reset local tracking
            Serial.println("[CAN TX] HOME -> Stepper 2 (Pitch) reset to 0");
            break;
        }
        case CMD_MODE: {
            // Mode is software-only — no CAN command needed
            Serial.printf("[MODE] Switched to %s\n", cmd.mode ? "MANUAL" : "AUTO");
            break;
        }
        default:
            Serial.printf("[WARN] Unknown command type: %d\n", cmd.cmdType);
            break;
    }
}

// =====================================================
// FREERTOS TASK 1: CAN RECEIVER (Priority 5 — Core 1)
// Parses all sensor CAN frames + actuator status (0x202)
// =====================================================
void canTask(void *pvParameters)
{
    Serial.println("[FreeRTOS] CAN Task Started on Core 1 (Priority 5)");
    twai_message_t rxMsg;
    static SensorPacket canPkt = {};

    for (;;) {
        if (twaiInitialized && twai_receive(&rxMsg, pdMS_TO_TICKS(5)) == ESP_OK) {

            lastCanRxTime = millis();

            if (!canOnline) {
                canOnline = true;
                Serial.println("[OK] CAN Bus ACTIVE.");
            }

            Serial.printf("[CAN RX] ID: 0x%03X DLC: %d\n", rxMsg.identifier, rxMsg.data_length_code);

            bool sendToQueue = false;

            switch (rxMsg.identifier) {

                case 0x100: { // System Status / Sequence & Health Flags
                    canPkt.sequence = rxMsg.data[0];
                    if (rxMsg.data_length_code >= 2) {
                        uint8_t healthFlags = rxMsg.data[1];
                        canPkt.gpsValid = (healthFlags & (1 << 1)) ? 1 : 0;
                        canPkt.bnoValid = (healthFlags & (1 << 2)) ? 1 : 0;
                        canPkt.uwValid  = (healthFlags & (1 << 5)) ? 1 : 0;
                    }
                    break;
                }
                case 0x101: { // Depth Sensor
                    memcpy(&canPkt.depthMm, rxMsg.data, sizeof(float));
                    break;
                }
                case 0x102: { // GPS Position
                    memcpy(&canPkt.latitude,  &rxMsg.data[0], sizeof(float));
                    memcpy(&canPkt.longitude, &rxMsg.data[4], sizeof(float));
                    if (canPkt.latitude != 0.0f || canPkt.longitude != 0.0f) {
                        canPkt.gpsValid = 1;
                    }
                    break;
                }
                case 0x104: { // BNO055 IMU (pitch, roll, yaw × 100 as int16)
                    int16_t p100, r100, y100;
                    memcpy(&p100, &rxMsg.data[0], sizeof(p100));
                    memcpy(&r100, &rxMsg.data[2], sizeof(r100));
                    memcpy(&y100, &rxMsg.data[4], sizeof(y100));
                    canPkt.pitch    = p100 / 100.0f;
                    canPkt.roll     = r100 / 100.0f;
                    canPkt.yaw      = y100 / 100.0f;
                    canPkt.bnoValid = 1;
                    break;
                }
                case 0x105: { // Underwater Ultrasonic Distance
                    memcpy(&canPkt.distanceMm, rxMsg.data, sizeof(float));
                    canPkt.uwValid = 1;
                    sendToQueue    = true;
                    break;
                }
                case 0x202: { // Actuator Status Report (from micro_Actuator)
                    if (rxMsg.data_length_code >= 8) {
                        memcpy(&latestActuatorPkt.pos1, &rxMsg.data[0], sizeof(int32_t));
                        memcpy(&latestActuatorPkt.pos2, &rxMsg.data[4], sizeof(int32_t));
                        Serial.printf("[CAN RX] Actuator Status -> Pos1=%ld Pos2=%ld\n",
                                      (long)latestActuatorPkt.pos1, (long)latestActuatorPkt.pos2);
                    }
                    break;
                }
            }

            // Update latest sensor snapshot for broadcast task
            latestSensorPkt = canPkt;

            // Post to SD log queue
            if (sendToQueue && sdLogQueue != NULL) {
                LogMessage msg;
                msg.packet = canPkt;
                strncpy(msg.source, "CAN", sizeof(msg.source));
                xQueueSend(sdLogQueue, &msg, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =====================================================
// FREERTOS TASK 2: SD CARD WRITER (Priority 4 — Core 1)
// =====================================================
void sdLogTask(void *pvParameters)
{
    Serial.println("[FreeRTOS] SD Log Task Started on Core 1 (Priority 4)");
    LogMessage msg;
    for (;;) {
        if (xQueueReceive(sdLogQueue, &msg, portMAX_DELAY) == pdTRUE) {
            logDataToSD(msg.packet, msg.source);
        }
    }
}

// =====================================================
// FREERTOS TASK 3: ESP-NOW SENSOR FALLBACK (Priority 2 — Core 0)
// Handles incoming ESP-NOW SensorPacket from micro_Sensor
// =====================================================
void espNowTask(void *pvParameters)
{
    Serial.println("[FreeRTOS] ESP-NOW Task Started on Core 0 (Priority 2)");
    for (;;) {
        if (newEspNowData) {
            newEspNowData = false;
            // Update latest snapshot from fallback ESP-NOW data
            latestSensorPkt = espNowBuffer;
            // Log to SD
            LogMessage msg;
            msg.packet = espNowBuffer;
            strncpy(msg.source, "ESP-NOW", sizeof(msg.source));
            xQueueSend(sdLogQueue, &msg, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =====================================================
// FREERTOS TASK 4: ESP-NOW BROADCAST TO BRIDGE (Priority 3 — Core 0)
// Sends sensor + actuator data to Bridge ESP32 at ~60 Hz (every 16 ms)
// Also processes incoming commands from Bridge and forwards to CAN
// =====================================================
void broadcastTask(void *pvParameters)
{
    Serial.println("[FreeRTOS] Broadcast Task Started on Core 0 (Priority 3)");
    uint32_t lastBroadcast = 0;

    for (;;) {
        uint32_t now = millis();

        // Broadcast at ~60 Hz (every 16 ms)
        if (espNowReady && (now - lastBroadcast >= 16)) {
            lastBroadcast = now;

            // Build and send sensor packet
            BridgeSensorPkt sPkt;
            sPkt.type        = BPKT_SENSOR;
            sPkt.sequence    = latestSensorPkt.sequence;
            sPkt.depthMm     = latestSensorPkt.depthMm;
            sPkt.distanceMm  = latestSensorPkt.distanceMm;
            sPkt.latitude    = latestSensorPkt.latitude;
            sPkt.longitude   = latestSensorPkt.longitude;
            sPkt.pitch       = latestSensorPkt.pitch;
            sPkt.roll        = latestSensorPkt.roll;
            sPkt.yaw         = latestSensorPkt.yaw;
            sPkt.imuValid    = latestSensorPkt.bnoValid;
            sPkt.gpsValid    = latestSensorPkt.gpsValid;
            sPkt.uwValid     = latestSensorPkt.uwValid;
            esp_now_send(broadcastMac, (uint8_t*)&sPkt, sizeof(sPkt));

            // Small gap between two ESP-NOW packets to avoid collision
            vTaskDelay(pdMS_TO_TICKS(2));

            // Send actuator status packet
            esp_now_send(broadcastMac, (uint8_t*)&latestActuatorPkt, sizeof(latestActuatorPkt));
        }

        // Handle pending bridge command
        if (newBridgeCommand) {
            newBridgeCommand = false;
            BridgeCommandPkt cmd;
            memcpy(&cmd, (void*)&pendingCommand, sizeof(BridgeCommandPkt));
            forwardCommandToCAN(cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =====================================================
// FREERTOS TASK 5: SYSTEM MONITOR & LED (Priority 1 — Core 0)
// =====================================================
void systemMonitorTask(void *pvParameters)
{
    Serial.println("[FreeRTOS] System Monitor & LED Task Started on Core 0 (Priority 1)");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_task_wdt_add(NULL);
#endif

    for (;;) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        esp_task_wdt_reset();
#endif

        // CAN Bus watchdog check
        if (twaiInitialized && canOnline) {
            if (millis() - lastCanRxTime > CAN_TIMEOUT_MS) {
                canOnline = false;
                Serial.println("[WARNING] CAN Bus TIMEOUT! (>3s). Relying on ESP-NOW fallback.");
            }
        }

        // Non-blocking LED status update
        updateLEDStatus();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =====================================================
// SETUP WATCHDOG
// =====================================================
void setupWatchdog()
{
    Serial.println("Initializing Task Watchdog Timer...");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms   = WDT_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_task_wdt_reconfigure(&twdt_config);
    esp_task_wdt_add(NULL);
#else
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
    esp_task_wdt_add(NULL);
#endif
    Serial.println("[OK] Task Watchdog Timer initialized (5s panic timeout).");
}

// =====================================================
// MAIN SETUP
// =====================================================
void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n==================================================");
    Serial.println(" AUV SUPERVISORY — CAN/ESP-NOW BRIDGE CONTROLLER");
    Serial.println("==================================================\n");

    statusLed.begin();
    statusLed.clear();
    statusLed.show();

    // Initialize shared resources
    sdLogQueue = xQueueCreate(20, sizeof(LogMessage));
    sdMutex    = xSemaphoreCreateMutex();

    // Initialize peripherals
    setupSD();
    setupUSBMSC();
    setupTWAI();
    setupESPNow();   // Always-on ESP-NOW (for bridge broadcast + sensor fallback)
    setupWatchdog();

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(canTask,           "CAN_Task",       4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(sdLogTask,         "SD_Task",        4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(broadcastTask,     "Broadcast_Task", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(espNowTask,        "ESP_NOW_Task",   3072, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(systemMonitorTask, "SysMon_Task",    3072, NULL, 1, NULL, 0);

    Serial.println("\n[READY] AUV Supervisory System Running!\n");
}

// =====================================================
// MAIN LOOP (Watchdog feed for loopTask)
// =====================================================
void loop()
{
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(500));
}