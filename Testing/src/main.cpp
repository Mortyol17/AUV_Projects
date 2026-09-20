#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>


// =====================================================
// AUV BRIDGE ESP32 FIRMWARE — DUAL USB-SERIAL TO ESP-NOW BRIDGE
// Baud:                  115200
// =====================================================

// =====================================================
// NATIVE ESP32 RGB LED CONFIGURATION (GPIO 48)
// =====================================================
#define RGB_LED_PIN 48
#define LED_BLINK_MS 30           // How long normal LED stays ON per flash
#define BLINK_COOLDOWN_MS 80      // Minimum gap between flashes for visible blinking
#define ERROR_DURATION_MS 1500    // Duration (ms) to keep error mode active on bad packet
#define ERROR_PULSE_MS 150        // Blink rate while in error mode

uint32_t ledOffTime = 0;
uint32_t lastBlinkTime = 0;
bool ledActive = false;

// Persistent Error State Variables
uint32_t errorUntilTime = 0;
uint32_t nextErrorToggle = 0;
bool errorLedState = false;

// Helper function using ESP32 Native neopixelWrite
void showStatusColor(uint8_t r, uint8_t g, uint8_t b)
{
    neopixelWrite(RGB_LED_PIN, r, g, b);
}

// Trigger persistent Red Error State
void triggerErrorBlink()
{
    errorUntilTime = millis() + ERROR_DURATION_MS;
}

// Non-blocking trigger for Packet-Specific Color Blink
void triggerColorBlink(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t now = millis();
    
    // Ignore normal packet blinks if currently in Red Error State
    if (now < errorUntilTime) return;

    if (now - lastBlinkTime >= BLINK_COOLDOWN_MS) {
        showStatusColor(r, g, b); // Turn ON specified color
        ledOffTime = now + LED_BLINK_MS;
        lastBlinkTime = now;
        ledActive = true;
    }
}

// Non-blocking update loop to handle LED timeouts and error pulsing
void updateLed()
{
    uint32_t now = millis();

    // 1. ACTIVE ERROR STATE: Persistent Red Blink
    if (now < errorUntilTime) {
        if (now >= nextErrorToggle) {
            errorLedState = !errorLedState;
            showStatusColor(errorLedState ? 255 : 0, 0, 0); // RED or OFF
            nextErrorToggle = now + ERROR_PULSE_MS;
        }
        return; // Skip normal packet LED handling
    }

    // Reset error LED state once error timer expires
    if (errorLedState) {
        showStatusColor(0, 0, 0);
        errorLedState = false;
    }

    // 2. NORMAL PACKET BLINK TIMEOUT
    if (ledActive && now >= ledOffTime) {
        showStatusColor(0, 0, 0); // Turn OFF
        ledActive = false;
    }
}

// =====================================================
// RAW SENSOR PACKET (matches micro_Sensor exactly)
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

// =====================================================
// BRIDGE PACKET STRUCTURES (must match micro_Supervisory)
// =====================================================
#define BPKT_SENSOR   0x01
#define BPKT_ACTUATOR 0x02
#define BPKT_COMMAND  0x03

#define CMD_STEPPER    1
#define CMD_BTS        2
#define CMD_HOME_YAW   3
#define CMD_HOME_PITCH 4
#define CMD_MODE       5

struct __attribute__((packed)) BridgeSensorPkt {
  uint8_t  type;
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

struct __attribute__((packed)) BridgeActuatorPkt {
  uint8_t  type;
  int32_t  pos1;
  int32_t  pos2;
  int8_t   btsSpeed;
};

struct __attribute__((packed)) BridgeCommandPkt {
  uint8_t  type;
  uint8_t  cmdType;
  int32_t  s1;
  int32_t  s2;
  int8_t   btsSpeed;
  uint8_t  mode;
};

// =====================================================
// GLOBALS
// =====================================================
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
String  serialBuffer   = "";
bool    espNowReady    = false;



// Forward declarations
void parseSerialCommand(const String& line);

// Broadcast telemetry JSON over USB Serial
void broadcastJson(const char* jsonStr)
{
    Serial.println(jsonStr);
}

// =====================================================
// SEND COMMAND VIA ESP-NOW
// =====================================================
void sendCommand(const BridgeCommandPkt& cmd)
{
    if (!espNowReady) return;
    triggerColorBlink(155, 0, 155); // PURPLE/MAGENTA: Command Packet Transmit
    esp_now_send(broadcastMac, (const uint8_t*)&cmd, sizeof(BridgeCommandPkt));
}

// =====================================================
// ESP-NOW RECEIVE CALLBACK
// =====================================================
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int len)
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
#endif
{
    if (len < 1) return;

    char jsonBuf[256];

    // ---- Source 1: Raw SensorPacket from micro_Sensor ----
    if (len == (int)sizeof(SensorPacket) && data[0] != BPKT_SENSOR
                                         && data[0] != BPKT_ACTUATOR
                                         && data[0] != BPKT_COMMAND) {
        triggerColorBlink(0, 155, 155); // CYAN: Sensor Packet Received

        const SensorPacket* pkt = (const SensorPacket*)data;
        snprintf(jsonBuf, sizeof(jsonBuf),
            "{\"type\":\"SENSOR\",\"src\":\"SENSOR_DIRECT\",\"seq\":%lu,"
            "\"depth\":%.1f,\"dist\":%.1f,\"lat\":%.5f,\"lon\":%.5f,"
            "\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f,"
            "\"imuOk\":%d,\"gpsOk\":%d,\"uwOk\":%d}",
            (unsigned long)pkt->sequence,
            pkt->depthMm, pkt->distanceMm,
            pkt->latitude, pkt->longitude,
            pkt->pitch, pkt->roll, pkt->yaw,
            pkt->bnoValid, pkt->gpsValid, pkt->uwValid
        );
        broadcastJson(jsonBuf);
        return;
    }

    uint8_t pktType = data[0];

    // ---- Source 2: Typed BridgeSensorPkt from micro_Supervisory (60 Hz Stream) ----
    if (pktType == BPKT_SENSOR && len == (int)sizeof(BridgeSensorPkt)) {
        triggerColorBlink(0, 155, 155); // CYAN: Sensor Packet Received

        const BridgeSensorPkt* pkt = (const BridgeSensorPkt*)data;
        snprintf(jsonBuf, sizeof(jsonBuf),
            "{\"type\":\"SENSOR\",\"src\":\"SUPERVISORY\",\"seq\":%lu,"
            "\"depth\":%.1f,\"dist\":%.1f,\"lat\":%.5f,\"lon\":%.5f,"
            "\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f,"
            "\"imuOk\":%d,\"gpsOk\":%d,\"uwOk\":%d}",
            (unsigned long)pkt->sequence,
            pkt->depthMm, pkt->distanceMm,
            pkt->latitude, pkt->longitude,
            pkt->pitch, pkt->roll, pkt->yaw,
            pkt->imuValid, pkt->gpsValid, pkt->uwValid
        );
        broadcastJson(jsonBuf);

    // ---- Source 2b: Actuator status from micro_Supervisory ----
    } else if (pktType == BPKT_ACTUATOR && len == (int)sizeof(BridgeActuatorPkt)) {
        triggerColorBlink(0, 200, 0); // YELLOW: Actuator Status Packet Received

        const BridgeActuatorPkt* pkt = (const BridgeActuatorPkt*)data;
        snprintf(jsonBuf, sizeof(jsonBuf),
            "{\"type\":\"ACTUATOR\",\"pos1\":%ld,\"pos2\":%ld,\"bts\":%d}",
            (long)pkt->pos1, (long)pkt->pos2, (int)pkt->btsSpeed
        );
        broadcastJson(jsonBuf);

    // ---- Source 3: INVALID/CORRUPTED PACKET SIZE HANDLER ----
    } else {
        triggerErrorBlink(); // PERSISTENT RED BLINK: Invalid packet length/header
        snprintf(jsonBuf, sizeof(jsonBuf), 
            "{\"error\":\"Corrupted or invalid packet\",\"len\":%d,\"type\":%d}", 
            len, pktType
        );
        broadcastJson(jsonBuf);
    }
}

// =====================================================
// PARSE COMMAND FROM LAPTOP SERIAL OR PHONE WEBSOCKET
// =====================================================
void parseSerialCommand(const String& line)
{
    BridgeCommandPkt cmd = {};
    cmd.type = BPKT_COMMAND;
    char ackBuf[128];

    if (line.startsWith("STEPPER:")) {
        cmd.cmdType = CMD_STEPPER;
        int sep = line.indexOf(':', 8);
        if (sep > 0) {
            cmd.s1 = (int32_t)line.substring(8, sep).toInt();
            cmd.s2 = (int32_t)line.substring(sep + 1).toInt();
        }
        sendCommand(cmd);
        snprintf(ackBuf, sizeof(ackBuf), "{\"ack\":\"STEPPER\",\"s1\":%ld,\"s2\":%ld}", (long)cmd.s1, (long)cmd.s2);
        broadcastJson(ackBuf);

    } else if (line.startsWith("BTS:")) {
        cmd.cmdType  = CMD_BTS;
        cmd.btsSpeed = (int8_t)constrain(line.substring(4).toInt(), -100, 100);
        sendCommand(cmd);
        snprintf(ackBuf, sizeof(ackBuf), "{\"ack\":\"BTS\",\"speed\":%d}", (int)cmd.btsSpeed);
        broadcastJson(ackBuf);

    } else if (line == "HOME_YAW") {
        cmd.cmdType = CMD_HOME_YAW;
        sendCommand(cmd);
        broadcastJson("{\"ack\":\"HOME_YAW\"}");

    } else if (line == "HOME_PITCH") {
        cmd.cmdType = CMD_HOME_PITCH;
        sendCommand(cmd);
        broadcastJson("{\"ack\":\"HOME_PITCH\"}");

    } else if (line.startsWith("MODE:")) {
        cmd.cmdType = CMD_MODE;
        cmd.mode    = line.endsWith("MANUAL") ? 1 : 0;
        sendCommand(cmd);
        snprintf(ackBuf, sizeof(ackBuf), "{\"ack\":\"MODE\",\"mode\":\"%s\"}", cmd.mode ? "MANUAL" : "AUTO");
        broadcastJson(ackBuf);

    } else if (line.length() > 0) {
        snprintf(ackBuf, sizeof(ackBuf), "{\"error\":\"Unknown command: %s\"}", line.c_str());
        broadcastJson(ackBuf);
    }
}



// =====================================================
// SETUP
// =====================================================
void setup()
{
    Serial.begin(115200);
    delay(1500);

    // Boot Test: Flash RGB to test native driver hardware
    showStatusColor(255, 0, 0); delay(200);   // Red
    showStatusColor(0, 255, 0); delay(200);   // Green
    showStatusColor(0, 0, 255); delay(200);   // Blue
    showStatusColor(0, 0, 0);                 // Off

    Serial.println("{\"info\":\"AUV Bridge ESP32 — Starting...\"}");

    // Station Mode (required for ESP-NOW, no AP)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("{\"error\":\"ESP-NOW init failed! Check board.\"}");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    // Register broadcast MAC
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        espNowReady = true;
    }

    Serial.printf("{\"info\":\"Bridge ready\",\"mac\":\"%s\",\"espnow\":%s}\n",
                  WiFi.macAddress().c_str(),
                  espNowReady ? "true" : "false");
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop()
{
    // Handle non-blocking LED off timeout & error pulsing
    updateLed();



    // Handle USB Serial inputs from Laptop
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            String trimmed = serialBuffer;
            trimmed.trim();
            if (trimmed.length() > 0) {
                parseSerialCommand(trimmed);
            }
            serialBuffer = "";
        } else if (serialBuffer.length() < 64) {
            serialBuffer += c;
        }
    }
    
    yield();
}