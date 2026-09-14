#include <Arduino.h>
#include <driver/twai.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include <Adafruit_NeoPixel.h>

// =====================================================
// PIN DEFINITIONS FOR ESP32-S3 MICRO_ACTUATOR
// =====================================================
// TWAI CAN Bus Pins
#define CAN_TX_PIN GPIO_NUM_38
#define CAN_RX_PIN GPIO_NUM_37

// Stepper Driver #1 (A4988)
#define STEP1_PIN 5
#define DIR1_PIN  4

// Stepper Driver #2 (A4988)
#define STEP2_PIN 7
#define DIR2_PIN  6

// BTS7960 Motor Driver (PWM Pins)
// Note: R_EN and L_EN tied to 3.3V via hardware jumper
#define BTS_RPWM_PIN 21
#define BTS_LPWM_PIN 47

// Built-in RGB Status LED (ESP32-S3 DevKitC-1)
#define RGB_LED_PIN 48

// Configuration Parameters
#define CAN_TIMEOUT_MS   2000  // Failsafe motor shutdown if no CAN command in 2s
#define WDT_TIMEOUT_SEC  5     // Watchdog timeout (5 seconds)
#define PWM_FREQ         20000 // 20 kHz PWM for BTS7960
#define PWM_RES          8     // 8-bit resolution (0-255)
#define MIN_STEP_DELAY_US 400  // Minimum delay between stepper steps (microsecond limit for max speed)

// =====================================================
// CAN MESSAGE IDENTIFIERS
// =====================================================
#define CAN_ID_STEPPER_CMD   0x200 // RX: Target positions (int32_t step1, int32_t step2)
#define CAN_ID_BTS_CMD       0x201 // RX: BTS7960 motor speed (-100 to +100)
#define CAN_ID_STATUS_REPORT 0x202 // TX: Current positions (int32_t pos1, int32_t pos2)
#define CAN_ID_HOME_CMD      0x203 // RX: Home/reset stepper position (uint8_t axis: 0x01=Yaw, 0x02=Pitch)

// =====================================================
// GLOBAL VARIABLES & STATE
// =====================================================
Adafruit_NeoPixel statusLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// Colors for LED
const uint32_t COLOR_OFF   = statusLed.Color(0, 0, 0);
const uint32_t COLOR_GREEN = statusLed.Color(0, 120, 0);
const uint32_t COLOR_RED   = statusLed.Color(120, 0, 0);
const uint32_t COLOR_BLUE  = statusLed.Color(0, 0, 120);

// Stepper Motor State (Position-based)
volatile int32_t targetPos1 = 0;
volatile int32_t currentPos1 = 0;
volatile int32_t targetPos2 = 0;
volatile int32_t currentPos2 = 0;

// BTS7960 Motor State (Percentage-based)
volatile int8_t targetBtsSpeed = 0; // -100 to +100
int8_t activeBtsSpeed = 0;

// System Status
bool twaiInitialized = false;
uint32_t lastCanRxTime = 0;
bool canConnected = false;

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================
void setupTWAI();
void setupWatchdog();
void setupMotors();
void updateBtsPwm(int8_t speedPct);
void updateLEDStatus();
void sendStatusReport();

// FreeRTOS Task Handles & Prototypes
void canTask(void *pvParameters);
void actuatorTask(void *pvParameters);

// =====================================================
// LEDC PWM HELPER FOR BTS7960 (ESP32 CORE COMPATIBLE)
// =====================================================
void setBtsPwmValues(uint8_t rPwm, uint8_t lPwm) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcWrite(BTS_RPWM_PIN, rPwm);
    ledcWrite(BTS_LPWM_PIN, lPwm);
#else
    ledcWrite(0, rPwm);
    ledcWrite(1, lPwm);
#endif
}

void setupMotors() {
    // Stepper GPIO pins
    pinMode(STEP1_PIN, OUTPUT);
    pinMode(DIR1_PIN, OUTPUT);
    pinMode(STEP2_PIN, OUTPUT);
    pinMode(DIR2_PIN, OUTPUT);

    digitalWrite(STEP1_PIN, LOW);
    digitalWrite(DIR1_PIN, LOW);
    digitalWrite(STEP2_PIN, LOW);
    digitalWrite(DIR2_PIN, LOW);

    // BTS7960 PWM Channels
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcAttach(BTS_RPWM_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(BTS_LPWM_PIN, PWM_FREQ, PWM_RES);
#else
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcAttachPin(BTS_RPWM_PIN, 0);
    ledcSetup(1, PWM_FREQ, PWM_RES);
    ledcAttachPin(BTS_LPWM_PIN, 1);
#endif

    setBtsPwmValues(0, 0);
    Serial.println("[OK] Motors Initialized (Steppers + BTS7960 PWM).");
}

void updateBtsPwm(int8_t speedPct) {
    speedPct = constrain(speedPct, -100, 100);
    activeBtsSpeed = speedPct;

    if (speedPct > 0) {
        // Forward Motion
        uint8_t pwmVal = map(speedPct, 0, 100, 0, 255);
        setBtsPwmValues(pwmVal, 0);
    } else if (speedPct < 0) {
        // Reverse Motion
        uint8_t pwmVal = map(-speedPct, 0, 100, 0, 255);
        setBtsPwmValues(0, pwmVal);
    } else {
        // Stop Motor
        setBtsPwmValues(0, 0);
    }
}

// =====================================================
// SETUP TWAI CAN BUS
// =====================================================
void setupTWAI() {
    Serial.println("Initializing TWAI CAN Bus...");
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        twaiInitialized = true;
        lastCanRxTime = millis();
        Serial.println("[OK] TWAI CAN Bus Initialized at 500 kbps.");
    } else {
        Serial.println("[ERROR] Failed to initialize TWAI CAN Bus.");
    }
}

// =====================================================
// WATCHDOG TIMER
// =====================================================
void setupWatchdog() {
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
    Serial.println("[OK] Watchdog Timer Initialized.");
}

// =====================================================
// TRANSMIT CAN STATUS REPORT
// =====================================================
void sendStatusReport() {
    if (!twaiInitialized) return;

    twai_message_t txMsg;
    txMsg.identifier = CAN_ID_STATUS_REPORT;
    txMsg.extd = 0;
    txMsg.rtr = 0;
    txMsg.data_length_code = 8;

    memcpy(&txMsg.data[0], (void*)&currentPos1, sizeof(int32_t));
    memcpy(&txMsg.data[4], (void*)&currentPos2, sizeof(int32_t));

    twai_transmit(&txMsg, pdMS_TO_TICKS(10));
}

// =====================================================
// FREERTOS TASK 1: CAN RECEIVER & TELEMETRY (CORE 1)
// Priority 5 (Highest)
// =====================================================
void canTask(void *pvParameters) {
    Serial.println("[FreeRTOS] CAN Task Started on Core 1 (Priority 5)");
    twai_message_t rxMsg;
    uint32_t lastReportTime = 0;

    for (;;) {
        // Check for incoming CAN messages
        if (twaiInitialized && twai_receive(&rxMsg, pdMS_TO_TICKS(5)) == ESP_OK) {
            lastCanRxTime = millis();
            canConnected = true;

            switch (rxMsg.identifier) {
                case CAN_ID_STEPPER_CMD: {
                    if (rxMsg.data_length_code >= 8) {
                        memcpy((void*)&targetPos1, &rxMsg.data[0], sizeof(int32_t));
                        memcpy((void*)&targetPos2, &rxMsg.data[4], sizeof(int32_t));
                        Serial.printf("[CAN RX] Stepper Target Pos -> #1: %ld, #2: %ld\n", targetPos1, targetPos2);
                    }
                    break;
                }
                case CAN_ID_BTS_CMD: {
                    if (rxMsg.data_length_code >= 1) {
                        targetBtsSpeed = (int8_t)rxMsg.data[0];
                        Serial.printf("[CAN RX] BTS7960 Target Speed -> %d%%\n", targetBtsSpeed);
                    }
                    break;
                }
                case CAN_ID_HOME_CMD: {
                    // Reset stepper position counters to zero (set home)
                    if (rxMsg.data_length_code >= 1) {
                        uint8_t axis = rxMsg.data[0];
                        if (axis == 0x01) { // Yaw (Stepper 1)
                            targetPos1  = 0;
                            currentPos1 = 0;
                            Serial.println("[CAN RX] HOME: Stepper 1 (Yaw) reset to 0");
                        } else if (axis == 0x02) { // Pitch (Stepper 2)
                            targetPos2  = 0;
                            currentPos2 = 0;
                            Serial.println("[CAN RX] HOME: Stepper 2 (Pitch) reset to 0");
                        } else if (axis == 0x00) { // Both axes
                            targetPos1  = 0; currentPos1 = 0;
                            targetPos2  = 0; currentPos2 = 0;
                            Serial.println("[CAN RX] HOME: Both steppers reset to 0");
                        }
                    }
                    break;
                }
            }
        }

        // Send status feedback every 200 ms
        if (millis() - lastReportTime >= 200) {
            lastReportTime = millis();
            sendStatusReport();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =====================================================
// FREERTOS TASK 2: ACTUATOR CONTROL & FAILSAFE (CORE 1)
// Priority 4
// =====================================================
void actuatorTask(void *pvParameters) {
    Serial.println("[FreeRTOS] Actuator Task Started on Core 1 (Priority 4)");
    esp_task_wdt_add(NULL);

    uint16_t stepYieldCounter = 0;

    for (;;) {
        // CAN Failsafe Check
        if (millis() - lastCanRxTime > CAN_TIMEOUT_MS) {
            if (canConnected) {
                canConnected = false;
                Serial.println("[WARNING] CAN Bus Timeout! Engaging motor safety shutdown.");
            }
            targetBtsSpeed = 0; // Emergency stop DC motor
        }

        // Update BTS7960 PWM
        if (activeBtsSpeed != targetBtsSpeed) {
            updateBtsPwm(targetBtsSpeed);
        }

        bool isStepping = false;

        // Stepper 1 Position Stepping Logic
        if (currentPos1 != targetPos1) {
            bool dir = (targetPos1 > currentPos1);
            digitalWrite(DIR1_PIN, dir ? HIGH : LOW);

            digitalWrite(STEP1_PIN, HIGH);
            delayMicroseconds(5);
            digitalWrite(STEP1_PIN, LOW);

            currentPos1 += (dir ? 1 : -1);
            isStepping = true;
        }

        // Stepper 2 Position Stepping Logic
        if (currentPos2 != targetPos2) {
            bool dir = (targetPos2 > currentPos2);
            digitalWrite(DIR2_PIN, dir ? HIGH : LOW);

            digitalWrite(STEP2_PIN, HIGH);
            delayMicroseconds(5);
            digitalWrite(STEP2_PIN, LOW);

            currentPos2 += (dir ? 1 : -1);
            isStepping = true;
        }

        esp_task_wdt_reset();

        if (isStepping) {
            delayMicroseconds(MIN_STEP_DELAY_US);
            // Periodically yield CPU time so lower-priority tasks (like loopTask) can execute
            if (++stepYieldCounter >= 20) {
                stepYieldCounter = 0;
                vTaskDelay(1);
            }
        } else {
            stepYieldCounter = 0;
            // When idle, yield to FreeRTOS scheduler
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// =====================================================
// LED STATUS UPDATE
// =====================================================
void updateLEDStatus() {
    static unsigned long lastFlash = 0;
    static bool ledState = false;

    if (canConnected) {
        // Green Heartbeat pulse
        if (millis() - lastFlash >= 500) {
            lastFlash = millis();
            ledState = !ledState;
            statusLed.setPixelColor(0, ledState ? COLOR_GREEN : COLOR_OFF);
            statusLed.show();
        }
    } else {
        // Red Fast Blink (Error / Offline)
        if (millis() - lastFlash >= 70) {
            lastFlash = millis();
            ledState = !ledState;
            statusLed.setPixelColor(0, ledState ? COLOR_RED : COLOR_OFF);
            statusLed.show();
        }
    }
}

// =====================================================
// ARDUINO SETUP & MAIN LOOP
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("=========================================");
    Serial.println("   MICRO_ACTUATOR FIRMWARE INITIALIZING  ");
    Serial.println("=========================================");

    statusLed.begin();
    statusLed.setBrightness(50);
    statusLed.setPixelColor(0, COLOR_BLUE);
    statusLed.show();

    setupMotors();
    setupTWAI();
    setupWatchdog();

    // Create FreeRTOS Tasks
    xTaskCreatePinnedToCore(canTask, "canTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(actuatorTask, "actuatorTask", 4096, NULL, 4, NULL, 1);

    Serial.println("[OK] micro_Actuator system ready.");
}

void loop() {
    esp_task_wdt_reset();
    updateLEDStatus();
    vTaskDelay(pdMS_TO_TICKS(50));
}