/*
 * ESP32-S3 Unterbau-Controller (ESP2)
 * VERSION: v7.0 FINAL (Calibration & Learning Mode)
 * STATUS: FULL CODE – COMPLETE
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <driver/pulse_cnt.h>
#include <Preferences.h>
#include <driver/ledc.h>
#include <driver/twai.h>
#include <TMCStepper.h>
#include <esp_task_wdt.h>

/* ==================== DEFINITIONS & ENUMS ==================== */
enum StatusLEDMode { LED_NORMAL, LED_EMERGENCY, LED_CAN_LOSS, LED_CALIBRATION };

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a7"


/* ==================== CONSTANTS ==================== */
#define MAX_GEAR              5
#define RPM_SHIFT_LOCK        3500
#define BATTERY_CUTOFF        10.5
#define BATTERY_FACTOR        4.895
#define OIL_TEMP_CRITICAL     110.0
#define BATTERY_SAMPLES       10
#define CAN_TIMEOUT_MS        2000
#define IGNITION_THRESHOLD    9.0
#define GAS_MAX_STEPS_RAW     100 // Interner Zähler-Maximalwert (vor Kalibrierung)
#define SERVICE_HOURS_LIMIT   10.0
#define CHAIN_SHIFTS_LIMIT    50
#define WDT_TIMEOUT           10

/* ==================== LEDC ==================== */
#define LEDC_SERVO_EXH_CH    LEDC_CHANNEL_0
#define LEDC_SERVO_GAS_CH    LEDC_CHANNEL_1
#define LEDC_SERVO_TIMER     LEDC_TIMER_0
#define LEDC_SERVO_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_SERVO_FREQ      50
#define LEDC_SERVO_RES       LEDC_TIMER_13_BIT
#define LEDC_SERVO_DUTY_MAX  8191

// PIN DEFINITIONS (aus b_PIN-Def)
#define PIN_BATTERY_ADC       1
#define PIN_EXHAUST_SERVO     2
#define PIN_OIL_TEMP          4
#define PIN_TMC_EN            5
#define PIN_TMC_UART          6
#define PIN_RPM_HALL          7
#define PIN_I2C_SDA           8
#define PIN_I2C_SCL           9
#define PIN_CAN_RX            10
#define PIN_CAN_TX            11
#define PIN_LED_SPOILER       12
#define PIN_TMC_STEP          13
#define PIN_TMC_DIR           14
#define PIN_LED_HECK_HL       15
#define PIN_LED_HECK_HR       16
#define PIN_LED_FRONT_VL      17
#define PIN_LED_FRONT_VR      18
#define PIN_GAS_SERVO         43
#define PIN_START_LED         44

// LED CONFIG (aus c_LED-Config)
#define NUM_FRONT_LEDS        50
#define NUM_HECK_HL           22
#define NUM_HECK_HR           22
#define NUM_SPOILER           187
#define BLINKER_OUTER_LEDS    35
#define SPOILER_BLINKER_LEFT  0
#define SPOILER_BLINKER_RIGHT 152
#define SPOILER_CENTER_START  80
#define SPOILER_CENTER_LEN    27

// TMC2209 CONFIG (aus d_)
#define R_SENSE               0.11f
#define STALL_VALUE           150
#define DRIVER_ADDRESS        0b00

// OLED CONFIG (aus e_)
#define SCREEN_WIDTH          128
#define SCREEN_HEIGHT         64
#define OLED_RESET            -1
#define SCREEN_ADDRESS        0x3C

// MCP23017 MAPPING (aus f_)
#define MCP1_ADDR             0x20
#define MCP2_ADDR             0x21
#define MCP1_USB_TASTER       0
#define MCP1_PEDAL_ENC_A      1
#define MCP1_PEDAL_ENC_B      6
#define MCP1_NEUTRAL          8
#define MCP1_OIL              9
#define MCP1_BRAKE            10
#define MCP1_TILT             13
#define MCP2_TRANS3           0
#define MCP2_USB_TRANS        1
#define MCP2_TRANS1           2
#define MCP2_FERN             3
#define MCP2_ABBLEND          4
#define MCP2_TRANS2           5
#define MCP2_DRS_R            6
#define MCP2_DRS_L            7
#define MCP2_PIEZO            8
#define MCP2_LED              9
#define MCP2_START_BTN        10
#define MCP2_ENDSTOP_R        11
#define MCP2_HUPE             14
#define MCP2_ZUENDUNG         15

/* ==================== EEPROM KEYS ==================== */
#define PREF_NAMESPACE    "esp2"
#define PREF_GEAR         "gear"
#define PREF_SHIFTS       "shifts"
#define PREF_CHAIN        "chain"
#define PREF_HOURS        "hours"
// Calibration Keys
#define PREF_GAS_MIN      "gas_min"
#define PREF_GAS_MAX      "gas_max"
#define PREF_SRV_GAS_MIN  "srv_g_min"
#define PREF_SRV_GAS_MAX  "srv_g_max"
#define PREF_SRV_EXH_MIN  "srv_e_min"
#define PREF_SRV_EXH_MAX  "srv_e_max"

/* ==================== GLOBAL OBJECTS ==================== */
Adafruit_MCP23X17 mcp1, mcp2;
Adafruit_AHTX0 aht20;
Adafruit_BMP280 bmp280;

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
BLECharacteristic* pCharInput = nullptr;
bool deviceConnected = false;
bool usbState = false;

uint32_t totalShifts = 0;
uint32_t chainShifts = 0;
float engineHours = 0.0;
float currentSpeed = 0.0;
uint8_t i2cStatus = 0;

// Calibration Variables
int32_t calGasMin = 0;
int32_t calGasMax = 60;
int calSrvGasMin = 0;
int calSrvGasMax = 180;
int calSrvExhMin = 0;
int calSrvExhMax = 180;

/* ==================== INSTANCES ==================== */

bool blinkerLeftState = false;
bool blinkerRightState = false;
bool warnBlinkState = false;
bool lastBlinkLeftBtn = false;
bool lastBlinkRightBtn = false;
bool lastWarnBtn = false;
unsigned long blinkerStartTime = 0;

