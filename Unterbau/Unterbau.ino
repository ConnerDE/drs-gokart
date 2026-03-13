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

/* ==================== PIN DEFINITIONS ==================== */
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

/* ==================== LED CONFIG ==================== */
#define NUM_FRONT_LEDS        50
#define NUM_HECK_HL           22
#define NUM_HECK_HR           22
#define NUM_SPOILER           187
#define BLINKER_OUTER_LEDS    35
#define SPOILER_BLINKER_LEFT  0
#define SPOILER_BLINKER_RIGHT 152
#define SPOILER_CENTER_START  80
#define SPOILER_CENTER_LEN    27

/* ==================== TMC2209 CONFIG ==================== */
#define R_SENSE               0.11f
#define STALL_VALUE           150
#define DRIVER_ADDRESS        0b00

/* ==================== OLED CONFIG ==================== */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

/* ==================== MCP23017 MAPPING ==================== */
#define MCP1_ADDR 0x20
#define MCP2_ADDR 0x21

// MCP1
#define MCP1_USB_TASTER   0    // 1/A0
#define MCP1_PEDAL_ENC_A  1    // 1/A1
#define MCP1_PEDAL_ENC_B  6    // 1/A6
#define MCP1_NEUTRAL      8    // 1/B0
#define MCP1_OIL          9    // 1/B1
#define MCP1_BRAKE        10   // 1/B2
#define MCP1_TILT         13   // 1/B5

// MCP2
#define MCP2_TRANS3       0    // 2/A0
#define MCP2_USB_TRANS    1    // 2/A1
#define MCP2_TRANS1       2    // 2/A2
#define MCP2_FERN         3    // 2/A3
#define MCP2_ABBLEND      4    // 2/A4
#define MCP2_TRANS2       5    // 2/A5
#define MCP2_DRS_R        6    // 2/A6
#define MCP2_DRS_L        7    // 2/A7
#define MCP2_PIEZO        8    // 2/B0
#define MCP2_LED          9    // 2/B1
#define MCP2_START_BTN    10   // 2/B2
#define MCP2_ENDSTOP_R    11   // 2/B3
#define MCP2_HUPE         14   // 2/B6
#define MCP2_ZUENDUNG     15   // 2/B7

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
OneWire oneWire(PIN_OIL_TEMP);
DallasTemperature ds18b20(&oneWire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Preferences prefs;
TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

CRGB ledsFrontVL[NUM_FRONT_LEDS];
CRGB ledsFrontVR[NUM_FRONT_LEDS];
CRGB ledsHeckHL[NUM_HECK_HL];
CRGB ledsHeckHR[NUM_HECK_HR];
CRGB ledsSpoiler[NUM_SPOILER];

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

/* ==================== RPM COUNTER ==================== */
class RPMCounter {
  uint32_t lastCalc = 0;
  uint16_t rpm = 0;
  pcnt_unit_handle_t pcnt_unit = NULL;
  pcnt_channel_handle_t pcnt_chan = NULL;

public:
  void begin() {
    pcnt_unit_config_t unit_config = {
      .low_limit = -4000,
      .high_limit = 4000,
      .flags = {}
    };
    pcnt_new_unit(&unit_config, &pcnt_unit);
    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 5000 };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);
    pcnt_chan_config_t chan_config = {
      .edge_gpio_num = PIN_RPM_HALL,
      .level_gpio_num = -1,
      .flags = {}
    };
    pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);
    pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
  }

  void update() {
    if (millis() - lastCalc < 100) return;
    if (!pcnt_unit) return;
    int count = 0;
    if (pcnt_unit_get_count(pcnt_unit, &count) == ESP_OK) {
      pcnt_unit_clear_count(pcnt_unit);
      rpm = abs(count) * 60000UL / 100;
    }
    lastCalc = millis();
  }
  uint16_t getRPM() const { return rpm; }
};

/* ==================== CAN RECEIVER ==================== */
class CanReceiver {
  unsigned long lastFrame = 0;
  uint8_t lastCounter = 0;
  unsigned long lastCounterChange = 0;

public:
  uint8_t exhaustAngle = 0;
  uint8_t dashPage = 0;
  uint8_t driveMode = 0;
  uint8_t lightMode = 0;
  uint8_t rgbMode = 0;
  uint16_t buttons = 0xFFFF;
  bool dataValid = false;

  void begin() {
    twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = (gpio_num_t)PIN_CAN_TX,
      .rx_io = (gpio_num_t)PIN_CAN_RX,
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 5,
      .rx_queue_len = 5,
      .alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED,
      .clkout_divider = 0,
      .intr_flags = ESP_INTR_FLAG_LEVEL1
    };
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
  }

  void update() {
    uint32_t alerts;
    if (twai_read_alerts(&alerts, 0) == ESP_OK) {
      if (alerts & TWAI_ALERT_BUS_OFF) twai_initiate_recovery();
      if (alerts & TWAI_ALERT_BUS_RECOVERED) twai_start();
    }

    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      if (message.data_length_code >= 8) {
        exhaustAngle = message.data[0];
        dashPage = message.data[1];
        driveMode = message.data[2];
        lightMode = message.data[3];
        rgbMode = message.data[4];
        buttons = message.data[5] | (message.data[6] << 8);
        uint8_t currentCounter = message.data[7];
        if (currentCounter != lastCounter) {
          lastCounter = currentCounter;
          lastCounterChange = millis();
        }
        dataValid = true;
        lastFrame = millis();
      }
    }
    if (millis() - lastFrame > CAN_TIMEOUT_MS) dataValid = false;
    if (millis() - lastCounterChange > 500) dataValid = false;
  }

  bool isBlinkRight()   { return !(buttons & (1 << 0)); }
  bool isShiftDown()    { return !(buttons & (1 << 1)); }
  bool isDRS()          { return !(buttons & (1 << 2)); }
  bool isReverse()      { return !(buttons & (1 << 3)); }
  bool isBlinkLeft()    { return !(buttons & (1 << 4)); }
  bool isShiftUp()      { return !(buttons & (1 << 5)); }
  bool isWarnBlink()    { return !(buttons & (1 << 7)); }
  bool isLaunch()       { return !(buttons & (1 << 9)); }
  bool isHorn()         { return !(buttons & (1 << 15)); }
};

/* ==================== GAS PEDAL & SERVO ==================== */
class GasPedal {
  int32_t encoderPos = 0;
  uint8_t lastEncState = 0;
  uint8_t currentGasPercent = 0;

public:
  void begin() {
    ledc_channel_config_t gas_conf = {
      .gpio_num = PIN_GAS_SERVO,
      .speed_mode = LEDC_SERVO_MODE,
      .channel = LEDC_SERVO_GAS_CH,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_SERVO_TIMER,
      .duty = 0,
      .hpoint = 0,
      .flags = {}
    };
    ledc_channel_config(&gas_conf);

    // Load Calibration
    calGasMin = prefs.getInt(PREF_GAS_MIN, 0);
    calGasMax = prefs.getInt(PREF_GAS_MAX, 60);
    calSrvGasMin = prefs.getInt(PREF_SRV_GAS_MIN, 0);
    calSrvGasMax = prefs.getInt(PREF_SRV_GAS_MAX, 180);
  }

  void cutThrottle() {
    uint32_t pulseWidth_us = map(calSrvGasMin, 0, 180, 1000, 2000);
    uint32_t duty = (uint32_t)((pulseWidth_us * (uint64_t)LEDC_SERVO_DUTY_MAX) / 20000ULL);
    ledc_set_duty(LEDC_SERVO_MODE, LEDC_SERVO_GAS_CH, duty);
    ledc_update_duty(LEDC_SERVO_MODE, LEDC_SERVO_GAS_CH);
  }

  void update(uint8_t driveMode, bool launchActive, uint16_t currentRPM) {
    bool a = mcp1.digitalRead(MCP1_PEDAL_ENC_A);
    bool b = mcp1.digitalRead(MCP1_PEDAL_ENC_B);
    uint8_t state = (a << 1) | b;

    if (state != lastEncState) {
      if ((lastEncState == 0b00 && state == 0b01) ||
          (lastEncState == 0b01 && state == 0b11) ||
          (lastEncState == 0b11 && state == 0b10) ||
          (lastEncState == 0b10 && state == 0b00)) {
        encoderPos++;
      } else if ((lastEncState == 0b00 && state == 0b10) ||
                 (lastEncState == 0b10 && state == 0b11) ||
                 (lastEncState == 0b11 && state == 0b01) ||
                 (lastEncState == 0b01 && state == 0b00)) {
        encoderPos--;
      }
      lastEncState = state;
      // Allow raw range for calibration, constrain later
      if (encoderPos < 0) encoderPos = 0;
      if (encoderPos > GAS_MAX_STEPS_RAW) encoderPos = GAS_MAX_STEPS_RAW;
    }

    // Calculate Percentage based on Calibration
    int32_t constrainedPos = constrain(encoderPos, calGasMin, calGasMax);
    currentGasPercent = map(constrainedPos, calGasMin, calGasMax, 0, 100);

    float inputNorm = (float)(constrainedPos - calGasMin) / (float)(calGasMax - calGasMin);
    float outputNorm = inputNorm;

    switch (driveMode) {
      case 0: outputNorm = inputNorm; break;
      case 1: outputNorm = pow(inputNorm, 1.5); break;
      case 2: outputNorm = inputNorm * inputNorm; break;
      case 3:
        outputNorm = inputNorm * 1.2;
        if (outputNorm > 1.0) outputNorm = 1.0;
        break;
    }

    int targetAngle = outputNorm * (calSrvGasMax - calSrvGasMin) + calSrvGasMin;

    if (launchActive && currentRPM > 2100) {
      targetAngle = calSrvGasMin;
    }

    uint32_t pulseWidth_us = map(targetAngle, 0, 180, 1000, 2000);
    uint32_t duty = (uint32_t)((pulseWidth_us * (uint64_t)LEDC_SERVO_DUTY_MAX) / 20000ULL);
    ledc_set_duty(LEDC_SERVO_MODE, LEDC_SERVO_GAS_CH, duty);
    ledc_update_duty(LEDC_SERVO_MODE, LEDC_SERVO_GAS_CH);
  }

  uint8_t getGasPercent() { return currentGasPercent; }
  int32_t getRawPos() { return encoderPos; }
};

/* ==================== SAFETY & START/STOP ==================== */
class SafetyModule {
  float batteryBuf[BATTERY_SAMPLES];
  uint8_t idx = 0;
  bool emergency = false;
  unsigned long lastLedUpdate = 0;
  int ledBrightness = 0;
  bool ledDir = true;
  unsigned long lastHourCalc = 0;

public:
  float batteryVoltage = 0;
  float oilTemp = 0;
  float outsideTemp = 0;
  float airPressure = 0;
  bool oilPresent = true;
  bool brakePressed = false;
  bool tiltDetected = false;
  bool neutralActive = false;
  bool ignitionOn = false;
  bool systemActive = false;
  uint16_t currentRPM = 0;
  StatusLEDMode ledMode = LED_NORMAL;

  void begin() {
    for (uint8_t i = 0; i < BATTERY_SAMPLES; i++) batteryBuf[i] = 12.0;

    ds18b20.begin();
    if (ds18b20.getDeviceCount() > 0) i2cStatus |= (1 << 5);

    if (aht20.begin()) i2cStatus |= (1 << 2);
    if (bmp280.begin(0x76)) i2cStatus |= (1 << 3);

    pinMode(PIN_START_LED, OUTPUT);

    mcp1.pinMode(MCP1_NEUTRAL, INPUT_PULLUP);
    mcp1.pinMode(MCP1_OIL, INPUT_PULLUP);
    mcp1.pinMode(MCP1_BRAKE, INPUT_PULLUP);
    mcp1.pinMode(MCP1_TILT, INPUT_PULLUP);
    mcp1.pinMode(MCP1_PEDAL_ENC_A, INPUT_PULLUP);
    mcp1.pinMode(MCP1_PEDAL_ENC_B, INPUT_PULLUP);
    mcp1.pinMode(MCP1_USB_TASTER, INPUT_PULLUP);

    for (uint8_t i = 0; i < 16; i++) {
      if (i == MCP2_START_BTN || i == MCP2_ENDSTOP_R) {
        mcp2.pinMode(i, INPUT_PULLUP);
      } else {
        mcp2.pinMode(i, OUTPUT);
        mcp2.digitalWrite(i, LOW);
      }
    }

    totalShifts = prefs.getUInt(PREF_SHIFTS, 0);
    chainShifts = prefs.getUInt(PREF_CHAIN, 0);
    engineHours = prefs.getFloat(PREF_HOURS, 0.0);
  }

  void update() {
    int raw = analogRead(PIN_BATTERY_ADC);
    float v = (raw / 4095.0) * 3.3 * BATTERY_FACTOR;
    batteryBuf[idx++] = v;
    idx %= BATTERY_SAMPLES;
    float sum = 0;
    for (float x : batteryBuf) sum += x;
    batteryVoltage = sum / BATTERY_SAMPLES;

    systemActive = (batteryVoltage > IGNITION_THRESHOLD);

    if (!systemActive) {
      ignitionOn = false;
      mcp2.digitalWrite(MCP2_ZUENDUNG, LOW);
      digitalWrite(PIN_START_LED, LOW);
      return;
    }

    if (ignitionOn && millis() - lastHourCalc > 60000) {
      lastHourCalc = millis();
      engineHours += (1.0 / 60.0);
      prefs.putFloat(PREF_HOURS, engineHours);
    }

    ds18b20.requestTemperatures();
    oilTemp = ds18b20.getTempCByIndex(0);
    if (oilTemp < -100.0) oilTemp = 0.0;

    sensors_event_t humidity, temp;
    aht20.getEvent(&humidity, &temp);
    float t1 = temp.temperature;
    float t2 = bmp280.readTemperature();
    float p = bmp280.readPressure();

    if (!isnan(t2) && t2 > -40 && t2 < 85) outsideTemp = (t1 + t2) / 2.0;
    else outsideTemp = t1;

    if (!isnan(p)) airPressure = p / 100.0F;

    neutralActive = !mcp1.digitalRead(MCP1_NEUTRAL);
    oilPresent = (mcp1.digitalRead(MCP1_OIL) == LOW);
    brakePressed = !mcp1.digitalRead(MCP1_BRAKE);
    tiltDetected = !mcp1.digitalRead(MCP1_TILT);

    bool nowEmergency = !oilPresent || tiltDetected || (oilTemp > OIL_TEMP_CRITICAL && oilTemp > 0);

    if (nowEmergency && !emergency) {
      emergency = true;
      ledMode = LED_EMERGENCY;
      mcp2.digitalWrite(MCP2_ZUENDUNG, LOW);
      ignitionOn = false;
      mcp2.digitalWrite(MCP2_PIEZO, HIGH);
    } else if (!nowEmergency && emergency) {
      emergency = false;
      ledMode = LED_NORMAL;
      mcp2.digitalWrite(MCP2_PIEZO, LOW);
    }

    static bool lastBtn = true;
    static unsigned long lastBtnTime = 0;
    bool btn = mcp2.digitalRead(MCP2_START_BTN);

    if (!btn && lastBtn && (millis() - lastBtnTime > 200)) {
      lastBtnTime = millis();
      if (ignitionOn) {
        ignitionOn = false;
        mcp2.digitalWrite(MCP2_ZUENDUNG, LOW);
      } else {
        if (brakePressed) {
          ignitionOn = true;
          mcp2.digitalWrite(MCP2_ZUENDUNG, HIGH);
        } else {
          ignitionOn = true;
          mcp2.digitalWrite(MCP2_ZUENDUNG, HIGH);
        }
      }
    }
    lastBtn = btn;

    updateStartLed();
  }

  void updateStartLed() {
    if (ledMode == LED_EMERGENCY) {
      if (millis() - lastLedUpdate > 100) {
        lastLedUpdate = millis();
        digitalWrite(PIN_START_LED, !digitalRead(PIN_START_LED));
      }
    } else if (ignitionOn && currentRPM > 300) {
      digitalWrite(PIN_START_LED, HIGH);
    } else if (systemActive) {
      if (millis() - lastLedUpdate > 20) {
        lastLedUpdate = millis();
        if (ledDir) {
          ledBrightness += 5;
          if (ledBrightness >= 255) ledDir = false;
        } else {
          ledBrightness -= 5;
          if (ledBrightness <= 0) ledDir = true;
        }
        analogWrite(PIN_START_LED, ledBrightness);
      }
    } else {
      digitalWrite(PIN_START_LED, LOW);
    }
  }

  bool isEmergencyActive() { return emergency; }
  bool canShiftDown(uint16_t rpm) { return rpm < RPM_SHIFT_LOCK; }
  void setCanLoss(bool lost) { ledMode = lost ? LED_CAN_LOSS : LED_NORMAL; }
};

extern GasPedal gasPedal;

/* ==================== GEARBOX ==================== */
class Gearbox {
  int8_t currentGear = 0;

public:
  void begin() {
    Serial1.begin(115200, SERIAL_8N1, PIN_TMC_UART, PIN_TMC_UART);
    delay(100);

    driver.begin();
    driver.toff(4);
    driver.blank_time(24);
    driver.rms_current(1200);
    driver.microsteps(16);
    driver.TCOOLTHRS(0xFFFFF);
    driver.SGTHRS(STALL_VALUE);

    pinMode(PIN_TMC_DIR, OUTPUT);
    pinMode(PIN_TMC_STEP, OUTPUT);
    pinMode(PIN_TMC_EN, OUTPUT);
    digitalWrite(PIN_TMC_EN, LOW);

    if (!mcp2.digitalRead(MCP2_ENDSTOP_R)) {
      currentGear = -1;
    } else {
      currentGear = prefs.getInt(PREF_GEAR, 0);
    }
  }

  int8_t getGear() const { return currentGear; }
  void setNeutral() { currentGear = 0; prefs.putInt(PREF_GEAR, currentGear); }

  void shiftUp(bool allow) {
    if (!allow) return;
    if (currentGear < MAX_GEAR) {
      if (stepMotor(true)) {
        currentGear++;
        prefs.putInt(PREF_GEAR, currentGear);
        incrementStats();
      } else {
        gasPedal.cutThrottle();
        unsigned long t0 = millis();
        while(millis() - t0 < 50) esp_task_wdt_reset();
        driver.rms_current(1500);
        digitalWrite(PIN_TMC_EN, LOW);
        if (stepMotor(true)) {
           currentGear++;
           prefs.putInt(PREF_GEAR, currentGear);
           incrementStats();
        }
        driver.rms_current(1200);
      }
    }
  }

  void shiftDown(bool allow) {
    if (!allow) return;
    if (currentGear > 1) {
      if (stepMotor(false)) {
        currentGear--;
        prefs.putInt(PREF_GEAR, currentGear);
        incrementStats();
      } else {
        gasPedal.cutThrottle();
        unsigned long t0 = millis();
        while(millis() - t0 < 50) esp_task_wdt_reset();
        driver.rms_current(1500);
        digitalWrite(PIN_TMC_EN, LOW);
        if (stepMotor(false)) {
           currentGear--;
           prefs.putInt(PREF_GEAR, currentGear);
           incrementStats();
        }
        driver.rms_current(1200);
      }
    }
  }

  void shiftToReverse(bool allow) {
    if (!allow || currentGear != 0) return;
    if (stepMotor(false)) {
      currentGear = -1;
      prefs.putInt(PREF_GEAR, currentGear);
    }
  }

  void incrementStats() {
    totalShifts++;
    chainShifts++;
    prefs.putUInt(PREF_SHIFTS, totalShifts);
    prefs.putUInt(PREF_CHAIN, chainShifts);
  }

  bool stepMotor(bool dir) {
    digitalWrite(PIN_TMC_DIR, dir ? HIGH : LOW);
    digitalWrite(PIN_TMC_EN, LOW);

    bool stalled = false;
    for (int i = 0; i < 400; i++) {
      if (driver.SG_RESULT() < 10) {
        stalled = true;
        break;
      }
      digitalWrite(PIN_TMC_STEP, HIGH);
      delayMicroseconds(800);
      digitalWrite(PIN_TMC_STEP, LOW);
      delayMicroseconds(800);
    }

    if (stalled) {
      digitalWrite(PIN_TMC_EN, HIGH);
      return false;
    }
    return true;
  }
};

/* ==================== EXHAUST SERVO ==================== */
void initExhaustServo() {
  ledc_channel_config_t exh_conf = {
    .gpio_num = PIN_EXHAUST_SERVO,
    .speed_mode = LEDC_SERVO_MODE,
    .channel = LEDC_SERVO_EXH_CH,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_SERVO_TIMER,
    .duty = 0,
    .hpoint = 0,
    .flags = {}
  };
  ledc_channel_config(&exh_conf);

  calSrvExhMin = prefs.getInt(PREF_SRV_EXH_MIN, 0);
  calSrvExhMax = prefs.getInt(PREF_SRV_EXH_MAX, 180);
}

void setExhaustServoAngle(uint8_t angle) {
  // Map 0-90 logic to calibrated servo range
  int targetAngle = calSrvExhMin;
  if (angle == 45) targetAngle = (calSrvExhMin + calSrvExhMax) / 2;
  if (angle == 90) targetAngle = calSrvExhMax;

  uint32_t pulseWidth_us = map(targetAngle, 0, 180, 1000, 2000);
  uint32_t duty = (uint32_t)((pulseWidth_us * (uint64_t)LEDC_SERVO_DUTY_MAX) / 20000ULL);
  ledc_set_duty(LEDC_SERVO_MODE, LEDC_SERVO_EXH_CH, duty);
  ledc_update_duty(LEDC_SERVO_MODE, LEDC_SERVO_EXH_CH);
}

/* ==================== LIGHTS MANAGER ==================== */
class LightsManager {
  bool lastDataValid = true;
  unsigned long lastBlinkerUpdate = 0;
  int16_t blinkerPosL = 0;
  int16_t blinkerPosR = 0;

public:
  void playWelcomeAnimation() {
    static bool done = false;
    static unsigned long t0 = 0;
    static uint8_t step = 0;
    if (done) return;

    unsigned long now = millis();
    CRGB orange = CRGB(255, 120, 0);

    switch (step) {
      case 0: t0 = now; step = 1; break;
      case 1: {
        if (now - t0 < 300) {
          uint8_t b = map(now - t0, 0, 300, 0, 255);
          fill_solid(ledsFrontVL, NUM_FRONT_LEDS, orange.nscale8(b));
          fill_solid(ledsFrontVR, NUM_FRONT_LEDS, orange.nscale8(b));
          fill_solid(ledsHeckHL, NUM_HECK_HL, orange.nscale8(b));
          fill_solid(ledsHeckHR, NUM_HECK_HR, orange.nscale8(b));
          fill_solid(ledsSpoiler, NUM_SPOILER, orange.nscale8(b));
          FastLED.show();
        } else { t0 = now; step = 2; }
      } break;
      case 2: if (now - t0 > 300) { t0 = now; step = 3; } break;
      case 3: {
        if (now - t0 < 300) {
          uint8_t b = map(now - t0, 0, 300, 255, 0);
          fill_solid(ledsFrontVL, NUM_FRONT_LEDS, orange.nscale8(b));
          fill_solid(ledsFrontVR, NUM_FRONT_LEDS, orange.nscale8(b));
          fill_solid(ledsHeckHL, NUM_HECK_HL, orange.nscale8(b));
          fill_solid(ledsHeckHR, NUM_HECK_HR, orange.nscale8(b));
          fill_solid(ledsSpoiler, NUM_SPOILER, orange.nscale8(b));
          FastLED.show();
        } else { done = true; }
      } break;
    }
  }

  void applyRGBEffect(CRGB* leds, uint16_t count, uint8_t mode, uint8_t driveMode) {
    static uint8_t hue = 0;
    static uint8_t pulse = 0;
    static bool pulseDir = true;
    static uint16_t pos = 0;
    hue += 2;

    switch (mode) {
      case 0: fill_solid(leds, count, CRGB::Black); break;
      case 1: {
        CRGB color;
        switch (driveMode) {
          case 0: color = CRGB(0, 255, 0); break;
          case 1: color = CRGB(0, 0, 255); break;
          case 2: color = CRGB(255, 0, 0); break;
          case 3: color = CRGB(128, 0, 128); break;
          default: color = CRGB::White; break;
        }
        fill_solid(leds, count, color);
      } break;
      case 2: fill_rainbow(leds, count, hue, 7); break;
      case 3: {
        if (pulseDir) { pulse += 3; if (pulse > 250) pulseDir = false; }
        else { pulse -= 3; if (pulse < 10) pulseDir = true; }
        fill_solid(leds, count, CHSV(96, 255, pulse));
      } break;
      case 4: {
        pos = (pos + 1) % (count * 2);
        fill_solid(leds, count, CRGB::Black);
        for (uint8_t i = 0; i < 8; i++) {
          int idx = (pos - i + count * 2) % (count * 2);
          if (idx < count) leds[idx] = CHSV((hue + i * 32) % 256, 255, 255 - i * 25);
        }
      } break;
      case 5: {
        fill_solid(leds, count, CRGB::Black);
        for (uint8_t i = 0; i < count / 5; i++) {
          uint8_t idx = random16(count);
          leds[idx] = CHSV(random8(), 200, 255);
        }
      } break;
      default: fill_solid(leds, count, CRGB::Black); break;
    }
  }

  void updateDynamicBlinker(CRGB* leds, uint16_t count, bool active, bool left) {
    if (!active) {
      fill_solid(leds, count, CRGB::Black);
      if (left) blinkerPosL = 0; else blinkerPosR = 0;
      return;
    }

    unsigned long now = millis();
    if (now - lastBlinkerUpdate > 15) {
      lastBlinkerUpdate = now;
      int16_t& pos = left ? blinkerPosL : blinkerPosR;
      uint16_t mid = count / 2;

      if (pos >= mid) { fill_solid(leds, count, CRGB::Black); pos = 0; return; }

      fill_solid(leds, count, CRGB::Black);
      CRGB orange = CRGB(255, 120, 0);

      if (left) {
        for (int i = 0; i <= pos && i < mid; i++) leds[i] = orange;
        pos++;
      } else {
        for (int i = count - 1; i >= mid && (count - 1 - i) <= pos; i--) leds[i] = orange;
        pos++;
      }
    }
  }

  void update(CanReceiver& can, SafetyModule& safety, bool launchActive) {
    playWelcomeAnimation();

    if (!can.dataValid && lastDataValid) {
      fill_solid(ledsFrontVL, NUM_FRONT_LEDS, CRGB::Black);
      fill_solid(ledsFrontVR, NUM_FRONT_LEDS, CRGB::Black);
      fill_solid(ledsHeckHL, NUM_HECK_HL, CRGB::Black);
      fill_solid(ledsHeckHR, NUM_HECK_HR, CRGB::Black);
      fill_solid(ledsSpoiler, NUM_SPOILER, CRGB::Black);
      mcp2.digitalWrite(MCP2_ABBLEND, LOW);
      FastLED.show();
      lastDataValid = false;
      return;
    }
    lastDataValid = can.dataValid;
    if (!can.dataValid) return;

    mcp2.digitalWrite(MCP2_ABBLEND, (can.lightMode == 1) ? HIGH : LOW);

    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    if (millis() - lastBlink > 500) { lastBlink = millis(); blinkState = !blinkState; }

    extern bool blinkerLeftState;
    extern bool blinkerRightState;
    extern bool warnBlinkState;

    bool warn = warnBlinkState;
    bool left = blinkerLeftState;
    bool right = blinkerRightState;
    bool brake = safety.brakePressed;

    mcp2.digitalWrite(MCP2_TRANS3, brake ? HIGH : LOW);

    // FRONT LEDS (Launch Control Bar)
    if (launchActive) {
      uint16_t rpm = safety.currentRPM;
      if (rpm > 2100) rpm = 2100;
      uint8_t fillCount = map(rpm, 0, 2100, 0, NUM_FRONT_LEDS);

      if (rpm >= 2050) {
        CRGB c = blinkState ? CRGB::White : CRGB::Black;
        fill_solid(ledsFrontVL, NUM_FRONT_LEDS, c);
        fill_solid(ledsFrontVR, NUM_FRONT_LEDS, c);
      } else {
        fill_solid(ledsFrontVL, NUM_FRONT_LEDS, CRGB::Black);
        fill_solid(ledsFrontVR, NUM_FRONT_LEDS, CRGB::Black);
        for(int i=0; i<fillCount; i++) ledsFrontVL[i] = CRGB::Green;
        for(int i=0; i<fillCount; i++) ledsFrontVR[i] = CRGB::Green;
      }
    } else {
      if (warn) {
        updateDynamicBlinker(ledsFrontVL, NUM_FRONT_LEDS, blinkState, true);
        updateDynamicBlinker(ledsFrontVR, NUM_FRONT_LEDS, blinkState, false);
      } else {
        updateDynamicBlinker(ledsFrontVL, NUM_FRONT_LEDS, left, true);
        updateDynamicBlinker(ledsFrontVR, NUM_FRONT_LEDS, right, false);
      }

      if (!warn && !left && !right) {
        uint8_t effectiveRgbMode = (can.lightMode > 0 && can.rgbMode == 0) ? 255 : can.rgbMode;
        if (effectiveRgbMode != 255) {
          applyRGBEffect(ledsFrontVL, NUM_FRONT_LEDS, effectiveRgbMode, can.driveMode);
          applyRGBEffect(ledsFrontVR, NUM_FRONT_LEDS, effectiveRgbMode, can.driveMode);
        }
      }
    }

    // SPOILER
    fill_solid(ledsSpoiler, NUM_SPOILER, CRGB::Black);

    if (safety.currentRPM > 3500) {
      CRGB warnColor = blinkState ? CRGB::DeepSkyBlue : CRGB::Red;
      for(int i=SPOILER_CENTER_START; i < SPOILER_CENTER_START + SPOILER_CENTER_LEN; i++) {
        ledsSpoiler[i] = warnColor;
      }
    } else {
      if (!warn && !left && !right && !brake) {
        uint8_t effectiveRgbMode = (can.lightMode > 0 && can.rgbMode == 0) ? 255 : can.rgbMode;
        if (effectiveRgbMode != 255) {
          applyRGBEffect(ledsSpoiler, NUM_SPOILER, effectiveRgbMode, can.driveMode);
        }
      }
    }

    CRGB spoilerBlinker = CRGB(255, 150, 0);
    CRGB spoilerBrake = CRGB(255, 0, 0);

    if (warn) {
      for (uint8_t i = 0; i < BLINKER_OUTER_LEDS; i++) {
        ledsSpoiler[SPOILER_BLINKER_LEFT + i] = blinkState ? spoilerBlinker : CRGB::Black;
        ledsSpoiler[SPOILER_BLINKER_RIGHT + i] = blinkState ? spoilerBlinker : CRGB::Black;
      }
    } else {
      if (left) for (uint8_t i = 0; i < BLINKER_OUTER_LEDS; i++)
        ledsSpoiler[SPOILER_BLINKER_LEFT + i] = blinkState ? spoilerBlinker : CRGB::Black;
      if (right) for (uint8_t i = 0; i < BLINKER_OUTER_LEDS; i++)
        ledsSpoiler[SPOILER_BLINKER_RIGHT + i] = blinkState ? spoilerBlinker : CRGB::Black;
      if (brake && !left && !right) {
        for (uint8_t i = 0; i < BLINKER_OUTER_LEDS; i++) {
          ledsSpoiler[SPOILER_BLINKER_LEFT + i] = spoilerBrake;
          ledsSpoiler[SPOILER_BLINKER_RIGHT + i] = spoilerBrake;
        }
      }
    }

    // HECK
    fill_solid(ledsHeckHL, NUM_HECK_HL, CRGB::Black);
    fill_solid(ledsHeckHR, NUM_HECK_HR, CRGB::Black);

    if (can.lightMode > 0) {
      CRGB rear = CRGB(255, 0, 0).nscale8(153);
      fill_solid(ledsHeckHL, NUM_HECK_HL, rear);
      fill_solid(ledsHeckHR, NUM_HECK_HR, rear);
    }
    if (brake) {
      fill_solid(ledsHeckHL, NUM_HECK_HL, CRGB::Red);
      fill_solid(ledsHeckHR, NUM_HECK_HR, CRGB::Red);
    }
    if (warn || left) fill_solid(ledsHeckHL, NUM_HECK_HL, blinkState ? CRGB::Orange : CRGB::Black);
    if (warn || right) fill_solid(ledsHeckHR, NUM_HECK_HR, blinkState ? CRGB::Orange : CRGB::Black);

    static unsigned long lastShow = 0;
    if (millis() - lastShow > 20) {
        FastLED.show();
        lastShow = millis();
    }
  }
};

/* ==================== BLE ==================== */
class BLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    std::string value = pChar->getValue();
    if (value.length() > 0) {
      String s = String(value.c_str());

      // Calibration Commands
      if (s == "CAL:GAS_MIN") {
        calGasMin = gasPedal.getRawPos();
        prefs.putInt(PREF_GAS_MIN, calGasMin);
      } else if (s == "CAL:GAS_MAX") {
        calGasMax = gasPedal.getRawPos();
        prefs.putInt(PREF_GAS_MAX, calGasMax);
      } else if (s.startsWith("SET:SRV_GAS_MIN:")) {
        calSrvGasMin = s.substring(16).toInt();
        prefs.putInt(PREF_SRV_GAS_MIN, calSrvGasMin);
      } else if (s.startsWith("SET:SRV_GAS_MAX:")) {
        calSrvGasMax = s.substring(16).toInt();
        prefs.putInt(PREF_SRV_GAS_MAX, calSrvGasMax);
      } else if (s.startsWith("SET:SRV_EXH_MIN:")) {
        calSrvExhMin = s.substring(16).toInt();
        prefs.putInt(PREF_SRV_EXH_MIN, calSrvExhMin);
      } else if (s.startsWith("SET:SRV_EXH_MAX:")) {
        calSrvExhMax = s.substring(16).toInt();
        prefs.putInt(PREF_SRV_EXH_MAX, calSrvExhMax);
      }

      // Reset Commands
      else if (s == "RESET_HOURS") {
        engineHours = 0;
        prefs.putFloat(PREF_HOURS, 0.0);
      } else if (s == "RESET_CHAIN") {
        chainShifts = 0;
        prefs.putUInt(PREF_CHAIN, 0);
      } else if (s.startsWith("SPD:")) {
        currentSpeed = s.substring(4).toFloat();
      }
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { deviceConnected = true; }
  void onDisconnect(BLEServer*) override { deviceConnected = false; }
};

void sendTelemetry(SafetyModule& safety, CanReceiver& can, GasPedal& gas, int8_t gear, bool launch) {
  static unsigned long last = 0;
  if (!deviceConnected || millis() - last < 500) return;
  last = millis();

  JsonDocument doc;
  doc["gear"] = gear;
  doc["rpm"] = safety.currentRPM;
  doc["bat"] = safety.batteryVoltage;
  doc["oil"] = safety.oilTemp;
  doc["mode"] = can.driveMode;
  doc["gas"] = gas.getGasPercent();
  doc["launch"] = launch;
  doc["light"] = can.lightMode;
  doc["rgb"] = can.rgbMode;
  doc["env"] = safety.outsideTemp;
  doc["press"] = safety.airPressure;
  doc["hours"] = engineHours;
  doc["shifts"] = totalShifts;

  // OBD Diagnostics (Bitmask)
  uint32_t diag = 0;
  if (safety.brakePressed) diag |= (1 << 0);
  if (safety.neutralActive) diag |= (1 << 1);
  if (safety.tiltDetected) diag |= (1 << 2);
  if (safety.oilPresent) diag |= (1 << 3);
  if (safety.ignitionOn) diag |= (1 << 4);
  if (can.dataValid) diag |= (1 << 5);
  // I2C Status (Bits 8-15)
  diag |= (i2cStatus << 8);

  doc["diag"] = diag;

  if (engineHours > SERVICE_HOURS_LIMIT) doc["warn"] = "SERVICE";
  else if (chainShifts > CHAIN_SHIFTS_LIMIT) doc["warn"] = "CHAIN";
  else doc["warn"] = "OK";

  String exhStr = "Closed";
  if (can.exhaustAngle == 45) exhStr = "Partial";
  if (can.exhaustAngle == 90) exhStr = "Open";
  doc["exh"] = exhStr;
  doc["exhAng"] = can.exhaustAngle;

  String json;
  serializeJson(doc, json);
  pCharacteristic->setValue(json.c_str());
  pCharacteristic->notify();
}

/* ==================== DISPLAY ==================== */
void updateDisplay(int8_t gear, SafetyModule& safety, CanReceiver& can) {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 100) return; // 10 FPS
  lastUpdate = millis();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (safety.isEmergencyActive()) {
    display.setTextSize(2);
    display.setCursor(10, 10);
    display.print("ERROR:");
    display.setTextSize(3);
    display.setCursor(10, 35);
    if (!safety.oilPresent) display.print("E1");
    else if (safety.tiltDetected) display.print("E2");
    else if (safety.oilTemp > OIL_TEMP_CRITICAL) display.print("E3");
    else if (safety.batteryVoltage < BATTERY_CUTOFF) display.print("E4");
  }
  else if (!can.dataValid) {
    display.setTextSize(2);
    display.setCursor(10, 25);
    display.print("NO CAN");
  }
  else {
    // Gear (Big)
    display.setTextSize(4);
    display.setCursor(5, 15);
    if (gear == 0) display.print("N");
    else if (gear == -1) display.print("R");
    else display.print(gear);

    // RPM Bar (Bottom)
    int barWidth = map(safety.currentRPM, 0, 4000, 0, 128);
    display.fillRect(0, 56, barWidth, 8, SSD1306_WHITE);

    // Info (Right Side)
    display.setTextSize(1);
    display.setCursor(60, 5);
    display.printf("BAT:%.1fV", safety.batteryVoltage);
    display.setCursor(60, 15);
    display.printf("OIL:%.0fC", safety.oilTemp);
    display.setCursor(60, 25);
    display.printf("SPD:%.0f", currentSpeed);

    // Mode
    display.setCursor(60, 35);
    switch(can.driveMode) {
      case 0: display.print("NRML"); break;
      case 1: display.print("SPRT"); break;
      case 2: display.print("S+"); break;
      case 3: display.print("RACE"); break;
    }
  }
  display.display();
}

/* ==================== INSTANCES ==================== */
RPMCounter rpm;
CanReceiver can;
SafetyModule safety;
Gearbox gearbox;
GasPedal gasPedal;
LightsManager lights;

bool blinkerLeftState = false;
bool blinkerRightState = false;
bool warnBlinkState = false;
bool lastBlinkLeftBtn = false;
bool lastBlinkRightBtn = false;
bool lastWarnBtn = false;
unsigned long blinkerStartTime = 0;

/* ==================== SETUP ==================== */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=================================");
  Serial.println("ESP32-S3 Unterbau-Controller v7.0");
  Serial.println("OLED & OBD Diagnostics & Calibration");
  Serial.println("=================================\n");

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  prefs.begin(PREF_NAMESPACE);

  // OLED Init
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    i2cStatus |= (1 << 4); // OLED OK
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("BOOTING...");
    display.display();
  }

  if (mcp1.begin_I2C(MCP1_ADDR)) {
    Serial.println("MCP1 initialized");
    i2cStatus |= (1 << 0);
  }

  if (mcp2.begin_I2C(MCP2_ADDR)) {
    Serial.println("MCP2 initialized");
    i2cStatus |= (1 << 1);
  }

  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_SERVO_MODE,
    .duty_resolution = LEDC_SERVO_RES,
    .timer_num = LEDC_SERVO_TIMER,
    .freq_hz = LEDC_SERVO_FREQ,
    .clk_cfg = LEDC_USE_RC_FAST_CLK
  };
  ledc_timer_config(&timer_conf);

  initExhaustServo();
  gasPedal.begin();

  FastLED.addLeds<WS2812B, PIN_LED_FRONT_VL, GRB>(ledsFrontVL, NUM_FRONT_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_FRONT_VR, GRB>(ledsFrontVR, NUM_FRONT_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_HECK_HL, GRB>(ledsHeckHL, NUM_HECK_HL).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_HECK_HR, GRB>(ledsHeckHR, NUM_HECK_HR).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_SPOILER, GRB>(ledsSpoiler, NUM_SPOILER).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(255);
  FastLED.setMaxRefreshRate(400);

  rpm.begin();
  can.begin();
  safety.begin();
  gearbox.begin();

  BLEDevice::init("ESP2");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* service = pServer->createService(SERVICE_UUID);

  pCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());

  pCharInput = service->createCharacteristic(
    "beb5483e-36e1-4688-b7f5-ea07361b26a8",
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharInput->setCallbacks(new BLECallbacks());

  service->start();
  BLEDevice::getAdvertising()->start();
  Serial.println("BLE Service started");

  // Show I2C Status on Boot
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("I2C CHECK:");
  display.printf("MCP1: %s\n", (i2cStatus & 1) ? "OK" : "ERR");
  display.printf("MCP2: %s\n", (i2cStatus & 2) ? "OK" : "ERR");
  display.printf("AHT20: %s\n", (i2cStatus & 4) ? "OK" : "ERR");
  display.printf("BMP280: %s\n", (i2cStatus & 8) ? "OK" : "ERR");
  display.display();
  delay(2000);
}

/* ==================== LOOP ==================== */
void loop() {
  esp_task_wdt_reset();  // Feed the watchdog

  can.update();
  rpm.update();
  safety.currentRPM = rpm.getRPM();
  safety.update();

  // Blinker Logic (Comfort Blinker)
  bool bL = can.isBlinkLeft();
  bool bR = can.isBlinkRight();
  bool bW = can.isWarnBlink();

  if (bW && !lastWarnBtn) {
    warnBlinkState = !warnBlinkState;
    if (warnBlinkState) { blinkerLeftState = false; blinkerRightState = false; }
  }
  lastWarnBtn = bW;

  if (!warnBlinkState && bL && !lastBlinkLeftBtn) {
    blinkerLeftState = !blinkerLeftState;
    blinkerRightState = false;
    if (blinkerLeftState) blinkerStartTime = millis();
  }
  lastBlinkLeftBtn = bL;

  if (!warnBlinkState && bR && !lastBlinkRightBtn) {
    blinkerRightState = !blinkerRightState;
    blinkerLeftState = false;
    if (blinkerRightState) blinkerStartTime = millis();
  }
  lastBlinkRightBtn = bR;

  // Auto-Cancel Blinker
  if ((blinkerLeftState || blinkerRightState) && !warnBlinkState) {
    if (millis() - blinkerStartTime > 10000 || currentSpeed > 30.0) {
      blinkerLeftState = false;
      blinkerRightState = false;
    }
  }

  bool launchActive = false;
  if (safety.ignitionOn && can.isLaunch() && safety.brakePressed) {
    launchActive = true;
  }

  gasPedal.update(can.driveMode, launchActive, safety.currentRPM);

  // USB Toggle (Debounced)
  static bool lastUsbTaster = true;
  static unsigned long lastUsbPress = 0;
  bool usbTasterPressed = !mcp1.digitalRead(MCP1_USB_TASTER);

  if (usbTasterPressed && !lastUsbTaster && (millis() - lastUsbPress > 200)) {
    usbState = !usbState;
    mcp2.digitalWrite(MCP2_USB_TRANS, usbState ? HIGH : LOW);
    lastUsbPress = millis();
  }
  lastUsbTaster = usbTasterPressed;

  bool drsActive = can.isDRS() || safety.brakePressed;
  mcp2.digitalWrite(MCP2_DRS_L, drsActive ? HIGH : LOW);
  mcp2.digitalWrite(MCP2_DRS_R, drsActive ? HIGH : LOW);

  safety.setCanLoss(!can.dataValid);

  if (!can.dataValid || !safety.systemActive) {
    blinkerLeftState = false;
    blinkerRightState = false;
    warnBlinkState = false;
    lights.update(can, safety, false);
    updateDisplay(gearbox.getGear(), safety, can);
    return;
  }

  setExhaustServoAngle(can.exhaustAngle);
  mcp2.digitalWrite(MCP2_HUPE, can.isHorn() ? HIGH : LOW);

  bool canShift = safety.canShiftDown(safety.currentRPM);

  static bool lastUp = false;
  bool up = can.isShiftUp();
  if (up && !lastUp && safety.ignitionOn) gearbox.shiftUp(canShift);
  lastUp = up;

  static bool lastDown = false;
  bool down = can.isShiftDown();
  if (down && !lastDown && safety.ignitionOn) gearbox.shiftDown(canShift);
  lastDown = down;

  // Reverse Logic
  static bool lastR = false;
  bool rev = can.isReverse() && can.isShiftDown() && safety.brakePressed;
  if (rev && !lastR && canShift && currentSpeed < 2.0) {
    gearbox.shiftToReverse(true);
  }
  lastR = rev;

  lights.update(can, safety, launchActive);
  sendTelemetry(safety, can, gasPedal, gearbox.getGear(), launchActive);
  updateDisplay(gearbox.getGear(), safety, can);
}
