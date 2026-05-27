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
  bool starterActive = false;
  bool hydPumpOverTemp = false;
  uint16_t currentRPM = 0;
  StatusLEDMode ledMode = LED_NORMAL;

  void begin() {
    for (uint8_t i = 0; i < BATTERY_SAMPLES; i++) batteryBuf[i] = 12.0;

    Serial.println("Initializing DS18B20...");
    ds18b20.begin();
    if (ds18b20.getDeviceCount() > 0) {
      i2cStatus |= (1 << 5);
      Serial.println("DS18B20 OK");
    } else {
      Serial.println("DS18B20 not found");
    }

    Serial.println("Initializing AHT20...");
    if (aht20.begin()) {
      i2cStatus |= (1 << 2);
      Serial.println("AHT20 OK");
    } else {
      Serial.println("AHT20 not found");
    }

    Serial.println("Initializing BMP280...");
    if (bmp280.begin(0x77)) {
      i2cStatus |= (1 << 3);
      Serial.println("BMP280 OK");
    } else {
      Serial.println("BMP280 not found");
    }

    pinMode(PIN_START_LED, OUTPUT);

    if (i2cStatus & (1 << 0)) {
      mcp1.pinMode(MCP1_NEUTRAL, INPUT_PULLUP);
      mcp1.pinMode(MCP1_OIL, INPUT_PULLUP);
      mcp1.pinMode(MCP1_BRAKE, INPUT_PULLUP);
      mcp1.pinMode(MCP1_TILT, INPUT_PULLUP);
      mcp1.pinMode(MCP1_USB_TASTER, INPUT_PULLUP);
    } else {
      Serial.println("Safety inputs disabled: MCP1 not available");
    }

    if (i2cStatus & (1 << 1)) {
      for (uint8_t i = 0; i < 16; i++) {
        if (i == MCP2_START_BTN || i == MCP2_ENDSTOP_R || i == MCP2_HYD_PUMP_THERM) {
          mcp2.pinMode(i, INPUT_PULLUP);
        } else {
          mcp2.pinMode(i, OUTPUT);
          mcp2.digitalWrite(i, LOW);
        }
      }
    } else {
      Serial.println("Safety outputs disabled: MCP2 not available");
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
      starterActive = false;
      if (i2cStatus & (1 << 1)) {
        mcp2.digitalWrite(MCP2_ZUENDUNG, LOW);
        mcp2.digitalWrite(MCP2_ESTARTER, LOW);
      }
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
    float t1 = outsideTemp;
    float t2 = NAN;
    float p = NAN;

    if (i2cStatus & (1 << 2)) {
      aht20.getEvent(&humidity, &temp);
      t1 = temp.temperature;
    }
    if (i2cStatus & (1 << 3)) {
      t2 = bmp280.readTemperature();
      p = bmp280.readPressure();
    }

    if (!isnan(t2) && t2 > -40 && t2 < 85 && (i2cStatus & (1 << 2))) outsideTemp = (t1 + t2) / 2.0;
    else outsideTemp = t1;

    if (!isnan(p)) airPressure = p / 100.0F;

    if (i2cStatus & (1 << 0)) {
      neutralActive = !mcp1.digitalRead(MCP1_NEUTRAL);
      oilPresent = (mcp1.digitalRead(MCP1_OIL) == LOW);
      brakePressed = !mcp1.digitalRead(MCP1_BRAKE);
      tiltDetected = !mcp1.digitalRead(MCP1_TILT);
    } else {
      neutralActive = true;
      oilPresent = true;
      brakePressed = false;
      tiltDetected = false;
    }

    hydPumpOverTemp = (i2cStatus & (1 << 1)) ? !mcp2.digitalRead(MCP2_HYD_PUMP_THERM) : false;

    bool nowEmergency = !oilPresent || tiltDetected || (oilTemp > OIL_TEMP_CRITICAL && oilTemp > 0);

    if (nowEmergency && !emergency) {
      emergency = true;
      ledMode = LED_EMERGENCY;
      starterActive = false;
      if (i2cStatus & (1 << 1)) {
        mcp2.digitalWrite(MCP2_ZUENDUNG, LOW);
        mcp2.digitalWrite(MCP2_ESTARTER, LOW);
      }
      ignitionOn = false;
      if (i2cStatus & (1 << 1)) mcp2.digitalWrite(MCP2_PIEZO, HIGH);
    } else if (!nowEmergency && emergency) {
      emergency = false;
      ledMode = LED_NORMAL;
      if (i2cStatus & (1 << 1)) mcp2.digitalWrite(MCP2_PIEZO, LOW);
    }

    static bool lastBtn = true;
    static unsigned long lastBtnTime = 0;
    bool btn = (i2cStatus & (1 << 1)) ? mcp2.digitalRead(MCP2_START_BTN) : true;
    bool startButtonPressed = !btn;

    if (startButtonPressed && lastBtn && (millis() - lastBtnTime > 200)) {
      lastBtnTime = millis();
      ignitionOn = !ignitionOn;
      if (i2cStatus & (1 << 1)) mcp2.digitalWrite(MCP2_ZUENDUNG, ignitionOn ? HIGH : LOW);
    }

    starterActive = startButtonPressed && ignitionOn && !emergency && currentRPM < STARTER_RPM_CUTOFF;
    if (i2cStatus & (1 << 1)) {
      mcp2.digitalWrite(MCP2_ESTARTER, starterActive ? HIGH : LOW);
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
  bool isStarterActive() const { return starterActive; }
  bool canShiftDown(uint16_t rpm) { return rpm < RPM_SHIFT_LOCK; }
  void setCanLoss(bool lost) { ledMode = lost ? LED_CAN_LOSS : LED_NORMAL; }
};
