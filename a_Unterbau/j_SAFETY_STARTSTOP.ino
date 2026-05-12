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
    if (bmp280.begin(0x77)) i2cStatus |= (1 << 3);

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
      ignitionOn = !ignitionOn;
      mcp2.digitalWrite(MCP2_ZUENDUNG, ignitionOn ? HIGH : LOW);
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