
/* ==================== GEARBOX ==================== */
class Gearbox {
  int8_t currentGear = 0;

public:
  void begin() {
    // TMC2209 PDN_UART is a single-wire UART. Keep RX and TX on the configured
    // hardware pin so Serial1 does not fall back to a default TX pin used by LEDs.
    Serial.println("Initializing Serial1 for TMC2209...");
    Serial1.begin(115200, SERIAL_8N1, PIN_TMC_UART, PIN_TMC_UART);
    delay(100);

    Serial.println("Initializing TMC2209 Driver...");
    driver.begin();
    driver.toff(4);
    driver.blank_time(24);
    driver.rms_current(1200);
    driver.microsteps(16);
    driver.TCOOLTHRS(0xFFFFF);
    driver.SGTHRS(STALL_VALUE);
    Serial.println("TMC2209 OK");

    pinMode(PIN_TMC_DIR, OUTPUT);
    pinMode(PIN_TMC_STEP, OUTPUT);
    pinMode(PIN_TMC_EN, OUTPUT);
    digitalWrite(PIN_TMC_EN, LOW);

    if ((i2cStatus & (1 << 1)) && !mcp2.digitalRead(MCP2_ENDSTOP_R)) {
      currentGear = -1;
    } else {
      currentGear = prefs.getInt(PREF_GEAR, 0);
    }

    Serial.println("Gearbox initialized");
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
