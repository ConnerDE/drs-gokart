/* ==================== GAS PEDAL & SERVO ==================== */
class GasPedal {
  int32_t potiRaw = 0;              // Aktueller ADC Wert vom Poti (0-32767)
  uint8_t currentGasPercent = 0;
  uint8_t asrCutPercent = 0;

  bool calibrating = false;
  int32_t calPotiMin = 500;         // Min Poti ADC Value (bottom)
  int32_t calPotiMax = 31000;       // Max Poti ADC Value (top)

  void clampServoCalibration() {
    calSrvGasMin = constrain(calSrvGasMin, 0, 180 - GAS_SERVO_TRAVEL_DEG);
    calSrvGasMax = constrain(calSrvGasMax, calSrvGasMin, calSrvGasMin + GAS_SERVO_TRAVEL_DEG);
    if ((calSrvGasMax - calSrvGasMin) > GAS_SERVO_TRAVEL_DEG) {
      calSrvGasMax = calSrvGasMin + GAS_SERVO_TRAVEL_DEG;
    }
  }

  void writeServoAngle(int targetAngle) {
    targetAngle = constrain(targetAngle, calSrvGasMin, calSrvGasMax);
    uint32_t pulseWidth_us = map(targetAngle, 0, 180, 1000, 2000);
    uint32_t duty = (uint32_t)((pulseWidth_us * (uint64_t)LEDC_SERVO_DUTY_MAX) / 20000ULL);
    ledc_set_duty(LEDC_SERVO_MODE, LEDC_SERVO_GAS_CH, duty);
    ledc_update_duty(LEDC_SERVO_MODE, LEDC_SERVO_GAS_CH);
  }

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

    // Load calibration from Preferences
    calPotiMin = prefs.getInt(PREF_GAS_MIN, 500);
    calPotiMax = prefs.getInt(PREF_GAS_MAX, 31000);
    calSrvGasMin = prefs.getInt(PREF_SRV_GAS_MIN, 0);
    calSrvGasMax = prefs.getInt(PREF_SRV_GAS_MAX, GAS_SERVO_TRAVEL_DEG);
    clampServoCalibration();
    prefs.putInt(PREF_SRV_GAS_MIN, calSrvGasMin);
    prefs.putInt(PREF_SRV_GAS_MAX, calSrvGasMax);

    // Kalibrierungsmodus aktiv wenn Min >= Max
    calibrating = (calPotiMin >= calPotiMax);
    if (calibrating) {
      Serial.println("GasPedal: Kalibrierungsmodus aktiv!");
    }
  }

  void cutThrottle() {
    writeServoAngle(calSrvGasMin);
  }

  void updateCalibration() {
    if (!calibrating) return;

    // Lese Poti vom ADS1115
    potiRaw = ads1115.readGasPoti();

    // Finde Min und Max
    static int32_t learnMin = 32767;
    static int32_t learnMax = 0;
    static unsigned long calibStartTime = 0;

    if (calibStartTime == 0) calibStartTime = millis();

    if (potiRaw < learnMin) learnMin = potiRaw;
    if (potiRaw > learnMax) learnMax = potiRaw;

    // Nach 5 Sekunden speichern wenn Range > 5000
    if (millis() - calibStartTime > 5000) {
      if ((learnMax - learnMin) > 5000) {
        calPotiMin = learnMin;
        calPotiMax = learnMax;
        prefs.putInt(PREF_GAS_MIN, calPotiMin);
        prefs.putInt(PREF_GAS_MAX, calPotiMax);
        calibrating = false;
        Serial.printf("GasPedal Kalibrierung abgeschlossen: %d - %d\n", calPotiMin, calPotiMax);
      }
    }
  }

  bool isCalibrating() { return calibrating; }

  void setPedalMinFromCurrent() {
    potiRaw = ads1115.readGasPoti();
    calPotiMin = potiRaw;
    prefs.putInt(PREF_GAS_MIN, calPotiMin);
  }

  void setPedalMaxFromCurrent() {
    potiRaw = ads1115.readGasPoti();
    calPotiMax = potiRaw;
    prefs.putInt(PREF_GAS_MAX, calPotiMax);
  }

  void update(uint8_t driveMode, bool launchActive, uint16_t currentRPM, bool neutralActive,
              bool asrActive, bool driftMode, uint8_t vRefKmh, int8_t steerPercent) {
    (void)neutralActive;

    // Lese aktuellen Poti Wert vom ADS1115
    potiRaw = ads1115.readGasPoti();
    if (calPotiMax <= calPotiMin) {
      cutThrottle();
      return;
    }

    // Constraint auf Kalibrierungsbereich
    int32_t constrainedPos = constrain(potiRaw, calPotiMin, calPotiMax);
    currentGasPercent = map(constrainedPos, calPotiMin, calPotiMax, 0, 100);

    // Normalisiert 0.0 - 1.0
    float inputNorm = (float)(constrainedPos - calPotiMin) / (float)(calPotiMax - calPotiMin);
    float outputNorm = inputNorm;

    // Drive Mode Mapping
    switch (driveMode) {
      case DRIVE_MODE_SPORT:
        outputNorm = pow(inputNorm, 1.5f);
        break;
      case DRIVE_MODE_OFFROAD:
        outputNorm = pow(inputNorm, 1.8f);
        if (inputNorm > 0.65f) outputNorm = 0.55f + (inputNorm - 0.65f) * 1.15f;
        outputNorm = constrain(outputNorm, 0.0f, 0.95f);
        break;
      case DRIVE_MODE_RACE:
        outputNorm = constrain(inputNorm * 1.2f, 0.0f, 1.0f);
        break;
    }

    if (driftMode) {
      outputNorm = constrain(inputNorm * 1.15f, 0.0f, 1.0f);
    }

    int targetAngle = (int)(outputNorm * (calSrvGasMax - calSrvGasMin) + calSrvGasMin);

    // Launch Control
    if (launchActive && currentRPM > 2100) {
      targetAngle = calSrvGasMin;
    }

    // ASR (Anti-Slip Regulation) - Drosselklappe reduzieren
    if (asrActive) {
      uint8_t cutStep = 3;
      if (driftMode) cutStep = 1;
      else if (driveMode == DRIVE_MODE_OFFROAD) cutStep = 2;
      if (asrCutPercent < 95) asrCutPercent = min(95, asrCutPercent + cutStep);
    } else if (asrCutPercent > 0) {
      asrCutPercent = (asrCutPercent > 8) ? asrCutPercent - 8 : 0;
    }

    if (asrCutPercent > 0) {
      int span = calSrvGasMax - calSrvGasMin;
      int cut = (span * asrCutPercent) / 100;
      targetAngle = max(calSrvGasMin, targetAngle - cut);
    }

    // Rollover Protection - Gas begrenzen bei Kurvenfahrt
    if ((float)vRefKmh > ROLLOVER_SPEED_KMH && abs(steerPercent) > ROLLOVER_STEER_PCT) {
      int steerMag = abs(steerPercent);
      int maxGasPct = map(steerMag, ROLLOVER_STEER_PCT, 100, 70, 25);
      int maxAngle = calSrvGasMin + (calSrvGasMax - calSrvGasMin) * maxGasPct / 100;
      if (targetAngle > maxAngle) targetAngle = maxAngle;
    }

    writeServoAngle(targetAngle);
  }

  uint8_t getGasPercent() { return currentGasPercent; }
  int32_t getRawPos() { return potiRaw; }
};
