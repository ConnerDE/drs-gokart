
/* ==================== CAN RECEIVER ==================== */
class CanReceiver {
  unsigned long lastFrame = 0;
  unsigned long lastChassisFrame = 0;
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

  // Fahrwerk-Knoten (ESP32-C3) @ 0x123
  uint8_t vRefKmh = 0;
  uint8_t vDriveKmh = 0;
  bool asrActive = false;
  bool asrOff = false;
  bool driftMode = false;
  bool asrSensorError = false;
  bool tractionIntervention = false;
  uint8_t currentSlip = 0;
  int16_t speedVLRaw = 0;
  int16_t speedVRRaw = 0;
  int16_t speedHARaw = 0;
  float speedVLKmh = 0.0f;
  float speedVRKmh = 0.0f;
  float speedHAKmh = 0.0f;
  uint8_t rawSpeedVL = 0;
  uint8_t rawSpeedVR = 0;
  bool chassisDataValid = false;

  // Lenkung-Knoten Daten (ESP32 Lenkung) - wird vom 3. ESP gesendet
  int8_t steerAngleCAN = 0;  // Lenkwinkel vom Lenkung-ESP (-100 bis +100)
  bool initialized = false;

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

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
      Serial.printf("CAN init failed: %d\n", err);
      initialized = false;
      return;
    }
    err = twai_start();
    if (err != ESP_OK) {
      Serial.printf("CAN start failed: %d\n", err);
      initialized = false;
      return;
    }
    initialized = true;
    Serial.println("CAN initialized successfully");
  }

  void update() {
    if (!initialized) {
      dataValid = false;
      return;
    }

    uint32_t alerts;
    if (twai_read_alerts(&alerts, 0) == ESP_OK) {
      if (alerts & TWAI_ALERT_BUS_OFF) twai_initiate_recovery();
      if (alerts & TWAI_ALERT_BUS_RECOVERED) twai_start();
    }

    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      if (message.identifier == CAN_ID_ASR && message.data_length_code == 8) {
        speedVLRaw = (int16_t)((message.data[0] << 8) | message.data[1]);
        speedVRRaw = (int16_t)((message.data[2] << 8) | message.data[3]);
        speedHARaw = (int16_t)((message.data[4] << 8) | message.data[5]);
        speedVLKmh = speedVLRaw / 10.0f;
        speedVRKmh = speedVRRaw / 10.0f;
        speedHAKmh = speedHARaw / 10.0f;
        rawSpeedVL = (uint8_t)constrain((int)roundf(speedVLKmh), 0, 255);
        rawSpeedVR = (uint8_t)constrain((int)roundf(speedVRKmh), 0, 255);
        vRefKmh = (uint8_t)constrain((int)roundf((speedVLKmh + speedVRKmh) * 0.5f), 0, 255);
        vDriveKmh = (uint8_t)constrain((int)roundf(speedHAKmh), 0, 255);
        uint8_t statusFlags = message.data[6];
        asrSensorError = (statusFlags & 0x01) != 0;
        driftMode = (statusFlags & 0x02) != 0;
        asrOff = (statusFlags & 0x04) != 0;
        tractionIntervention = (statusFlags & 0x08) != 0;
        currentSlip = constrain(message.data[7], 0, 100);
        asrActive = tractionIntervention;
        chassisDataValid = true;
        lastChassisFrame = millis();
        currentSpeed = (float)vRefKmh;
      } else if (message.identifier == CAN_ID_CHASSIS && message.data_length_code >= 6) {
        vRefKmh = message.data[0];
        vDriveKmh = message.data[1];
        asrActive = message.data[2] != 0;
        tractionIntervention = asrActive;
        asrOff = (message.data[3] & 0x01) != 0;
        driftMode = (message.data[3] & 0x02) != 0;
        // Kompatibilität: C3 sendet Drift aktuell als Byte3=1 ohne Bitmaske
        if (message.data[3] == 1 && (message.data[3] & 0x02) == 0) {
          driftMode = true;
          asrOff = false;
        }
        if (asrActive) {
          currentSlip = driftMode ? ASR_SLIP_DRIFT_PCT :
                        ((driveMode == DRIVE_MODE_OFFROAD) ? ASR_SLIP_OFFROAD_PCT : ASR_SLIP_NORMAL_PCT);
        } else {
          currentSlip = 0;
        }
        rawSpeedVL = message.data[4];
        rawSpeedVR = message.data[5];
        chassisDataValid = true;
        lastChassisFrame = millis();
        currentSpeed = (float)vRefKmh;
      } else if (message.data_length_code >= 8) {
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
    if (millis() - lastChassisFrame > CAN_TIMEOUT_MS) {
      chassisDataValid = false;
      asrActive = false;
      tractionIntervention = false;
      currentSlip = 0;
    }
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

  bool isOffroadMode() const { return driveMode == DRIVE_MODE_OFFROAD; }
  uint8_t slipLimitPercent() const {
    if (asrOff) return 100;
    if (driftMode) return ASR_SLIP_DRIFT_PCT;
    if (isOffroadMode()) return ASR_SLIP_OFFROAD_PCT;
    return ASR_SLIP_NORMAL_PCT;
  }
  bool shouldCutForSlip() const {
    if (!chassisDataValid || asrOff) return false;
    if (asrSensorError) return true;
    if (!tractionIntervention && currentSlip <= slipLimitPercent()) return false;
    return currentSlip >= slipLimitPercent();
  }
  bool slipLampActive() const {
    return chassisDataValid && !asrOff && (tractionIntervention || currentSlip >= slipLimitPercent());
  }
};
