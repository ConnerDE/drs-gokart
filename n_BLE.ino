
/* ==================== BLE ==================== */
class BLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String value = pChar->getValue();
    if (value.length() > 0) {
      String s = String(value.c_str());

      // Calibration Commands
      if (s == "CAL:GAS_MIN") {
        gasPedal.setPedalMinFromCurrent();
        calGasMin = gasPedal.getRawPos();
      } else if (s == "CAL:GAS_MAX") {
        gasPedal.setPedalMaxFromCurrent();
        calGasMax = gasPedal.getRawPos();
      } else if (s == "CAL:STEER_LEFT") {
        steering.setLeftLimit();
      } else if (s == "CAL:STEER_CENTER") {
        steering.setCenter();
      } else if (s == "CAL:STEER_RIGHT") {
        steering.setRightLimit();
      } else if (s.startsWith("SET:SRV_GAS_MIN:")) {
        calSrvGasMin = constrain(s.substring(16).toInt(), 0, 180 - GAS_SERVO_TRAVEL_DEG);
        calSrvGasMax = constrain(calSrvGasMax, calSrvGasMin, calSrvGasMin + GAS_SERVO_TRAVEL_DEG);
        prefs.putInt(PREF_SRV_GAS_MIN, calSrvGasMin);
        prefs.putInt(PREF_SRV_GAS_MAX, calSrvGasMax);
      } else if (s.startsWith("SET:SRV_GAS_MAX:")) {
        calSrvGasMax = constrain(s.substring(16).toInt(), calSrvGasMin, calSrvGasMin + GAS_SERVO_TRAVEL_DEG);
        prefs.putInt(PREF_SRV_GAS_MAX, calSrvGasMax);
      } else if (s.startsWith("SET:SRV_EXH_MIN:")) {
        calSrvExhMin = constrain(s.substring(16).toInt(), 0, 180);
        prefs.putInt(PREF_SRV_EXH_MIN, calSrvExhMin);
      } else if (s.startsWith("SET:SRV_EXH_MAX:")) {
        calSrvExhMax = constrain(s.substring(16).toInt(), 0, 180);
        prefs.putInt(PREF_SRV_EXH_MAX, calSrvExhMax);
      }

      // Reset Commands
      else if (s == "RESET_HOURS") {
        engineHours = 0;
        prefs.putFloat(PREF_HOURS, 0.0);
      } else if (s == "RESET_CHAIN") {
        chainShifts = 0;
        prefs.putUInt(PREF_CHAIN, 0);
      } else if (s == "RESET_ENERGY") {
        currentSensor.resetEnergy();
      } else if (s == "CAL:CURRENT_ZERO") {
        currentSensor.calibrateZero();
      } else if (s.startsWith("SPD:")) {
        currentSpeed = constrain(s.substring(4).toFloat(), 0.0f, 255.0f);
      }
      // OBD / Test Commands
      else if (s == "OBD:ON") {
        obdMode = true;
        forceBleTelemetry = true;
      } else if (s == "OBD:OFF") {
        obdMode = false;
        forceBleTelemetry = true;
      } else if (s == "OBD:SNAP") {
        forceBleTelemetry = true;
      } else if (s == "TEST:START") {
        bleTestMode = true;
        obdMode = true;
        bleTestStep = 1;
        forceBleTelemetry = true;
      } else if (s == "TEST:STOP") {
        bleTestMode = false;
        stopBleTestModeOutputs();
        forceBleTelemetry = true;
      } else if (s == "TEST:NEXT") {
        bleTestMode = true;
        obdMode = true;
        bleTestStep = (bleTestStep % 7) + 1;
        forceBleTelemetry = true;
      } else if (s.startsWith("TEST:STEP:")) {
        bleTestMode = true;
        obdMode = true;
        bleTestStep = constrain(s.substring(10).toInt(), 0, 7);
        forceBleTelemetry = true;
      }
      // DRS Show Mode
      else if (s == "DRS:SHOW_ON") {
        setDRSShowMode(true);
      } else if (s == "DRS:SHOW_OFF") {
        setDRSShowMode(false);
      }
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { deviceConnected = true; }
  void onDisconnect(BLEServer*) override { deviceConnected = false; }
};

static const char* okText(bool ok) {
  return ok ? "OK" : "MISS";
}

void addObdData(JsonDocument& doc, SafetyModule& safety, CanReceiver& can, GasPedal& gas, int8_t gear) {
  doc["obd"] = obdMode;
  doc["test"] = bleTestMode;
  doc["testStep"] = bleTestStep;
  doc["uptime_ms"] = millis();
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["reset"] = (int)esp_reset_reason();

  JsonObject hw = doc["hw"].to<JsonObject>();
  hw["mcp1"] = okText(i2cStatus & (1 << 0));
  hw["mcp2"] = okText(i2cStatus & (1 << 1));
  hw["aht20"] = okText(i2cStatus & (1 << 2));
  hw["bmp280"] = okText(i2cStatus & (1 << 3));
  hw["oled"] = USE_OLED_DISPLAY ? okText(i2cStatus & (1 << 4)) : "OFF";
  hw["ds18b20"] = okText(i2cStatus & (1 << 5));
  hw["ads1115"] = okText(ads1115.initialized);
  hw["can"] = can.initialized ? (can.dataValid ? "OK" : "NO_DATA") : "MISS";
  hw["canChassis"] = can.chassisDataValid ? "OK" : "NO_DATA";
  hw["egt"] = egtSensor.isValid() ? "OK" : "NO_DATA";

  JsonObject in = doc["in"].to<JsonObject>();
  in["brake"] = safety.brakePressed;
  in["neutral"] = safety.neutralActive;
  in["oil"] = safety.oilPresent;
  in["tilt"] = safety.tiltDetected;
  in["hydHot"] = safety.hydPumpOverTemp;
  in["usbBtn"] = (i2cStatus & (1 << 0)) ? !mcp1.digitalRead(MCP1_USB_TASTER) : false;
  in["startBtn"] = (i2cStatus & (1 << 1)) ? !mcp2.digitalRead(MCP2_START_BTN) : false;
  in["revEnd"] = (i2cStatus & (1 << 1)) ? !mcp2.digitalRead(MCP2_ENDSTOP_R) : false;

  JsonObject adc = doc["adc"].to<JsonObject>();
  adc["gasRaw"] = gas.getRawPos();
  adc["gasPct"] = gas.getGasPercent();
  adc["steerRaw"] = steering.readRaw();
  adc["steerPct"] = steering.getSteerPercent();
  adc["currRaw"] = ads1115.readCurrentSensor();
  adc["currV"] = currentSensor.getSensorVoltageV();

  JsonObject out = doc["out"].to<JsonObject>();
  out["gear"] = gear;
  out["ignition"] = safety.ignitionOn;
  out["starter"] = safety.isStarterActive();
  out["drs"] = isDRSActive();
  out["usb"] = usbState;
  out["horn"] = can.isHorn();
  out["exhAng"] = can.exhaustAngle;
}

void sendTelemetry(SafetyModule& safety, CanReceiver& can, GasPedal& gas, int8_t gear, bool launch) {
  static unsigned long last = 0;
  if (!deviceConnected) return;
  if (!forceBleTelemetry && millis() - last < 500) return;
  forceBleTelemetry = false;
  last = millis();

  JsonDocument doc;
  doc["gear"] = gear;
  if (can.chassisDataValid) {
    doc["vRef"] = can.vRefKmh;
    doc["vDrv"] = can.vDriveKmh;
    doc["asr"] = can.shouldCutForSlip();
    doc["slip"] = can.slipLampActive();
    doc["slipPct"] = can.currentSlip;
    doc["slipLimit"] = can.slipLimitPercent();
    doc["asrErr"] = can.asrSensorError;
    doc["asrOff"] = can.asrOff;
    doc["drift"] = can.driftMode;
    doc["vVL"] = can.speedVLKmh;
    doc["vVR"] = can.speedVRKmh;
    doc["vHA"] = can.speedHAKmh;
  }
  doc["rpm"] = safety.currentRPM;
  doc["bat"] = safety.batteryVoltage;
  doc["bat_mV"] = (uint16_t)(safety.batteryVoltage * 1000.0f);
  doc["oil"] = safety.oilTemp;
  doc["mode"] = can.driveMode;
  doc["gas"] = gas.getGasPercent();
  doc["launch"] = launch;
  doc["starter"] = safety.isStarterActive();
  doc["light"] = can.lightMode;
  doc["rgb"] = can.rgbMode;
  doc["env"] = safety.outsideTemp;
  doc["press"] = safety.airPressure;
  doc["hours"] = engineHours;
  doc["shifts"] = totalShifts;

  // Batterie-/Stromsensor Daten
  doc["curr_A"] = currentSensor.getCurrentA();
  doc["curr_mA"] = currentSensor.getCurrentMA();
  doc["power_W"] = currentSensor.PowerWatt();
  doc["energy_Wh"] = currentSensor.getEnergyWh();
  doc["curr_peak_A"] = currentSensor.getPeakCurrentA();
  doc["curr_zero_V"] = currentSensor.getZeroOffsetV();
  doc["egt"] = egtSensor.getTemperatureC();
  doc["egtOK"] = egtSensor.isValid();
  doc["steerRaw"] = steering.readRaw();
  doc["hydHot"] = safety.hydPumpOverTemp;

  // OBD Diagnostics (Bitmask)
  uint32_t diag = 0;
  if (safety.brakePressed) diag |= (1 << 0);
  if (safety.neutralActive) diag |= (1 << 1);
  if (safety.tiltDetected) diag |= (1 << 2);
  if (safety.oilPresent) diag |= (1 << 3);
  if (safety.ignitionOn) diag |= (1 << 4);
  if (can.dataValid) diag |= (1 << 5);
  if (safety.hydPumpOverTemp) diag |= (1 << 6);
  // I2C Status (Bits 8-15)
  diag |= (i2cStatus << 8);

  doc["diag"] = diag;

  if (safety.hydPumpOverTemp) doc["warn"] = "HYD_PUMP_TEMP";
  else if (engineHours > SERVICE_HOURS_LIMIT) doc["warn"] = "SERVICE";
  else if (chainShifts > CHAIN_SHIFTS_LIMIT) doc["warn"] = "CHAIN";
  else doc["warn"] = "OK";

  String exhStr = "Closed";
  if (can.exhaustAngle == 45) exhStr = "Partial";
  if (can.exhaustAngle == 90) exhStr = "Open";
  doc["exh"] = exhStr;
  doc["exhAng"] = can.exhaustAngle;
  doc["drs"]      = (uint8_t)getDRSState();  // 0=Disabled, 1=Armed, 2=Active
  doc["drsShow"]  = drsShowMode;

  if (obdMode || bleTestMode) {
    addObdData(doc, safety, can, gas, gear);
  }

  String json;
  serializeJson(doc, json);
  pCharacteristic->setValue(json.c_str());
  pCharacteristic->notify();
}

void stopBleTestModeOutputs() {
  setDRS(false);
  gasPedal.cutThrottle();
  setExhaustServoAngle(0);
  blinkerLeftState = false;
  blinkerRightState = false;
  warnBlinkState = false;
  if (i2cStatus & (1 << 1)) {
    mcp2.digitalWrite(MCP2_USB_TRANS, LOW);
    mcp2.digitalWrite(MCP2_TRANS1, LOW);
    mcp2.digitalWrite(MCP2_TRANS3, LOW);
    mcp2.digitalWrite(MCP2_FERN, LOW);
    mcp2.digitalWrite(MCP2_ABBLEND, LOW);
    mcp2.digitalWrite(MCP2_PIEZO, LOW);
    mcp2.digitalWrite(MCP2_HUPE, LOW);
  }
  fill_solid(ledsFrontVL, NUM_FRONT_LEDS, CRGB::Black);
  fill_solid(ledsFrontVR, NUM_FRONT_LEDS, CRGB::Black);
  fill_solid(ledsHeckHL, NUM_HECK_HL, CRGB::Black);
  fill_solid(ledsHeckHR, NUM_HECK_HR, CRGB::Black);
  fill_solid(ledsSpoiler, NUM_SPOILER, CRGB::Black);
  FastLED.show();
}

void runBleTestMode() {
  static uint8_t lastStep = 255;
  static unsigned long lastPulse = 0;
  static bool pulse = false;

  if (!bleTestMode) return;
  if (safety.ignitionOn || safety.currentRPM > STARTER_RPM_CUTOFF) {
    bleTestMode = false;
    stopBleTestModeOutputs();
    return;
  }

  if (bleTestStep != lastStep) {
    stopBleTestModeOutputs();
    lastStep = bleTestStep;
  }

  if (millis() - lastPulse > 500) {
    lastPulse = millis();
    pulse = !pulse;
  }

  switch (bleTestStep) {
    case 1:
      fill_solid(ledsFrontVL, NUM_FRONT_LEDS, CRGB::White);
      fill_solid(ledsFrontVR, NUM_FRONT_LEDS, CRGB::White);
      FastLED.show();
      break;
    case 2:
      fill_solid(ledsHeckHL, NUM_HECK_HL, CRGB::Red);
      fill_solid(ledsHeckHR, NUM_HECK_HR, CRGB::Red);
      fill_solid(ledsSpoiler, NUM_SPOILER, CRGB::Red);
      FastLED.show();
      break;
    case 3:
      blinkerLeftState = pulse;
      blinkerRightState = pulse;
      warnBlinkState = true;
      lights.update(can, safety, false, steering.getAbglichSteerPercent());
      break;
    case 4:
      if (i2cStatus & (1 << 1)) {
        mcp2.digitalWrite(MCP2_ABBLEND, pulse ? HIGH : LOW);
        mcp2.digitalWrite(MCP2_FERN, !pulse ? HIGH : LOW);
      }
      break;
    case 5:
      setDRS(pulse);
      break;
    case 6:
      setExhaustServoAngle(pulse ? 90 : 0);
      break;
    case 7:
      if (i2cStatus & (1 << 1)) {
        mcp2.digitalWrite(MCP2_PIEZO, pulse ? HIGH : LOW);
        mcp2.digitalWrite(MCP2_HUPE, pulse ? HIGH : LOW);
      }
      break;
    default:
      stopBleTestModeOutputs();
      break;
  }
}
