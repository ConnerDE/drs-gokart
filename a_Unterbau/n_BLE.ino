
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
      // DRS Show Mode
      else if (s == "DRS:SHOW_ON") {
        drsShowMode = true;
      } else if (s == "DRS:SHOW_OFF") {
        drsShowMode = false;
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

  doc["drs"]      = (uint8_t)getDRSState();  // 0=Disabled, 1=Armed, 2=Active
  doc["drsShow"]  = drsShowMode;
}

