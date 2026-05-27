#if 0
/* Legacy steering tab. Active class is in c5_STEERING.ino. */
/* ==================== LENKWINKEL (ADS1115 + CAN) ==================== */
class SteeringInput {
  int minRaw = 500;
  int centerRaw = 16000;
  int maxRaw = 31000;
  int8_t lastSteerPercent = 0;
  static constexpr int DEADZONE = 300;
  static constexpr int CAN_TOLERANCE = 20;

  int8_t canSteerPercent = 0;

public:
  void begin() {
    minRaw = prefs.getInt(PREF_STEER_MIN, 500);
    centerRaw = prefs.getInt(PREF_STEER_CENTER, 16000);
    maxRaw = prefs.getInt(PREF_STEER_MAX, 31000);
    Serial.printf("SteeringInput: Cal min/center/max = %d/%d/%d\n", minRaw, centerRaw, maxRaw);
  }

  int16_t readRaw() {
    return ads1115.readSteerCalib();
  }

  void setLeftLimit() {
    minRaw = readRaw();
    prefs.putInt(PREF_STEER_MIN, minRaw);
  }

  void setCenter() {
    centerRaw = readRaw();
    prefs.putInt(PREF_STEER_CENTER, centerRaw);
  }

  void setRightLimit() {
    maxRaw = readRaw();
    prefs.putInt(PREF_STEER_MAX, maxRaw);
  }

  int8_t getSteerPercent() {
    int raw = readRaw();
    int delta = raw - centerRaw;
    if (abs(delta) < DEADZONE) return 0;

    int range = (delta < 0) ? (centerRaw - minRaw) : (maxRaw - centerRaw);
    range = max(range, DEADZONE + 1);
    int pct = map(abs(delta), DEADZONE, range, 0, 100);
    pct = constrain(pct, 0, 100);
    lastSteerPercent = (delta < 0) ? (int8_t)(-pct) : (int8_t)pct;
    return lastSteerPercent;
  }

  void setCANSteerPercent(int8_t percent) {
    canSteerPercent = percent;
  }

  int8_t getAbglichSteerPercent() {
    int8_t localSteer = getSteerPercent();

    if (canSteerPercent != 0) {
      int8_t diff = abs(localSteer - canSteerPercent);
      if (diff > CAN_TOLERANCE) {
        static unsigned long lastWarn = 0;
        if (millis() - lastWarn > 5000) {
          Serial.printf("Steering Discrepancy: Local=%d, CAN=%d (Diff=%d%%)\n",
                        localSteer, canSteerPercent, diff);
          lastWarn = millis();
        }
      }
      return (int8_t)((localSteer * 70 + canSteerPercent * 30) / 100);
    }

    return localSteer;
  }

  int8_t getLastSteer() const { return lastSteerPercent; }
  int getMinRaw() const { return minRaw; }
  int getCenterRaw() const { return centerRaw; }
  int getMaxRaw() const { return maxRaw; }
};
#endif
