/* ==================== CURRENT SENSOR (ACS758 100A) ==================== */
class CurrentSensor {
  float currentMA = 0;
  float powerWatt = 0;
  float peakCurrentA = 0;
  float energyWh = 0;
  uint16_t batteryVoltageADC = 0;
  unsigned long lastEnergyUpdate = 0;

  float zeroOffsetV = CURRENT_SENSOR_ZERO_OFFSET_V;
  static constexpr float SENSITIVITY_MV_PER_A = CURRENT_SENSOR_SENSITIVITY_MV_PER_A;  // ACS758: 26.5mV/A
  static constexpr float NOISE_FLOOR_A = CURRENT_SENSOR_NOISE_FLOOR_A;
  static constexpr float MAX_CURRENT_A = CURRENT_SENSOR_MAX_A;

  public:
  void begin() {
    Serial.println("Initializing Current Sensor (ACS758 100A)...");
    Serial.printf("  Sensitivity: %.1f mV/A\n", SENSITIVITY_MV_PER_A);
    Serial.printf("  Zero Offset: %.2f V\n", zeroOffsetV);
    Serial.printf("  Max Current: %.0f A\n", MAX_CURRENT_A);
    zeroOffsetV = prefs.getFloat(PREF_CURR_ZERO, CURRENT_SENSOR_ZERO_OFFSET_V);
    Serial.printf("  Calibrated Offset: %.3f V\n", zeroOffsetV);
  }

  void update(uint16_t batteryVoltage_mV) {
    unsigned long now = millis();
    if (!ads1115.initialized) {
    lastEnergyUpdate = now;
    return;
  }

    int16_t currentRaw = ads1115.readCurrentSensor();
    float voltage = ads1115.rawToVolts(currentRaw);

    // Konvertierung: (V - Offset[V]) * 1000mV/V / Sensitivity[mV/A]
    float currentA = ((voltage - zeroOffsetV) * 1000.0f) / SENSITIVITY_MV_PER_A;

    // Rauschunterdrückung - unter Noise Floor = 0A
    if (fabsf(currentA) < NOISE_FLOOR_A) {
    currentA = 0.0f;
  }

    // Limitierung auf physikalischen Bereich (-100A bis +100A)
    currentA = constrain(currentA, -MAX_CURRENT_A, MAX_CURRENT_A);

    currentMA = currentA * 1000.0f;
    powerWatt = (float)batteryVoltage_mV / 1000.0f * fabsf(currentA);
    peakCurrentA = max(peakCurrentA, fabsf(currentA));

    if (lastEnergyUpdate != 0 && now >= lastEnergyUpdate) {
    energyWh += calculateEnergyWh(powerWatt, now - lastEnergyUpdate);
  }
    lastEnergyUpdate = now;
    batteryVoltageADC = batteryVoltage_mV;
  }

  float getSensorVoltageV() const {
    return ads1115.rawToVolts(ads1115.readCurrentSensor());
  }

  void calibrateZero() {
    if (!ads1115.initialized) return;
    // Liest Spannung wenn KEIN Strom fließt (Fahrzeug stillgestellt, alle Verbraucher aus)
    zeroOffsetV = getSensorVoltageV();
    prefs.putFloat(PREF_CURR_ZERO, zeroOffsetV);
    currentMA = 0;
    powerWatt = 0;
    Serial.printf("Current Sensor Zero-Offset calibrated: %.3f V\n", zeroOffsetV);
  }

  void resetEnergy() {
    energyWh = 0;
    peakCurrentA = 0;
    lastEnergyUpdate = millis();
  }

  float getCurrentMA() const { return currentMA; }
  float getCurrentA() const { return currentMA / 1000.0f; }
  float PowerWatt() const { return powerWatt; }
  uint16_t getBatteryVoltage() const { return batteryVoltageADC; }
  float getPeakCurrentA() const { return peakCurrentA; }
  float getEnergyWh() const { return energyWh; }
  float getZeroOffsetV() const { return zeroOffsetV; }

  static float calculateEnergyWh(float powerW, unsigned long timeMS) {
    return (powerW * timeMS) / (1000.0f * 3600.0f);
  }
};