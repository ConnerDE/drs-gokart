#if 0
/* Legacy current sensor tab. Active class is in c3_CURRENT_SENSOR.ino. */
/* ==================== CURRENT SENSOR (ACS770) ==================== */
// ACS770 30A Stromsensor über ADS1115 ADC
// Pin: Connector 2.26 -> ADS1115 Eingang 3 (Kanal 3)
// Konvertierung: 0-5V ADC -> 0-30A Strom (Sensor: 166mV/A @ ±30A)

class CurrentSensor {
  float currentMA = 0;           // Strom in mA
  float powerWatt = 0;           // Leistung in W
  uint16_t batteryVoltageADC = 0;

  static constexpr float VREF = 6.144;     // ADS1115 Referenz (±6.144V)
  static constexpr float OFFSET = 2.5;    // Offsetspannung (2.5V bei 0A)
  static constexpr float SENSITIVITY = 166.0; // mV/A

public:
  void begin() {
    Serial.println("Initializing Current Sensor...");
    // ADC wird von ADS1115 initialisiert
  }

  void update(uint16_t batteryVoltage_mV) {
    if (!ads1115.initialized) return;

    // Lese Stromsensor Kanal vom ADS1115
    int16_t currentRaw = ads1115.readCurrentSensor();
    float voltage = ads1115.rawToVolts(currentRaw);  // in V

    // Konvertiere auf Strom (ACS770: 166mV/A, Offset 2.5V)
    float offsetVoltage = voltage - OFFSET;  // Differenz zu Nullpunkt
    currentMA = (offsetVoltage * 1000.0) / SENSITIVITY;  // in mA, kann negativ sein!

    // Berechne Leistung: P = U * I
    powerWatt = (float)batteryVoltage_mV / 1000.0 * abs(currentMA) / 1000.0;  // in W

    batteryVoltageADC = batteryVoltage_mV;
  }

  float getSensorVoltageV() const {
    return ads1115.rawToVolts(ads1115.readCurrentSensor());
  }

  float getCurrentMA() const { return currentMA; }
  float getCurrentA() const { return currentMA / 1000.0; }
  float PowerWatt() const { return powerWatt; }
  uint16_t getBatteryVoltage() const { return batteryVoltageADC; }

  // Energieverbrauch berechnen (Wh)
  static float calculateEnergyWh(float powerW, unsigned long timeMS) {
    return (powerW * timeMS) / (1000.0 * 3600.0);
  }
};

// Globale Instanz
CurrentSensor currentSensor;
#endif

