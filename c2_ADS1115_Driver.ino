/* ==================== ADS1115 ADC DRIVER ==================== */
// Adafruit ADS1115 - 4-Kanal 16-Bit ADC über I2C (1 LSB = 187.5µV @ ±6.144V)
// Kanäle: 0=Gaspedal, 2=Lenkwinkel, 3=Stromsensor (ACS758 100A)

class ADS1115Driver {
  Adafruit_ADS1115 ads;  // 0x48

  public:
  bool initialized = false;

  void begin() {
    Serial.println("Initializing ADS1115 ADC...");
    if (!ads.begin(ADS1115_ADDRESS)) {
    Serial.println("ADS1115 not found!");
    return;
  }

    // Konfiguriere alle Kanäle
    ads.setGain(GAIN_TWOTHIRDS);  // +/- 6.144V
    ads.setDataRate(RATE_ADS1115_128SPS);  // 128 samples per second
    Serial.println("ADS1115 OK");
    initialized = true;
  }

  // Lese Kanal mit Mittelwertbildung - OHNE zu blockieren
  int16_t readChannel(uint8_t channel, uint8_t samples = 5) {
    if (!initialized) return 0;
    int32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
    sum += ads.readADC_SingleEnded(channel);
    // KEIN delay(2) - das blockiert den Watchdog!
    // Die ADS1115 Lib wartet bereits intern!
    delayMicroseconds(100);  // Nur minimaler Delay für Bus
  }
    return sum / samples;
  }

  // Konvertiere ADC Value zu Spannung (mV)
  float readVoltage(uint8_t channel, uint8_t samples = 5) {
    if (!initialized) return 0;
    int16_t raw = readChannel(channel, samples);
    return ads.computeVolts(raw) * 1000.0;  // in mV
  }

  float rawToVolts(int16_t raw) {
    if (!initialized) return 0.0f;
    return ads.computeVolts(raw);
  }

  // Gaspedal Kanal (ADS1115 Eingang 0) - 10k Poti
  int16_t readGasPoti() {
    return readChannel(ADS1115_CHANNEL_GAS, 3);  // 3 Samples für schnelle Response
  }

  // Stromsensor Kanal (ADS1115 Eingang 3) - ACS758 100A
  int16_t readCurrentSensor() {
    return readChannel(ADS1115_CHANNEL_CURRENT, 5);  // 5 Samples für Stabilität
  }

  // Lenkwinkel Kalibrierungs-Kanal (ADS1115 Eingang 2)
  int16_t readSteerCalib() {
    return readChannel(ADS1115_CHANNEL_STEER, 3);
  }
};

// Globale ADS1115 Instanz
ADS1115Driver ads1115;