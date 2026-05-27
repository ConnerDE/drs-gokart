/* ==================== EGT MAX31855 (TYPE K) ==================== */
class EGTMax31855 {
  float temperatureC = 0.0f;
  uint8_t faultFlags = 0;
  bool valid = false;
  unsigned long lastRead = 0;

  uint32_t readRaw32() {
    uint32_t value = 0;

    mcp1.digitalWrite(MCP1_EGT_CS, LOW);
    delayMicroseconds(2);
    for (uint8_t i = 0; i < 32; i++) {
      mcp1.digitalWrite(MCP1_EGT_SCLK, LOW);
      delayMicroseconds(1);
      value <<= 1;
      if (mcp1.digitalRead(MCP1_EGT_MISO)) value |= 1;
      mcp1.digitalWrite(MCP1_EGT_SCLK, HIGH);
      delayMicroseconds(1);
    }
    mcp1.digitalWrite(MCP1_EGT_CS, HIGH);

    return value;
  }

public:
  void begin() {
    if (!(i2cStatus & (1 << 0))) {
      Serial.println("EGT disabled: MCP1 not available");
      return;
    }
    mcp1.pinMode(MCP1_EGT_SCLK, OUTPUT);
    mcp1.pinMode(MCP1_EGT_CS, OUTPUT);
    mcp1.pinMode(MCP1_EGT_MISO, INPUT);
    mcp1.digitalWrite(MCP1_EGT_CS, HIGH);
    mcp1.digitalWrite(MCP1_EGT_SCLK, LOW);
    Serial.println("EGT MAX31855 initialized");
  }

  void update() {
    if (!(i2cStatus & (1 << 0))) return;
    if (millis() - lastRead < 250) return;
    lastRead = millis();

    uint32_t raw = readRaw32();
    faultFlags = raw & 0x07;
    valid = ((raw & 0x00010000UL) == 0) && faultFlags == 0;
    if (!valid) return;

    int16_t tempRaw = (raw >> 18) & 0x3FFF;
    if (tempRaw & 0x2000) tempRaw |= 0xC000;
    temperatureC = tempRaw * 0.25f;
  }

  float getTemperatureC() const { return temperatureC; }
  bool isValid() const { return valid; }
  uint8_t getFaultFlags() const { return faultFlags; }
};
