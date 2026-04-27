
/* ==================== SETUP ==================== */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=================================");
  Serial.println("ESP32-S3 Unterbau-Controller v7.0");
  Serial.println("OLED & OBD Diagnostics & Calibration");
  Serial.println("=================================\n");

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  prefs.begin(PREF_NAMESPACE);

  // OLED Init
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    i2cStatus |= (1 << 4); // OLED OK
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("BOOTING...");
    display.display();
  }

  if (mcp1.begin_I2C(MCP1_ADDR)) {
    Serial.println("MCP1 initialized");
    i2cStatus |= (1 << 0);
  }

  if (mcp2.begin_I2C(MCP2_ADDR)) {
    Serial.println("MCP2 initialized");
    i2cStatus |= (1 << 1);
  }

  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_SERVO_MODE,
    .duty_resolution = LEDC_SERVO_RES,
    .timer_num = LEDC_SERVO_TIMER,
    .freq_hz = LEDC_SERVO_FREQ,
    .clk_cfg = LEDC_USE_RC_FAST_CLK
  };
  ledc_timer_config(&timer_conf);

  initExhaustServo();
  gasPedal.begin();

  FastLED.addLeds<WS2812B, PIN_LED_FRONT_VL, GRB>(ledsFrontVL, NUM_FRONT_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_FRONT_VR, GRB>(ledsFrontVR, NUM_FRONT_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_HECK_HL, GRB>(ledsHeckHL, NUM_HECK_HL).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_HECK_HR, GRB>(ledsHeckHR, NUM_HECK_HR).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_SPOILER, GRB>(ledsSpoiler, NUM_SPOILER).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(255);
  FastLED.setMaxRefreshRate(400);

  rpm.begin();
  can.begin();
  safety.begin();
  gearbox.begin();

  BLEDevice::init("ESP2");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* service = pServer->createService(SERVICE_UUID);

  pCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());

  pCharInput = service->createCharacteristic(
    "beb5483e-36e1-4688-b7f5-ea07361b26a8",
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharInput->setCallbacks(new BLECallbacks());

  service->start();
  BLEDevice::getAdvertising()->start();
  Serial.println("BLE Service started");

  // Show I2C Status on Boot
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("I2C CHECK:");
  display.printf("MCP1: %s\n", (i2cStatus & 1) ? "OK" : "ERR");
  display.printf("MCP2: %s\n", (i2cStatus & 2) ? "OK" : "ERR");
  display.printf("AHT20: %s\n", (i2cStatus & 4) ? "OK" : "ERR");
  display.printf("BMP280: %s\n", (i2cStatus & 8) ? "OK" : "ERR");
  display.display();
  delay(2000);
}
