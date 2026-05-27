
/* ==================== SETUP ==================== */
static void initTaskWatchdogAfterBoot() {
  esp_task_wdt_deinit();

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };

  esp_err_t err = esp_task_wdt_init(&wdt_config);
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_add(NULL);
    Serial.println("Task watchdog active");
  } else {
    Serial.printf("Task watchdog init failed: %d\n", err);
  }
  Serial.flush();
}

static void feedTaskWatchdog() {
  esp_task_wdt_reset();
}

void setup() {
  Serial.begin(115200);
  unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < 1500) {
    delay(10);
  }
  Serial.println("\n\n=================================");
  Serial.println("ESP32-S3 Unterbau-Controller v7.0");
  Serial.println("OLED & OBD Diagnostics & Calibration");
  Serial.printf("Reset reason CPU0: %d\n", (int)esp_reset_reason());
  Serial.println("=================================\n");
  Serial.flush();

  initTaskWatchdogAfterBoot();
  feedTaskWatchdog();

  Serial.println("Initializing Wire (I2C)...");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  delay(100);
  feedTaskWatchdog();

  Serial.println("Initializing Preferences...");
  prefs.begin(PREF_NAMESPACE);
  delay(100);
  feedTaskWatchdog();

  // OLED Init
  if (USE_OLED_DISPLAY) {
    Serial.println("Initializing OLED Display...");
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
      Serial.println("OLED OK");
    }
  } else {
    Serial.println("OLED disabled");
  }
  delay(100);
  feedTaskWatchdog();

  Serial.println("Initializing MCP1...");
  if (mcp1.begin_I2C(MCP1_ADDR)) {
    Serial.println("MCP1 initialized");
    i2cStatus |= (1 << 0);
  } else {
    Serial.println("MCP1 FAILED!");
  }
  delay(50);
  feedTaskWatchdog();

  Serial.println("Initializing MCP2...");
  if (mcp2.begin_I2C(MCP2_ADDR)) {
    Serial.println("MCP2 initialized");
    i2cStatus |= (1 << 1);
  } else {
    Serial.println("MCP2 FAILED!");
  }
  delay(50);
  feedTaskWatchdog();

  Serial.println("Configuring LEDC Timer...");
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_SERVO_MODE,
    .duty_resolution = LEDC_SERVO_RES,
    .timer_num = LEDC_SERVO_TIMER,
    .freq_hz = LEDC_SERVO_FREQ,
    .clk_cfg = LEDC_USE_RC_FAST_CLK
  };
  esp_err_t timer_err = ledc_timer_config(&timer_conf);
  if (timer_err != ESP_OK) {
    Serial.printf("LEDC timer config failed: %d\n", timer_err);
  } else {
    Serial.println("LEDC timer OK");
  }
  delay(50);
  feedTaskWatchdog();

  Serial.println("Initializing ADS1115 ADC...");
  ads1115.begin();
  delay(100);
  feedTaskWatchdog();

  Serial.println("Initializing Exhaust Servo...");
  initExhaustServo();
  feedTaskWatchdog();

  Serial.println("Initializing Gas Pedal...");
  gasPedal.begin();
  Serial.println("Gas Pedal OK");
  feedTaskWatchdog();

  Serial.println("Initializing Current Sensor...");
  currentSensor.begin();
  Serial.println("Current Sensor OK");
  delay(50);
  feedTaskWatchdog();

  Serial.println("Initializing EGT MAX31855...");
  egtSensor.begin();
  Serial.println("EGT OK");
  delay(50);
  feedTaskWatchdog();

  Serial.println("Initializing FastLED...");
  FastLED.addLeds<WS2812B, PIN_LED_FRONT_VL, GRB>(ledsFrontVL, NUM_FRONT_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_FRONT_VR, GRB>(ledsFrontVR, NUM_FRONT_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_HECK_HL, GRB>(ledsHeckHL, NUM_HECK_HL).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_HECK_HR, GRB>(ledsHeckHR, NUM_HECK_HR).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, PIN_LED_SPOILER, GRB>(ledsSpoiler, NUM_SPOILER).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(255);
  FastLED.setMaxRefreshRate(400);
  Serial.println("FastLED OK");
  delay(50);
  feedTaskWatchdog();

  Serial.println("Initializing RPM Counter...");
  rpm.begin();
  Serial.println("RPM Counter OK");
  feedTaskWatchdog();

  Serial.println("Initializing CAN...");
  can.begin();
  Serial.println("CAN OK");
  feedTaskWatchdog();

  Serial.println("Initializing Steering...");
  steering.begin();
  Serial.println("Steering OK");
  feedTaskWatchdog();

  Serial.println("Initializing Safety Module...");
  safety.begin();
  Serial.println("Safety OK");
  feedTaskWatchdog();

  Serial.println("Initializing Gearbox...");
  gearbox.begin();
  Serial.println("Gearbox OK");
  feedTaskWatchdog();

  Serial.println("Initializing BLE...");
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
  feedTaskWatchdog();

  // Show I2C Status on Boot
  if (i2cStatus & (1 << 4)) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("I2C CHECK:");
    display.printf("MCP1: %s\n", (i2cStatus & 1) ? "OK" : "ERR");
    display.printf("MCP2: %s\n", (i2cStatus & 2) ? "OK" : "ERR");
    display.printf("AHT20: %s\n", (i2cStatus & 4) ? "OK" : "ERR");
    display.printf("BMP280: %s\n", (i2cStatus & 8) ? "OK" : "ERR");
    display.display();
  }
  for (uint8_t i = 0; i < 20; i++) {
    delay(100);
    feedTaskWatchdog();
  }
}
