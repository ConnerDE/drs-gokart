// GLOBAL
OneWire oneWire(PIN_OIL_TEMP);
DallasTemperature ds18b20(&oneWire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

CRGB ledsFrontVL[NUM_FRONT_LEDS];
CRGB ledsFrontVR[NUM_FRONT_LEDS];
CRGB ledsHeckHL[NUM_HECK_HL];
CRGB ledsHeckHR[NUM_HECK_HR];
CRGB ledsSpoiler[NUM_SPOILER];

RPMCounter rpm;
CanReceiver can;
SafetyModule safety;
Gearbox gearbox;
GasPedal gasPedal;
LightsManager lights;