/* ==================== PIN DEFINITIONS (HARDWARE PINOUT) ==================== */
// Diese Datei dokumentiert die genaue Hardware-Pinbelegung basierend auf der Steckerbestückung

// Connector 1 (GRÜN) - GPIO & Sensoren
#define PIN_BATTERY_ADC       1        // Zündschloss Input - 12V Batterie (ADC)
#define PIN_EXHAUST_SERVO     2        // Auspuff Data SG90 - Servo Signal
#define PIN_STEER_ADC         3        // Lenkwinkel Sensor (ADS1115 ADC Eingang 2)
#define PIN_OIL_TEMP          4        // Motor Temperatur - DS18B20 OneWire
#define PIN_TMC_EN            5        // TMC2209 Enable
#define PIN_TMC_UART          6        // TMC2209 UART Schnittstelle
#define PIN_RPM_HALL          7        // Motor RPM - Drehzahlsensor Hall (A3144)
#define PIN_I2C_SDA           8        // I2C SDA (OLED Display, BMP280, AHT20)
#define PIN_I2C_SCL           9        // I2C SCL (OLED Display, BMP280, AHT20)
#define PIN_CAN_RX            10       // CAN RX - SN65HVD Transceiver
#define PIN_CAN_TX            11       // CAN TX - SN65HVD Transceiver
#define PIN_LED_SPOILER       12       // Spoiler Beleuchtung - WS2812B (187 LEDs)
#define PIN_TMC_STEP          13       // TMC2209 Step
#define PIN_TMC_DIR           14       // TMC2209 Direction
#define PIN_LED_HECK_HL       15       // Mode H/L - WS2812B Hecklicht Links (22 LEDs)
#define PIN_LED_HECK_HR       16       // Mode H/R - WS2812B Hecklicht Rechts (22 LEDs)
#define PIN_LED_FRONT_VL      17       // Mode V/L - WS2812B Frontlicht Links (50 LEDs)
#define PIN_LED_FRONT_VR      18       // Mode V/R - WS2812B Frontlicht Rechts (50 LEDs)

// Connector 2 (BLAU) - Erweiterungen
#define PIN_GAS_SERVO         43       // Vergaser Servo SG90 - Gaspedal/Drossel
#define PIN_START_LED         44       // Start-Led - Statusanzeige PWM

// Frei / Ungenutzte GPIO
#define PIN_FREE_GPIO21       21       // Pull-Down verfügbar
#define PIN_FREE_GPIO47       47       // Pull-Down verfügbar
#define PIN_FREE_GPIO48       48       // Pull-Down verfügbar

// USB Pins (intern ESP32-S3)
#define PIN_USB_DATA_POS      20       // USB D+
#define PIN_USB_DATA_NEG      19       // USB D−
#define PIN_BOOT_MODE         0        // Boot Mode

/* ==================== MCP23017 PIN MAPPING ==================== */
// MCP23017 Port Expander - ADRESSE 0x20 (MCP1)
#define MCP1_ADDR             0x20

// MCP1 Port A (Eingänge)
#define MCP1_USB_TASTER       0        // USB Taster / Schalter
#define MCP1_PEDAL_ENC_A      1        // Gaspedal Encoder Kanal A
#define MCP1_PEDAL_ENC_B      6        // Gaspedal Encoder Kanal B

// MCP1 Port B (gemischt)
#define MCP1_BRAKE            10       // Bremsschalter (Sensor)
#define MCP1_OIL              9        // Öldruck / Ölstand (Sensor)
#define MCP1_NEUTRAL          8        // Getriebe N Sensor
#define MCP1_TILT             13       // Kippschalter
#define MCP1_EGT_SCLK         12       // MAX31855 SCLK (MCP1/B4)
#define MCP1_EGT_MISO         14       // MAX31855 MISO (MCP1/B6)
#define MCP1_EGT_CS           15       // MAX31855 CS (MCP1/B7)

// MCP23017 Port Expander - ADRESSE 0x21 (MCP2)
#define MCP2_ADDR             0x21

// MCP2 Port A (Transistor Outputs)
#define MCP2_USB_TRANS        1        // USB Transistor NPN (Umschaltung)
#define MCP2_TRANS1           2        // Transistor 1 freier Ausgang
#define MCP2_FERN             3        // Fernlicht (Transistor NPN)
#define MCP2_ABBLEND          4        // Abblendlicht (Transistor NPN)
#define MCP2_TRANS2           5        // Transistor 2 / Relais Ansteuerung
#define MCP2_ESTARTER         MCP2_TRANS2 // A5: E-Starter Transistor
#define MCP2_DRS_R            6        // DRS Rechts (Transistor NPN)
#define MCP2_DRS_L            7        // DRS Links (Transistor NPN)

// MCP2 Port B (Ausgänge)
#define MCP2_PIEZO            8        // Piezo Buzzer (Ausgang)
#define MCP2_LED              9        // Status LED (Ausgang)
#define MCP2_START_BTN        10       // Start Stop Taster (Eingang)
#define MCP2_ENDSTOP_R        11       // Endschalter Getriebe (Eingang)
#define MCP2_HYD_PUMP_THERM   12       // B4: Thermoschalter Hydraulikpumpe (Eingang, aktiv LOW)
#define MCP2_HUPE             14       // Hupe (Transistor NPN)
#define MCP2_ZUENDUNG         15       // Zündunterbrechung (Transistor NPN)

/* ==================== I2C EXP ADDRESSES ==================== */
#define OLED_ADDRESS          0x3C     // SSD1306 OLED Display Address
#define AHT20_ADDRESS         0x38     // AHT20 Temperatur & Feuchte Sensor
#define BMP280_ADDRESS        0x77     // BMP280 Barometer Sensor
#define ADS1115_ADDRESS       0x48     // ADS1115 ADC (Gaspedal, Stromsensor, Lenkwinkel)

/* ==================== ADS1115 ADC CHANNELS ==================== */
#define ADS1115_CHANNEL_GAS    0       // Eingang 0: Gaspedal Poti (0-10k)
#define ADS1115_CHANNEL_STEER  2       // Eingang 2: Lenkwinkel Sensor
#define ADS1115_CHANNEL_CURRENT 3      // Eingang 3: Stromsensor (ACS758 100A, 26.5mV/A)
/* ==================== DRIVE MODES ==================== */
#define DRIVE_MODE_NORMAL      0
#define DRIVE_MODE_SPORT       1
#define DRIVE_MODE_OFFROAD     2
#define DRIVE_MODE_RACE        3
#define ASR_SLIP_NORMAL_PCT    10
#define ASR_SLIP_OFFROAD_PCT   25
#define ASR_SLIP_DRIFT_PCT     50

/* ==================== LED CONFIGURATION ==================== */
#define NUM_FRONT_LEDS        50       // WS2812B Front LEDs pro Seite
#define NUM_HECK_HL           22       // WS2812B Heck Links LEDs
#define NUM_HECK_HR           22       // WS2812B Heck Rechts LEDs
#define NUM_SPOILER           187      // WS2812B Spoiler LEDs
#define BLINKER_OUTER_LEDS    35       // Blinker äußere LEDs
#define SPOILER_BLINKER_LEFT  0        // Spoiler Links Blinker Start Index
#define SPOILER_BLINKER_RIGHT 152      // Spoiler Rechts Blinker Start Index
#define SPOILER_CENTER_START  80       // Spoiler Mitte Start Index
#define SPOILER_CENTER_LEN    27       // Spoiler Mitte Länge

/* ==================== SCREEN CONFIGURATION ==================== */
#define SCREEN_WIDTH          128      // SSD1306 Width
#define SCREEN_HEIGHT         64       // SSD1306 Height
#define OLED_RESET            -1       // Reset wird übers MCU gemacht
#define SCREEN_ADDRESS        0x3C     // SSD1306 I2C Address

/* ==================== TMC2209 STEPPER CONFIGURATION ==================== */
#define R_SENSE               0.11f    // TMC2209 Sense Resistor
#define STALL_VALUE           150      // Stalldetection Threshold
#define DRIVER_ADDRESS        0b00     // TMC2209 Slave Address

/* ==================== LEDC SERVO CONFIGURATION ==================== */
#define LEDC_SERVO_EXH_CH     LEDC_CHANNEL_0  // Auspuff Servo PWM Channel
#define LEDC_SERVO_GAS_CH     LEDC_CHANNEL_1  // Gas Servo PWM Channel
#define LEDC_SERVO_TIMER      LEDC_TIMER_0    // Shared Timer
#define LEDC_SERVO_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_SERVO_FREQ       50               // 50 Hz für Standard Servo
#define LEDC_SERVO_RES        LEDC_TIMER_13_BIT
#define LEDC_SERVO_DUTY_MAX   8191

