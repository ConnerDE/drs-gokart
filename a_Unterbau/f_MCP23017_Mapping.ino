/* ==================== MCP23017 PORT EXPANDER MAPPING ==================== */
// MCP23017 I2C Port Expander für GPIO-Erweiterung
// - MCP1 @ 0x20: Hauptsensoren (Bremse, Neutral, Öl, Kippschalter, Pedal)
// - MCP2 @ 0x21: Ausgabenschalter & Transistoren (Licht, Hupe, Zündung, DRS)

// Adafruit MCP23X17-Bibliothek Pin-Nummern (0-7 = Port A, 8-15 = Port B)

/* ==================== MCP23017/1 @ 0x20 (EINGÄNGE) ==================== */
// Port A (0-7) - Eingänge
#define MCP1_USB_TASTER       0        // A0: USB Taster / Schalter (Connector 2.4)
#define MCP1_PEDAL_ENC_A      1        // A1: Gaspedal Encoder Kanal A
#define MCP1_PEDAL_ENC_B      6        // A6: Gaspedal Encoder Kanal B

// Port B (8-15) - Eingänge
#define MCP1_NEUTRAL          8        // B0: Getriebe Neutral Sensor
#define MCP1_OIL              9        // B1: Motor Öldruck / Ölstand
#define MCP1_BRAKE            10       // B2: Bremsschalter
#define MCP1_TILT             13       // B5: Kippschalter

/* ==================== MCP23017/2 @ 0x21 (AUSGÄNGE) ==================== */
// Port A (0-7) - Transistor Ausgänge
#define MCP2_TRANS3           0        // A0: Transistor 3
#define MCP2_USB_TRANS        1        // A1: USB Transistor
#define MCP2_TRANS1           2        // A2: Transistor 1
#define MCP2_FERN             3        // A3: Fernlicht
#define MCP2_ABBLEND          4        // A4: Abblendlicht
#define MCP2_TRANS2           5        // A5: Transistor 2 / Relais
#define MCP2_ESTARTER         MCP2_TRANS2 // A5: E-Starter Transistor
#define MCP2_DRS_R            6        // A6: DRS Rechts
#define MCP2_DRS_L            7        // A7: DRS Links

// Port B (8-15) - Direkte Ausgänge & Eingänge
#define MCP2_PIEZO            8        // B0: Piezo Buzzer
#define MCP2_LED              9        // B1: Status LED
#define MCP2_START_BTN        10       // B2: Start/Stop Taster
#define MCP2_ENDSTOP_R        11       // B3: Endschalter Getriebe
#define MCP2_HYD_PUMP_THERM   12       // B4: Thermoschalter Hydraulikpumpe
#define MCP2_HUPE             14       // B6: Hupe
#define MCP2_ZUENDUNG         15       // B7: Zündunterbrechung

