
/* ==================== DISPLAY ==================== */
void updateDisplay(int8_t physicalGear, SafetyModule& safety, CanReceiver& can,
                   int8_t virtualGear, bool showVirtualGear) {
  if (!USE_OLED_DISPLAY) return;
  if (!(i2cStatus & (1 << 4))) return;

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 100) return;
  lastUpdate = millis();

  int8_t displayGear = physicalGear;
  if (showVirtualGear && virtualGear > 0) {
    displayGear = virtualGear;
  } else if (showVirtualGear && can.vRefKmh < (uint8_t)GEAR_MIN_SPEED_KMH) {
    displayGear = 0;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (safety.isEmergencyActive()) {
    display.setTextSize(2);
    display.setCursor(10, 10);
    display.print("ERROR:");
    display.setTextSize(3);
    display.setCursor(10, 35);
    if (!safety.oilPresent) display.print("E1");
    else if (safety.tiltDetected) display.print("E2");
    else if (safety.oilTemp > OIL_TEMP_CRITICAL) display.print("E3");
    else if (safety.batteryVoltage < BATTERY_CUTOFF) display.print("E4");
  }
  else if (!can.dataValid) {
    display.setTextSize(2);
    display.setCursor(10, 25);
    display.print("NO CAN");
  }
  else {
    display.setTextSize(4);
    display.setCursor(5, 15);
    if (displayGear == 0) display.print("N");
    else if (displayGear == -1) display.print("R");
    else display.print(displayGear);

    int barWidth = map(safety.currentRPM, 0, 4000, 0, 128);
    display.fillRect(0, 56, barWidth, 8, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(60, 5);
    display.printf("BAT:%.1fV", safety.batteryVoltage);
    display.setCursor(60, 15);
    display.printf("OIL:%.0fC", safety.oilTemp);
    display.setCursor(60, 25);
    if (can.chassisDataValid) {
      display.printf("V:%u/%u", can.vRefKmh, can.vDriveKmh);
    } else {
      display.printf("SPD:%.0f", currentSpeed);
    }
    display.setCursor(60, 35);
    switch(can.driveMode) {
      case DRIVE_MODE_NORMAL: display.print("NRML"); break;
      case DRIVE_MODE_SPORT: display.print("SPRT"); break;
      case DRIVE_MODE_OFFROAD: display.print("OFFR"); break;
      case DRIVE_MODE_RACE: display.print("RACE"); break;
    }
    if (safety.hydPumpOverTemp) {
      display.setCursor(60, 45);
      display.print("HYD");
    } else if (can.chassisDataValid && can.slipLampActive()) {
      display.setCursor(60, 45);
      display.print("SLIP");
    }
    if (can.driftMode) {
      display.setCursor(90, 45);
      display.print("DRF");
    }
  }
  display.display();
}
