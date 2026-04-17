
/* ==================== LOOP ==================== */
void loop() {
  esp_task_wdt_reset();  // Feed the watchdog

  can.update();
  rpm.update();
  safety.currentRPM = rpm.getRPM();
  safety.update();

  // Blinker Logic (Comfort Blinker)
  bool bL = can.isBlinkLeft();
  bool bR = can.isBlinkRight();
  bool bW = can.isWarnBlink();

  if (bW && !lastWarnBtn) {
    warnBlinkState = !warnBlinkState;
    if (warnBlinkState) { blinkerLeftState = false; blinkerRightState = false; }
  }
  lastWarnBtn = bW;

  if (!warnBlinkState && bL && !lastBlinkLeftBtn) {
    blinkerLeftState = !blinkerLeftState;
    blinkerRightState = false;
    if (blinkerLeftState) blinkerStartTime = millis();
  }
  lastBlinkLeftBtn = bL;

  if (!warnBlinkState && bR && !lastBlinkRightBtn) {
    blinkerRightState = !blinkerRightState;
    blinkerLeftState = false;
    if (blinkerRightState) blinkerStartTime = millis();
  }
  lastBlinkRightBtn = bR;

  // Auto-Cancel Blinker
  if ((blinkerLeftState || blinkerRightState) && !warnBlinkState) {
    if (millis() - blinkerStartTime > 10000 || currentSpeed > 30.0) {
      blinkerLeftState = false;
      blinkerRightState = false;
    }
  }

  bool launchActive = false;
  if (safety.ignitionOn && can.isLaunch() && safety.brakePressed) {
    launchActive = true;
  }

  gasPedal.update(can.driveMode, launchActive, safety.currentRPM);

  // USB Toggle (Debounced)
  static bool lastUsbTaster = true;
  static unsigned long lastUsbPress = 0;
  bool usbTasterPressed = !mcp1.digitalRead(MCP1_USB_TASTER);

  if (usbTasterPressed && !lastUsbTaster && (millis() - lastUsbPress > 200)) {
    usbState = !usbState;
    mcp2.digitalWrite(MCP2_USB_TRANS, usbState ? HIGH : LOW);
    lastUsbPress = millis();
  }
  lastUsbTaster = usbTasterPressed;

  bool systemOK = can.isAlive() && !safety.emergencyActive() && voltageOK && !overTemp; // großteil noch implementieren

  static bool lastDRSButton = false;
  bool currentDRSButton = can.isDRS();

  if (currentDRSButton && !lastDRSButton) {
    drsButtonPressed();
  }

  lastDRSButton = currentDRSButton;

  drsUpdate(speed, throttle, safety.brakePressed(), rpm, systemOK);
  bool drsActive = isDRSActive();
  void setDRS(bool drsActive) {
    mcp2.digitalWrite(MCP2_DRS_L, drsActive ? HIGH : LOW);
    mcp2.digitalWrite(MCP2_DRS_R, drsActive ? HIGH : LOW);
  }
  
  safety.setCanLoss(!can.dataValid);

  if (!can.dataValid || !safety.systemActive) {
    blinkerLeftState = false;
    blinkerRightState = false;
    warnBlinkState = false;
    lights.update(can, safety, false);
    updateDisplay(gearbox.getGear(), safety, can);
    return;
  }

  setExhaustServoAngle(can.exhaustAngle);
  mcp2.digitalWrite(MCP2_HUPE, can.isHorn() ? HIGH : LOW);

  bool canShift = safety.canShiftDown(safety.currentRPM);

  static bool lastUp = false;
  bool up = can.isShiftUp();
  if (up && !lastUp && safety.ignitionOn) gearbox.shiftUp(canShift);
  lastUp = up;

  static bool lastDown = false;
  bool down = can.isShiftDown();
  if (down && !lastDown && safety.ignitionOn) gearbox.shiftDown(canShift);
  lastDown = down;

  // Reverse Logic
  static bool lastR = false;
  bool rev = can.isReverse() && can.isShiftDown() && safety.brakePressed;
  if (rev && !lastR && canShift && currentSpeed < 2.0) {
    gearbox.shiftToReverse(true);
  }
  lastR = rev;

  lights.update(can, safety, launchActive);
  sendTelemetry(safety, can, gasPedal, gearbox.getGear(), launchActive);
  updateDisplay(gearbox.getGear(), safety, can);
}