
/* ==================== LIGHTS MANAGER ==================== */
class LightsManager {
  bool lastDataValid = true;
  unsigned long lastBlinkerUpdate = 0;
  int16_t blinkerPosL = 0;
  int16_t blinkerPosR = 0;

public:
  void playWelcomeAnimation() {
    static bool done = false;
    static unsigned long t0 = 0;
    static uint8_t step = 0;
    if (done) return;

    unsigned long now = millis();
    CRGB orange = CRGB(255, 120, 0);

    switch (step) {
      case 0: t0 = now; step = 1; break;
      case 1: {
        if (now - t0 < 300) {
          uint8_t b = map(now - t0, 0, 300, 0, 255);
          fill_solid(ledsFrontVL, NUM_FRONT_LEDS, orange.nscale8(b));
          fill_solid(ledsFrontVR, NUM_FRONT_LEDS, orange.nscale8(b));
          fill_solid(ledsHeckHL, NUM_HECK_HL, orange.nscale8(b));
          fill_solid(ledsHeckHR, NUM_HECK_HR, orange.nscale8(b));
          fill_solid(ledsSpoiler, NUM_SPOILER, orange.nscale8(b));
          FastLED.show();
        } else { t0 = now; step = 2; }
      } break;
      case 2: if (now - t0 > 300) { t0 = now; step = 3; } break;
      case 3: {
        if (now - t0 < 300) {
          uint8_t b = map(now - t0, 0, 300, 255, 0);
          fill_solid(ledsFrontVL, NUM_FRONT_LEDS, orange.nscale8(b));
          fill_solid(ledsFrontVR, NUM_FRONT_LEDS, orange.nscale8(b));
          fill_solid(ledsHeckHL, NUM_HECK_HL, orange.nscale8(b));
          fill_solid(ledsHeckHR, NUM_HECK_HR, orange.nscale8(b));
          fill_solid(ledsSpoiler, NUM_SPOILER, orange.nscale8(b));
          FastLED.show();
        } else { done = true; }
      } break;
    }
  }

  void applyRGBEffect(CRGB* leds, uint16_t count, uint8_t mode, uint8_t driveMode) {
    static uint8_t hue = 0;
    static uint8_t pulse = 0;
    static bool pulseDir = true;
    static uint16_t pos = 0;
    hue += 2;

    switch (mode) {
      case 0: fill_solid(leds, count, CRGB::Black); break;
      case 1: {
        CRGB color;
        switch (driveMode) {
          case DRIVE_MODE_NORMAL: color = CRGB(0, 255, 0); break;
          case DRIVE_MODE_SPORT: color = CRGB(0, 0, 255); break;
          case DRIVE_MODE_OFFROAD: color = CRGB(255, 140, 0); break;
          case DRIVE_MODE_RACE: color = CRGB(128, 0, 128); break;
          default: color = CRGB::White; break;
        }
        fill_solid(leds, count, color);
      } break;
      case 2: fill_rainbow(leds, count, hue, 7); break;
      case 3: {
        if (pulseDir) { pulse += 3; if (pulse > 250) pulseDir = false; }
        else { pulse -= 3; if (pulse < 10) pulseDir = true; }
        fill_solid(leds, count, CHSV(96, 255, pulse));
      } break;
      case 4: {
        pos = (pos + 1) % (count * 2);
        fill_solid(leds, count, CRGB::Black);
        for (uint8_t i = 0; i < 8; i++) {
          int idx = (pos - i + count * 2) % (count * 2);
          if (idx < count) leds[idx] = CHSV((hue + i * 32) % 256, 255, 255 - i * 25);
        }
      } break;
      case 5: {
        fill_solid(leds, count, CRGB::Black);
        for (uint8_t i = 0; i < count / 5; i++) {
          uint8_t idx = random16(count);
          leds[idx] = CHSV(random8(), 200, 255);
        }
      } break;
      default: fill_solid(leds, count, CRGB::Black); break;
    }
  }

  void updateCorneringLight(CRGB* ledsVL, CRGB* ledsVR, uint16_t count,
                            int8_t steerPercent, uint8_t rawVL, uint8_t rawVR) {
    if (steerPercent == 0) return;

    float vVL = (float)rawVL / 10.0f;
    float vVR = (float)rawVR / 10.0f;
    float diff = vVL - vVR;
    if (fabsf(diff) < 0.05f) return;

    int steerMag = abs(steerPercent);
    uint8_t intensity = map(steerMag, 5, 100, 40, 255);
    uint8_t spread = map(steerMag, 5, 100, 4, count / 3);
    spread = constrain(spread, 4, count / 2);

    CRGB* target = nullptr;
    bool fromOutside = false;

    if (steerPercent < 0 && diff > 0) {
      target = ledsVL;
      fromOutside = true;
    } else if (steerPercent > 0 && diff < 0) {
      target = ledsVR;
      fromOutside = false;
    } else if (steerPercent < 0) {
      target = ledsVL;
      fromOutside = true;
    } else {
      target = ledsVR;
      fromOutside = false;
    }

    if (!target) return;

    CRGB cornerColor = CRGB(180, 200, 255);
    for (uint8_t i = 0; i < spread && i < count; i++) {
      uint16_t idx = fromOutside ? i : (count - 1 - i);
      uint8_t fade = 255 - (uint8_t)((i * 220) / spread);
      target[idx] = cornerColor.nscale8((fade * intensity) / 255);
    }
  }

  void updateDynamicBlinker(CRGB* leds, uint16_t count, bool active, bool left) {
    if (!active) {
      fill_solid(leds, count, CRGB::Black);
      if (left) blinkerPosL = 0; else blinkerPosR = 0;
      return;
    }

    unsigned long now = millis();
    if (now - lastBlinkerUpdate > 15) {
      lastBlinkerUpdate = now;
      int16_t& pos = left ? blinkerPosL : blinkerPosR;
      uint16_t mid = count / 2;

      if (pos >= mid) { fill_solid(leds, count, CRGB::Black); pos = 0; return; }

      fill_solid(leds, count, CRGB::Black);
      CRGB orange = CRGB(255, 120, 0);

      if (left) {
        for (int i = 0; i <= pos && i < mid; i++) leds[i] = orange;
        pos++;
      } else {
        for (int i = count - 1; i >= mid && (count - 1 - i) <= pos; i--) leds[i] = orange;
        pos++;
      }
    }
  }

  void update(CanReceiver& can, SafetyModule& safety, bool launchActive, int8_t steerPercent) {
    playWelcomeAnimation();

    if (!can.dataValid && lastDataValid) {
      fill_solid(ledsFrontVL, NUM_FRONT_LEDS, CRGB::Black);
      fill_solid(ledsFrontVR, NUM_FRONT_LEDS, CRGB::Black);
      fill_solid(ledsHeckHL, NUM_HECK_HL, CRGB::Black);
      fill_solid(ledsHeckHR, NUM_HECK_HR, CRGB::Black);
      fill_solid(ledsSpoiler, NUM_SPOILER, CRGB::Black);
      if (i2cStatus & (1 << 1)) mcp2.digitalWrite(MCP2_ABBLEND, LOW);
      FastLED.show();
      lastDataValid = false;
      return;
    }
    lastDataValid = can.dataValid;
    if (!can.dataValid) return;

    if (i2cStatus & (1 << 1)) mcp2.digitalWrite(MCP2_ABBLEND, (can.lightMode == 1) ? HIGH : LOW);

    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    if (millis() - lastBlink > 500) { lastBlink = millis(); blinkState = !blinkState; }

    extern bool blinkerLeftState;
    extern bool blinkerRightState;
    extern bool warnBlinkState;

    bool warn = warnBlinkState;
    bool left = blinkerLeftState;
    bool right = blinkerRightState;
    bool brake = safety.brakePressed;

    if (i2cStatus & (1 << 1)) mcp2.digitalWrite(MCP2_TRANS3, brake ? HIGH : LOW);

    // FRONT LEDS (Launch Control Bar)
    if (launchActive) {
      uint16_t rpm = safety.currentRPM;
      if (rpm > 2100) rpm = 2100;
      uint8_t fillCount = map(rpm, 0, 2100, 0, NUM_FRONT_LEDS);

      if (rpm >= 2050) {
        CRGB c = blinkState ? CRGB::White : CRGB::Black;
        fill_solid(ledsFrontVL, NUM_FRONT_LEDS, c);
        fill_solid(ledsFrontVR, NUM_FRONT_LEDS, c);
      } else {
        fill_solid(ledsFrontVL, NUM_FRONT_LEDS, CRGB::Black);
        fill_solid(ledsFrontVR, NUM_FRONT_LEDS, CRGB::Black);
        for(int i=0; i<fillCount; i++) ledsFrontVL[i] = CRGB::Green;
        for(int i=0; i<fillCount; i++) ledsFrontVR[i] = CRGB::Green;
      }
    } else {
      if (warn) {
        updateDynamicBlinker(ledsFrontVL, NUM_FRONT_LEDS, blinkState, true);
        updateDynamicBlinker(ledsFrontVR, NUM_FRONT_LEDS, blinkState, false);
      } else {
        updateDynamicBlinker(ledsFrontVL, NUM_FRONT_LEDS, left, true);
        updateDynamicBlinker(ledsFrontVR, NUM_FRONT_LEDS, right, false);
      }

      if (!warn && !left && !right) {
        uint8_t effectiveRgbMode = (can.lightMode > 0 && can.rgbMode == 0) ? 255 : can.rgbMode;
        if (effectiveRgbMode != 255) {
          applyRGBEffect(ledsFrontVL, NUM_FRONT_LEDS, effectiveRgbMode, can.driveMode);
          applyRGBEffect(ledsFrontVR, NUM_FRONT_LEDS, effectiveRgbMode, can.driveMode);
        }
        if (can.chassisDataValid) {
          updateCorneringLight(ledsFrontVL, ledsFrontVR, NUM_FRONT_LEDS,
                               steerPercent, can.rawSpeedVL, can.rawSpeedVR);
        }
      }
    }

    // SPOILER
    fill_solid(ledsSpoiler, NUM_SPOILER, CRGB::Black);

    if (safety.currentRPM > 3500) {
      CRGB warnColor = blinkState ? CRGB::DeepSkyBlue : CRGB::Red;
      for(int i=SPOILER_CENTER_START; i < SPOILER_CENTER_START + SPOILER_CENTER_LEN; i++) {
        ledsSpoiler[i] = warnColor;
      }
    } else {
      if (!warn && !left && !right && !brake) {
        uint8_t effectiveRgbMode = (can.lightMode > 0 && can.rgbMode == 0) ? 255 : can.rgbMode;
        if (effectiveRgbMode != 255) {
          applyRGBEffect(ledsSpoiler, NUM_SPOILER, effectiveRgbMode, can.driveMode);
        }
      }
    }

    CRGB spoilerBlinker = CRGB(255, 150, 0);
    CRGB spoilerBrake = CRGB(255, 0, 0);

    if (warn) {
      for (uint8_t i = 0; i < BLINKER_OUTER_LEDS; i++) {
        ledsSpoiler[SPOILER_BLINKER_LEFT + i] = blinkState ? spoilerBlinker : CRGB::Black;
        ledsSpoiler[SPOILER_BLINKER_RIGHT + i] = blinkState ? spoilerBlinker : CRGB::Black;
      }
    } else {
      if (left) for (uint8_t i = 0; i < BLINKER_OUTER_LEDS; i++)
        ledsSpoiler[SPOILER_BLINKER_LEFT + i] = blinkState ? spoilerBlinker : CRGB::Black;
      if (right) for (uint8_t i = 0; i < BLINKER_OUTER_LEDS; i++)
        ledsSpoiler[SPOILER_BLINKER_RIGHT + i] = blinkState ? spoilerBlinker : CRGB::Black;
      if (brake && !left && !right) {
        for (uint8_t i = 0; i < BLINKER_OUTER_LEDS; i++) {
          ledsSpoiler[SPOILER_BLINKER_LEFT + i] = spoilerBrake;
          ledsSpoiler[SPOILER_BLINKER_RIGHT + i] = spoilerBrake;
        }
      }
    }

    // HECK
    fill_solid(ledsHeckHL, NUM_HECK_HL, CRGB::Black);
    fill_solid(ledsHeckHR, NUM_HECK_HR, CRGB::Black);

    if (can.lightMode > 0) {
      CRGB rear = CRGB(255, 0, 0).nscale8(153);
      fill_solid(ledsHeckHL, NUM_HECK_HL, rear);
      fill_solid(ledsHeckHR, NUM_HECK_HR, rear);
    }
    if (brake) {
      fill_solid(ledsHeckHL, NUM_HECK_HL, CRGB::Red);
      fill_solid(ledsHeckHR, NUM_HECK_HR, CRGB::Red);
    }
    if (warn || left) fill_solid(ledsHeckHL, NUM_HECK_HL, blinkState ? CRGB::Orange : CRGB::Black);
    if (warn || right) fill_solid(ledsHeckHR, NUM_HECK_HR, blinkState ? CRGB::Orange : CRGB::Black);

    static unsigned long lastShow = 0;
    if (millis() - lastShow > 20) {
        FastLED.show();
        lastShow = millis();
    }
  }
};
