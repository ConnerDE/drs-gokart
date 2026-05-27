
/* ==================== VIRTUELLE GANGERKENNUNG ==================== */
class VirtualGearDetector {
  static constexpr float GEAR_RATIOS[5] = { 18.71f, 12.03f, 8.02f, 6.02f, 5.34f };
  int8_t detectedGear = 0;

public:
  void update(uint16_t motorRpm, uint8_t vDriveKmh) {
    if ((float)vDriveKmh < GEAR_MIN_SPEED_KMH) {
      detectedGear = 0;
      return;
    }

    float vMs = (float)vDriveKmh / 3.6f;
    float wheelRpm = (vMs / (2.0f * PI * WHEEL_RADIUS_M)) * 60.0f;
    if (wheelRpm < 30.0f) {
      detectedGear = 0;
      return;
    }

    float ratio = (float)motorRpm / wheelRpm;
    (void)GEAR_HARDWARE_C; // Primär-/Achsübersetzung (Referenz)

    int8_t bestGear = 0;
    float bestDelta = 999.0f;
    for (uint8_t i = 0; i < 5; i++) {
      float delta = fabsf(ratio - GEAR_RATIOS[i]);
      if (delta < bestDelta) {
        bestDelta = delta;
        bestGear = (int8_t)(i + 1);
      }
    }

    if (bestDelta <= GEAR_RATIO_TOLERANCE) {
      detectedGear = bestGear;
    } else {
      detectedGear = 0;
    }
  }

  int8_t getGear() const { return detectedGear; }
};
