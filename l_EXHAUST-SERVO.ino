
#if 0
/* Legacy exhaust servo implementation. Active implementation is in a_Unterbau.ino. */
/* ==================== EXHAUST SERVO ==================== */
void initExhaustServo() {
  ledc_channel_config_t exh_conf = {
    .gpio_num = PIN_EXHAUST_SERVO,
    .speed_mode = LEDC_SERVO_MODE,
    .channel = LEDC_SERVO_EXH_CH,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_SERVO_TIMER,
    .duty = 0,
    .hpoint = 0,
    .flags = {}
  };
  ledc_channel_config(&exh_conf);

  calSrvExhMin = prefs.getInt(PREF_SRV_EXH_MIN, 0);
  calSrvExhMax = prefs.getInt(PREF_SRV_EXH_MAX, 180);
}

void setExhaustServoAngle(uint8_t angle) {
  // Map 0-90 logic to calibrated servo range
  int targetAngle = calSrvExhMin;
  if (angle == 45) targetAngle = (calSrvExhMin + calSrvExhMax) / 2;
  if (angle == 90) targetAngle = calSrvExhMax;

  uint32_t pulseWidth_us = map(targetAngle, 0, 180, 1000, 2000);
  uint32_t duty = (uint32_t)((pulseWidth_us * (uint64_t)LEDC_SERVO_DUTY_MAX) / 20000ULL);
  ledc_set_duty(LEDC_SERVO_MODE, LEDC_SERVO_EXH_CH, duty);
  ledc_update_duty(LEDC_SERVO_MODE, LEDC_SERVO_EXH_CH);
}
#endif
