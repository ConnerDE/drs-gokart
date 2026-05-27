
/* ==================== RPM COUNTER ==================== */
class RPMCounter {
  uint32_t lastCalc = 0;
  uint16_t rpm = 0;
  pcnt_unit_handle_t pcnt_unit = NULL;
  pcnt_channel_handle_t pcnt_chan = NULL;

public:
  void begin() {
    Serial.println("RPM Counter: configuring PCNT");
    pcnt_unit_config_t unit_config = {
      .low_limit = -4000,
      .high_limit = 4000,
      .flags = {}
    };
    esp_err_t err = pcnt_new_unit(&unit_config, &pcnt_unit);
    if (err != ESP_OK || pcnt_unit == NULL) {
      Serial.printf("RPM Counter: pcnt_new_unit failed: %d\n", err);
      pcnt_unit = NULL;
      return;
    }

    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 5000 };
    err = pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);
    if (err != ESP_OK) {
      Serial.printf("RPM Counter: glitch filter failed: %d\n", err);
    }

    pcnt_chan_config_t chan_config = {
      .edge_gpio_num = PIN_RPM_HALL,
      .level_gpio_num = -1,
      .flags = {}
    };
    err = pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);
    if (err != ESP_OK || pcnt_chan == NULL) {
      Serial.printf("RPM Counter: pcnt_new_channel failed: %d\n", err);
      pcnt_unit = NULL;
      pcnt_chan = NULL;
      return;
    }

    err = pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (err != ESP_OK) Serial.printf("RPM Counter: edge action failed: %d\n", err);
    err = pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (err != ESP_OK) Serial.printf("RPM Counter: level action failed: %d\n", err);
    err = pcnt_unit_enable(pcnt_unit);
    if (err != ESP_OK) Serial.printf("RPM Counter: enable failed: %d\n", err);
    err = pcnt_unit_clear_count(pcnt_unit);
    if (err != ESP_OK) Serial.printf("RPM Counter: clear failed: %d\n", err);
    err = pcnt_unit_start(pcnt_unit);
    if (err != ESP_OK) Serial.printf("RPM Counter: start failed: %d\n", err);
  }

  void update() {
    if (millis() - lastCalc < 100) return;
    if (!pcnt_unit) return;
    int count = 0;
    if (pcnt_unit_get_count(pcnt_unit, &count) == ESP_OK) {
      pcnt_unit_clear_count(pcnt_unit);
      rpm = abs(count) * 60000UL / 100;
    }
    lastCalc = millis();
  }
  uint16_t getRPM() const { return rpm; }
};
