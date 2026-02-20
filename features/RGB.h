class RGBCKSwitch {
  private:
    // Co-Anode Logic, 01: press start, 10: press end, 00: released, 11: still pressing
    struct Button {
      uint8_t pin;
      RGBCKSwitch* parent;
      void setup() { pinMode(pin, INPUT_PULLDOWN); }
      void xtask() {
        uint8_t v1 = 0, v2 = 0, vx = 0, count = 0;
        uint32_t ts = 0, ms = 0;
        while (digitalRead(pin)) delay(10); // ensure button is released on startup
        while (1) {
          v2 = digitalRead(pin);
          vx = v1 > v2 ? 10 : v1 < v2 ? 01 : v2 ? 11 : 00;
          ms = millis() - ts;
          switch (vx) {
            case 01: ts = millis(); break;
            case 10: if (ms > 100) ts = millis(), count++; break;
            case 00: if (ms > 500 && count) parent->onClick(count), count = 0; break;
            case 11: if (ms > 3000) parent->onClick(3), ts = millis(), count = 0xFF; break; // trigger count overflow to 0 on release
          }
          v1 = v2, delay(37);
        }
      }
    } K = {PIN_K, this};

    void onClick(uint8_t clicks) {
      switch (clicks) {
        case 1: ok ? onSingleClickAgain() : void(state = CONNECTED); break;
        case 2: sleep(); break;
        case 3: esp_restart(); break;
      }
    }

    void onSingleClickAgain() {
      static uint32_t lastToggle = 0;
      if (
        lastToggle == 0 ||
        millis() - lastToggle > ONE_SECOND
      ) lastToggle = millis(), P.putBool("ble", useBLE = !useBLE);
    }

    // voltage divider(1/2,100kΩ) + average(x20) + discharge curve
    struct Battery {
      uint8_t pin, lv = 0;
      void setup() { analogSetAttenuation(ADC_11db); } 
      void xtask() {
        float v1 = 0, v2 = 0;
        uint32_t ts = 0, mv = 0;
        while (1) {
          if (ts == 0 || millis() - ts > BATTERY_READ_INTERVAL) {
            ts = millis();
            v2 = medianAverage() * 2.0;
            v1 = v1 == 0 ? v2 : (v1 * 0.7) + (v2 * 0.3);
            mv = uint32_t(v1);
            lv = mapMilliVolt(mv);
            if (abs((int)lv - (int)batteryLevel) > 3) logR("%dmV, %d%%", mv, batteryLevel = lv);
          }
          delay(BATTERY_READ_INTERVAL);
        }
      }
      float medianAverage() {
        int volt[30];
        for (int i = 0; i < 30; i++) volt[i] = analogReadMilliVolts(pin), delay(20);
        std::sort(volt, volt + 30);
        return std::accumulate(volt + 10, volt + 20, 0) / 10.0;
      }
      uint8_t mapMilliVolt(uint32_t mv) {
        if (mv >= 4200) return 100;
        if (mv >= 4000) return map(mv, 4000, 4200, 80, 100); // 4.0-4.2V: 80-100%
        if (mv >= 3800) return map(mv, 3800, 4000, 50, 80);  // 3.8-4.0V: 50-80%
        if (mv >= 3700) return map(mv, 3700, 3800, 20, 50);  // 3.7-3.8V: 20-50%
        if (mv >= 3500) return map(mv, 3500, 3700, 5,  20);  // 3.5-3.7V: 5-20%
        if (mv >= 3300) return map(mv, 3300, 3500, 0,  5);   // 3.3-3.5V: 0-5%
        return 0;
      }
    } L = {BATTERY_ADC_PIN};

    // 0: off, 1: on, 2: blink, 3: manually
    struct Color {
      uint8_t pin, mode = 3;
      void setup() { pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); }
      void on()    { digitalWrite(pin, LOW); }
      void off()   { digitalWrite(pin, HIGH); }
      void tick()  { mode == 0 ? off() : mode == 1 ? on() : mode == 2 ? on() : void(); }
      void tock()  { mode == 0 ? off() : mode == 1 ? on() : mode == 2 ? off() : void(); }
    } R = {PIN_R}, G = {PIN_G}, B = {PIN_B};

    SemaphoreHandle_t rgbing = xSemaphoreCreateMutex();
    bool lock() const { return ok && xSemaphoreTake(rgbing, pdMS_TO_TICKS(100)) == pdTRUE; }
    void unlock() const { xSemaphoreGive(rgbing); }

    // store preferred settings, autoload on next start
    struct Storage {
      Preferences db;
      char* bucket = "config";
      void setup() { db.begin(bucket, false), useBLE = db.getBool("ble", false); }
      void putBool(const char* key, bool value) { db.putBool(key, value); }
    } P;

  public:
    uint8_t state = UNKNOWN;
    StateShortcut<RGBCKSwitch, CONNECTED> ok{this};

    void setup() {
      R.setup(), G.setup(), B.setup(), K.setup(), L.setup(), P.setup();

      xTaskCreate(run<Button>, "button", 3072, &K, 1, NULL);
      xTaskCreate(run<Battery>, "battery", 3072, &L, 1, NULL);
      xTaskCreate(run<RGBCKSwitch>, "color", 2048, this, 1, NULL);

      esp_reset_reason_t r = esp_reset_reason();
      state = (
        r == ESP_RST_POWERON ||
        r == ESP_RST_DEEPSLEEP ||
        r == ESP_RST_SW
      ) ? CONNECTED : READY;
    }

    void xtask() {
      while (1) {
        R.tick(), G.tick(), B.tick(); delay(100);
        R.tock(), G.tock(), B.tock(); delay(100);
      }
    }

    void color(bool usb_ok, bool ble_ok, bool nrf_ok) {
      if (lock()) {
        uint8_t r = 0, g = 0, b = 0;
        if (batteryLevel < BATTERY_LOW_LEVEL) {
          r = 2;
        } else {
          if (usb_ok) useBLE ? (b = ble_ok ? 1 : 2) : (g = nrf_ok ? 1 : 2);
          else g = 2, b = 2;
        }
        R.mode = r, G.mode = g, B.mode = b, unlock();
      }
    }

    void sleep() {
      logR("BYE"), Serial.flush();

      digitalWrite(KEYBOARD_POWER_CTRL, KEYBOARD_POWER_OFF);
      delay(50);

      rtc_gpio_init(PIN_K);
      rtc_gpio_set_direction(PIN_K, RTC_GPIO_MODE_INPUT_ONLY);
      rtc_gpio_pulldown_en(PIN_K);
      rtc_gpio_pullup_dis(PIN_K);

      esp_sleep_enable_ext1_wakeup(1ULL << PIN_K, ESP_EXT1_WAKEUP_ANY_HIGH);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
      esp_deep_sleep_start();
    }
};
