class Keyboard {
  private:
    uint32_t lastActive = millis();

    usb_host_config_t        uConf = { .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    hid_host_driver_config_t hConf = { .callback = onConnect, .callback_arg = this };
    hid_host_device_config_t kConf = { .callback = onReport, .callback_arg = this };
    hid_host_device_handle_t kbd_0 = nullptr;

    static void onConnect(hid_host_device_handle_t device, hid_host_driver_event_t event, void* arg) {
      if (event != HID_HOST_DRIVER_EVENT_CONNECTED) return;

      hid_host_dev_params_t params;
      hid_host_device_get_params(device, &params);
      if (params.proto != HID_PROTOCOL_KEYBOARD) return;
      
      static_cast<Keyboard*>(arg)->kbd_0 = device;
      xTaskNotifyGive(xTaskGetHandle("kbd")); // async init with xtask avoid blocking
    }

    static void onReport(hid_host_device_handle_t device, hid_host_interface_event_t event, void* arg) {
      if (event != HID_HOST_INTERFACE_EVENT_INPUT_REPORT) return;

      size_t i;
      uint8_t buf[64] = {0}, report[8] = {0};
      hid_host_device_get_raw_input_report_data(device, buf, sizeof(buf), &i);

      memcpy(report, i > 8 ? buf + 1 : buf, 8);
      xQueueSend(QI, report, 0);

      if (memcmp(report, KEY_RELEASE, 8)) static_cast<Keyboard*>(arg)->lastActive = millis();
    }

    void init() {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      hid_host_device_open(kbd_0, &kConf);
      hid_class_request_set_protocol(kbd_0, HID_REPORT_PROTOCOL_REPORT);
      hid_host_device_start(kbd_0);

      state = CONNECTED, logU("OK");
    }

  public:
    uint8_t state = UNKNOWN;
    StateShortcut<Keyboard, READY> ready{this};
    StateShortcut<Keyboard, CONNECTED> ok{this};
    StateShortcut<Keyboard, IDLE_SLEEP> idle{this};

    void setup() {
      char* tags[] = {"ENUM", "USB HOST", "hid-host"};
      for (auto tag:tags) esp_log_level_set(tag, ESP_LOG_NONE);

      pinMode(KEYBOARD_POWER_CTRL, OUTPUT);
      digitalWrite(KEYBOARD_POWER_CTRL, KEYBOARD_POWER_OFF);

      usb_host_install(&uConf);
      hid_host_install(&hConf);
      state = READY;
    }

    void begin() {
      if (!ready) return;

      xTaskCreate([](void* arg) { while(1) usb_host_lib_handle_events(portMAX_DELAY, NULL);  }, "usb", 4096, NULL, 5, NULL);
      xTaskCreate([](void* arg) { while(1) hid_host_handle_events(portMAX_DELAY); }, "hid", 4096, NULL, 5, NULL);
      xTaskCreate([](void* arg) { while(1) static_cast<Keyboard*>(arg)->init(); }, "kbd", 4096, this, 5, NULL);
      xTaskCreate(run<Keyboard>, "idle", 2048, this, 1, NULL);

      digitalWrite(KEYBOARD_POWER_CTRL, KEYBOARD_POWER_ON);
      state = CONNECTING;
    }

    void xtask() {
      while (1) {
        if (millis() - lastActive > MAX_IDLE_MS) state = IDLE_SLEEP;
        delay(TEN_SECONDS);
      }
    }

    void capsNumScroll(uint8_t lockState = 0x00) {
      if (ok)
        hid_class_request_set_report(kbd_0, HID_REPORT_TYPE_OUTPUT, 0, &lockState, 1);
    }
};