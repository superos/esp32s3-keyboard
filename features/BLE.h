#define BT_MIN            0x06    // 7.5ms:  High Speed (Apple/Windows Preferred)
#define BT_MAX            0x10    // 20ms:   Balanced
#define BT_LATENCY        0       // 0:      Slave latency (number of intervals allowed to skip) - 0: No latency
#define BT_TIMEOUT        600     // 6s:     Safe timeout

#define KB_SIG            0x02    // USB Implementers Forum (SIG assigned source)
#define KB_VID            0x0006  // Vendor ID (Generic / Broadcom)
#define KB_PID            0x0001  // Product ID
#define KB_VERSION        0x0111  // Product Version (1.11)
#define KB_COUNTRY        0x20    // Country Code: 32 (China), 33 (US) - HID Spec 1.11
#define KB_APPEARANCE     0x03C1  // Appearance: Keyboard (HID profile standard)

class Bluetooth : public NimBLEServerCallbacks, public NimBLECharacteristicCallbacks {
  private:
    bool lastUse = !useBLE; // first trigger even no changes
    uint16_t conn = 0xFFFF;

    SemaphoreHandle_t __lock = xSemaphoreCreateMutex();
    bool lock() const { return xSemaphoreTake(__lock, pdMS_TO_TICKS(100)) == pdTRUE; }
    void unlock() const { xSemaphoreGive(__lock); }

    NimBLEServer*          server = nullptr;
    NimBLEHIDDevice*          hid = nullptr;
    NimBLECharacteristic*   input = nullptr;
    NimBLECharacteristic*  output = nullptr;
    NimBLEAdvertising*  broadcast = nullptr;

    void onConnect(NimBLEServer* s, NimBLEConnInfo& c) {
      conn = c.getConnHandle();
      s->updateConnParams(conn, BT_MIN, BT_MAX, BT_LATENCY, BT_TIMEOUT);
      hid->setBatteryLevel(batteryLevel, true);
      state = CONNECTED, logB("OK");
    }

    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& c, int r) {
      state = DISCONNECTED, logB("LOST(%d)", r);
      conn = 0xFFFF;
      useBLE ? s->startAdvertising() : false;
    }

    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) {
      std::string value = characteristic->getValue();
      if (value.length()) {
        uint8_t lockState = value[0];
        xQueueSend(QO, &lockState, 0);
      }
    }
    
    void onToggleBLE() {
      if (!lock()) return;
      uint8_t __ = lastUse < useBLE ? 01 : lastUse > useBLE ? 10 : 00;
      switch (__) {
        case 01: broadcast->isAdvertising() || broadcast->start(); break;
        case 10: broadcast->isAdvertising() && broadcast->stop(), conn == 0xFFFF || server->disconnect(conn); break;
      }
      lastUse = useBLE, unlock();
    }

  public:
    uint8_t state = UNKNOWN;
    StateShortcut<Bluetooth, READY> ready{this};
    StateShortcut<Bluetooth, CONNECTED> ok{this};

    void setup() {
      NimBLEDevice::init(bleName);
      NimBLEDevice::setSecurityAuth(true, true, true);

      server = NimBLEDevice::createServer();
      server->setCallbacks(this);

      hid = new NimBLEHIDDevice(server);
      hid->setManufacturer("Generic");
      hid->setPnp(KB_SIG, KB_VID, KB_PID, KB_VERSION);
      hid->setHidInfo(KB_COUNTRY, KB_VERSION);
      hid->setReportMap((uint8_t*)StandardReport, sizeof(StandardReport));
      hid->setBatteryLevel(100);
      input = hid->getInputReport(0); // for sending key reports
      output = hid->getOutputReport(0); // for receiving LED state
      output->setCallbacks(this);
      hid->startServices();

      broadcast = NimBLEDevice::getAdvertising();
      broadcast->setAppearance(KB_APPEARANCE);
      broadcast->addServiceUUID(hid->getHidService()->getUUID());
      broadcast->addServiceUUID(hid->getBatteryService()->getUUID());
      broadcast->setMinInterval(32);
      broadcast->setMaxInterval(48);

      state = READY;
    }

    void begin() {
      if (!ready) return onToggleBLE();
      xTaskCreate(run<Bluetooth>, "ble", 3072, this, 1, NULL);
      state = CONNECTING;
    }

    void xtask() { // battery service and connection params
      static uint8_t lv = 100;
      while (1) {
        if (ok)
          if (lv != batteryLevel)
            hid->setBatteryLevel(lv = batteryLevel, true);
        delay(TEN_SECONDS);
      }
    }

    void send(uint8_t* keys) {
      if (ok)
        input->setValue(keys, 8), input->notify();
    }
};
