RF24 radio(PIN_CE, PIN_CSN);

class Si24R1 {
  private:
    uint8_t ack = 0, err = 0;
    uint32_t ping = 0, pong = 0, logTime = 0, logInterval = 5 * ONE_MINUTE;

    SemaphoreHandle_t __lock = xSemaphoreCreateMutex();
    bool lock() const { return xSemaphoreTake(__lock, pdMS_TO_TICKS(100)) == pdTRUE; }
    void unlock() const { xSemaphoreGive(__lock); }

    void readAck() {
      if (!radio.isAckPayloadAvailable()) return;
      uint8_t lockState;
      radio.read(&lockState, 1);
      xQueueSend(QO, &lockState, 0);
    }

    bool transmit(const void* buf, uint8_t len) {
      if (!lock()) return false;
      bool hasAck = radio.write(buf, len) ? (readAck(), true) : false;
      unlock();
      ping += 1, pong += hasAck;
      if (millis() - logTime > logInterval) {
        if (ping > 0) logN("%.1f%%", (float)pong * 100.0f / ping);
        logTime = millis(), ping = 0, pong = 0;
      }
      return hasAck;
    }
    
  public:
    uint8_t state = UNKNOWN;
    StateShortcut<Si24R1, READY> ready{this};
    StateShortcut<Si24R1, CONNECTED> ok{this};

    void setup() {
      if (
        SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN) &&
        radio.begin() &&
        radio.isChipConnected()
      ) return void(state = READY);

      radio.printDetails(), logN("ERR");
    }

    void begin() {
      if (!ready) return;

      radio.setChannel(101);
      radio.setDataRate(RF24_2MBPS);
      radio.setPALevel(RF24_PA_MAX);

      radio.enableDynamicPayloads();
      radio.enableAckPayload();

      radio.openWritingPipe(ADDRESS);
      radio.stopListening();

      xTaskCreate(run<Si24R1>, "nrf", 3072, this, 1, NULL);
      state = CONNECTING;
    }

    void xtask() { // connection health monitor
      while (1) {
        if (useBLE) {
          delay(200); continue;
        }
        bool hello = transmit("#r_u_ok", 8);
        if (hello) {
          err = 0, ack > 99 ? ack = 3 : ack++;
          if (state != CONNECTED && ack > 3) state = CONNECTED, logN("OK");
        } else {
          ack = 0, err > 99 ? err = 3 : err++;
          if (state == CONNECTED && err > 3) state = DISCONNECTED, logN("LOST(%d)", err);
        }
        delay(ok ? ONE_SECOND : 200);
      }
    }

    void send(uint8_t* keys) {
      if (ok)
        transmit(keys, 8);
    }
};