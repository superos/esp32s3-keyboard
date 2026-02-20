#include "config.h"

bool useBLE = true;
uint8_t batteryLevel = 0;
RGBCKSwitch rgb; Keyboard usb; Si24R1 nrf; Bluetooth ble;

void workflow() {
  if (rgb.ok) {
    if (usb.ready) usb.begin();
    else if (usb.ok) nrf.begin(), ble.begin();
    else if (usb.idle) rgb.sleep();
  }
  delay(100);
}

void handleInput() { // usb -> nrf/ble -> pc
  uint8_t keys[8] = {0};
  if (xQueueReceive(QI, &keys, portMAX_DELAY)) ble.ok ? ble.send(keys) : nrf.send(keys);
}

void handleOutput() { // pc -> nrf/ble -> usb
  uint8_t s1 = 0xFF, s2 = 0x00;
  if (xQueueReceive(QO, &s2, portMAX_DELAY)) s2 != s1 ? usb.capsNumScroll((s1 = s2)) : void();
}

void setup() {
  Serial.begin(115200);
  rgb.setup(), usb.setup(), ble.setup(), nrf.setup();
  xTaskCreate([](void* arg) { while(1) workflow(); }, "flow", 3072, NULL, 1, NULL);
  xTaskCreate([](void* arg) { while(1) handleInput(); }, "input", 3072, NULL, 3, NULL);
  xTaskCreate([](void* arg) { while(1) handleOutput(); }, "output", 3072, NULL, 3, NULL);
}

void loop() {
  rgb.color(usb.ok, ble.ok, nrf.ok);
  delay(50);
}
