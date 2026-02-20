#include <Arduino.h>

/**
 * required by @features/RGB.h
 * Provides RGB and button, battery monitoring, settings storage, rtc wakeup
 */
#include <numeric>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <Preferences.h>

/**
 * required by @features/USB.h
 * Provides USB Host support for HID devices on ESP32
 * https://components.espressif.com/components/espressif/usb_host_hid
 * PROJECT
 * ├── keyboard.ino
 * ├── hid_host.c     # put .c with .ino let arduino auto handle it
 * └── usb_host_hid   # put headers here and correct path in `hid_host.c`
 *     ├── hid.h
 *     └── hid_host.h
 */
#include "usb_host_hid/hid_host.h"

/**
 * required by @features/NRF.h
 * Provides support for nRF24L01+ wireless transceivers
 * https://github.com/nRF24/RF24 
 */
#include <RF24.h>

/**
 * required by @features/BLE.h
 * Provides a lightweight BLE stack for Arduino
 * https://github.com/h2zero/NimBLE-Arduino
 */
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

/**
 * Logging & Time Macros
 */
#define logR(fmt, ...) Serial.printf("[RGB] " fmt "\n", ##__VA_ARGS__)
#define logU(fmt, ...) Serial.printf("[USB] " fmt "\n", ##__VA_ARGS__)
#define logN(fmt, ...) Serial.printf("[NRF] " fmt "\n", ##__VA_ARGS__)
#define logB(fmt, ...) Serial.printf("[BLE] " fmt "\n", ##__VA_ARGS__)
#define ONE_SECOND (1000)
#define ONE_MINUTE (60 * 1000)
#define TEN_SECONDS (10 * ONE_SECOND)
#define TEN_MINUTES (10 * ONE_MINUTE)

/**
 * Global State Enumeration and StateShortcuts
 */
enum GlobalState { UNKNOWN = 0, READY, CONNECTING, CONNECTED, DISCONNECTED, IDLE_SLEEP };

template<typename T, uint8_t V> 
struct StateShortcut { 
  const T* p; 
  operator bool() const { return p->state == V; } 
};

/**
 * xTaskCreate Task Runner Shortcut
 * define a member function `void xtask()` in your class as the task entry point
 * usage: xTaskCreate(run<YourClass>, "taskname", stacksize, this, priority, NULL);
 */
template <typename T>
static void run(void* arg) { static_cast<T*>(arg)->xtask(); }

/**
 * Global Queue Definitions
 * QI: input reports from USB, aka key press/release
 * QO: output reports from NRF/BLE, aka led lock states
 */
static const QueueHandle_t QI = xQueueCreate(10, sizeof(uint8_t[8]));
static const QueueHandle_t QO = xQueueCreate(10, sizeof(uint8_t));
static const uint8_t KEY_RELEASE[8] = {0};

/**
 * Keyboard Power Control
 * manually power on VBUS after USB HOST started to ensure keyboard detected(USB enumeration)
 * AO3401: S = 5v-Vin, D = 5v-Vout, G + 10kΩ = S
 * S8050: B + 5kΩ = GPIO, E = GND, C = AO3401 G
 * GPIO output mode + pulldown, GPIO write 0 (default) -=> D = 0v, write 1 => D = 5v
 * 
 * ┌───┐ESP32.USB ┌───┐KEYBOARD
 * │GND├──────────┤GND├────────────┐
 * │D- ├──────────┤D- │            │
 * │D+ ├──────────┤D+ │            │
 * │5v ┼┐        ┌┼5v │            │
 * └───┘│        │└───┘            │
 *      │    ┌───┼───┐AO3401   ┌───┼───┐S8050
 *      ├────┼S  D  G┼────┬────┼C  E  B┼────5kΩ──── GPIO(USB_POWER_CTRL)
 *      │    └───────┘    │    └───────┘
 *      └───────10kΩ──────┘
 */
static const gpio_num_t KEYBOARD_POWER_CTRL = GPIO_NUM_3;
static const uint8_t    KEYBOARD_POWER_ON   = HIGH;
static const uint8_t    KEYBOARD_POWER_OFF  = LOW;

/**
 * Co-Anode RGBCK Switch Pin Definitions
 * C: 3.3v, VDD33
 * K: input, 0 = released, 1 = pressed
 * R: output, 0 = on, 1 = off, status of batteryLevel, BLINK when low, otherwise OFF
 * G: output, 0 = on, 1 = off, status of usb keyboard, BLINK when connecting, OFF when usb connected but ble connecting, ON when usb & ble both connected
 * B: output, 0 = on, 1 = off, status of ble keyboard, BLINK when connecting, OFF when connected
 * current limiting resistor for led: R = 150Ω, G = 50Ω, B = 50Ω
 * 
 * ┌───────────────────────────┐J1
 * │3v3 ... 4  5  6  7  ... GND│
 * └─┼──────┼──┼──┼──┼─────────┘
 *   │     150 50 50 │
 *   └───┐  │  │  │  │
 *      ┌┼──┼──┼──┼──┼┐ Co-Anode RGB Switch
 *      │C  R  G  B  K│
 *      └─────────────┘
 */
static const gpio_num_t PIN_R = GPIO_NUM_4;
static const gpio_num_t PIN_G = GPIO_NUM_5;
static const gpio_num_t PIN_B = GPIO_NUM_6;
static const gpio_num_t PIN_K = GPIO_NUM_7;

/**
 * Battery Monitoring Definitions
 * BATTERY_ADC_PIN: ADC pin to read battery voltage
 * BATTERY_LOW_LEVEL: battery percentage threshold to trigger low battery warning
 * BATTERY_READ_INTERVAL: interval in milliseconds to read battery level
 * 
 * ┌─────────────────────────┬──100kΩ──┐
 * │ Li-ion battery with PCB │         ├── GPIO_INPUT (ADC 1.5v ~ 2.1v map to 3.0v ~ 4.2v)
 * └─────────────────────────┴──100kΩ──┘
 */
static const gpio_num_t BATTERY_ADC_PIN = GPIO_NUM_8;
static const uint8_t    BATTERY_LOW_LEVEL = 10;
static const uint32_t   BATTERY_READ_INTERVAL = 15 * ONE_MINUTE;

/**
 * Dual-Mode K Pin
 * GPIO default input mode with PULLDOWN
 *  1. single click: CONNECTED => begin
 *  2. double click: DISCONNECTED => sleep
 * two sleep triggers defined:
 *  1. hardware: double click RGBCK button
 *  2. software: keyboard idle MAX_IDLE_MINUTES
 * long press RGBCK button switch NRF/BLE will trigger soft restart
 */
static const uint8_t MAX_IDLE_MINUTES = 15;
static const uint32_t MAX_IDLE_MS = MAX_IDLE_MINUTES * ONE_MINUTE;

/**
 * 
 * !!! Arduino Board Configuration PSRAM = DISABLED to avoid conflict GPIOs 35-39 !!!
 * 
 * NRF24L01+ Pin Definitions
 * MOSI: Master Out Slave In pin for SPI communication
 * SCK: Serial Clock pin for SPI communication
 * MISO: Master In Slave Out pin for SPI communication
 * CE: Chip Enable pin to activate the NRF24L01+ for transmission or reception
 * CSN: Chip Select Not pin to select the NRF24L01+ device on the SPI bus
 * ESP32S3 3v3 and 5v cannot drive enough current, extra power required for NRF24L01+: 115mA@0dBm, 90mA@-6dBm
 * 
 * ┌───────────────────────────────────────┐J1 
 * │3v3   ...      9  10 11 12 13  ...  GND│
 * └───────────────┼──┼──┼──┼──┼───────────┘
 *     GND ─────┐  │  │  │  │  │
 *     3v3 ──┐  │  │  │  │  │  │
 *         ┌─┼──┼──┼──┼──┼──┼──┼─┐
 *         │  nRF24L01+ module   │
 *         └─────────────────────┘
 * Standard nRF24L01+ pinout (top to bottom):
 * 1.GND 2.VCC 3.CE 4.CSN 5.SCK 6.MOSI 7.MISO 8.IRQ
 */
static const uint8_t PIN_CE     = 9;   // GPIO 9  → nRF24 Pin 3 (CE)
static const uint8_t PIN_CSN    = 10;  // GPIO 10 → nRF24 Pin 4 (CSN)
static const uint8_t PIN_SCK    = 11;  // GPIO 11 → nRF24 Pin 5 (SCK)
static const uint8_t PIN_MOSI   = 12;  // GPIO 12 → nRF24 Pin 6 (MOSI)
static const uint8_t PIN_MISO   = 13;  // GPIO 13 → nRF24 Pin 7 (MISO)
static const uint8_t ADDRESS[5] = { 'G', '8', '0', 'S', '3' };

/**
 * Standard HID Report Descriptor for a keyboard
 * Defines the format of input and output reports for a standard keyboard device.
 * Input reports include modifier keys, reserved byte, and key codes.
 * Output reports include LED states for Num Lock, Caps Lock, Scroll Lock, etc.
 * Reference: https://www.usb.org/sites/default/files/documents/hid1_11.pdf (Section 7.2.1)
 */
static const uint8_t StandardReport[] = {
  0x05, 0x01, // Usage Page (Generic Desktop)
  0x09, 0x06, // Usage (Keyboard)
  0xA1, 0x01, // Collection (Application)
  0x05, 0x07, //   Usage Page (Key Codes)
  0x19, 0xE0, //   Usage Minimum (224)
  0x29, 0xE7, //   Usage Maximum (231)
  0x15, 0x00, //   Logical Minimum (0)
  0x25, 0x01, //   Logical Maximum (1)
  0x75, 0x01, //   Report Size (1)
  0x95, 0x08, //   Report Count (8)
  0x81, 0x02, //   Input (Data, Variable, Absolute) ; Modifier byte
  0x95, 0x01, //   Report Count (1)
  0x75, 0x08, //   Report Size (8)
  0x81, 0x01, //   Input (Constant) ; Reserved byte
  0x95, 0x05, //   Report Count (5)
  0x75, 0x01, //   Report Size (1)
  0x05, 0x08, //   Usage Page (LEDs)
  0x19, 0x01, //   Usage Minimum (1)
  0x29, 0x05, //   Usage Maximum (5)
  0x91, 0x02, //   Output (Data, Variable, Absolute) ; LED report
  0x95, 0x01, //   Report Count (1)
  0x75, 0x03, //   Report Size (3)
  0x91, 0x01, //   Output (Constant) ; LED report padding
  0x95, 0x06, //   Report Count (6)
  0x75, 0x08, //   Report Size (8)
  0x15, 0x00, //   Logical Minimum (0)
  0x25, 0x65, //   Logical Maximum (101)
  0x05, 0x07, //   Usage Page (Key Codes)
  0x19, 0x00, //   Usage Minimum (0)
  0x29, 0x65, //   Usage Maximum (101)
  0x81, 0x00, //   Input (Data, Array)
  0xC0        // End Collection
};

// Global Scope Bluetooth Service Variables
static char bleName[] = "CHERRY";
extern bool useBLE;
extern uint8_t batteryLevel;

/**
 * Feature Module Includes
 * USB: USB Keyboard functionality
 * BLE: Bluetooth Low Energy Keyboard functionality
 * NRF: nRF24L01+ Wireless Keyboard functionality
 * RGB: RGBCK Switch functionality
 */
#include "features/RGB.h"
#include "features/USB.h"
#include "features/BLE.h"
#include "features/NRF.h"