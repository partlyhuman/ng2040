#include <RF24.h>
#include "rf24_ng_controller.h"

#ifdef USE_RGB
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel led(1, 16, NEO_GRB);
#endif

#define JOY_PIN_COUNT 10
//                                         L  R  D  U  A  B  C  D  SEL STA
const uint8_t joyInPins[JOY_PIN_COUNT] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
const uint32_t ledColors[JOY_PIN_COUNT] = { 0x101010, 0x101010, 0x101010, 0x101010, 0x300000, 0x181800, 0x003000, 0x000030, 0x200020, 0x200020 };
const uint8_t ADDRS[][3] = { "P1", "P2" };
typedef uint16_t payload_t;

// Manual polling and sending in addition to interrupt-driven
// If an input is dropped, this controls the maximum duration before a new one is sent
// 16ms = 1 frame @ 60fps
const uint32_t JOYSTICK_POLL_MS = 100;

// How long with no inputs before the radio fully powers down
// Powering up takes some time but it may not be noticeable, in which case we'd want to power down more aggressively
// N seconds
const uint32_t POWER_DOWN_AFTER_IDLES = 2 * 1000 / JOYSTICK_POLL_MS;

uint32_t joyPinsMask;
uint32_t currentColor, nextColor;
bool isBatteryLow;
volatile bool isJoystickIdle;
volatile bool isRadioPowered;
volatile bool isRadioTx;
volatile uint32_t idles;

struct repeating_timer ledTimer;
struct repeating_timer batteryCheckTimer;

RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);

void setupRadio() {
  int playerNum = digitalRead(PIN_SW_PLAYER) == LOW ? 2 : 1;
  bool highPower = digitalRead(PIN_SW_POWER) == LOW;

  // Radio power savings: low power, slowest data rate, smallest possible address width, 2 byte payload, no acks
  radio.setPALevel(highPower ? RF24_PA_MAX : RF24_PA_LOW);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);  // RF24_1MBPS RF24_2MBPS RF24_250KBPS
  radio.setRetries(0, 0);
  radio.setPayloadSize(sizeof(payload_t));
  radio.setAddressWidth(3);  // 3 is minimum (in our case "P1\0")
  radio.setAutoAck(false);
  radio.disableDynamicPayloads();

  radio.openWritingPipe(ADDRS[playerNum - 1]);

#ifdef USE_RGB
  led.setPixelColor(0, highPower ? 0xff00ff : 0x101010);
  led.show();
  delay(100);
  led.setPixelColor(0, playerNum == 1 ? 0x00ff00 : 0x0000ff);
  led.show();
  delay(100);
  led.clear();
  led.show();
#else
  digitalWrite(PIN_LED, HIGH);
#endif
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  pinMode(PIN_VSENSE, INPUT);
  pinMode(PIN_LED, OUTPUT_2MA);
  pinMode(PIN_SW_PLAYER, INPUT_PULLUP);
  pinMode(PIN_SW_POWER, INPUT_PULLUP);

  SPI1.begin();

  // Pathetic attempt to save a tiny sliver of power
  gpio_set_drive_strength(12, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(14, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(15, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(RADIO_CS_PIN, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(RADIO_CE_PIN, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(PIN_POWER_RADIO, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(PIN_LED, GPIO_DRIVE_STRENGTH_2MA);

  // Turn on radio power
  pinMode(PIN_POWER_RADIO, OUTPUT);
  digitalWrite(PIN_POWER_RADIO, HIGH);
  delay(10);  // <2MS startup time for LDO

  bool success = radio.begin(&SPI1);

  if (!success) {
#ifdef USE_RGB
    led.setPixelColor(0, 0xff0000);
    led.show();
#else
    // blink really fast
    ledBlink(100);
#endif
    return;
  }

  // Joystick pins are all input/pullup
  for (uint8_t pin : joyInPins) {
    // When you hit a button IMMEDIATELY send it, in addition to the regular polling
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin), joystickISR, CHANGE);

    joyPinsMask |= (1ul << pin);
  }

  setupRadio();
  isRadioPowered = true;

  add_repeating_timer_ms(509, batteryCheckCallback, NULL, &batteryCheckTimer);
}

bool ledCallback(struct repeating_timer *t) {
  static bool blinkOn = false;
  blinkOn = !blinkOn;
  digitalWrite(PIN_LED, blinkOn);
  return true;
}

void ledBlink(uint32_t interval) {
  cancel_repeating_timer(&ledTimer);

  if (interval == 0) {
    digitalWrite(PIN_LED, HIGH);
    return;
  }

  add_repeating_timer_ms(interval, ledCallback, NULL, &ledTimer);
}

inline payload_t joystickPoll() {
  uint32_t bits = gpio_get_all() & joyPinsMask;
  isJoystickIdle = bits == joyPinsMask;
  idles = isJoystickIdle ? idles + 1 : 0;
  nextColor = isJoystickIdle ? 0x000030 : 0x000080;
  return bits >> 2;
}

// My "clever" way to measure battery power when the battery itself is powering the reference voltage:
// VBatt => 3.3V regulator on RP2040 Zero => VRef
// VBatt => 3.0V regulator on NG2040 board => A1 pin, measure
// Using the simplified idea that for a regulator, VOut = Min(VReg, VIn),
// note that for 3.0 < VBatt < 3.3, the 3.0V regulator will output 3.0V but the 3.3V regulator will be dropping below 3.3V to VBatt.
// This way we can flip the script, and MEASURE the 3.0V regulator in order to INFER a drop in the reference voltage.
// Again this really only works between 3.0V and 3.3V, but that's exactly when we want to warn that battery is low.
bool batteryCheckCallback(struct repeating_timer *t) {
  // We need to know what our scale/resolution is
  const int bits = 12;
  analogReadResolution(bits);
  float scale = (float)(1 << bits);

  const float LOW_BATTERY_V = 3.25;

  // VMeasured = 3.0 / VRef
  // rearranging, VRef = 3.0 / VMeasured
  // VRef = MIN(VBatt, 3.3)
  const float fixed3_0 = 3.0;
  float measured = analogRead(PIN_VSENSE) / scale;
  float vref = fixed3_0 / measured;

#ifdef USE_RGB
  if (vref >= 3.3) {
    led.setPixelColor(0, 0xffffff);
  } else if (vref >= 3.25) {
    led.setPixelColor(0, 0x0000ff);
  } else if (vref >= 3.2) {
    led.setPixelColor(0, 0x008080);
  } else if (vref >= 3.15) {
    led.setPixelColor(0, 0x00ff00);
  } else if (vref >= 3.1) {
    led.setPixelColor(0, 0x808000);
  } else if (vref >= 3.05) {
    led.setPixelColor(0, 0xff0000);
  } else {
    led.setPixelColor(0, 0x100000);
  }
  led.show();
#endif

  if (vref < 1.0) {
    // Pin is probably floating, ignore
    return true;
  }

  static int lowBattReadings = 0;
  if (vref < LOW_BATTERY_V) {
    // Make sure this is not a fluke by capturing multiple readings
    if (++lowBattReadings > 10) {
      isBatteryLow = true;
      // Slow blink
      ledBlink(800);
      // Stop measuring
      return false;
    }
  }

  return true;
}

void joystickISR() {
  payload_t payload = joystickPoll();
  if (isRadioPowered && idles < POWER_DOWN_AFTER_IDLES) {
    // Previous states are of no use to us
    radio.flush_tx();
    // Send new state immediately.
    // Non-blocking, add to FIFO and start sending, safe for ISR. Note from docs:
    // // This function now leaves the CE pin high, so the radio
    // // will remain in TX or STANDBY-II Mode until a txStandBy() command is issued.
    radio.startFastWrite(&payload,
                         sizeof(payload_t),
                         false,  // multicast
                         true);  // startTx
    isRadioTx = true;
  }
}

void loop() {
  absolute_time_t timeout = make_timeout_time_ms(JOYSTICK_POLL_MS);
  best_effort_wfe_or_timeout(timeout);

  if (isRadioTx) {
    // If we woke up from a transmit ISR, complete it
    if (radio.txStandBy()) {
      isRadioTx = false;
    }
  } else {
    // This could be scheduled, or we could have woken up because the joystick was touched and we need to power up
    payload_t payload = joystickPoll();

    if (isRadioPowered && idles > POWER_DOWN_AFTER_IDLES && !isRadioTx) {
      isRadioPowered = false;
      radio.powerDown();
      if (!isBatteryLow) {
        // Simulate dimming, but battery low flash should not be messed with
        ledBlink(15);
      }
    } else if (!isRadioPowered && !isJoystickIdle) {
      radio.powerUp();
      if (!isBatteryLow) {
        ledBlink(0);
      }
      isRadioPowered = true;
    }

    if (isRadioPowered) {
      // This is fine it does send right away, it just won't sleep until the next manual poll (whatever)
      // radio.startFastWrite(&payload, sizeof(payload_t), false, true);
      // isRadioTx = true;

      // Blocking write and back to standby when done
      // radio.flush_tx(); // Shouldn't be needed
      radio.write(&payload, sizeof(payload_t));
      // Basic write does fall back to standby on its own
    }
  }

#ifdef USE_RGB
  if (nextColor != currentColor) {
    led.setPixelColor(0, nextColor);
    led.show();
    currentColor = nextColor;
  }
#endif
}