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
const uint32_t JOYSTICK_POLL_MS = 16;

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
  // initialize all pins to output low - does anything?
  gpio_set_dir_all_bits(~0);
  gpio_put_all(0);

  pinMode(PIN_BATT_VSENSE, INPUT);
  pinMode(PIN_LED, OUTPUT);
  // pinMode(PICO_DEFAULT_LED_PIN, OUTPUT);
  pinMode(PIN_SW_PLAYER, INPUT_PULLUP);
  pinMode(PIN_SW_POWER, INPUT_PULLUP);
  // Should we try changing the drive strength of the GPIO pins?

  SPI1.begin();

  // Turn on radio power
  pinMode(PIN_POWER_RADIO, OUTPUT);
  digitalWrite(PIN_POWER_RADIO, HIGH);
  delay(10);  // <2MS startup time for LDO

  bool success = radio.begin(&SPI1);
  isRadioPowered = success;

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
}

bool ledBlinkTimer(struct repeating_timer *t) {
  static bool blinkOn = false;
  blinkOn = !blinkOn;
  digitalWrite(PIN_LED, blinkOn);
  return true;
}

void ledBlink(uint32_t interval) {
  static struct repeating_timer timer;
  cancel_repeating_timer(&timer);

  if (interval == 0) {
    digitalWrite(PIN_LED, HIGH);
    return;
  }

  add_repeating_timer_ms(interval, ledBlinkTimer, NULL, &timer);
}

inline payload_t joystickPoll() {
  uint32_t bits = gpio_get_all() & joyPinsMask;
  isJoystickIdle = bits == joyPinsMask;
  idles = isJoystickIdle ? idles + 1 : 0;
  nextColor = isJoystickIdle ? 0x000030 : 0x000080;
  return bits >> 2;
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
    //                   buf,            len,               multicast, startTx
    radio.startFastWrite(&payload, sizeof(payload_t), false, true);
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
      // In case there is an in-flight send
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