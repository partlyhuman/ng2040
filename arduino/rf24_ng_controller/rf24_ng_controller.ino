#include <RF24.h>
#include <Ticker.h>
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
const uint32_t JOYSTICK_POLL_MS = 33;

// Clock down loop by going to (CPU) sleep between iterations
// Controls the maximum time radio will be in TX (STANDBY-I) state
const uint32_t LOOP_SLEEP_MS = 2;

// How long with no inputs before the radio fully powers down
// Powering up takes some time but it may not be noticeable, in which case we'd want to power down more aggressively
// N seconds
const uint32_t POWER_DOWN_AFTER_IDLES = 2 * 1000 / JOYSTICK_POLL_MS;

static_assert(LOOP_SLEEP_MS < JOYSTICK_POLL_MS, "Loop will sleep over joystick poll invocations");

uint32_t joyPinsMask;
uint32_t currentColor, nextColor;
volatile bool isJoystickIdle = false;
volatile bool isRadioPowered = true;
volatile bool isRadioTx = false;
bool isBatteryLow = false;
volatile uint32_t idles = 0;

RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);
Ticker ledBlinkTicker(ledBlink, 500);
Ticker lowPowerSampleTicker(lowPowerSample, 2100);
Ticker joystickLoopTicker(joystickLoop, JOYSTICK_POLL_MS);

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

  if (!success) {
#ifdef USE_RGB
    led.setPixelColor(0, 0xff0000);
    led.show();
#else
    // blink really fast
    ledBlinkTicker.interval(100);
    ledBlinkTicker.start();
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

  lowPowerSampleTicker.start();
}

void ledBlink() {
  static bool blinkOn = false;
  blinkOn = !blinkOn;
  digitalWrite(PIN_LED, blinkOn);
}

void lowPowerSample() {
  // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
  const float conversion_factor = 3.3f / (1 << 12);
  float volts = analogRead(PIN_BATT_VSENSE) * conversion_factor;

  // Map 0-1023 range to 0V-3.3V
  // float volts = map(analogRead(PIN_BATT_VSENSE), 0, 1023, 0, 3300) / 1000.0;
  
  if (volts < 1.0) {
    // Pin is probably floating, ignore
    return;
  }

  // 10K resistor dropped around 0.015V experimentally
  // Could simplify if cutoff >= 3.3V then just check against max 1024
  if (volts < (3.3 - 0.015)) {
    isBatteryLow = true;
    // Slow blink
    ledBlinkTicker.interval(500);
    ledBlinkTicker.start();
    // Permanent state, no rechecking needed
    lowPowerSampleTicker.stop();
  }
}

void radioPower() {
  // use the volatile isRadioPowered instead of locking
  if (isJoystickIdle) {
    if (isRadioPowered && !isRadioTx && idles > POWER_DOWN_AFTER_IDLES) {
      isRadioPowered = false;
      radio.powerDown();
      if (!isBatteryLow) {
        // Simulate dimming, but battery low flash should not be messed with
        ledBlinkTicker.interval(10);
        ledBlinkTicker.start();
      }
    }
  } else {
    if (!isRadioPowered) {
      radio.powerUp();
      if (!isBatteryLow) {
        ledBlinkTicker.stop();
        digitalWrite(PIN_LED, HIGH);
      }
      isRadioPowered = true;
    }
  }
}

inline payload_t joystickPoll() {
  uint32_t gpio_current = gpio_get_all() & joyPinsMask;
  isJoystickIdle = gpio_current == joyPinsMask;
  idles = isJoystickIdle ? idles + 1 : 0;
  nextColor = isJoystickIdle ? 0x000030 : 0x000080;
  return gpio_current >> 2;
}

inline void radioSend(payload_t payload) {
  if (!isRadioPowered) return;
  
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

void joystickISR() {
  payload_t payload = joystickPoll();
  if (idles < POWER_DOWN_AFTER_IDLES) {
    radioSend(payload);
  }
}

void joystickLoop() {
  payload_t payload = joystickPoll();

  // Observe how long we've been idle and power down or up if needed
  radioPower();

  radioSend(payload);

#ifdef USE_RGB
  if (nextColor != currentColor) {
    led.setPixelColor(0, nextColor);
    led.show();
    currentColor = nextColor;
  }
#endif
}

void loop() {
  joystickLoopTicker.update();

  // Block until any pending sends are complete and go into standby mode
  // Go after the joystick loop so we can take care of the most recent send immediately
  if (isRadioTx) {
    noInterrupts();
    if (radio.txStandBy()) {
      isRadioTx = false;
    }
    interrupts();
  }

  lowPowerSampleTicker.update();
  ledBlinkTicker.update();

  // Power saving strategy - does it have an effect vs letting loop run?
  sleep_ms(LOOP_SLEEP_MS);
}