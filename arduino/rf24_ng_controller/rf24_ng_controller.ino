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
const uint16_t POWER_DOWN_AFTER_IDLES = 2 * 1000 / RATE;  // N seconds
typedef uint16_t payload_t;

uint32_t joyPinsMask;
uint32_t currentColor, nextColor;
volatile bool isIdle;
volatile bool isPowered = true;
bool isBatteryLow = false;
volatile uint32_t idles = 0;

RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);
Ticker ledBlinkTicker(ledBlink, 500);
Ticker lowPowerSampleTicker(lowPowerSample, 2100);
Ticker joystickLoopTicker(joystickLoop, RATE);

void setupRadio() {
  int playerNum = digitalRead(PIN_SW_PLAYER) == LOW ? 2 : 1;
  bool highPower = digitalRead(PIN_SW_POWER) == LOW;

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
  joystickLoopTicker.start();
}

inline payload_t joystickPoll() {
  uint32_t gpio_current = gpio_get_all() & joyPinsMask;
  isIdle = gpio_current == joyPinsMask;
  idles = isIdle ? idles + 1 : 0;
  nextColor = isIdle ? 0x000030 : 0x000080;
  return gpio_current >> 2;
}

void joystickISR() {
  payload_t payloadBuffer = joystickPoll();
  if (!isPowered || idles >= POWER_DOWN_AFTER_IDLES) return;

  // Previous states are of no use to us
  radio.flush_tx();
  // Send new state immediately.
  // Non-blocking, add to FIFO and start sending, safe for ISR. Note from docs:
  // // This function now leaves the CE pin high, so the radio
  // // will remain in TX or STANDBY-II Mode until a txStandBy() command is issued.
  // So standby happens after next scheduled send.
  //                   buf,            len,               multicast, startTx
  radio.startFastWrite(&payloadBuffer, sizeof(payload_t), false, true);
}

void updateRadioPowered() {
  if (isIdle) {
    if (isPowered && idles > POWER_DOWN_AFTER_IDLES) {
      isPowered = false;
      // In case there is an in-flight send
      delayMicroseconds(100);
      radio.powerDown();
      if (!isBatteryLow) {
        // Simulate dimming, but battery low flash should not be messed with
        ledBlinkTicker.interval(10);
        ledBlinkTicker.start();
      }
    }
  } else {
    if (!isPowered) {
      radio.powerUp();
      if (!isBatteryLow) {
        ledBlinkTicker.stop();
        digitalWrite(PIN_LED, HIGH);
      }
      isPowered = true;
    }
  }
}

void joystickLoop() {
  payload_t payloadBuffer = joystickPoll();

  updateRadioPowered();

  if (!isPowered) return;

  // Disable interrupts to ensure exclusive access to the radio during the blocking write
  // Blocking is fine in main loop.
  // Drop back to standby mode when fifo is flushed.
  noInterrupts();
  radio.flush_tx();
  radio.write(&payloadBuffer, sizeof(payload_t));
  radio.txStandBy();
  interrupts();

#ifdef USE_RGB
  if (nextColor != currentColor) {
    led.setPixelColor(0, nextColor);
    led.show();
    currentColor = nextColor;
  }
#endif
}

void lowPowerSample() {
  if (isBatteryLow) return;

  float volts = map(analogRead(PIN_BATT_VSENSE), 0, 1023, 0, 3300) / 1000.0;
  if (volts < 0.5f) {
    // Probably not connected, ignore
    return;
  }

  // Could simplify if cutoff >= 3.3V then just check against max 1024
  if (volts < 3.25f) {
    isBatteryLow = true;
    // Slow blink
    ledBlinkTicker.interval(500);
    ledBlinkTicker.start();
    // Permanent state, no rechecking needed
    lowPowerSampleTicker.stop();
  }
}

void ledBlink() {
  static bool blinkOn = false;
  blinkOn = !blinkOn;
  digitalWrite(PIN_LED, blinkOn);
}

void loop() {
  joystickLoopTicker.update();
  lowPowerSampleTicker.update();
  ledBlinkTicker.update();
}