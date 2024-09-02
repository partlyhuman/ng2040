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

typedef uint16_t payload_t;

uint32_t joyPinsMask;
const uint8_t ADDRS[][3] = { "P1", "P2" };
payload_t payload = 0;
uint32_t currentColor, nextColor;

RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);
Ticker lowPowerBlinkTicker(lowPowerBlink, 500);
Ticker lowPowerSampleTicker(lowPowerSample, 2100);
Ticker joystickPollTicker(joystickPoll, RATE);


void setupRadio() {
  int playerNum = digitalRead(PIN_SW_PLAYER) == LOW ? 2 : 1;
  bool highPower = digitalRead(PIN_SW_POWER) == LOW;

  radio.setPALevel(highPower ? RF24_PA_MAX : RF24_PA_LOW);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);  // RF24_1MBPS RF24_2MBPS RF24_250KBPS
  radio.setRetries(0, 0);
  radio.setPayloadSize(sizeof(payload_t));
  radio.setAddressWidth(3);  // Set the address width from 3 to 5 bytes (24, 32 or 40 bit)
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
  pinMode(PICO_DEFAULT_LED_PIN, OUTPUT);
  pinMode(PIN_SW_PLAYER, INPUT_PULLUP);
  pinMode(PIN_SW_POWER, INPUT_PULLUP);
  // Should we try changing the drive strength of the GPIO pins?

  SPI1.begin();

  // Turn on radio power
  pinMode(PIN_POWER_RADIO, OUTPUT);
  digitalWrite(PIN_POWER_RADIO, HIGH);
  delay(50);  // <2MS startup time for LDO

  bool success = radio.begin(&SPI1);

  if (!success) {
#ifdef USE_RGB
    led.setPixelColor(0, 0xff0000);
    led.show();
#else
    // blink really fast
    lowPowerBlinkTicker.interval(100);
    lowPowerBlinkTicker.start();
#endif
    return;
  }

  // Joystick pins are all input/pullup
  for (uint8_t pin : joyInPins) {
    // When you hit a button IMMEDIATELY send it, in addition to the regular polling
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin), joystickPoll, CHANGE);

    joyPinsMask |= (1ul << pin);
  }

  setupRadio();

  lowPowerSampleTicker.start();
  joystickPollTicker.start();
}

void joystickPoll() {
  uint32_t gpio_current = gpio_get_all() & joyPinsMask;
  bool idleState = gpio_current == joyPinsMask;

  nextColor = idleState ? 0x000030 : 0x000080;
  // payload = 0;
  // for (int i = 0; i < JOY_PIN_COUNT; i++) {
  //   payload |= (((gpio_current >> joyInPins[i]) & 0x1) << i);
  // }

  // manually unrolled, update if any changes to pin numbers or payload format
  payload =
    gpio_get(2) | (gpio_get(3) << 1) | (gpio_get(4) << 2) | (gpio_get(5) << 3) | (gpio_get(6) << 4) | (gpio_get(7) << 5) | (gpio_get(8) << 6) | (gpio_get(9) << 7) | (gpio_get(10) << 8) | (gpio_get(11) << 9);

  radio.write(&payload, sizeof(payload_t));
}

void lowPowerSample() {
  if (lowPowerBlinkTicker.state() == STOPPED) {
    float volts = map(analogRead(PIN_BATT_VSENSE), 0, 1023, 0, 3300) / 1000.0;
    if (volts < 0.5f) {
      // Probably not connected, ignore
      return;
    }

    // Could simplify if cutoff >= 3.3V then just check against max 1024
    if (volts < 3.25f) {
      lowPowerBlinkTicker.start();
    }
  }
}

static bool blinkOn = false;
void lowPowerBlink() {
  blinkOn = !blinkOn;
  digitalWrite(PIN_LED, blinkOn);
}

void loop() {
  lowPowerBlinkTicker.update();
  lowPowerSampleTicker.update();
  joystickPollTicker.update();

#ifdef USE_RGB
  if (nextColor != currentColor) {
    led.setPixelColor(0, nextColor);
    led.show();
    currentColor = nextColor;
  }
#endif
}