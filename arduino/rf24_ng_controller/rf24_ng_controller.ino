#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <RF24.h>
#include <SPI.h>

#undef PLAYER2
#undef DEBUG

#ifdef PLAYER2
#define RADIO_CHANNEL 0
const uint8_t address[][6] = { "P7", "P8", "P9", "PA", "PB", "PC" }; //P2
#else
#define RADIO_CHANNEL 119
const uint8_t address[][6] = { "P1", "P2", "P3", "P4", "P5", "P6" }; //P1
#endif

#define RADIO_LEVEL RF24_PA_LOW
#define PIN_CE 20
#define PIN_CS 17
#define SENDER 1
#define JOY_PIN_COUNT 10
#define DEBUGLED(t) gpio_put(PICO_DEFAULT_LED_PIN, t)

//                                         L  R  D  U  A  B  C  D  SEL STA
const uint8_t joyInPins[JOY_PIN_COUNT] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
const uint8_t joyWakePin = joyInPins[9];  // START
static uint32_t joyPinsMask;

typedef uint16_t payload_t;
static payload_t payload = 0;

static RF24 radio(PIN_CE, PIN_CS);

void flashLED(int times = 10, int ms = 50) {
  bool on = true;
  for (int i = 0; i < times; i++, on = !on) {
    DEBUGLED(on);
    sleep_ms(ms);
  }
  DEBUGLED(false);
}

void setupRadio() {
  radio.setPALevel(RADIO_LEVEL);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_1MBPS);  // RF24_1MBPS RF24_2MBPS RF24_250KBPS
  radio.setRetries(2, 50);
  radio.setPayloadSize(sizeof(payload_t));
  radio.setAddressWidth(3); // TODO can we make this 2, or 1?
  radio.setAutoAck(false);
  radio.disableDynamicPayloads();

  radio.openWritingPipe(address[SENDER]);
  radio.openReadingPipe(1, address[!SENDER]);
  radio.stopListening();
}


void setup() {
  DEBUGLED(true);
#ifdef DEBUG
  Serial.begin();
#endif
  radio.begin();

  gpio_set_dir(PIN_CS, OUTPUT);
  gpio_put(PIN_CS, true);

  if (!radio.begin()) {
    panic("RADIO ERROR");
  }

  setupRadio();

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, OUTPUT);

  // Joystick pins are all input/pullup
  for (uint8_t pin : joyInPins) {
    gpio_init(pin);
    gpio_set_dir(pin, INPUT);
    gpio_pull_up(pin);

    joyPinsMask |= (1ul << pin);
  }

#ifdef DEBUG
  Serial.printf("joyPinsMask: %lx\n", joyPinsMask);
  radio.printPrettyDetails();
#endif

  flashLED();
}

// TODO consider underclocking
// TODO consider throttling rate
void loop() {
  uint32_t gpio_current = gpio_get_all() & joyPinsMask;
  bool idleState = gpio_current == joyPinsMask;
  DEBUGLED(!idleState);

  // manually unrolled, update if any changes to pin numbers or payload format
  payload =
    gpio_get(2) | (gpio_get(3) << 1) | (gpio_get(4) << 2) | (gpio_get(5) << 3) | (gpio_get(6) << 4) | (gpio_get(7) << 5) | (gpio_get(8) << 6) | (gpio_get(9) << 7) | (gpio_get(10) << 8) | (gpio_get(11) << 9);


  radio.write(&payload, sizeof(payload_t));
}