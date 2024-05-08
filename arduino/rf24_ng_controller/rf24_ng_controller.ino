#include <RF24.h>
#include <Adafruit_NeoPixel.h>

#undef DEBUG
#define USE_RGB

#define RATE 7             // milliseconds between sends. 16 = 60fps. 4 = 250fps
#define RADIO_CHANNEL 119  // Which RF channel to communicate on, 0-125
#define RADIO_CE_PIN 26
#define RADIO_CS_PIN 13
#define PIN_SW_POWER 28
#define PIN_SW_PLAYER 29

#define JOY_PIN_COUNT 10
//                                         L  R  D  U  A  B  C  D  SEL STA
const uint8_t joyInPins[JOY_PIN_COUNT] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
const uint32_t ledColors[JOY_PIN_COUNT] = { 0x101010, 0x101010, 0x101010, 0x101010, 0x300000, 0x181800, 0x003000, 0x000030, 0x200020, 0x200020 };

static uint32_t joyPinsMask;

const uint8_t ADDRS[][3] = { "P1", "P2" };

typedef uint16_t payload_t;
static payload_t payload = 0;

static RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);
static Adafruit_NeoPixel led(1, 16, NEO_GRB);

void flashLED(int times = 10, int ms = 50) {
  bool on = true;
  for (int i = 0; i < times; i++, on = !on) {
    led.setPixelColor(0, 0xffffff);
    led.show();
    sleep_ms(ms);
  }
  led.clear();
  led.show();
}

void setupRadio() {
  int playerNum = digitalRead(PIN_SW_PLAYER) == LOW ? 2 : 1;
  bool highPower = digitalRead(PIN_SW_POWER) == LOW;

  radio.setPALevel(highPower ? RF24_PA_MAX : RF24_PA_LOW);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_1MBPS);  // RF24_1MBPS RF24_2MBPS RF24_250KBPS
  radio.setRetries(0, 0);
  radio.setPayloadSize(sizeof(payload_t));
  radio.setAddressWidth(3);  // Set the address width from 3 to 5 bytes (24, 32 or 40 bit)
  radio.setAutoAck(false);
  radio.disableDynamicPayloads();

  radio.openWritingPipe(ADDRS[playerNum - 1]);
  // No reading pipe needed, right?
  // radio.openReadingPipe(1, address[!SENDER]);
  // radio.stopListening();
}


void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  SPI1.begin();
  bool success = radio.begin(&SPI1);

  if (!success) {
    led.setPixelColor(0, 0xff0000);
    led.show();
    panic("RADIO ERROR");
  }

  // Joystick pins are all input/pullup
  for (uint8_t pin : joyInPins) {
    gpio_init(pin);
    gpio_set_dir(pin, INPUT);
    gpio_pull_up(pin);

    joyPinsMask |= (1ul << pin);
  }

  pinMode(PICO_DEFAULT_LED_PIN, OUTPUT);
  pinMode(PIN_SW_PLAYER, INPUT_PULLUP);
  pinMode(PIN_SW_POWER, INPUT_PULLUP);

  setupRadio();

#ifdef DEBUG
  Serial.printf("joyPinsMask: %lx\n", joyPinsMask);
  radio.printPrettyDetails();
#endif

  flashLED();
}

static unsigned long lastTime = 0;
void loop() {
  // Throttle sending
  unsigned long now = millis();
  if (now - lastTime < RATE) {
    return;
  }
  lastTime = now;

  uint32_t gpio_current = gpio_get_all() & joyPinsMask;
  bool idleState = gpio_current == joyPinsMask;
#ifdef USE_RGB
  led.setPixelColor(0, idleState ? 0x100000 : 0x400000);
  led.show();
#endif

  // payload = 0;
  // for (uint8_t pin : joyInPins) {
  //   payload |= ((gpio_current >> pin) & 0x1);
  //   payload <<= 1;
  // }

  // manually unrolled, update if any changes to pin numbers or payload format
  payload =
    gpio_get(2) | (gpio_get(3) << 1) | (gpio_get(4) << 2) | (gpio_get(5) << 3) | (gpio_get(6) << 4) | (gpio_get(7) << 5) | (gpio_get(8) << 6) | (gpio_get(9) << 7) | (gpio_get(10) << 8) | (gpio_get(11) << 9);

  radio.write(&payload, sizeof(payload_t));
}