#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include <printf.h>

#undef DEBUG
#define USE_IRQ
#define USE_RGB

#define DEBUGLED(t)
#define RADIO_CHANNEL 119 // Which RF channel to communicate on, 0-125
#define RADIO_IRQ_PIN 27
#define RADIO_CE_PIN 26
#define RADIO_CS_PIN 13
#define PIN_SW_POWER 28
#define PIN_SW_PLAYER 29

#define JOY_PIN_COUNT 10
//                                          L  R  D  U  A  B  C  D  SEL STA
const uint8_t joyOutPins[JOY_PIN_COUNT] = { 2, 3, 1, 0, 4, 5, 6, 7, 9, 8 };
const uint32_t ledColors[JOY_PIN_COUNT] = { 0x101010, 0x101010, 0x101010, 0x101010, 0x300000, 0x181800, 0x003000, 0x000030, 0x200020, 0x200020 };

const uint8_t ADDRS[][3] = {"P1", "P2"};

typedef uint16_t payload_t;
static payload_t payload = 0;
static payload_t lastPayload = 0;

static RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);
static Adafruit_NeoPixel led(1, 16, NEO_GRB);

inline void updateJoystick() {
  uint32_t color = 0;
  for (int i = 0; i < JOY_PIN_COUNT; i++) {
    bool pressed = !bitRead(payload, i);
    // TODO make this a batch like gpio_set...?
    pinMode(joyOutPins[i], pressed ? OUTPUT : INPUT);
#ifdef USE_RGB
    if (pressed) color = ledColors[i];
#endif
  }
#ifdef USE_RGB
  static uint32_t lastColor = 0;
  if (color != lastColor) {
    led.setPixelColor(0, color);
    led.show();
    lastColor = color;
  }
#endif
}

void rxInterrupt() {
  // declare variables for IRQ masks
  bool tx_ds, tx_df, rx_dr;
  // get values for IRQ masks
  radio.whatHappened(tx_ds, tx_df, rx_dr);
  // whatHappened() clears the IRQ masks also. This is required for
  // continued TX operations when a transmission fails.
  // clearing the IRQ masks resets the IRQ pin to its inactive state (HIGH)

  if (radio.available()) {
    radio.read(&payload, sizeof(payload));
    updateJoystick();
#ifdef DEBUG
    //  radio.printDetails();
    Serial.println(payload, BIN);
#endif
  }
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  SPI1.begin();
  bool success = radio.begin(&SPI1);

  if (!success) {
#ifdef DEBUG
    Serial.print("RADIO ERROR. ");
    Serial.print(" SCK:");
    Serial.print(PIN_SPI1_SCK, DEC);
    Serial.print(" MOSI:");
    Serial.print(PIN_SPI1_MOSI, DEC);
    Serial.print(" MISO:");
    Serial.print(PIN_SPI1_MISO, DEC);
    Serial.print(" CSN:");
    Serial.print(PIN_SPI1_SS, DEC);
    Serial.println("");
#endif
    led.setPixelColor(0, 0xff0000);
    led.show();
    panic("RADIO ERROR");
  }
  
  pinMode(PIN_SW_PLAYER, INPUT_PULLUP);
  pinMode(PIN_SW_POWER, INPUT_PULLUP);
  int playerNum = digitalRead(PIN_SW_PLAYER) == LOW ? 2 : 1;
  bool highPower = digitalRead(PIN_SW_POWER) == LOW;
  led.setPixelColor(0, playerNum == 1 ? 0x00ff00 : 0x0000ff);
  led.show();

  radio.setPALevel(highPower ? RF24_PA_HIGH : RF24_PA_LOW);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_1MBPS);  // RF24_1MBPS RF24_2MBPS RF24_250KBPS
  radio.setRetries(0, 0);
  radio.setPayloadSize(sizeof(payload_t));
  radio.setAddressWidth(3); // Set the address width from 3 to 5 bytes (24, 32 or 40 bit)
  radio.setAutoAck(false);
  radio.disableDynamicPayloads();

  // radio.openWritingPipe(address[SENDER]);
  radio.openReadingPipe(0, ADDRS[playerNum - 1]);
  radio.startListening();

#ifdef USE_IRQ
  radio.maskIRQ(1, 1, 0);  // args = "data_sent", "data_fail", "data_ready
  pinMode(RADIO_IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RADIO_IRQ_PIN), rxInterrupt, FALLING);
#endif

  for (int i = 0; i < JOY_PIN_COUNT; i++) {
    pinMode(joyOutPins[i], INPUT);
  }

#ifdef DEBUG
  printf_begin();              // needed only once for printing details
  radio.printDetails();        // (smaller) function that prints raw register values
  radio.printPrettyDetails();  // (larger) function that prints human readable data
#endif
}

void loop() {
#ifndef USE_IRQ
  if (radio.available()) {
    radio.read(&payload, sizeof(payload_t));
#ifdef DEBUG
    Serial.println(payload);  // print the payload's value
#endif
    updateJoystick();
  }
#endif
}
