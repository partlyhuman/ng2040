#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#undef DEBUG
#define USE_IRQ
#define SENDER 0

#define DEBUGLED(t)

#define RADIO_IRQ_PIN 27
#define RADIO_CE_PIN 26
#define RADIO_CS_PIN 13
#define RADIO_LEVEL RF24_PA_MIN
#define RADIO_CHANNEL 119

RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);

typedef uint16_t payload_t;

#define JOY_PIN_COUNT 10

//                                    L  R  D  U  A  B  C  D  SEL STA
uint8_t joyOutPins[JOY_PIN_COUNT] = { 2, 3, 1, 0, 4, 5, 6, 7,  9,  8 };
uint8_t address[][6] = { "P1", "P2", "P3", "P4", "P5", "P6" };
static payload_t payload = 0;
static payload_t lastPayload = 0;

void rxInterrupt() {
  bool tx_ds, tx_df, rx_dr;                 // declare variables for IRQ masks
  radio.whatHappened(tx_ds, tx_df, rx_dr);  // get values for IRQ masks
  // whatHappened() clears the IRQ masks also. This is required for
  // continued TX operations when a transmission fails.
  // clearing the IRQ masks resets the IRQ pin to its inactive state (HIGH)

  if (radio.available()) {
    radio.read(&payload, sizeof(payload));

    for (int i = 0; i < JOY_PIN_COUNT; i++) {
      bool pressed = !(payload & (1 << i));
      pinMode(joyOutPins[i], pressed ? OUTPUT : INPUT);
    }

#ifdef DEBUG
    //  radio.printDetails();
    Serial.println(payload, BIN);
#endif
  }
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
#endif

  SPI1.begin();

  // initialize the transceiver on the SPI bus
  bool success = radio.begin(&SPI1);

  if (!success) {
#ifdef DEBUG
    Serial.print("RADIO ERROR. ");
    Serial.print(" SCK:");
    Serial.print(PIN_SPI1_SCK   , DEC);
    Serial.print(" MOSI:");
    Serial.print(PIN_SPI1_MOSI  , DEC);
    Serial.print(" MISO:");
    Serial.print(PIN_SPI1_MISO  , DEC);
    Serial.print(" CSN:");
    Serial.print(PIN_SPI1_SS    , DEC);
    Serial.println("");
#endif
    while (true);
  }


  radio.setPALevel(RADIO_LEVEL);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_1MBPS); // RF24_1MBPS RF24_2MBPS RF24_250KBPS
  radio.setPayloadSize(sizeof(payload_t));
  radio.setAddressWidth(3);
  radio.setAutoAck(false);
  radio.disableDynamicPayloads();

  radio.openWritingPipe(address[SENDER]);
  radio.openReadingPipe(1, address[!SENDER]);
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
  printf_begin();             // needed only once for printing details
  radio.printDetails();       // (smaller) function that prints raw register values
  radio.printPrettyDetails(); // (larger) function that prints human readable data
#endif

}

void loop() {
#ifndef USE_IRQ
  if (radio.available()) {
    radio.read(&payload, sizeof(payload_t));
#ifdef DEBUG
    Serial.println(payload);  // print the payload's value
#endif
    for (int i = 0; i < JOY_PIN_COUNT; i++) {
      bool pressed = !(payload & (1 << i));
      pinMode(joyOutPins[i], pressed ? OUTPUT : INPUT);
      // digitalWrite(joyOutPins[i], LOW);
    }
  }
#endif
}
