#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#undef DEBUG
#define SENDER 0

#define DEBUGLED(t) digitalWrite(16, t)

#define RADIO_IRQ_PIN 27
#define RADIO_CE_PIN 26
#define RADIO_CS_PIN 13

RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);

typedef uint16_t payload_t;

#define JOY_PIN_COUNT 10

uint8_t joyOutPins[JOY_PIN_COUNT] = { 2, 3, 1, 0, 4, 5, 6, 7,  9,  8 };


// Let these addresses be used for the pair
uint8_t address[][6] = { "P1", "P2", "P3", "P4", "P5", "P6" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
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
  }

  if (payload == lastPayload) {
    return;
  }

  lastPayload = payload;
  for (int i = 0; i < JOY_PIN_COUNT; i++) {
    bool pressed = !(payload & (1 << i));
    pinMode(joyOutPins[i], pressed ? OUTPUT : INPUT);
    //    if (pressed) digitalWrite(joyOutPins[i], LOW);
    //    digitalWrite(joyOutPins[i], pressed ? LOW : HIGH);
  }


#ifdef DEBUG
  //  radio.printDetails();
  Serial.println(payload, BIN);
#endif

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


  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other. MIN/LOW/HIGH/MAX
  radio.setPALevel(RF24_PA_MIN);  // RF24_PA_MAX is default.

  radio.setChannel(24);
  //  radio.setDataRate(RF24_250KBPS); // RF24_1MBPS RF24_2MBPS RF24_250KBPS
  //  radio.setRetries(2, 10);


  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload_t));

  radio.setAddressWidth(3);
  //  radio.setAutoAck(false);
  //  radio.disableAckPayload();
  //  radio.disableDynamicPayloads();

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[SENDER]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!SENDER]);  // using pipe 1

  // additional setup specific to the node's role
  radio.maskIRQ(1, 1, 0);  // args = "data_sent", "data_fail", "data_ready
  radio.startListening();  // put radio in RX mode

  pinMode(RADIO_IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RADIO_IRQ_PIN), rxInterrupt, FALLING);

  for (int i = 0; i < JOY_PIN_COUNT; i++) {
    digitalWrite(joyOutPins[i], LOW);
    pinMode(joyOutPins[i], INPUT);
  }

#ifdef DEBUG
  // For debugging info
  printf_begin();             // needed only once for printing details
  radio.printDetails();       // (smaller) function that prints raw register values
  radio.printPrettyDetails(); // (larger) function that prints human readable data
#endif

}

void loop() {
}
