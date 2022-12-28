#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define DEBUG
#define SENDER 0

#ifdef _BOARDS_WAVESHARE_RP2040_ZERO_H
#define DEBUGLED(t) digitalWrite(16, t)
#else
#define DEBUGLED(t) digitalWrite(PICO_DEFAULT_LED_PIN, t)
#endif

#define TEST_BTN_PIN 6

typedef uint16_t payload_t;

#define JOY_PIN_COUNT 10

//                                    L  R  D  U  A  B  C  D  SEL STA
#if SENDER
uint8_t joyInPins[JOY_PIN_COUNT] =  { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
#else
uint8_t joyOutPins[JOY_PIN_COUNT] = { 2, 3, 1, 0, 4, 5, 6, 7,  9,  8 };
#endif


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
    if (pressed) digitalWrite(joyOutPins[i], LOW);
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

#if SENDER
  RF24 radio(20, 17);  // CE, CSN
#else

#define RADIO_IRQ_PIN 15
  SPI.pins(
  RF24 radio(14, 13);  // CE, CSN
#endif


  // initialize the transceiver on the SPI bus
  bool success = false;
  do {
    success = radio.begin();
#ifdef DEBUG
    if (!success) {
      Serial.print("RADIO ERROR. ");
      Serial.print("SPI bus:");
      Serial.print(PICO_DEFAULT_SPI, DEC);
      Serial.print(" SCK:");
      Serial.print(PICO_DEFAULT_SPI_SCK_PIN, DEC);
      Serial.print(" TX:");
      Serial.print(PICO_DEFAULT_SPI_TX_PIN, DEC);
      Serial.print(" RX:");
      Serial.print(PICO_DEFAULT_SPI_RX_PIN, DEC);
      Serial.print(" CSN:");
      Serial.print(PICO_DEFAULT_SPI_CSN_PIN, DEC);
      Serial.println("");
    }
#endif
  } while (!success);

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other. MIN/LOW/HIGH/MAX
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

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
#if SENDER
  radio.stopListening();  // put radio in TX mode

  pinMode(PICO_DEFAULT_LED_PIN, OUTPUT);
  pinMode(TEST_BTN_PIN, INPUT_PULLUP);

  for (int i = 0; i < JOY_PIN_COUNT; i++) {
    pinMode(joyInPins[i], INPUT_PULLUP);
  }
#else
  radio.maskIRQ(1, 1, 0);  // args = "data_sent", "data_fail", "data_ready
  radio.startListening();  // put radio in RX mode

  pinMode(RADIO_IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RADIO_IRQ_PIN), rxInterrupt, FALLING);

  for (int i = 0; i < JOY_PIN_COUNT; i++) {
    pinMode(joyOutPins[i], INPUT);
  }
#endif

#ifdef DEBUG
  // For debugging info
  printf_begin();             // needed only once for printing details
  radio.printDetails();       // (smaller) function that prints raw register values
  radio.printPrettyDetails(); // (larger) function that prints human readable data
#endif

}

void loop() {

#if SENDER
  // This device is a TX node

  payload_t buttons = 0;

  // single button test
  //  buttons |= (digitalRead(TEST_BTN_PIN) == LOW);

  for (int i = 0; i < JOY_PIN_COUNT; i++) {
    buttons |= (digitalRead(joyInPins[i]) << i);
  }

  if (buttons == payload) {
    return;
  }

  payload = buttons;

  DEBUGLED(HIGH);
  unsigned long start_timer = micros();                // start the timer
  bool report = radio.write(&payload, sizeof(payload_t));  // transmit & save the report
  unsigned long end_timer = micros();                  // end the timer
  DEBUGLED(LOW);

#ifdef DEBUG
  Serial.println(payload, BIN);
  if (report) {
    Serial.print(F("Transmission successful! "));  // payload was delivered
    Serial.print(F("Time to transmit = "));
    Serial.print(end_timer - start_timer);  // print the timer result
    Serial.print(F(" us. Sent: "));
    Serial.println(payload);  // print payload sent
  } else {
    Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
  }
#endif

#else
  //  // This device is a RX node
  //  while (radio.available()) {
  //    radio.read(&payload, sizeof(payload_t));             // fetch payload from FIFO
  //#ifdef DEBUG
  //    //    Serial.print(F("Received "));
  //    //    Serial.print(bytes);  // print the size of the payload
  //    //    Serial.print(F(" bytes on pipe "));
  //    //    Serial.print(pipe);  // print the pipe number
  //    //    Serial.print(F(": "));
  //    Serial.println(payload);  // print the payload's value
  //#endif
  //    digitalWrite(LED_BUILTIN, payload > 0);
  //  }
#endif
}
