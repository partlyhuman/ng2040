#undef DEBUG
#undef USE_RGB

#define RATE 8            // milliseconds between sends. 16 = 60fps. 4 = 250fps
#define RADIO_CHANNEL 28  // Which RF channel to communicate on, 0-125
#define RADIO_CE_PIN 26
#define RADIO_CS_PIN 13
#define PIN_SW_POWER 28
#define PIN_SW_PLAYER 29
#define PIN_POWER_RADIO 0
#define PIN_BATT_VSENSE A1
#define PIN_LED 1

void ledBlink();
void lowPowerSample();
void joystickLoop();
void joystickISR();