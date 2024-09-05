#undef DEBUG
#undef USE_RGB

// Which RF channel to communicate on, 0-125
#define RADIO_CHANNEL 28
#define RADIO_CE_PIN 26
#define RADIO_CS_PIN 13
#define PIN_SW_POWER 28
#define PIN_SW_PLAYER 29
#define PIN_POWER_RADIO 0
#define PIN_BATT_VSENSE A1
#define PIN_LED 1



// void lowPowerSample() {
//   // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
//   const float conversion_factor = 3.3f / (1 << 12);
//   float volts = analogRead(PIN_BATT_VSENSE) * conversion_factor;

//   // Map 0-1023 range to 0V-3.3V
//   // float volts = map(analogRead(PIN_BATT_VSENSE), 0, 1023, 0, 3300) / 1000.0;

//   if (volts < 1.0) {
//     // Pin is probably floating, ignore
//     return;
//   }

//   // 10K resistor dropped around 0.015V experimentally
//   // Could simplify if cutoff >= 3.3V then just check against max 1024
//   if (volts < (3.3 - 0.015)) {
//     isBatteryLow = true;
//     // Slow blink
//     ledBlinkTicker.interval(500);
//     ledBlinkTicker.start();
//     // Permanent state, no rechecking needed
//     lowPowerSampleTicker.stop();
//   }
// }