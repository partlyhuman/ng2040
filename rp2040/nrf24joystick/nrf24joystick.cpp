#include <cstdio>
#include <RF24.h>
#include <hardware/clocks.h>
#include <hardware/rosc.h>
#include <hardware/structs/scb.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "pico/sleep.h"

#define RADIO_CHANNEL 119
#define RADIO_LEVEL RF24_PA_MIN
#define PIN_CE 20
#define PIN_CS 17
#define SENDER 1
#define JOY_PIN_COUNT 10
#define DEBUGLED(t) gpio_put(PICO_DEFAULT_LED_PIN, t)
#define TIMEOUT_SEC 60

//                                    L  R  D  U  A  B  C  D  SEL STA
const uint8_t joyInPins[JOY_PIN_COUNT] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
const uint8_t address[][6] = {"P1", "P2", "P3", "P4", "P5", "P6"};
const uint8_t joyWakePin = joyInPins[9]; // START
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
    radio.setDataRate(RF24_1MBPS); // RF24_1MBPS RF24_2MBPS RF24_250KBPS
    radio.setRetries(2, 50);
    radio.setPayloadSize(sizeof(payload_t));
    radio.setAddressWidth(3);
    radio.setAutoAck(false);
    radio.disableDynamicPayloads();

    radio.openWritingPipe(address[SENDER]);
    radio.openReadingPipe(1, address[!SENDER]);
    radio.stopListening();
}

void deepSleep() {
    printf("GOING DOWN...\n");
    flashLED();

    uint scb_orig = scb_hw->scr;
    uint clock0_orig = clocks_hw->sleep_en0;
    uint clock1_orig = clocks_hw->sleep_en1;

    radio.powerDown();
    DEBUGLED(false);

    sleep_run_from_xosc();
    sleep_goto_dormant_until_pin(joyWakePin, true, false);

    // ----- DEEP SLEEP -----

    // Credit: https://ghubcoder.github.io/posts/awaking-the-pico/
    //Re-enable ring Oscillator control
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

    //reset procs back to default
    scb_hw->scr = scb_orig;
    clocks_hw->sleep_en0 = clock0_orig;
    clocks_hw->sleep_en1 = clock1_orig;

    //reset clocks
    clocks_init();
    stdio_init_all();

    radio.powerUp();
    setupRadio();

    printf("COMING UP!\n");
    flashLED(3);
}

void setup() {
    stdio_init_all();

    printf("OK\n");

    printf("Radio setup...\n");
    radio.begin();

    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, true);

    if (!radio.begin()) {
        panic("RADIO ERROR");
    }

    setupRadio();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO::DIRECTION_OUT);

    // Joystick pins are all input/pullup
    for (uint8_t pin: joyInPins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO::DIRECTION_IN);
        gpio_pull_up(pin);

        joyPinsMask |= (1ul << pin);
    }

    printf("joyPinsMask: %lx\n", joyPinsMask);
    radio.printPrettyDetails();

    flashLED();
}

// TODO consider switching to interrupts and underclocking
inline void loop() {
    static bool idleState = false;
    static absolute_time_t idleSleepTime = at_the_end_of_time;

    // indicate awake
    DEBUGLED(true);

    uint32_t gpio_current = gpio_get_all() & joyPinsMask;

    if (gpio_current == joyPinsMask) {
        // idle
        if (!idleState) {
            printf("idle start...\n");
            idleState = true;
            idleSleepTime = make_timeout_time_ms(TIMEOUT_SEC * 1000);
        } else if (time_reached(idleSleepTime)) {
            // indicate asleep
            DEBUGLED(false);
            deepSleep();
            idleState = false;
        }
    } else {
        idleState = false;
    }

    //    payload = 0;
    //    for (int i = 0; i < JOY_PIN_COUNT; i++) {
    //        payload |= gpio_get(joyInPins[i]) << i;
    //    }

    // manually unrolled, update if any changes to pin numbers or payload format
    payload =
            gpio_get(2) |
            (gpio_get(3) << 1) |
            (gpio_get(4) << 2) |
            (gpio_get(5) << 3) |
            (gpio_get(6) << 4) |
            (gpio_get(7) << 5) |
            (gpio_get(8) << 6) |
            (gpio_get(9) << 7) |
            (gpio_get(10) << 8) |
            (gpio_get(11) << 9);


    radio.write(&payload, sizeof(payload_t));
}


int main() {
    setup();
    while (true) {
        loop();
    }
    return 0;
}