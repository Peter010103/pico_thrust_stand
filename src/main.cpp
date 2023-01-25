#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "shoot.h"
#include "utils.h"

void loop() {
    char key_input[1024];

    scanf("%1024s", key_input);
    printf("Byte: %1024s", key_input);

    // l (led)
    if (strcmp(key_input, "l") == 0) {
        utils::flash_led(LED_BUILTIN);
    }

    // b - beep
    if (strcmp(key_input, "b") == 0) {
        shoot::throttle_code = 1;
        shoot::telemetry = 1;
    }

    // r - rise
    if (strcmp(key_input, "r") == 0) {
        if (shoot::throttle_code >= ZERO_THROTTLE and
            shoot::throttle_code <= MAX_THROTTLE) {
            // Check for max throttle
            if (shoot::throttle_code == MAX_THROTTLE) {
                printf("Max Throttle reached\n");
            } else {
                shoot::throttle_code += 50;
                if (shoot::throttle_code > MAX_THROTTLE)
                    shoot::throttle_code = MAX_THROTTLE;
                printf("Throttle: %i\n", shoot::throttle_code - ZERO_THROTTLE);
            }
        } else {
            printf("Motor is not in throttle mode\n");
        }
    }

    // f - fall
    if (strcmp(key_input, "f") == 0) {
        if (shoot::throttle_code <= MAX_THROTTLE &&
            shoot::throttle_code >= ZERO_THROTTLE) {
            if (shoot::throttle_code == ZERO_THROTTLE) {
                printf("Throttle is zero\n");
            } else {
                shoot::throttle_code -= 50;
                printf("Throttle: %i\n", shoot::throttle_code - ZERO_THROTTLE);
            }
        } else {
            printf("Motor is not in throttle mode\n");
        }
    }

    // q - disarm
    // Not sure how to disarm yet. Maybe set throttle to 0 and don't send a
    // cmd for some secs?
    if (key_input == 32) {
        shoot::throttle_code = ZERO_THROTTLE;
        shoot::telemetry = 0;
        printf("Throttle: %i\n", 0);
    }

    // NOTE: In DEBUG mode, sending a DSHOT Frame takes a lot of time!
    // So it may seem as if the PICO is unable to detect key presses
    // while sending commands!
    // But is this even needed?
    shoot::send_dshot_frame();
    printf("Finished processing byte.\n");
}

int main() {
    int32_t thrust;
    int key_input;

    // Load cell calibration
    int32_t zero_val = 150622;

    stdio_init_all();

    gpio_init(LED_BUILTIN);
    gpio_set_dir(LED_BUILTIN, GPIO_OUT);

    for (int i = 0; i < 3; i++) {
        gpio_put(LED_BUILTIN, 1);  // Set pin 25 to high
        printf("LED ON!\n");
        sleep_ms(1000);  // 0.5s delay

        gpio_put(LED_BUILTIN, 0);  // Set pin 25 to low
        printf("LED OFF!\n");
        sleep_ms(1000);  // 0.5s delay
    }

    printf("Setting up load cell\n");
    hx711_t hx;
    hx711_init(&hx, CLKPIN, DATPIN, pio0, &hx711_noblock_program,
               &hx711_noblock_program_init);
    hx711_power_up(&hx, hx711_gain_128);
    hx711_wait_settle(hx711_rate_10);

    // pwm config
    // Note that PWM needs to be setup first,
    // because the dma dreq requires tts::pwm_slice_num
    tts::pwm_setup();

    // dma config
    tts::dma_setup();

    // Set repeating timer
    // NOTE: this can be put in main loop to start
    // repeating timer on key press (e.g. a for arm)
    shoot::rt_setup();

    sleep_ms(1500);

    tts::print_gpio_setup();
    tts::print_dshot_setup();
    tts::print_dma_setup();

    printf("Initial throttle: %i", shoot::throttle_code);
    printf("\tInitial telemetry: %i", shoot::telemetry);

    while (1) {
        key_input = getchar_timeout_us(0);
        update_signal(key_input);
        sleep_ms(100);
        if (hx711_get_value_noblock(&hx, &thrust)) {
            printf("%li\n", thrust - zero_val);
        }
    }
}

