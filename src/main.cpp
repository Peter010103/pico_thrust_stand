#include <stdio.h>
#include <string.h>

#include "hx711.h"
#include "hx711_noblock.pio.h"
#include "pico/stdlib.h"
#include "shoot.h"
#include "utils.h"

void update_signal(int &key_input) {
    // l (led)
    if (key_input == 108) {
        utils::flash_led(LED_BUILTIN);
    }

    // a (arm)
    // if (strcmp(key_input, ) == 97)
    // {
    //     // Disable timer
    //     cancel_repeating_timer(&send_frame_rt);
    //     // Arm motor
    //     arm_motor();
    //     // Re-enable timer
    //     dma_alarm_rt_state =
    //     alarm_pool_add_repeating_timer_us(pico_alarm_pool,
    //     DMA_ALARM_PERIOD, repeating_send_dshot_frame, NULL,
    //     &send_frame_rt); printf("Re-enabled repeating DMA alarm:
    //     "); printfln(dma_alarm_rt_state);

    //     return;
    // }

    // b - beep
    if (key_input == 98) {
        shoot::throttle_code = 1;
        shoot::telemetry = 1;
    }

    // s - spin
    // if (strcmp(key_input, ) == 115)
    // {
    //     // Disable timer
    //     cancel_repeating_timer(&send_frame_rt);
    //     // Ramp up to speed ARM_THROTTLE + 100
    //     ramp_motor();
    //     // Re-enable timer
    //     dma_alarm_rt_state =
    //     alarm_pool_add_repeating_timer_us(pico_alarm_pool,
    //     DMA_ALARM_PERIOD, repeating_send_dshot_frame, NULL,
    //     &send_frame_rt); printf("Re-enabled repeating DMA alarm:
    //     "); printfln(dma_alarm_rt_state);
    // }

    // r - rise
    if (key_input == 114) {
        if (shoot::throttle_code >= ZERO_THROTTLE and
            shoot::throttle_code <= MAX_THROTTLE) {
            // Check for max throttle
            if (shoot::throttle_code == MAX_THROTTLE) {
                printf("Max Throttle reached\n");
            } else {
                shoot::throttle_code += 1;
                printf("Throttle: %i", shoot::throttle_code);
            }
        } else {
            printf("Motor is not in throttle mode\n");
        }
    }

    // f - fall
    if (key_input == 102) {
        if (shoot::throttle_code <= MAX_THROTTLE &&
            shoot::throttle_code >= ZERO_THROTTLE) {
            if (shoot::throttle_code == ZERO_THROTTLE) {
                printf("Throttle is zero\n");
            } else {
                shoot::throttle_code -= 1;
                printf("Throttle: %i", shoot::throttle_code);
            }
        } else {
            printf("Motor is not in throttle mode\n");
        }
    }

    // q - disarm
    if (key_input == 32) {
        shoot::throttle_code = ZERO_THROTTLE;
        shoot::telemetry = 0;
    }

    shoot::send_dshot_frame();
    printf("Finished processing byte.\n");
}

int main() {
    int32_t thrust;
    int key_input;

    stdio_init_all();
    gpio_init(LED_BUILTIN);
    gpio_set_dir(LED_BUILTIN, GPIO_OUT);

    printf("Setting up load cell\n");
    hx711_t hx;
    hx711_init(&hx, CLKPIN, DATPIN, pio0, &hx711_noblock_program,
               &hx711_noblock_program_init);
    hx711_power_up(&hx, hx711_gain_128);
    hx711_wait_settle(hx711_rate_10);

    if (hx711_get_value_timeout(&hx, 250000, &thrust)) {
        printf("%li\n", thrust);
    }

    printf("Setting up dshot\n");
    tts::pwm_setup();
    tts::dma_setup();
    shoot::rt_setup();

    sleep_ms(1500);

    // tts::print_gpio_setup();
    // tts::print_dshot_setup();
    // tts::print_dma_setup();

    printf("Initial throttle:\n %i\n", shoot::throttle_code);
    printf("Initial telemetry:\n %i\n", shoot::telemetry);

    while (1) {
        key_input = getchar_timeout_us(0);
        update_signal(key_input);
        if (hx711_get_value_noblock(&hx, &thrust)) {
            printf("%li\n", thrust);
        }
        utils::flash_led(LED_BUILTIN);
    }
}

