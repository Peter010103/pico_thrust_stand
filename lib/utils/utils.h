#pragma once
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

namespace utils {
void print_pwm_config(const pwm_config &conf);
void flash_led(const int &pin);

} // namespace utils
