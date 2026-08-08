#include "driver/gpio.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_RUN    GPIO_NUM_26
#define PIN_BRAKE  GPIO_NUM_25
#define PIN_LEFT   GPIO_NUM_33
#define PIN_RIGHT  GPIO_NUM_32

static void config_pin(gpio_num_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0); 
}

void tail_lights_init(void) {
    config_pin(PIN_RUN);
    config_pin(PIN_BRAKE);
    config_pin(PIN_LEFT);
    config_pin(PIN_RIGHT);
}

void tail_lights_set_running(bool enable) {
    gpio_set_level(PIN_RUN, enable ? 1 : 0);
}

void tail_lights_set_brake(bool enable) {
    gpio_set_level(PIN_BRAKE, enable ? 1 : 0);
}

void tail_lights_set_left_turn(bool enable) {
    gpio_set_level(PIN_LEFT, enable ? 1 : 0);
}

void tail_lights_set_right_turn(bool enable) {
    gpio_set_level(PIN_RIGHT, enable ? 1 : 0);
}

void tail_lights_off_all(void) {
    tail_lights_set_running(false);
    tail_lights_set_brake(false);
    tail_lights_set_left_turn(false);
    tail_lights_set_right_turn(false);
}

void app_main(void){
    tail_lights_init();
    while (1) {
        tail_lights_set_running(true);
        vTaskDelay(pdMS_TO_TICKS(1500));
        tail_lights_set_running(false);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}