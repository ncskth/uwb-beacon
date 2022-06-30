#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "hardware.h"

void init_rgb();
void init_uwb();


void app_main() {
    init_rgb();
}

void init_rgb() {
    // led
    ledc_timer_config_t led_timer_conf_r = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_LED_R,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config_t led_timer_conf_g = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_LED_G,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config_t led_timer_conf_b = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_LED_B,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    ledc_channel_config_t led_channel_conf_r = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_LED_R,
        .timer_sel = TIMER_LED_R,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED_R,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };
    ledc_channel_config_t led_channel_conf_g = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_LED_G,
        .timer_sel = TIMER_LED_G,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED_G,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };
    ledc_channel_config_t led_channel_conf_b = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_LED_B,
        .timer_sel = TIMER_LED_B,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED_B,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };

    ledc_timer_config(&led_timer_conf_r);
    ledc_timer_config(&led_timer_conf_g);
    ledc_timer_config(&led_timer_conf_b);

    ledc_channel_config(&led_channel_conf_r);
    ledc_channel_config(&led_channel_conf_g);
    ledc_channel_config(&led_channel_conf_b);

    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_R, 0);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_G, 0);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_B, 255);
    vTaskDelay(100);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_R, 0);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_G, 255);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_B, 0);

    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_R);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_G);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_B);
}
