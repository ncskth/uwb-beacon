#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <stdint.h>
#include <esp_event.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#include "wifi.h"
#include "uwb.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "hardware.h"
#include "pt.h"
#include "uwb_definitions.h"

#define VBAT_K ((470 + 1200) / (470))
#define VBAT_LP_GAIN 0.9

#define GREEN 0,255,0
#define BLUE 0,0,255
#define RED 255,0,0
#define YELLOW 255,40,0
#define PINK 255, 20, 20

bool low_power = false;

uint8_t purpose;
uint8_t node_id;
int32_t pos_x;
int32_t pos_y;
int32_t pos_z;
uint8_t positioning_system_status;

void init_rgb();
void set_led(uint8_t r, uint8_t g, uint8_t b);

void app_main() {
    printf("version 4\n");
    esp_event_loop_create_default();

    //init ISR
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    //init flash
    esp_err_t err;
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    size_t len = 1;
    nvs_handle_t nvs;
    err = nvs_open("storage", NVS_READWRITE, &nvs);
    err = nvs_get_blob(nvs, "node_id", &node_id, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        node_id = 255;
        nvs_set_blob(nvs, "node_id", &node_id, 1);
    }
    len = 4;
    nvs_get_blob(nvs, "pos_x", &pos_x, &len);
    nvs_get_blob(nvs, "pos_y", &pos_y, &len);
    nvs_get_blob(nvs, "pos_z", &pos_z, &len);
    len = 1;
    nvs_get_blob(nvs, "purpose", &purpose, &len);
    nvs_get_blob(nvs, "system_status", &positioning_system_status, &len);

    gpio_set_direction(PIN_BUTTON_SENSE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON_SENSE, GPIO_PULLUP_ONLY);
    gpio_set_level(PIN_VBAT_SENSE_GND, 0);
    gpio_set_direction(PIN_VBAT_SENSE_GND, GPIO_MODE_INPUT_OUTPUT);

    init_rgb();
    set_led(0, 0, 255);
    init_wifi();
    init_uwb();

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, &adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("eFuse Vref\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Two Point\n");
    } else {
        printf("Default\n");
    }
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(CHANNEL_BAT_SENSE, ADC_ATTEN_11db);
    set_led(0, 255, 0);

    float vbat = 4.0;
    while (true) {
        vTaskDelay(100);
        uint16_t raw_adc = adc1_get_raw(CHANNEL_BAT_SENSE);
        float new_vbat = esp_adc_cal_raw_to_voltage(raw_adc, &adc_chars) / 1000.0 * VBAT_K;
        vbat = new_vbat * (1 - VBAT_LP_GAIN) + vbat * VBAT_LP_GAIN;

        if (vbat < 3.4) {
            set_led(RED);
        } else 
        if (positioning_system_status == UWB_SYSTEM_STATUS_GOOD) {
            set_led(GREEN);
        } else
        if (positioning_system_status == UWB_SYSTEM_STATUS_CALIBRATING) {
            set_led(YELLOW);
        } else
        if (positioning_system_status == UWB_SYSTEM_STATUS_ERROR) {
            set_led(PINK);
        }
    }
}

void set_led(uint8_t r, uint8_t g, uint8_t b) {
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_R, r / 4);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_G, g / 4);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_B, b / 4);

    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_R);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_G);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_B);   
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

    set_led(0,0,0);
}