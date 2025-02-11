#include "Common-define.h"

#define ESC1_pin 5
#define ESC2_pin 17
#define ESC3_pin 16
#define ESC4_pin 2
#define MAX_PWM (819)
#define MIN_PWM (410)

static const char *CONTROL_TAG = "control_task";


void configureESC() {
    ledc_timer_config_t ledc_timer0_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer0_config));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ESC1_pin,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = ESC2_pin;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel.gpio_num = ESC3_pin;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL_3;
    ledc_channel.gpio_num = ESC4_pin;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, MIN_PWM);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, MIN_PWM);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, MIN_PWM);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, MIN_PWM);

    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);

    ESP_LOGI(CONTROL_TAG, "ESC configured");
}

void MotorControlTask(void *parameter) {
    ESC_out_t esc_data;

    while (1) {
        xQueueReceive(ESC_data_queue, &esc_data, portMAX_DELAY);

        if (esc_data.ESC1_out > 2500 || esc_data.ESC2_out > 2500 || esc_data.ESC3_out > 2500 || esc_data.ESC4_out > 2500) {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, MIN_PWM);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, MIN_PWM);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, MIN_PWM);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, MIN_PWM);
            continue;
        }

        // Set duty cycle for each ESC
        for (int i = 0; i < 4; i++) {
            uint16_t esc_output = (i == 0) ? esc_data.ESC1_out : (i == 1) ? esc_data.ESC2_out : (i == 2) ? esc_data.ESC3_out : esc_data.ESC4_out;
            if (esc_output < 1000) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, i, MIN_PWM);
            } else if (esc_output > 2000) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, i, MAX_PWM);
            } else {
                float duty = MIN_PWM + ((double)MAX_PWM - (double)MIN_PWM) * ((esc_output - 1000) / 1000.0);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, i, (uint32_t)duty);
            }
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, i);
        }
    }
}