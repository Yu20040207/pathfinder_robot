#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_direction.h"
#include "motor_ramp.h"
#include "quadrature.h"

static const char *TAG = "fwr_motor_pulse";
static const int PULSE_MS = 5000;
static const int DC_TEST_MS = 500;
static const uint32_t DUTY = 256;  // 25% of a 10-bit PWM period.
static const uint32_t FULL_DUTY = 1023;
static const gpio_num_t encoder_a[2] = {GPIO_NUM_8, GPIO_NUM_10};
static const gpio_num_t encoder_b[2] = {GPIO_NUM_9, GPIO_NUM_11};
static portMUX_TYPE encoder_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t encoder_count[2];
static volatile uint32_t encoder_skipped[2];
static volatile uint8_t encoder_previous[2];

static uint8_t read_encoder_state(unsigned motor)
{
    return (uint8_t)((gpio_get_level(encoder_a[motor]) << 1) |
                     gpio_get_level(encoder_b[motor]));
}

static void encoder_edge(void *argument)
{
    const unsigned motor = (unsigned)(uintptr_t)argument;
    const uint8_t current = read_encoder_state(motor);
    portENTER_CRITICAL_ISR(&encoder_lock);
    const uint8_t previous = encoder_previous[motor];
    const int8_t step = quadrature_step(previous, current);
    if (previous != current && step == 0) {
        ++encoder_skipped[motor];
    }
    encoder_count[motor] += step;
    encoder_previous[motor] = current;
    portEXIT_CRITICAL_ISR(&encoder_lock);
}

static void report_encoders(void)
{
    int32_t a, b;
    uint32_t a_skipped, b_skipped;
    portENTER_CRITICAL(&encoder_lock);
    a = encoder_count[0];
    b = encoder_count[1];
    a_skipped = encoder_skipped[0];
    b_skipped = encoder_skipped[1];
    portEXIT_CRITICAL(&encoder_lock);
    ESP_LOGI(TAG, "encoder A=%ld skipped=%lu B=%ld skipped=%lu (4x edges, uncalibrated)",
             (long)a, (unsigned long)a_skipped, (long)b, (unsigned long)b_skipped);
}

static void stop_both(void)
{
    for (int channel = LEDC_CHANNEL_0; channel <= LEDC_CHANNEL_3; ++channel) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
    }
}

static void run_forward_ramp(uint32_t total_steps, bool start_boost)
{
    const ledc_channel_t left = (ledc_channel_t)motor_channel(MOTOR_LEFT, MOTOR_FORWARD);
    const ledc_channel_t right = (ledc_channel_t)motor_channel(MOTOR_RIGHT, MOTOR_FORWARD);
    stop_both();
    ESP_LOGI(TAG, "Forward ramp: both wheels, %lu s, start boost=%s; 'x' aborts.",
             (unsigned long)(total_steps / 10U), start_boost ? "yes" : "no");
    for (uint32_t step = 0; step <= total_steps; ++step) {
        const int incoming = getchar();
        if (incoming == 'x' || incoming == 'X') {
            ESP_LOGW(TAG, "Forward ramp aborted by serial command");
            break;
        }
        const uint32_t duty = start_boost ? motor_start_ramp_duty(step)
                                          : motor_ramp_duty(step, total_steps);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, left, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, left));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, right, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, right));
        if (step < total_steps) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    stop_both();
    ESP_LOGI(TAG, "STOP after forward ramp");
}

static void run_both_steady(void)
{
    const ledc_channel_t left = (ledc_channel_t)motor_channel(MOTOR_LEFT, MOTOR_FORWARD);
    const ledc_channel_t right = (ledc_channel_t)motor_channel(MOTOR_RIGHT, MOTOR_FORWARD);
    stop_both();
    ESP_LOGI(TAG, "Both-wheel diagnostic: 50%% PWM for 1 s; 'x' aborts.");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, left, 512));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, left));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, right, 512));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, right));
    for (int tick = 0; tick < 10; ++tick) {
        vTaskDelay(pdMS_TO_TICKS(100));
        const int incoming = getchar();
        if (incoming == 'x' || incoming == 'X') {
            ESP_LOGW(TAG, "Both-wheel diagnostic aborted by serial command");
            break;
        }
    }
    stop_both();
    ESP_LOGI(TAG, "STOP after both-wheel diagnostic");
}

void app_main(void)
{
    const gpio_num_t pins[] = {GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7};
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        ESP_ERROR_CHECK(gpio_reset_pin(pins[i]));
        ESP_ERROR_CHECK(gpio_set_level(pins[i], 0));
        ESP_ERROR_CHECK(gpio_set_direction(pins[i], GPIO_MODE_OUTPUT));
    }

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = GPIO_NUM_4,  // AIN1
        .duty = 0,
        .hpoint = 0,
    };
    const gpio_num_t drive_pins[] = {GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7};
    for (int index = 0; index < 4; ++index) {
        channel.channel = (ledc_channel_t)index;
        channel.gpio_num = drive_pins[index];
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
    stop_both();

    const gpio_config_t encoder_input = {
        .pin_bit_mask = (1ULL << GPIO_NUM_8) | (1ULL << GPIO_NUM_9) |
                        (1ULL << GPIO_NUM_10) | (1ULL << GPIO_NUM_11),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&encoder_input));
    for (unsigned motor = 0; motor < 2; ++motor) {
        encoder_previous[motor] = read_encoder_state(motor);
    }
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (unsigned motor = 0; motor < 2; ++motor) {
        ESP_ERROR_CHECK(gpio_isr_handler_add(encoder_a[motor], encoder_edge, (void *)(uintptr_t)motor));
        ESP_ERROR_CHECK(gpio_isr_handler_add(encoder_b[motor], encoder_edge, (void *)(uintptr_t)motor));
    }

    setvbuf(stdin, NULL, _IONBF, 0);
    ESP_LOGI(TAG, "Outputs idle. 'a'/'b': 5 s 25%%; '1'/'2': AIN1 50/75%%; '3'/'4': BIN1 50/75%%; '5': left forward 50%%; '6': right reverse 50%%; 't': 4 s ramp; 'T': 15 s ramp; 'V': 15 s with start boost; 'u': both 50%% 1 s; 'x': stop.");
    ESP_LOGI(TAG, "Keep wheels suspended. Disconnect motor power immediately if anything unexpected happens.");
    ESP_LOGI(TAG, "Encoder inputs GPIO8/9/10/11 require <=3.3 V level shifting. 'r': report; 'z': zero.");
    while (true) {
        const int command = getchar();
        if (command == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (command == 'x' || command == 'X') {
            stop_both();
            ESP_LOGI(TAG, "STOP");
            continue;
        }
        if (command == 't' || command == 'T' || command == 'V') {
            run_forward_ramp(command == 't' ? 40U : 150U, command == 'V');
            continue;
        }
        if (command == 'u') {
            run_both_steady();
            continue;
        }
        if (command == 'r') {
            report_encoders();
            continue;
        }
        if (command == 'z') {
            portENTER_CRITICAL(&encoder_lock);
            encoder_count[0] = 0;
            encoder_count[1] = 0;
            encoder_skipped[0] = 0;
            encoder_skipped[1] = 0;
            portEXIT_CRITICAL(&encoder_lock);
            ESP_LOGI(TAG, "Encoder counts zeroed");
            continue;
        }
        if (command >= '1' && command <= '6') {
            const bool motor_a = command == '1' || command == '2' || command == '5';
            const ledc_channel_t selected = command == '5'
                ? (ledc_channel_t)motor_channel(MOTOR_LEFT, MOTOR_FORWARD)
                : command == '6'
                    ? (ledc_channel_t)motor_channel(MOTOR_RIGHT, MOTOR_REVERSE)
                    : command <= '2' ? LEDC_CHANNEL_0 : LEDC_CHANNEL_2;
            const uint32_t duty = (command == '2' || command == '4') ? 768 : 512;
            stop_both();
            ESP_LOGI(TAG, "PWM diagnostic Motor_%c channel=%d: 500 ms, %u/1024 duty", motor_a ? 'A' : 'B', (int)selected, (unsigned)duty);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, selected, duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, selected));
            vTaskDelay(pdMS_TO_TICKS(DC_TEST_MS));
            stop_both();
            ESP_LOGI(TAG, "STOP after %d ms PWM diagnostic", DC_TEST_MS);
            report_encoders();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (command == 'c' || command == 'd') {
            const ledc_channel_t selected = command == 'c' ? LEDC_CHANNEL_0 : LEDC_CHANNEL_2;
            stop_both();
            ESP_LOGI(TAG, "DC diagnostic Motor_%c: 500 ms full duty", command == 'c' ? 'A' : 'B');
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, selected, FULL_DUTY));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, selected));
            vTaskDelay(pdMS_TO_TICKS(DC_TEST_MS));
            stop_both();
            ESP_LOGI(TAG, "STOP after %d ms DC diagnostic", DC_TEST_MS);
            report_encoders();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (command != 'a' && command != 'b') {
            continue;
        }
        const ledc_channel_t selected = command == 'a' ? LEDC_CHANNEL_0 : LEDC_CHANNEL_2;
        stop_both();
        ESP_LOGI(TAG, "Pulse Motor_%c", command == 'a' ? 'A' : 'B');
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, selected, DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, selected));
        vTaskDelay(pdMS_TO_TICKS(PULSE_MS));
        stop_both();
        ESP_LOGI(TAG, "STOP after %d ms", PULSE_MS);
        report_encoders();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
