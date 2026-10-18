#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "control.h"
#include "protocol.h"
#include "motor_direction.h"
#include "quadrature.h"

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static base_control_t controller;
static volatile int32_t counts[2];
static volatile uint32_t skipped[2];
static volatile uint8_t previous[2];
static const gpio_num_t ea[2] = {GPIO_NUM_8, GPIO_NUM_10};
static const gpio_num_t eb[2] = {GPIO_NUM_9, GPIO_NUM_11};
static const gpio_num_t motor_pins[4] = {GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7};

static uint8_t encoder_state(unsigned motor) {
    return (gpio_get_level(ea[motor])<<1) | gpio_get_level(eb[motor]);
}
static void edge(void *arg) {
    unsigned motor = (unsigned)(uintptr_t)arg;
    portENTER_CRITICAL_ISR(&lock);
    uint8_t current = encoder_state(motor);
    int8_t delta = quadrature_step(previous[motor], current);
    if (previous[motor]!=current && delta==0) ++skipped[motor];
    counts[motor] = (int32_t)((uint32_t)counts[motor]+(uint32_t)(int32_t)delta);
    previous[motor] = current;
    portEXIT_CRITICAL_ISR(&lock);
}
static void pwm(int channel, float duty) {
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, (uint32_t)lroundf(duty*1024)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}
static void drive(unsigned motor, float duty) {
    int forward = motor_channel((motor_side_t)motor, MOTOR_FORWARD);
    int reverse = motor_channel((motor_side_t)motor, MOTOR_REVERSE);
    if (duty>0) { pwm(reverse, 0); pwm(forward, duty); }
    else { pwm(forward, 0); pwm(reverse, -duty); }
}
static void control_task(void *arg) {
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        float out[2];
        portENTER_CRITICAL(&lock);
        base_tick(&controller, esp_timer_get_time(), counts[0], counts[1], skipped[0], skipped[1]);
        out[0] = controller.pwm[0]; out[1] = controller.pwm[1];
        portEXIT_CRITICAL(&lock);
        drive(0, out[0]); drive(1, out[1]);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
    }
}
static void stop(void) {
    portENTER_CRITICAL(&lock);
    base_stop(&controller);
    portEXIT_CRITICAL(&lock);
}
static void report(bool legacy) {
    base_control_t snapshot;
    portENTER_CRITICAL(&lock);
    snapshot = controller;
    portEXIT_CRITICAL(&lock);
    if (legacy) {
        ESP_LOGI("fwr_motor_pulse", "encoder A=%ld skipped=%lu B=%ld skipped=%lu (4x edges, calibrated)",
            (long)snapshot.counts[0], (unsigned long)snapshot.skipped[0],
            (long)snapshot.counts[1], (unsigned long)snapshot.skipped[1]);
    } else {
        printf("S %lld %ld %ld %.2f %.2f %.2f %.2f %.4f %.4f %d %d\n",
            (long long)esp_timer_get_time(), (long)snapshot.counts[0], (long)snapshot.counts[1],
            (double)snapshot.measured[0], (double)snapshot.measured[1],
            (double)snapshot.setpoint[0], (double)snapshot.setpoint[1],
            (double)snapshot.pwm[0], (double)snapshot.pwm[1], snapshot.active, snapshot.fault);
        printf("S2 %lld %ld %ld %lu %lu %.2f %.2f %.2f %.2f %.4f %.4f %d %d\n",
            (long long)esp_timer_get_time(), (long)snapshot.counts[0], (long)snapshot.counts[1],
            (unsigned long)snapshot.skipped[0], (unsigned long)snapshot.skipped[1],
            (double)snapshot.measured[0], (double)snapshot.measured[1],
            (double)snapshot.setpoint[0], (double)snapshot.setpoint[1],
            (double)snapshot.pwm[0], (double)snapshot.pwm[1], snapshot.active, snapshot.fault);
    }
}
void app_main(void) {
    for (int i=0; i<4; ++i) {
        ESP_ERROR_CHECK(gpio_reset_pin(motor_pins[i]));
        ESP_ERROR_CHECK(gpio_set_level(motor_pins[i], 0));
        ESP_ERROR_CHECK(gpio_set_direction(motor_pins[i], GPIO_MODE_OUTPUT));
    }
    ledc_timer_config_t timer = {
        .speed_mode=LEDC_LOW_SPEED_MODE, .duty_resolution=LEDC_TIMER_10_BIT,
        .timer_num=LEDC_TIMER_0, .freq_hz=10000, .clk_cfg=LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    for (int i=0; i<4; ++i) {
        ledc_channel_config_t config = {
            .gpio_num=motor_pins[i], .speed_mode=LEDC_LOW_SPEED_MODE,
            .channel=(ledc_channel_t)i, .intr_type=LEDC_INTR_DISABLE,
            .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&config));
    }
    gpio_config_t inputs = {
        .pin_bit_mask=(1ULL<<8)|(1ULL<<9)|(1ULL<<10)|(1ULL<<11),
        .mode=GPIO_MODE_INPUT, .pull_up_en=GPIO_PULLUP_DISABLE,
        .pull_down_en=GPIO_PULLDOWN_DISABLE, .intr_type=GPIO_INTR_ANYEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&inputs));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (unsigned i=0; i<2; ++i) {
        previous[i] = encoder_state(i);
        ESP_ERROR_CHECK(gpio_isr_handler_add(ea[i], edge, (void *)(uintptr_t)i));
        ESP_ERROR_CHECK(gpio_isr_handler_add(eb[i], edge, (void *)(uintptr_t)i));
    }
    base_init(&controller, esp_timer_get_time(), counts[0], counts[1]);
    configASSERT(xTaskCreate(control_task, "base_control", 4096, NULL, 10, NULL)==pdPASS);
    setvbuf(stdin, NULL, _IONBF, 0);
    puts("BASE_CONTROL_V2");
    puts("v L R (mm/s, <=100), d L R (forward duty <=0.65), b L R (loaded mm/s 150-200), a D (right-left mm/s <=50), y +/-1 (straight sync), t +/-1 (turn sync), x stop, c clear idle fault, r encoders, s state");
    char line[80];
    size_t used=0;
    bool discard=false;
    int64_t line_start=0, last_report=0;
    for (;;) {
        int64_t now = esp_timer_get_time();
        if (used && now-line_start>200000) { stop(); used=0; discard=true; }
        int ch = getchar();
        if (ch==EOF) {
            if (now-last_report>=100000) { report(false); last_report=now; }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (ch=='x' || ch=='X') { stop(); used=0; discard=false; continue; }
        if (discard) { if (ch=='\n') discard=false; continue; }
        if (!used && ch=='?') { puts("BASE_CONTROL_V2"); continue; }
        if (!used && (ch=='r' || ch=='s')) { report(ch=='r'); continue; }
        if (!used && ch=='c') {
            portENTER_CRITICAL(&lock);
            if (!controller.active) controller.fault=BASE_OK;
            portEXIT_CRITICAL(&lock);
            continue;
        }
        if (ch=='\r') continue;
        if (ch=='\n') {
            if (used) {
                line[used]='\0';
                float left, right;
                if (parse_turn(line, &left)) {
                    portENTER_CRITICAL(&lock);
                    base_turn_command(&controller, (int)left, esp_timer_get_time());
                    portEXIT_CRITICAL(&lock);
                } else if (parse_sync(line, &left)) {
                    portENTER_CRITICAL(&lock);
                    base_sync_command(&controller, (int)left, esp_timer_get_time());
                    portEXIT_CRITICAL(&lock);
                } else if (parse_arc(line, &left)) {
                    portENTER_CRITICAL(&lock);
                    base_arc_command(&controller, left, esp_timer_get_time());
                    portEXIT_CRITICAL(&lock);
                } else if (parse_loaded(line, &left, &right)) {
                    portENTER_CRITICAL(&lock);
                    base_loaded_command(&controller, left, right, esp_timer_get_time());
                    portEXIT_CRITICAL(&lock);
                } else if (parse_duty(line, &left, &right)) {
                    portENTER_CRITICAL(&lock);
                    base_duty_command(&controller, left, right, esp_timer_get_time());
                    portEXIT_CRITICAL(&lock);
                } else if (parse_velocity(line, &left, &right)) {
                    portENTER_CRITICAL(&lock);
                    base_command(&controller, left, right, esp_timer_get_time());
                    portEXIT_CRITICAL(&lock);
                } else stop();
                used=0;
            }
            continue;
        }
        if (used==0) line_start=now;
        if (used>=sizeof(line)-1) { stop(); used=0; discard=true; }
        else line[used++]=(char)ch;
    }
}
