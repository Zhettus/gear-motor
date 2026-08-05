#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"


#define PIN_INA        GPIO_NUM_26     // ZK-BM1 IN1  (PWM)
#define PIN_INB        GPIO_NUM_27     // ZK-BM1 IN2  (PWM)
#define PIN_ENC_A      GPIO_NUM_32     // encoder channel A
#define PIN_ENC_B      GPIO_NUM_33     // encoder channel B

#define PWM_FREQ_HZ    1000        //pwm > 2000 for zk-bm1
#define PWM_RES         LEDC_TIMER_10_BIT
#define PWM_MAX        1023
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_INA     LEDC_CHANNEL_0   // drives PIN_INA
#define LEDC_CH_INB     LEDC_CHANNEL_1   // drives PIN_INB

//Encoder

#define ENCODER_CPT           16
#define COUNTS_PER_MOTOR_REV  (4 * ENCODER_CPT)
#define GEAR_RATIO            131.0f   // motor rev per output rev

#define MOTOR_RPM_AT_MAX      9000.0f


#define CONTROL_DT_MS         20
#define KP                    0.03f
#define KI                    0.10f
#define KD                    0.0f
#define INTEG_LIMIT           ((float)PWM_MAX)
static volatile float g_target_output_rpm = 0.0f;
static volatile bool  g_drive_enabled     = false;
static pcnt_unit_handle_t g_pcnt = NULL;

static void zk_output(float signed_duty)
{
    float mag = fabsf(signed_duty);
    if (mag > PWM_MAX) mag = PWM_MAX;
    uint32_t duty = (uint32_t)(mag + 0.5f);

    if (signed_duty >= 0.0f) {
        // forward: PWM on IN1, IN2 held low
        // pull the opposite input low FIRST so the two are never both high
        ledc_set_duty(LEDC_MODE, LEDC_CH_INB, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CH_INB);
        ledc_set_duty(LEDC_MODE, LEDC_CH_INA, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CH_INA);
    } else {
        // reverse: PWM on IN2, IN1 held low
        ledc_set_duty(LEDC_MODE, LEDC_CH_INA, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CH_INA);
        ledc_set_duty(LEDC_MODE, LEDC_CH_INB, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CH_INB);
    }
}

static void zk_coast(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CH_INA, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CH_INA);
    ledc_set_duty(LEDC_MODE, LEDC_CH_INB, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CH_INB);
}

static void control_task(void *arg)
{
    const float dt = CONTROL_DT_MS / 1000.0f;
    float integral = 0.0f;
    float prev_err = 0.0f;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_DT_MS));

        int count = 0;
        pcnt_unit_get_count(g_pcnt, &count);
        pcnt_unit_clear_count(g_pcnt);

        float meas_motor_rpm =
            ((float)count / (float)COUNTS_PER_MOTOR_REV) / dt * 60.0f;

        if (!g_drive_enabled) {
            integral = 0.0f;
            prev_err = 0.0f;
            zk_coast();
            continue;
        }

        float target_motor_rpm = g_target_output_rpm * GEAR_RATIO;
        float err = target_motor_rpm - meas_motor_rpm;

        float ff = (target_motor_rpm / MOTOR_RPM_AT_MAX) * PWM_MAX;

        integral += KI * err * dt;
        if (integral >  INTEG_LIMIT) integral =  INTEG_LIMIT;
        if (integral < -INTEG_LIMIT) integral = -INTEG_LIMIT;

        float deriv = (err - prev_err) / dt;
        prev_err = err;

        float out = ff + KP * err + integral + KD * deriv;
        if (out >  PWM_MAX) out =  PWM_MAX;
        if (out < -PWM_MAX) out = -PWM_MAX;

        zk_output(out);
    }
}

static void stopMotor(void)
{
    g_target_output_rpm = 0.0f;
    g_drive_enabled = false;
}

static void moveForward(float seconds, float outputRpm)
{
    g_target_output_rpm = +outputRpm;
    g_drive_enabled = true;
    vTaskDelay(pdMS_TO_TICKS((uint32_t)(seconds * 1000.0f)));
    stopMotor();
}

static void moveBackward(float seconds, float outputRpm)
{
    g_target_output_rpm = -outputRpm;
    g_drive_enabled = true;
    vTaskDelay(pdMS_TO_TICKS((uint32_t)(seconds * 1000.0f)));
    stopMotor();
}

static void drive_init(void)
{
    // PWM timer shared by both input channels
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);

    // IN1 channel
    ledc_channel_config_t ccfg_a = {
        .gpio_num   = PIN_INA,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CH_INA,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&ccfg_a);

    // IN2 channel
    ledc_channel_config_t ccfg_b = {
        .gpio_num   = PIN_INB,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CH_INB,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&ccfg_b);

    zk_coast();

    // PCNT quadrature decoder (x4)
    pcnt_unit_config_t ucfg = {
        .high_limit =  30000,
        .low_limit  = -30000,
    };
    pcnt_new_unit(&ucfg, &g_pcnt);

    pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(g_pcnt, &fcfg);

    // Channel A: count edges of A, direction from level of B
    pcnt_chan_config_t ca = {
        .edge_gpio_num  = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B,
    };
    pcnt_channel_handle_t ch_a = NULL;
    pcnt_new_channel(g_pcnt, &ca, &ch_a);
    pcnt_channel_set_edge_action(ch_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ch_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // Channel B: count edges of B, direction from level of A
    pcnt_chan_config_t cb = {
        .edge_gpio_num  = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A,
    };
    pcnt_channel_handle_t ch_b = NULL;
    pcnt_new_channel(g_pcnt, &cb, &ch_b);
    pcnt_channel_set_edge_action(ch_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ch_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(g_pcnt);
    pcnt_unit_clear_count(g_pcnt);
    pcnt_unit_start(g_pcnt);

    xTaskCreate(control_task, "motor_ctl", 4096, NULL, 5, NULL);
}

void app_main(void)
{
    drive_init();
    moveForward(1.3f, 10.0f); //по часовой
/*
    for (;;) {
        moveForward(5.0f, 30.0f);
        vTaskDelay(pdMS_TO_TICKS(500));
        moveBackward(5.0f, 30.0f);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    */
}