#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>   
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/i2c.h"
#include "esp_timer.h" 
#include "esp_log.h"

#define PIN_INA        GPIO_NUM_26     // ZK-BM1 IN1 (PWM)
#define PIN_INB        GPIO_NUM_27     // ZK-BM1 IN2 (PWM)
#define PIN_ENC_A      GPIO_NUM_32     // Encoder channel A
#define PIN_ENC_B      GPIO_NUM_33     // Encoder channel B

#define PIN_I2C_SDA    GPIO_NUM_21     // INA219 SDA
#define PIN_I2C_SCL    GPIO_NUM_22     // INA219 SCL

//  INA219
#define I2C_PORT_NUM   I2C_NUM_0
#define INA219_ADDR    0x40
#define INA219_REG_SHUNTV 0x01
#define INA219_REG_BUSV  0x02

// motor and pwm params
#define PWM_FREQ_HZ    1000            // PWM < 2000 for ZK-BM1
#define PWM_RES         LEDC_TIMER_10_BIT
#define PWM_MAX        1023
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_INA     LEDC_CHANNEL_0
#define LEDC_CH_INB     LEDC_CHANNEL_1

// encoder and gearbox
#define ENCODER_CPT           16
#define COUNTS_PER_MOTOR_REV  (4 * ENCODER_CPT)          // 64 (4x quadrature)
#define GEAR_RATIO            131.0f                      // motor revs per output rev
#define MOTOR_RPM_AT_MAX      4500.0f
#define COUNTS_PER_OUTPUT_REV ((int64_t)COUNTS_PER_MOTOR_REV * 131)  // 8384

// pid and control
#define CONTROL_DT_MS         20
#define KP                    0.10f
#define KI                    0.50f
#define KD                    0.0f
#define INTEG_LIMIT           (PWM_MAX * 0.50f)
#define MIN_PWM               150.0f


#define DOOR_TRAVEL_REVS      3.0f
#define DOOR_TRAVEL_COUNTS    ((int64_t)(DOOR_TRAVEL_REVS * COUNTS_PER_OUTPUT_REV))

/* How close (in counts) is "arrived". ~1/20 of an output turn. */
#define POS_TOL_COUNTS        (COUNTS_PER_OUTPUT_REV / 20)

/* Speeds (output-shaft RPM): cruise, then slow down near the target. */
#define DOOR_MOVE_RPM         7.0f
#define APPROACH_RPM          3.0f
#define APPROACH_WINDOW_COUNTS (COUNTS_PER_OUTPUT_REV / 4)   // slow + suppress-trip zone

/* Obstacle detection. NOTE: your old CURRENT_MAX_VALUE 300.0 was not physical --
 * ina219_read_current_amps() returns AMPS, so 300 would mean 300 A. Set this to
 * a value between the motor's free-run current and its blocked/stall current.
 * CALIBRATE: watch the logged current while the door runs free vs. while you
 * hold it, and pick a threshold in between. */
#define OBSTACLE_CURRENT_A    1.5f
#define STARTUP_GRACE_MS      600      // ignore the current in-rush right after a start
#define STALL_RPM_THRESH      1.0f     // output ~motor rpm treated as "not moving"
#define STALL_THRESHOLD_LOOPS 25       // 25 * 20ms = 500ms of pushing-but-stopped

static const char *TAG = "DOOR";

typedef enum { DOOR_CLOSED = 0, DOOR_OPEN = 1 } door_state_t;
typedef enum { CMD_OPEN = 0, CMD_CLOSE = 1 }    door_cmd_t;

/* --- status published by control_task (read-only for everyone else) --- */
static volatile door_state_t g_state       = DOOR_CLOSED;  // where the door is
static volatile bool         g_moving      = false;        // a move is in progress
static volatile bool         g_last_blocked = false;       // last move ended in a retreat
static volatile float        g_current_a   = 0.0f;         // latest INA219 current (A)

/* --- internals --- */
static volatile float  g_target_output_rpm = 0.0f;   // (kept for compatibility / debug)
static volatile bool   g_drive_enabled     = false;  // (kept; unused by state machine)
static pcnt_unit_handle_t g_pcnt = NULL;
static QueueHandle_t   g_cmd_q = NULL;

/* Absolute position, owned exclusively by control_task. CLOSED == 0. */
static int64_t g_position = 0;

/* Counts that correspond to a given door state. */
static inline int64_t counts_of_state(door_state_t s)
{
    return (s == DOOR_OPEN) ? DOOR_TRAVEL_COUNTS : 0;
}

// ===================== INA219 I2C (legacy driver) =====================
static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_PORT_NUM, &conf);
    i2c_driver_install(I2C_PORT_NUM, conf.mode, 0, 0, 0);
}

static float ina219_read_current_amps(void)
{
    uint8_t reg = INA219_REG_SHUNTV;
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_master_write_read_device(I2C_PORT_NUM, INA219_ADDR,
                                                  &reg, 1, data, 2, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) return -999.0f;
    int16_t raw_shunt = (int16_t)((data[0] << 8) | data[1]);
    return (float)raw_shunt * 0.0001f;   // 100 uA/bit for 0.1 ohm shunt
}

static float ina219_read_bus_voltage(void)
{
    uint8_t reg = INA219_REG_BUSV;
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_master_write_read_device(I2C_PORT_NUM, INA219_ADDR,
                                                  &reg, 1, data, 2, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) return -999.0f;
    int16_t raw_bus = (int16_t)((data[0] << 8) | data[1]);
    return (float)(raw_bus >> 3) * 0.004f;   // bits [15:3], 4 mV LSB
}

/* Slow logging task: prints voltage + the latest current sampled by the
 * control loop. The FAST current sampling for obstacle detection happens in
 * control_task, not here (4 Hz would be far too slow to catch a pinch). */
static void ina219_task(void *arg)
{
    for (;;) {
        float voltage = ina219_read_bus_voltage();
        if (voltage < -100.0f) {
            ESP_LOGE(TAG, "INA219 read error (check I2C wiring SDA:21 SCL:22)");
        } else {
            ESP_LOGI(TAG, "Bus %.2f V | I %.3f A | pos %.2f rev | state %s%s",
                     voltage, g_current_a,
                     (double)g_position / (double)COUNTS_PER_OUTPUT_REV,
                     g_state == DOOR_OPEN ? "OPEN" : "CLOSED",
                     g_moving ? " (moving)" : "");
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ===================== motor output =====================
static void zk_output(float signed_duty)
{
    float mag = fabsf(signed_duty);
    if (mag > 0.1f && mag < MIN_PWM) mag = MIN_PWM;
    if (mag > PWM_MAX) mag = PWM_MAX;
    uint32_t duty = (uint32_t)(mag + 0.5f);

    if (signed_duty >= 0.0f) {          // OPEN direction (see sign note in header)
        ledc_set_duty(LEDC_MODE, LEDC_CH_INB, 0);   ledc_update_duty(LEDC_MODE, LEDC_CH_INB);
        ledc_set_duty(LEDC_MODE, LEDC_CH_INA, duty);ledc_update_duty(LEDC_MODE, LEDC_CH_INA);
    } else {                            // CLOSE direction
        ledc_set_duty(LEDC_MODE, LEDC_CH_INA, 0);   ledc_update_duty(LEDC_MODE, LEDC_CH_INA);
        ledc_set_duty(LEDC_MODE, LEDC_CH_INB, duty);ledc_update_duty(LEDC_MODE, LEDC_CH_INB);
    }
}

static void zk_coast(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CH_INA, 0); ledc_update_duty(LEDC_MODE, LEDC_CH_INA);
    ledc_set_duty(LEDC_MODE, LEDC_CH_INB, 0); ledc_update_duty(LEDC_MODE, LEDC_CH_INB);
}

// ===================== control + state machine =====================
static void control_task(void *arg)
{
    const float dt = CONTROL_DT_MS / 1000.0f;

    /* velocity-loop state */
    float integral = 0.0f, prev_err = 0.0f, last_out = 0.0f;

    /* move / state-machine state */
    bool         moving        = false;
    bool         retreating    = false;
    door_state_t final_state   = DOOR_CLOSED;   // state to adopt when target reached
    int64_t      target_counts = 0;
    int64_t      grace_until_us = 0;
    int          stall_counter = 0;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_DT_MS));

        // ---- measure: accumulate absolute position, compute RPM ----
        int delta = 0;
        pcnt_unit_get_count(g_pcnt, &delta);
        pcnt_unit_clear_count(g_pcnt);
        g_position += delta;
        float meas_motor_rpm = ((float)delta / (float)COUNTS_PER_MOTOR_REV) / dt * 60.0f;

        // ---- fast current sample for obstacle detection ----
        float cur = ina219_read_current_amps();
        if (cur > -100.0f) g_current_a = fabsf(cur);

        // ---- accept a new command only while idle ----
        if (!moving) {
            door_cmd_t cmd;
            if (xQueueReceive(g_cmd_q, &cmd, 0) == pdTRUE) {
                door_state_t want = (cmd == CMD_OPEN) ? DOOR_OPEN : DOOR_CLOSED;
                if (want != g_state) {
                    final_state    = want;
                    target_counts  = counts_of_state(want);
                    retreating     = false;
                    stall_counter  = 0;
                    integral       = 0.0f; prev_err = 0.0f;
                    grace_until_us = esp_timer_get_time() + (int64_t)STARTUP_GRACE_MS * 1000;
                    moving         = true;
                    g_moving       = true;
                    g_last_blocked = false;
                    ESP_LOGI(TAG, "MOVE -> %s (target %lld cnt)",
                             want == DOOR_OPEN ? "OPEN" : "CLOSED", (long long)target_counts);
                }
            }
        }

        if (!moving) { zk_coast(); continue; }

        int64_t err_counts = target_counts - g_position;
        bool    near_target = llabs(err_counts) < APPROACH_WINDOW_COUNTS;

        // ---- arrived? ----
        if (llabs(err_counts) <= POS_TOL_COUNTS) {
            zk_coast();
            moving   = false;  g_moving = false;
            g_state  = final_state;
            g_last_blocked = retreating;   // if this move was a retreat, flag it
            integral = 0.0f;   last_out = 0.0f;   stall_counter = 0;
            ESP_LOGI(TAG, "ARRIVED %s%s",
                     final_state == DOOR_OPEN ? "OPEN" : "CLOSED",
                     retreating ? " (retreated - path was blocked)" : "");
            continue;
        }

        // ---- obstacle detection (armed after grace, suppressed near the ends) ----
        int64_t now_us = esp_timer_get_time();
        bool armed   = (now_us > grace_until_us) && !near_target;
        bool overcur = armed && (g_current_a > OBSTACLE_CURRENT_A);

        bool pushing = fabsf(last_out) > (PWM_MAX * 0.2f);
        bool stopped = fabsf(meas_motor_rpm) < STALL_RPM_THRESH;
        if (armed && pushing && stopped) stall_counter++; else stall_counter = 0;
        bool stalled = stall_counter > STALL_THRESHOLD_LOOPS;

        if (overcur || stalled) {
            const char *why = overcur ? "OVERCURRENT" : "STALL";
            if (!retreating) {
                // interference on the way -> go back to the state we started from
                retreating     = true;
                final_state    = g_state;                 // g_state still holds the origin
                target_counts  = counts_of_state(final_state);
                grace_until_us = now_us + (int64_t)STARTUP_GRACE_MS * 1000;
                stall_counter  = 0; integral = 0.0f;
                ESP_LOGW(TAG, "OBSTACLE [%s] -> retreat to %s",
                         why, final_state == DOOR_OPEN ? "OPEN" : "CLOSED");
            } else {
                // blocked even while retreating -> stop hard, don't loop forever
                zk_coast();
                moving = false; g_moving = false; g_last_blocked = true;
                ESP_LOGE(TAG, "OBSTACLE [%s] during retreat -> ABORT, motor off", why);
                continue;
            }
            // recompute error for this same loop after flipping the target
            err_counts  = target_counts - g_position;
            near_target = llabs(err_counts) < APPROACH_WINDOW_COUNTS;
        }

        // ---- velocity setpoint toward target (slow down near the end) ----
        float dir   = (err_counts > 0) ? 1.0f : -1.0f;
        float speed = near_target ? APPROACH_RPM : DOOR_MOVE_RPM;
        float target_output_rpm = dir * speed;
        g_target_output_rpm = target_output_rpm;

        // ---- existing feed-forward + PI velocity loop (now signed) ----
        float target_motor_rpm = target_output_rpm * GEAR_RATIO;
        float err = target_motor_rpm - meas_motor_rpm;
        float ff  = (target_motor_rpm / MOTOR_RPM_AT_MAX) * PWM_MAX;

        integral += KI * err * dt;
        if (integral >  INTEG_LIMIT) integral =  INTEG_LIMIT;
        if (integral < -INTEG_LIMIT) integral = -INTEG_LIMIT;
        float deriv = (err - prev_err) / dt; prev_err = err;

        float out = ff + KP * err + integral + KD * deriv;
        if (out >  PWM_MAX) out =  PWM_MAX;
        if (out < -PWM_MAX) out = -PWM_MAX;

        last_out = out;
        zk_output(out);
    }
}

// ===================== command API (call from app_main) =====================
static void door_command(door_cmd_t c) { xQueueSend(g_cmd_q, &c, portMAX_DELAY); }

static void door_wait_idle(void)
{
    vTaskDelay(pdMS_TO_TICKS(60));            // let control_task pick up the command
    while (g_moving) vTaskDelay(pdMS_TO_TICKS(50));
}

// ===================== init =====================
static void drive_init(void)
{
    i2c_init();

    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER,
        .duty_resolution = PWM_RES, .freq_hz = PWM_FREQ_HZ, .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg_a = {
        .gpio_num = PIN_INA, .speed_mode = LEDC_MODE, .channel = LEDC_CH_INA,
        .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0, .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&ccfg_a);
    ledc_channel_config_t ccfg_b = ccfg_a;
    ccfg_b.gpio_num = PIN_INB; ccfg_b.channel = LEDC_CH_INB;
    ledc_channel_config(&ccfg_b);
    zk_coast();

    pcnt_unit_config_t ucfg = { .high_limit = 30000, .low_limit = -30000 };
    pcnt_new_unit(&ucfg, &g_pcnt);
    pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(g_pcnt, &fcfg);

    pcnt_chan_config_t ca = { .edge_gpio_num = PIN_ENC_A, .level_gpio_num = PIN_ENC_B };
    pcnt_channel_handle_t ch_a = NULL;
    pcnt_new_channel(g_pcnt, &ca, &ch_a);
    pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_chan_config_t cb = { .edge_gpio_num = PIN_ENC_B, .level_gpio_num = PIN_ENC_A };
    pcnt_channel_handle_t ch_b = NULL;
    pcnt_new_channel(g_pcnt, &cb, &ch_b);
    pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(g_pcnt);
    pcnt_unit_clear_count(g_pcnt);
    pcnt_unit_start(g_pcnt);

    g_cmd_q = xQueueCreate(4, sizeof(door_cmd_t));

    /* HOMING ASSUMPTION: the door is assumed fully CLOSED at power-up, so
     * position 0 == closed. If you can't guarantee that, add a homing routine:
     * drive CLOSE at low speed until a stall (RPM ~ 0 while pushing), then set
     * g_position = 0 and g_state = DOOR_CLOSED before accepting commands. */
    g_position = 0;
    g_state    = DOOR_CLOSED;

    xTaskCreate(control_task, "motor_ctl",  4096, NULL, 5, NULL);
    xTaskCreate(ina219_task,  "ina219_mon", 3072, NULL, 3, NULL);
}

void app_main(void)
{
    drive_init();

    for (;;) {
        /*
        door_command(CMD_OPEN);
        door_wait_idle();
        ESP_LOGI(TAG, "open cycle done (blocked=%d)", g_last_blocked);
        vTaskDelay(pdMS_TO_TICKS(3000));

        door_command(CMD_CLOSE);
        door_wait_idle();
        ESP_LOGI(TAG, "close cycle done (blocked=%d)", g_last_blocked);
        vTaskDelay(pdMS_TO_TICKS(5000));*/
        ESP_LOGI(TAG, "open-loop test: forward 60%%");
        zk_output(600);                       // ~60% duty, straight to the driver
        vTaskDelay(pdMS_TO_TICKS(2000));
        zk_output(0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}