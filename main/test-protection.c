#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

#define PIN_MOTOR_IN1        GPIO_NUM_26
#define PIN_MOTOR_IN2        GPIO_NUM_27
#define PIN_ENC_A            GPIO_NUM_32
#define PIN_ENC_B            GPIO_NUM_33
#define PIN_I2C_SDA          GPIO_NUM_21
#define PIN_I2C_SCL          GPIO_NUM_22

#define MOTOR_MAX_RPM        4500
#define GEAR_RATIO           131
#define ENCODER_BASE_CPT     16    
#define QUAD_DECODE_MULT     4      

#define COUNTS_PER_MOTOR_REV   ((int64_t)ENCODER_BASE_CPT * QUAD_DECODE_MULT)
#define COUNTS_PER_OUTPUT_REV  (COUNTS_PER_MOTOR_REV * GEAR_RATIO)   /* = 8384 */

#define DOOR_OPEN_RPM        20.0f
#define DOOR_CLOSE_RPM       15.0f
#define DOOR_MOVE_TIME_MS    4000

#define PWM_RESOLUTION       LEDC_TIMER_10_BIT     /* 0..1023                  */
#define PWM_DUTY_MAX         1023
#define PWM_FREQ_HZ          1000                  /* 1 kHz: the ZK-BM1 cannot *
                                                    * switch cleanly at 20kHz - *
                                                    * high freq => no torque    */
#define PWM_MIN_DUTY         250                   /* floor: don't stall light */
#define PWM_CEILING_DUTY     950                   /* leave regulation headroom*/
#define KICKSTART_DUTY       614                   /* ~60% break stiction      */
#define KICKSTART_MS         100
#define RETREAT_DUTY         500                   /* open-loop retreat power  */
#define BRAKE_PULSE_MS       60

#define OVERCURRENT_LIMIT_A  1.5f     /* absolute current trip                */
#define CURRENT_DELTA_A      0.8f     /* sharp dI over one poll interval      */
#define STALL_RPM_THRESH     1.0f     /* output RPM considered "not moving"   */
#define STALL_DUTY_THRESH    30.0f    /* only flag stall while pushing >30%   */
#define STALL_HOLD_MS        600      /* stall must persist this long         */
#define STARTUP_GRACE_MS     800      /* ignore safety during startup inertia */

#define MAX_RETRIES          3
#define RETREAT_MIN_MS       1500
#define RETREAT_PAUSE_MS     100
#define OBSTACLE_RETRY_DELAY 3000     /* cool-down after a retreat            */

#define CONTROL_LOOP_MS      20       /* 50 Hz control + stall loop           */
#define INA_POLL_MS          5        /* 200 Hz current polling               */
#define MOTOR_RPM_AT_MAX     4500.0f  /* for the feedforward duty estimate    */
#define CTRL_KP              5.0f     /* duty per output-RPM error            */
#define CTRL_KI              1.5f     /* duty per output-RPM-second           */
#define INTEG_LIMIT_DUTY     250.0f   /* anti-windup clamp on integral term   */

#define INA219_ADDR          0x40
#define INA219_I2C_HZ        100000   /* 100 kHz: forgiving of weak pull-ups  */
#define INA219_REG_CONFIG    0x00
#define INA219_REG_SHUNT     0x01
#define INA219_REG_BUS       0x02
#define INA219_REG_CALIB     0x05
#define INA219_CONFIG_32V_2A 0x399F   /* 32V bus, /8 gain (+-320mV), 12-bit    */
#define INA219_CALIB_VALUE   4096     /* for 0.1ohm, 100uA/bit current LSB     */
#define SHUNT_LSB_VOLTS      0.00001f /* shunt-voltage LSB = 10 uV             */
#define SHUNT_OHMS           0.1f

#define PCNT_HI_LIMIT        10000
#define PCNT_LO_LIMIT       (-10000)
#define PCNT_GLITCH_NS       1000

typedef enum {
    MOTOR_COAST = 0,
    MOTOR_FORWARD,      /* door OPEN  direction */
    MOTOR_REVERSE,      /* door CLOSE direction */
    MOTOR_BRAKE,
} motor_dir_t;

typedef struct {
    bool        motion_active;   /* control task owns motor when true         */
    motor_dir_t dir;             /* current commanded direction               */
    float       target_rpm;      /* output-shaft RPM setpoint                 */
    int         commanded_duty;  /* last duty written by control task         */
    float       measured_rpm;    /* latest output-shaft RPM                   */
    float       current_a;       /* latest INA219 current magnitude           */
    int64_t     grace_until_us;  /* safety armed only after this timestamp    */
} sys_state_t;


static const char *TAG      = "DOOR";
static const char *TAG_INA  = "INA219";
static const char *TAG_CTRL = "CTRL";

static i2c_master_bus_handle_t s_i2c_bus  = NULL;
static i2c_master_dev_handle_t s_ina_dev  = NULL;
static pcnt_unit_handle_t      s_pcnt      = NULL;

static SemaphoreHandle_t s_state_mtx = NULL;
static EventGroupHandle_t s_evt      = NULL;
static sys_state_t s_state;     

#define EVT_OBSTACLE_CURRENT   (1u << 0)
#define EVT_OBSTACLE_STALL     (1u << 1)
#define EVT_OBSTACLE_ANY       (EVT_OBSTACLE_CURRENT | EVT_OBSTACLE_STALL)


static inline void state_lock(void)   { xSemaphoreTake(s_state_mtx, portMAX_DELAY); }
static inline void state_unlock(void) { xSemaphoreGive(s_state_mtx); }

/* Publish an obstacle-free regulated-motion command. */
static void state_begin_motion(motor_dir_t dir, float target_rpm, int seed_duty,
                               int64_t grace_until_us)
{
    state_lock();
    s_state.motion_active  = true;
    s_state.dir            = dir;
    s_state.target_rpm     = target_rpm;
    s_state.commanded_duty = seed_duty;
    s_state.grace_until_us = grace_until_us;
    state_unlock();
}

static void state_end_motion(void)
{
    state_lock();
    s_state.motion_active = false;
    state_unlock();
}


#define LEDC_CH_IN1   LEDC_CHANNEL_0
#define LEDC_CH_IN2   LEDC_CHANNEL_1
#define LEDC_TIMER    LEDC_TIMER_0
#define LEDC_MODE     LEDC_LOW_SPEED_MODE

static void motor_write_raw(uint32_t d1, uint32_t d2)
{
    if (d1 > PWM_DUTY_MAX) d1 = PWM_DUTY_MAX;
    if (d2 > PWM_DUTY_MAX) d2 = PWM_DUTY_MAX;
    ledc_set_duty(LEDC_MODE, LEDC_CH_IN1, d1);
    ledc_set_duty(LEDC_MODE, LEDC_CH_IN2, d2);
    ledc_update_duty(LEDC_MODE, LEDC_CH_IN1);
    ledc_update_duty(LEDC_MODE, LEDC_CH_IN2);
}

static void motor_drive(motor_dir_t dir, uint32_t duty)
{
    switch (dir) {
        case MOTOR_FORWARD: motor_write_raw(duty, 0);                 break;
        case MOTOR_REVERSE: motor_write_raw(0, duty);                 break;
        case MOTOR_BRAKE:   motor_write_raw(PWM_DUTY_MAX, PWM_DUTY_MAX); break;
        case MOTOR_COAST:
        default:            motor_write_raw(0, 0);                    break;
    }
}

static void motor_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t ch1 = {
        .gpio_num   = PIN_MOTOR_IN1,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CH_IN1,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config_t ch2 = ch1;
    ch2.gpio_num = PIN_MOTOR_IN2;
    ch2.channel  = LEDC_CH_IN2;

    ESP_ERROR_CHECK(ledc_channel_config(&ch1));
    ESP_ERROR_CHECK(ledc_channel_config(&ch2));

    motor_drive(MOTOR_COAST, 0);
    ESP_LOGI(TAG, "Motor / LEDC ready (%.1f kHz, %d-bit)",
             PWM_FREQ_HZ / 1000.0, 10);
}


static esp_err_t ina219_write_reg(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(s_ina_dev, buf, sizeof(buf), pdMS_TO_TICKS(50));
}

static esp_err_t ina219_read_reg(uint8_t reg, uint16_t *out)
{
    uint8_t rx[2] = { 0 };
    esp_err_t err = i2c_master_transmit_receive(s_ina_dev, &reg, 1,
                                                rx, sizeof(rx),
                                                pdMS_TO_TICKS(50));
    if (err == ESP_OK) {
        *out = ((uint16_t)rx[0] << 8) | rx[1];
    }
    return err;
}

static esp_err_t ina219_read_current(float *amps)
{
    uint16_t raw;
    esp_err_t err = ina219_read_reg(INA219_REG_SHUNT, &raw);
    if (err != ESP_OK) {
        return err;
    }
    int16_t signed_raw = (int16_t)raw;     
    float   v_shunt    = signed_raw * SHUNT_LSB_VOLTS;
    *amps = v_shunt / SHUNT_OHMS;
    return ESP_OK;
}

static esp_err_t ina219_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = PIN_I2C_SCL,
        .sda_io_num                   = PIN_I2C_SDA,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = INA219_ADDR,
        .scl_speed_hz    = INA219_I2C_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_ina_dev));

    /* Probe before configuring so a wiring fault is reported clearly. */
    esp_err_t err = i2c_master_probe(s_i2c_bus, INA219_ADDR, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INA, "No response at 0x%02X (%s).", INA219_ADDR,
                 esp_err_to_name(err));
        ESP_LOGE(TAG_INA, "Check: 4.7k pull-ups on SDA/SCL, 3V3 power, "
                          "common GND, SDA=GPIO%d/SCL=GPIO%d not swapped, "
                          "addr straps (A0/A1=GND for 0x40).",
                 PIN_I2C_SDA, PIN_I2C_SCL);
        return err;
    }

    /* All bus traffic below returns its error instead of aborting, so a
     * flaky sensor halts the controller cleanly rather than panic-looping. */
    err = ina219_write_reg(INA219_REG_CALIB, INA219_CALIB_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INA, "calibration write failed: %s", esp_err_to_name(err));
        return err;
    }
    err = ina219_write_reg(INA219_REG_CONFIG, INA219_CONFIG_32V_2A);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INA, "config write failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Read-back sanity check on the config register. */
    uint16_t cfg = 0;
    err = ina219_read_reg(INA219_REG_CONFIG, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INA, "config read-back failed: %s", esp_err_to_name(err));
        return err;
    }
    if (cfg != INA219_CONFIG_32V_2A) {
        ESP_LOGW(TAG_INA, "Config read-back mismatch: wrote 0x%04X got 0x%04X",
                 INA219_CONFIG_32V_2A, cfg);
    }
    ESP_LOGI(TAG_INA, "INA219 ready at 0x%02X", INA219_ADDR);
    return ESP_OK;
}

/* ==========================================================================
 *  ENCODER (PCNT quadrature, 4x, read-and-clear per loop)
 * ========================================================================== */

static void encoder_init(void)
{
    /* The counter is read-and-cleared every control loop, so per-loop deltas
     * stay tiny and never approach the limits — no accumulation needed. This
     * avoids any dependency on accum_count and can't silently saturate.       */
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HI_LIMIT,
        .low_limit  = PCNT_LO_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_pcnt));

    pcnt_glitch_filter_config_t gf = { .max_glitch_ns = PCNT_GLITCH_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt, &gf));

    /* Channel A: edge on A, level from B. */
    pcnt_chan_config_t ch_a_cfg = {
        .edge_gpio_num  = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B,
    };
    pcnt_channel_handle_t ch_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt, &ch_a_cfg, &ch_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_a,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Channel B: edge on B, level from A (gives 4x decoding). */
    pcnt_chan_config_t ch_b_cfg = {
        .edge_gpio_num  = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A,
    };
    pcnt_channel_handle_t ch_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt, &ch_b_cfg, &ch_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_b,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_b,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt));
    ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt));
    ESP_LOGI(TAG_CTRL, "Encoder ready (%lld counts/output-rev)",
             (long long)COUNTS_PER_OUTPUT_REV);
}

/* Return the counts accumulated since the previous call, then reset to 0. */
static int encoder_read_and_clear(void)
{
    int c = 0;
    pcnt_unit_get_count(s_pcnt, &c);
    pcnt_unit_clear_count(s_pcnt);
    return c;
}


static void ina219_monitor_task(void *arg)
{
    (void)arg;
    float prev_a    = 0.0f;
    bool  have_prev = false;

    ESP_LOGI(TAG_INA, "Current monitor task started (%d Hz)", 1000 / INA_POLL_MS);

    for (;;) {
        float amps;
        esp_err_t err = ina219_read_current(&amps);

        if (err != ESP_OK) {
            ESP_LOGE(TAG_INA, "read failed: %s", esp_err_to_name(err));
            have_prev = false;            
            vTaskDelay(pdMS_TO_TICKS(INA_POLL_MS));
            continue;
        }

        amps = fabsf(amps);                

        bool    active;
        int64_t grace_until;
        state_lock();
        s_state.current_a = amps;
        active            = s_state.motion_active;
        grace_until       = s_state.grace_until_us;
        state_unlock();

        if (active && esp_timer_get_time() > grace_until) {
            bool over  = amps > OVERCURRENT_LIMIT_A;
            bool spike = have_prev && ((amps - prev_a) > CURRENT_DELTA_A);
            if (over || spike) {
                ESP_LOGW(TAG_INA,
                         "OBSTACLE (current): I=%.2fA dI=%.2fA [%s]",
                         amps, have_prev ? amps - prev_a : 0.0f,
                         over ? "over-limit" : "spike");
                xEventGroupSetBits(s_evt, EVT_OBSTACLE_CURRENT);
            }
        }

        prev_a    = amps;
        have_prev = true;
        vTaskDelay(pdMS_TO_TICKS(INA_POLL_MS));
    }
}


static void motor_control_task(void *arg)
{
    (void)arg;
    int64_t stall_start_us = 0;            
    float   integral       = 0.0f;          

    const float dt              = CONTROL_LOOP_MS / 1000.0f;
    const float rpm_per_min_fac = 60.0f / dt; 
    TickType_t  last_wake       = xTaskGetTickCount();

    encoder_read_and_clear();

    ESP_LOGI(TAG_CTRL, "Control task started (%d Hz)", 1000 / CONTROL_LOOP_MS);

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_LOOP_MS));

        int   delta    = encoder_read_and_clear();
        float revs     = (float)abs(delta) / (float)COUNTS_PER_OUTPUT_REV;
        float door_rpm = revs * rpm_per_min_fac;

        bool        active;
        motor_dir_t dir;
        float       target;
        int64_t     grace_until;
        state_lock();
        s_state.measured_rpm = door_rpm;
        active               = s_state.motion_active;
        dir                  = s_state.dir;
        target               = s_state.target_rpm;
        grace_until          = s_state.grace_until_us;
        state_unlock();

        if (!active) {
            stall_start_us = 0;           
            integral       = 0.0f;
            continue;
        }

        float ff    = (target * GEAR_RATIO / MOTOR_RPM_AT_MAX) * PWM_DUTY_MAX;
        float error = target - door_rpm;

        integral += CTRL_KI * error * dt;
        if (integral >  INTEG_LIMIT_DUTY) integral =  INTEG_LIMIT_DUTY;
        if (integral < -INTEG_LIMIT_DUTY) integral = -INTEG_LIMIT_DUTY;

        float out = ff + CTRL_KP * error + integral;
        if (out < PWM_MIN_DUTY)     out = PWM_MIN_DUTY;    
        if (out > PWM_CEILING_DUTY) out = PWM_CEILING_DUTY;
        int duty = (int)(out + 0.5f);

        motor_drive(dir, (uint32_t)duty);

        state_lock();
        s_state.commanded_duty = duty;
        state_unlock();

        int64_t now_us = esp_timer_get_time();
        if (now_us <= grace_until) {
            stall_start_us = 0;
            continue;
        }

        float duty_pct = 100.0f * (float)duty / (float)PWM_DUTY_MAX;
        bool  pushing  = duty_pct > STALL_DUTY_THRESH;
        bool  stopped  = door_rpm < STALL_RPM_THRESH;

        if (pushing && stopped) {
            if (stall_start_us == 0) {
                stall_start_us = now_us;
            } else if ((now_us - stall_start_us) > (int64_t)STALL_HOLD_MS * 1000) {
                ESP_LOGW(TAG_CTRL,
                         "OBSTACLE (stall): duty=%.0f%% rpm=%.2f for >%dms",
                         duty_pct, door_rpm, STALL_HOLD_MS);
                xEventGroupSetBits(s_evt, EVT_OBSTACLE_STALL);
                stall_start_us = 0;
            }
        } else {
            stall_start_us = 0;            
        }
    }
}


static const char *dir_name(motor_dir_t d)
{
    return (d == MOTOR_FORWARD) ? "OPEN(fwd)" : "CLOSE(rev)";
}


static esp_err_t move_door(motor_dir_t dir, float target_rpm, uint32_t move_ms)
{
    if (dir != MOTOR_FORWARD && dir != MOTOR_REVERSE) {
        ESP_LOGE(TAG, "move_door: invalid direction");
        return ESP_ERR_INVALID_ARG;
    }
    motor_dir_t retreat_dir = (dir == MOTOR_FORWARD) ? MOTOR_REVERSE : MOTOR_FORWARD;

    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        ESP_LOGI(TAG, "=== move %s | target %.1f RPM | attempt %d/%d ===",
                 dir_name(dir), target_rpm, attempt, MAX_RETRIES);

        state_end_motion();
        xEventGroupClearBits(s_evt, EVT_OBSTACLE_ANY);

        ESP_LOGI(TAG, "kickstart %dms @ duty %d", KICKSTART_MS, KICKSTART_DUTY);
        motor_drive(dir, KICKSTART_DUTY);
        vTaskDelay(pdMS_TO_TICKS(KICKSTART_MS));

        int64_t start_us    = esp_timer_get_time();
        int64_t grace_until = start_us + (int64_t)STARTUP_GRACE_MS * 1000;
        state_begin_motion(dir, target_rpm, PWM_MIN_DUTY, grace_until);

        EventBits_t bits         = 0;
        uint32_t    elapsed_ms   = 0;
        bool        completed    = false;

        for (;;) {
            elapsed_ms = (uint32_t)((esp_timer_get_time() - start_us) / 1000);
            if (elapsed_ms >= move_ms) {
                completed = true;
                break;
            }
            bits = xEventGroupWaitBits(s_evt, EVT_OBSTACLE_ANY,
                                       pdFALSE, pdFALSE,
                                       pdMS_TO_TICKS(CONTROL_LOOP_MS));
            if (bits & EVT_OBSTACLE_ANY) {
                elapsed_ms = (uint32_t)((esp_timer_get_time() - start_us) / 1000);
                break;
            }
        }

        state_end_motion();

        if (completed) {
            motor_drive(MOTOR_BRAKE, PWM_DUTY_MAX);
            vTaskDelay(pdMS_TO_TICKS(BRAKE_PULSE_MS));
            motor_drive(MOTOR_COAST, 0);
            ESP_LOGI(TAG, "move %s COMPLETE (%ums)", dir_name(dir),
                     (unsigned)elapsed_ms);
            return ESP_OK;
        }

        const char *cause = (bits & EVT_OBSTACLE_CURRENT) ? "OVERCURRENT"
                          : (bits & EVT_OBSTACLE_STALL)   ? "STALL"
                          : "UNKNOWN";
        ESP_LOGW(TAG, "OBSTACLE [%s] after %ums -> retreat", cause,
                 (unsigned)elapsed_ms);

        motor_drive(MOTOR_COAST, 0);
        vTaskDelay(pdMS_TO_TICKS(RETREAT_PAUSE_MS));

        uint32_t retreat_ms = (elapsed_ms > RETREAT_MIN_MS) ? elapsed_ms
                                                            : RETREAT_MIN_MS;
        ESP_LOGI(TAG, "retreat %s %ums @ duty %d",
                 dir_name(retreat_dir), (unsigned)retreat_ms, RETREAT_DUTY);
        motor_drive(retreat_dir, RETREAT_DUTY);
        vTaskDelay(pdMS_TO_TICKS(retreat_ms));

        motor_drive(MOTOR_COAST, 0);
        ESP_LOGI(TAG, "coast + cool-down %dms", OBSTACLE_RETRY_DELAY);
        vTaskDelay(pdMS_TO_TICKS(OBSTACLE_RETRY_DELAY));

    }

    motor_drive(MOTOR_COAST, 0);
    ESP_LOGE(TAG, "CRITICAL: move %s ABORTED after %d attempts - door blocked",
             dir_name(dir), MAX_RETRIES);
    return ESP_FAIL;
}


void app_main(void)
{
    ESP_LOGI(TAG, "Door safety controller booting...");

    s_state_mtx = xSemaphoreCreateMutex();
    s_evt       = xEventGroupCreate();
    if (!s_state_mtx || !s_evt) {
        ESP_LOGE(TAG, "Failed to allocate RTOS sync objects - halting");
        return;
    }
    memset(&s_state, 0, sizeof(s_state));

    if (ina219_init() != ESP_OK) {
        ESP_LOGE(TAG, "INA219 init failed - halting for safety");
        return;                   
    }
    encoder_init();
    motor_init();

    BaseType_t ok1 = xTaskCreatePinnedToCore(ina219_monitor_task, "ina_mon",
                                             4096, NULL, 6, NULL, tskNO_AFFINITY);
    BaseType_t ok2 = xTaskCreatePinnedToCore(motor_control_task, "ctrl_loop",
                                             4096, NULL, 7, NULL, 1);
    if (ok1 != pdPASS || ok2 != pdPASS) {
        ESP_LOGE(TAG, "Failed to create safety tasks - halting");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "System ready. Idle current: %.3f A", s_state.current_a);

    for (;;) {
        esp_err_t r;

        r = move_door(MOTOR_FORWARD, DOOR_OPEN_RPM, DOOR_MOVE_TIME_MS);
        ESP_LOGI(TAG, "OPEN result: %s", esp_err_to_name(r));
        vTaskDelay(pdMS_TO_TICKS(3000));   

        r = move_door(MOTOR_REVERSE, DOOR_CLOSE_RPM, DOOR_MOVE_TIME_MS);
        ESP_LOGI(TAG, "CLOSE result: %s", esp_err_to_name(r));
        vTaskDelay(pdMS_TO_TICKS(5000));   
    }
}