#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/i2c.h"
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
#define COUNTS_PER_MOTOR_REV  (4 * ENCODER_CPT)
#define GEAR_RATIO            131.0f   // Motor revs per output rev
#define MOTOR_RPM_AT_MAX      4500.0f 

// pid and control
#define CONTROL_DT_MS         20
#define KP                    0.10f 
#define KI                    0.50f
#define KD                    0.0f
#define INTEG_LIMIT           (PWM_MAX * 0.50f)
#define MIN_PWM               150.0f

#define MAX_CURRENT           0.35f

static const char *TAG = "MOTOR_SYSTEM";

static volatile float g_target_output_rpm = 0.0f;
static volatile bool  g_drive_enabled     = false;
static pcnt_unit_handle_t g_pcnt = NULL;

//timer variables
typedef enum {
    MODE_IDLE,
    MODE_NORMAL,
    MODE_REVERSING
} auto_reverse_mode_t;

static volatile auto_reverse_mode_t g_auto_mode = MODE_IDLE;
static volatile bool g_interference_tripped = false;
static volatile float g_latest_current = 0.0f;
static volatile TickType_t g_move_start_tick = 0;
static volatile TickType_t g_reverse_start_tick = 0;
static volatile TickType_t g_move_duration_ticks = 0;

static void stopMotor(void); // Forward declaration

// INA219 I2C
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

    esp_err_t ret = i2c_master_write_read_device(I2C_PORT_NUM, INA219_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) {
        return -999.0f; 
    }

    int16_t raw_shunt = (int16_t)((data[0] << 8) | data[1]);
    return (float)raw_shunt * 0.0001f;
}

static float ina219_read_bus_voltage(void)
{
    uint8_t reg = INA219_REG_BUSV;
    uint8_t data[2] = {0};

    esp_err_t ret = i2c_master_write_read_device(I2C_PORT_NUM, INA219_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) {
        return -999.0f;
    }

    int16_t raw_bus = (int16_t)((data[0] << 8) | data[1]);
    return (float)(raw_bus >> 3) * 0.004f;
}

// Task dedicated to monitoring and logging INA219 sensor data
static void ina219_task(void *arg)
{
    int print_counter = 0;
    
    for (;;) {
        float current = ina219_read_current_amps();
        float voltage = ina219_read_bus_voltage();

        if (current > -100.0f) {
            g_latest_current = fabsf(current); // Update global variable for fast access
        }

        // Print only 4 times per second (every 12th loop of 20ms) to avoid spamming the console
        if (++print_counter >= 12) {
            if (current < -100.0f || voltage < -100.0f) {
                ESP_LOGE(TAG, "INA219 Read Error! Check I2C wiring (SDA:21, SCL:22)");
            } else {
                ESP_LOGI(TAG, "Bus V: %.2f V | Curr: %.3f A (%.1f mA)", 
                         voltage, g_latest_current, g_latest_current * 1000.0f);
            }
            print_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Poll every 20ms for fast collision detection
    }
}

// motor control
static void zk_output(float signed_duty)
{
    float mag = fabsf(signed_duty);
    
    if (mag > 0.1f && mag < MIN_PWM) {
        mag = MIN_PWM; 
    }

    if (mag > PWM_MAX) mag = PWM_MAX;
    uint32_t duty = (uint32_t)(mag + 0.5f);

    if (signed_duty >= 0.0f) {
        ledc_set_duty(LEDC_MODE, LEDC_CH_INB, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CH_INB);
        ledc_set_duty(LEDC_MODE, LEDC_CH_INA, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CH_INA);
    } else {
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
    int stall_counter = 0;
    const int STALL_THRESHOLD = 25; 

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
            stall_counter = 0;
            zk_coast();
            continue;
        }

        // auto reverse logic
        TickType_t current_tick = xTaskGetTickCount();

        if (g_auto_mode == MODE_NORMAL) {
            if (g_latest_current > MAX_CURRENT) {
                
                ESP_LOGW(TAG, "INTERFERENCE, Current: %.2f A", g_latest_current);
                g_interference_tripped = true;
                // time calculation
                g_move_duration_ticks = current_tick - g_move_start_tick;
                
                // state changing
                g_auto_mode = MODE_REVERSING;
                
                // direction reverse 
                g_target_output_rpm = -g_target_output_rpm; 
                g_reverse_start_tick = current_tick;
                integral = 0.0f; 
                prev_err = 0.0f;
            }
        } 
        else if (g_auto_mode == MODE_REVERSING) {
            // check reverse time 
            if ((current_tick - g_reverse_start_tick) >= g_move_duration_ticks) {
                ESP_LOGI(TAG, "Finished backing up. Stopping motor.");
                stopMotor(); 
                integral = 0.0f;
                prev_err = 0.0f;
                zk_coast();
                continue; 
            }
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

        // Stall Detection Logic
        if (fabsf(out) > (PWM_MAX * 0.2f) && fabsf(meas_motor_rpm) < 1.0f) {
            stall_counter++;
            if (stall_counter > STALL_THRESHOLD) {
                ESP_LOGW(TAG, "STALL DETECTED! Shutting down motor.");
                stopMotor(); 
                integral = 0.0f;
                zk_coast();
                continue;
            }
        } else {
            stall_counter = 0;
        }

        zk_output(out);
    }
}

static void stopMotor(void)
{
    g_target_output_rpm = 0.0f;
    g_drive_enabled = false;
    g_auto_mode = MODE_IDLE; // Reset state
}

static void drive_init(void)
{
    // Initialize I2C Bus for INA219
    i2c_init();

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

    // Channel A
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

    // Channel B
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

    // Create Motor PID task
    xTaskCreate(control_task, "motor_ctl", 4096, NULL, 5, NULL);

    // Create INA219 current monitoring task
    xTaskCreate(ina219_task, "ina219_mon", 3072, NULL, 3, NULL);
}

static void set_motor_speed_rpm(float target_rpm)
{
    //auto reverse: no commands
    if (g_auto_mode == MODE_REVERSING && target_rpm != 0.0f) {
        return; 
    }

    if (target_rpm == 0.0f) {
        stopMotor();
    } else {
        g_target_output_rpm = target_rpm;
        //fresh -> recording start time
        if (!g_drive_enabled || g_auto_mode == MODE_IDLE) {
            g_move_start_tick = xTaskGetTickCount();
            g_auto_mode = MODE_NORMAL;
        }
        g_drive_enabled = true;
    }
}
//cycle interrupted -> returns; false if normal
static bool run_motor_and_wait(float target_rpm, uint32_t wait_ms)
{
    g_interference_tripped = false; //flag reset
    set_motor_speed_rpm(target_rpm);

    uint32_t elapsed_ms = 0;
    const uint32_t step_ms = 50; // status every 50ms

    while (elapsed_ms < wait_ms) {
        if (g_interference_tripped) {
            // interference -> wait reverse -> return to idle
            while (g_auto_mode != MODE_IDLE) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            return true; //app main -> interrupted message
        }
        
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        elapsed_ms += step_ms;
    }

    return false; 
}


void app_main(void)
{
    drive_init();
    
    for (;;) {
        if (run_motor_and_wait(7.0f, 5000)) {
            continue; 
        }

        set_motor_speed_rpm(0.0f);
        vTaskDelay(pdMS_TO_TICKS(100));

        if (run_motor_and_wait(-7.0f, 5000)) {
            continue;
        }

        set_motor_speed_rpm(0.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

