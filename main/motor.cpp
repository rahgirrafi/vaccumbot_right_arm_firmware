
#include "vaccum_firmware.hpp"

// ========= PWM / GPIO init =========
void motors_init(){
    ESP_LOGI("MOTOR", "Initializing LEDC PWM for single motor control...");
   
    // Configure LEDC PWM timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT, // 13-bit resolution
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000, // 1 kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI("MOTOR", "LEDC timer configured successfully");

    // Channel for motor dir 1
    ledc_channel_config_t ledc_channel = {
        .gpio_num = MOTOR_PWM_GPIO1,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = MOTOR_LEDC_CHANNEL1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure motor channel: %s", esp_err_to_name(err));
        return;
    }

    
    // Channel for motor dir 2
    ledc_channel.channel = MOTOR_LEDC_CHANNEL2;
    ledc_channel.gpio_num = MOTOR_PWM_GPIO2;
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure right arm channel: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGD("MOTOR", "LEDC PWM channel configured successfully");
}void set_motor_pwm(uint8_t ledc_channel, float pwm_frac)
{
    // pwm_frac in range -1..1
    if (pwm_frac > 1.0f) pwm_frac = 1.0f;
    if (pwm_frac < -1.0f) pwm_frac = -1.0f;
    
    // Calculate duty for 13-bit resolution (0-8191)
    // LEDC_TIMER_13_BIT = 2^13 - 1 = 8191
    int duty = (int)(fabsf(pwm_frac) * 8191.0f);

    esp_err_t err1 = ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ledc_channel, duty);
    esp_err_t err2 = ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ledc_channel);
    
    if (err1 != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to set duty for channel %d: %s", ledc_channel, esp_err_to_name(err1));
    }
    if (err2 != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to update duty for channel %d: %s", ledc_channel, esp_err_to_name(err2));
    }
}

void set_motor_pwm_special(uint8_t ledc_channel, float pwm_frac, int8_t direction)
{
    //if direction is 1, set put pwm to channel 1, 0 to channel 2
    if(direction == 1){
        set_motor_pwm(MOTOR_LEDC_CHANNEL1, fabsf(pwm_frac));
        set_motor_pwm(MOTOR_LEDC_CHANNEL2, 0.0f);
    
    } else if(direction == -1){
        set_motor_pwm(MOTOR_LEDC_CHANNEL1, 0.0f);
        set_motor_pwm(MOTOR_LEDC_CHANNEL2, fabsf(pwm_frac));
    } else {
        //stop motor
        set_motor_pwm(MOTOR_LEDC_CHANNEL1, 0.0f);
        set_motor_pwm(MOTOR_LEDC_CHANNEL2, 0.0f);   
    }
    
}

// Advanced motor control task with position and speed PID
void motor_control_task(void *arg)
{
    ESP_LOGD("MOTOR_CONTROL", "Motor control task starting with PID...");
    
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    static uint32_t log_counter = 0;
    
    // PID state variables
    float last_speed_error = 0.0f;
    float error = 0.0f;

    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        const TickType_t timeout_ticks = pdMS_TO_TICKS(500); // 500ms timeout
        bool arm_state_timeout = (last_arm_state_update_time == 0) || 
                                 ((current_time - last_arm_state_update_time) > timeout_ticks);
        
        if (arm_state_timeout) {
            // Stop arm motors if no updates received within timeout period
            set_motor_pwm(MOTOR_LEDC_CHANNEL1, 0.0f);
            set_motor_pwm(MOTOR_LEDC_CHANNEL2, 0.0f);
            if (log_counter % 40 == 0) {
                ESP_LOGW("DRIVE_CONTROL_TASK", "Arm state timeout - motors stopped (no updates for >500ms)");
            }
            log_counter++;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }
        // Get position error from arm controller subscription
        error = joint_pos_error;

        // Get current speed from AS5600 sensor (RPM)
        float current_speed = 0.0f;
        if (g_as5600) {
            current_speed = g_as5600->get_rpm();
        }

        // Target speed is 0.11 RPM (adjust as needed)
        const float target_speed = 0.11f;
        
        // Calculate speed error (target - current)
        float speed_error = target_speed - current_speed;

        speed_error = (speed_error/target_speed) * ARM_MAX_PWM;

        float integral = 0;
        // PID control for speed
        float derivative = (speed_error - last_speed_error) / (CONTROL_PERIOD_MS / 1000.0f);
        integral = integral + speed_error * (CONTROL_PERIOD_MS / 1000.0f);
        float output = speed_error * ARM_KP + derivative * ARM_KD + integral * ARM_KI;

        last_speed_error = speed_error;

        // Normalize to PWM range
        float pwm_frac = output / (float)ARM_MAX_PWM;

        // Clamp to [-1.0, 1.0]
        if (pwm_frac > 1.0f) pwm_frac = 1.0f;
        if (pwm_frac < -1.0f) pwm_frac = -1.0f;

        // Determine direction based on position error
        // If error > 0: need to move forward (positive direction)
        // If error < 0: need to move backward (negative direction)
      
        if (fabsf(error) > 0.001f) {
            float direction = (error > 0.0f) ? 1.0f : -1.0f;
            set_motor_pwm_special(MOTOR_LEDC_CHANNEL1, pwm_frac, (int8_t)direction); 
            // Logging every 40 iterations
        if (log_counter % 40 == 0) {
            ESP_LOGI("MOTOR_CONTROL", "Pos Error: %.2f°, Speed: %.2f RPM, PWM: %.2f", 
                     error, current_speed, pwm_frac);
        }
        }
        else {
            // Within deadband, stop motor
            set_motor_pwm_special(MOTOR_LEDC_CHANNEL1, 0.0f, 0); 
            if (log_counter % 40 == 0) {
            ESP_LOGI("MOTOR_CONTROL", "Motor within deadband, stopping motor. Pos Error: %.2f°", error);
        }
        }
        

        log_counter++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    
}
}