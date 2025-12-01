#include "vaccum_firmware.hpp"

// Initialize single AS5600 sensor
void as5600_init(void)
{
    ESP_LOGD("SENSOR", "Initializing AS5600 sensor...");
    
    // Create I2C instance for AS5600
    static espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
    });

    // Velocity filter
    static constexpr float filter_cutoff_hz = 4.0f;
    static constexpr float encoder_update_period = 0.01f; // 10ms (100 Hz)
    
    static espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period});
    
    // Filter function
    auto filter_fn = [&filter](float raw) -> float { return filter.update(raw); };

    // Create AS5600 instance
    g_as5600 = new espp::As5600(
        {.write_then_read =
             std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
         .velocity_filter = filter_fn,
         .update_period = std::chrono::duration<float>(encoder_update_period),
         .log_level = espp::Logger::Verbosity::WARN});

    ESP_LOGI("SENSOR", "AS5600 sensor initialized on I2C_NUM_0 (SDA=%d, SCL=%d)", 
             I2C_SDA_GPIO, I2C_SCL_GPIO);
}

// Sensor sampling task
void sensor_sample_task(void *arg)
{
    ESP_LOGD("SENSOR_TASK", "Sensor sampling task starting...");
    
    as5600_init();
    
    (void)arg;
    int log_counter = 0;
    TickType_t last_wake = xTaskGetTickCount();
    
    ESP_LOGD("SENSOR_TASK", "Sensor sampling task initialized");
    
    while (1) {
        // Read AS5600 sensor data
        float angle_deg = 0.0f;
        float angle_rad = 0.0f;
        float rpm = 0.0f;
        float count = 0.0f;
        
        if (g_as5600) {
            angle_deg = g_as5600->get_degrees();
            angle_rad = g_as5600->get_radians();
            rpm = g_as5600->get_rpm();
            count = g_as5600->get_count();
        }

        // Update sensor message with mutex protection - only publish radian angle
        if (xSemaphoreTake(sensor_msg_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            // Store only the angle in radians
            sensor_data_msg.data = angle_rad;
            
            xSemaphoreGive(sensor_msg_mutex);
            
            if (log_counter % 50 == 0) {
                ESP_LOGI("SENSOR_TASK", "Sensor updated - Angle: %.2f° (%.4f rad), RPM: %.2f, Count: %.2f", 
                         angle_deg, angle_rad, rpm, count);
            }
        } else {
            ESP_LOGW("SENSOR_TASK", "Failed to take sensor_msg_mutex");
        }

        log_counter++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ENCODER_SAMPLE_MS));
    }
}
