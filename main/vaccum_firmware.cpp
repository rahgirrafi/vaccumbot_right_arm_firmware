/*
ESP32 micro-ROS + AS5600 - Single Motor and Sensor Example

This simplified example controls one motor and reads one AS5600 magnetic encoder sensor,
publishing the sensor data via micro-ROS using a FixedArray8 message.
*/

#include "vaccum_firmware.hpp"
#include "global_declarations.hpp"

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32 single motor + AS5600 sensor node");
    
    // Initialize network interface for micro-ROS
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    // Initialize mutexes
    motor_mutex = xSemaphoreCreateMutex();
    sensor_msg_mutex = xSemaphoreCreateMutex();
    joint_state_mutex = xSemaphoreCreateMutex();

    // Initialize motor PWM
    motors_init();
    ESP_LOGD(TAG, "Motor PWM initialized");

    // Initialize micro-ROS communication
    micro_ros_init_and_create_comm();

    // Create tasks
    xTaskCreate(sensor_sample_task, "sensor_task", 4096, NULL, 2, NULL);
    xTaskCreate(motor_control_task, "motor_ctrl", 4096, NULL, 3, NULL);
    xTaskCreate(micro_ros_spin_task, "micro_ros", 8192, NULL, 1, NULL);
    
    ESP_LOGD(TAG, "All tasks launched successfully");
}


