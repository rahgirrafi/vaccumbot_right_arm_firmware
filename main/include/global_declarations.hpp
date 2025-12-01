#pragma once
#include "vaccum_firmware.hpp"
#include <custom_interfaces/msg/float32_fixed_array8.h>

// Motor control
float target_motor_speed = 0.0f;
SemaphoreHandle_t motor_mutex;

// Single AS5600 sensor
espp::As5600 *g_as5600 = nullptr;

// micro-ROS
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_publisher_t sensor_pub;
rcl_subscription_t joint_state_sub;

// Message storage - Float32 for radian angle
std_msgs__msg__Float32 sensor_data_msg;

// Joint state array message storage
custom_interfaces__msg__Float32FixedArray8 joint_state_array_msg;
SemaphoreHandle_t joint_state_mutex = NULL;

// Position error from arm controller (for single joint)
float joint_pos_error = 0.0f;

// Mutex for protecting sensor message access
SemaphoreHandle_t sensor_msg_mutex = NULL;
TickType_t last_arm_state_update_time = 0;

const char *TAG = "single_motor_node";
