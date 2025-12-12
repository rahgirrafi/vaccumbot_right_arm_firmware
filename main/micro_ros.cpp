#include "vaccum_firmware.hpp"
#include "micro_ros.hpp"

void joint_state_sub_callback(const void *msgin)
{
    const custom_interfaces__msg__Float32FixedArray8 *arr = (const custom_interfaces__msg__Float32FixedArray8 *)msgin;
    last_arm_state_update_time = xTaskGetTickCount();

    // Extract position error from the array (assuming it's at index 0)
    joint_pos_error = arr->element[2];

    // Store full array message with mutex protection
    if (xSemaphoreTake(joint_state_mutex, (TickType_t)10) == pdTRUE) {
        // Copy all 8 elements
        for (int i = 0; i < 8; i++) {
            joint_state_array_msg.element[i] = arr->element[i];
        }
        xSemaphoreGive(joint_state_mutex);
    }
}

void micro_ros_init_and_create_comm(void)
{
    ESP_LOGD("MICRO_ROS", "Initializing micro-ROS communication...");
    
    // Initialize the Float32 message structure (simple, no array allocation needed)
    std_msgs__msg__Float32__init(&sensor_data_msg);
    sensor_data_msg.data = 0.0f;
    
    // Initialize joint state array message
    custom_interfaces__msg__Float32FixedArray8__init(&joint_state_array_msg);
    for (int i = 0; i < 8; i++) {
        joint_state_array_msg.element[i] = 0.0f;
    }
    
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    
    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    #endif
    
    ESP_LOGD("MICRO_ROS", "Initializing support with options...");
    rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE("MICRO_ROS", "rclc_support_init_with_options failed with code %d: %s", (int)rc, rcl_get_error_string().str);
        ESP_LOGE("MICRO_ROS", "Make sure micro-ROS agent is running at %s:%s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        ESP_LOGE("MICRO_ROS", "Check WiFi connectivity and network configuration");
        fflush(stdout);
        vTaskDelete(NULL);
    }
    
    ESP_LOGD("MICRO_ROS", "Support initialization successful!");
    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "right_mcu__motor_sensor_node", "", &support));      

    ESP_LOGD("MICRO_ROS", "Creating publisher for angle (radians)...");
    
    // Create publisher for sensor data - using standard Float32 message
    rcl_ret_t pub_rc = rclc_publisher_init_default(
            &sensor_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/right_arm_angle_rad");
    
    if (pub_rc != RCL_RET_OK) {
        ESP_LOGE("MICRO_ROS", "Publisher init failed with code %d: %s", (int)pub_rc, rcl_get_error_string().str);
        fflush(stdout);
        vTaskDelete(NULL);
    }

    ESP_LOGD("MICRO_ROS", "Publisher created successfully!");
    
    // Create subscription for joint state array
    ESP_LOGD("MICRO_ROS", "Creating subscription for joint state array...");
    RCCHECK(
        rclc_subscription_init_default(
            &joint_state_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, Float32FixedArray8),
            "/joint_state_array")
    );
    ESP_LOGD("MICRO_ROS", "Subscription created successfully!");
    
    // Initialize executor (needed for subscriptions)
    ESP_LOGD("MICRO_ROS", "Creating executor...");
    executor = rclc_executor_get_zero_initialized_executor();
    // Executor with 1 handle (subscription)
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &joint_state_sub, &joint_state_array_msg, &joint_state_sub_callback, ON_NEW_DATA));
    ESP_LOGD("MICRO_ROS", "Executor created successfully!");
    
    ESP_LOGI("MICRO_ROS", "Micro-ROS initialization complete!");
}
void micro_ros_spin_task(void *arg)
{
    ESP_LOGD("MICRO_ROS_TASK", "Micro-ROS task starting (publish + subscription)...");

    // Publish rate: 100ms (10 Hz)
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    static uint32_t log_counter = 0;

    while (1) {
        // Spin executor to process subscription callbacks
        rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        if (spin_ret != RCL_RET_OK && log_counter % 50 == 0) {
            ESP_LOGW("MICRO_ROS_TASK", "Executor spin failed: %d", spin_ret);
        }
        
         // Publish sensor data (angle in radians) - use local copy to minimize mutex hold time
        std_msgs__msg__Float32 local_sensor_msg;
        bool got_mutex = false;
        
        if (xSemaphoreTake(sensor_msg_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            local_sensor_msg = sensor_data_msg;
            xSemaphoreGive(sensor_msg_mutex);
            got_mutex = true;
        } else {
            if (log_counter % 20 == 0) {
                ESP_LOGW("MICRO_ROS_TASK", "Failed to take sensor_msg_mutex");
            }
        }
        
        if (got_mutex) {
            rcl_ret_t ret = rcl_publish(&sensor_pub, &local_sensor_msg, NULL);
            if (ret != RCL_RET_OK) {
                if (log_counter % 10 == 0) {
                    ESP_LOGW("MICRO_ROS_TASK", "Angle publish failed: %d", ret);
                }
            } else {
                if (log_counter % 10 == 0) {
                    ESP_LOGI("MICRO_ROS_TASK", "Angle published: %.4f rad", local_sensor_msg.data);
                }
            }
        }
        log_counter++;
        vTaskDelay(xDelay);
    }
}
