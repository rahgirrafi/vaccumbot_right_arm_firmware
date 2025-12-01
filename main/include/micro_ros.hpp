#ifndef MICRO_ROS_HPP
#define MICRO_ROS_HPP
// micro-ROS / rclc includes

#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = (fn); \
  if ((temp_rc) != RCL_RET_OK) { \
    printf("RCCHECK FAILED %s:%d rc=%d -> %s\n", __FILE__, __LINE__, (int)temp_rc, rcl_get_error_string().str); \
    fflush(stdout); \
    vTaskDelete(NULL); \
  } \
}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = (fn); if((temp_rc) != RCL_RET_OK){printf("RCSOFTCHECK %s:%d rc=%d -> %s\n", __FILE__, __LINE__, (int)temp_rc, rcl_get_error_string().str);}}

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Use standard ROS2 messages
#include <std_msgs/msg/float32.h>
#include <custom_interfaces/msg/float32_fixed_array8.h>

extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_allocator_t allocator;
extern rclc_support_t support;

// micro-ROS publisher and subscription
extern rcl_publisher_t sensor_pub;
extern rcl_subscription_t joint_state_sub;

// message storage
extern std_msgs__msg__Float32 sensor_data_msg;
extern custom_interfaces__msg__Float32FixedArray8 joint_state_array_msg;

// Mutex and error variable
extern SemaphoreHandle_t joint_state_mutex;
extern float joint_pos_error;

void micro_ros_init_and_create_comm(void);
void micro_ros_spin_task(void *arg);
void joint_state_sub_callback(const void *msgin);

#endif // MICRO_ROS_HPP__

