#pragma once
// espp includes for AS5600
#include "as5600.hpp"
#include "i2c.hpp"
#include "butterworth_filter.hpp"
#include "task.hpp"

// ========= User configuration (adjust to your board) =========
// I2C bus configuration for single AS5600 sensor
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22

// Message for sensor data - Float32 for radian angle
extern std_msgs__msg__Float32 sensor_data_msg;

// Mutex for protecting sensor message access
extern SemaphoreHandle_t sensor_msg_mutex;

// Single AS5600 encoder
extern espp::As5600 *g_as5600;

void as5600_init(void);
void sensor_sample_task(void *arg);

