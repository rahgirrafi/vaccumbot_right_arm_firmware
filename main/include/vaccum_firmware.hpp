#ifndef VACUUM_FIRMWARE_HPP
#define VACUUM_FIRMWARE_HPP

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <chrono>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "sdkconfig.h"

#include  "motor.hpp"
#include "micro_ros.hpp"
#include "encoder.hpp"
#include "esp_event.h"
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <uros_network_interfaces.h>
using namespace std::chrono_literals;
extern const char *TAG ;

// Timings - ULTRA-CONSERVATIVE to prevent network buffer corruption
#define SENSOR_PUBLISH_PERIOD_MS 200 // 5 Hz - Match encoder sampling rate
#define CONTROL_PERIOD_MS 20 // 50 Hz
#define ENCODER_SAMPLE_MS 200 // 5 Hz - Keep sensor sampling rate

// Robot params
#define WHEEL_RADIUS 0.022f
#define WHEEL_SEPARATION 0.10f

void print_mem(const char *tag);
void print_memory_info();

#endif // VACUUM_FIRMWARE_HPP__