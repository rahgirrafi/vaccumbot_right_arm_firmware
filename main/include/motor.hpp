#pragma once

// TB6612FNG Motor Driver GPIO pins for Motor 3 (Group 3)
#define MOTOR_IN1_GPIO    GPIO_NUM_4     // Direction control 1
#define MOTOR_IN2_GPIO    GPIO_NUM_2    // Direction control 2

// PWM Configuration for GPIO_5 (MOTOR3_IN1_GPIO)
#define MOTOR_PWM_GPIO1   MOTOR_IN1_GPIO
#define MOTOR_PWM_GPIO2   MOTOR_IN2_GPIO
#define MOTOR_LEDC_CHANNEL1 LEDC_CHANNEL_0
#define MOTOR_LEDC_CHANNEL2 LEDC_CHANNEL_1

#define MOTOR_PWM_TIMER   LEDC_TIMER_0

// PID defaults for arm motor control (tune these)
#define ARM_KI 0.0f
#define ARM_KP 1.2f
#define ARM_KD 0.01f
#define ARM_MAX_PWM 8191

// Motor control target
extern float target_motor_speed;
extern SemaphoreHandle_t motor_mutex;
extern TickType_t last_arm_state_update_time;

// Position error from arm controller
extern float joint_pos_error;

void motors_init();
void set_motor_pwm(uint8_t ledc_channel, float pwm_frac);
void motor_control_task(void *arg);

