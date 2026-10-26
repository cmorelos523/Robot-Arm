#ifndef SERVOS_H
#define SERVOS_H


#include <stdio.h>
#include <inttypes.h>
#include "math.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"


// Servo control
#define LEDC_FREQ               (50)
#define SERVO_1_CTRL_PIN        (GPIO_NUM_4)
#define SERVO_2_CTRL_PIN        (GPIO_NUM_1)
#define LEDC_NEUTRAL_POS_DUTY   (614)
#define LEDC_MIN_POS_DUTY       (210)
#define LEDC_MAX_POS_DUTY       (1020)

// Servo PWM to angle translation
#define SLOPE_SERVO             (4.5)
#define INTERCEPT_SERVO         (210)

// Starting angles and coordinates
#define TOP_LEFT_S1     (51)
#define TOP_LEFT_S2     (127)   
#define START_X         (0)
#define START_Y         (0)

// Globals to keep track of angles
extern uint8_t current_s1_angle;
extern uint8_t current_s2_angle;


/**
 * @brief   Moves servo 1 to the specified angle
 * @details The Micro Servo 9g requires a PWM signal at 50 Hz as an input to control the angle of the motor. Each duty cycle value between 2.5% and 12.5%
 *          maps to an angle between 0° and 180° respectively. A simple linear function was used to map this values, its slope and intercept are defined as
 *          SLOPE_SERVO and INTERCEPT_SERVO
 * @param   angle: The angle to turn to
 * @return  None
 */
void set_angle_servo_1(uint8_t angle);

/**
 * @brief   Moves servo 2 to the specified angle
 * @details The Micro Servo 9g requires a PWM signal at 50 Hz as an input to control the angle of the motor. Each duty cycle value between 2.5% and 12.5%
 *          maps to an angle between 0° and 180° respectively. A simple linear function was used to map this values, its slope and intercept are defined as
 *          SLOPE_SERVO and INTERCEPT_SERVO
 * @param   angle: The angle to turn to
 * @return  None
 */
void set_angle_servo_2(uint8_t angle);

/**
 * @brief   This functions coordinates the movement of both motors
 * @details Divides the target angle into steps and calls the set_angle_servo_x() functions every step to smoothtly transition into the target position 
 * @param   targer_s1: Target angle of servo 1
 * @param   target_s2: Target angle of servo 2
 * @return  None
 */
void move_arm_to_target(uint8_t target_s1, uint8_t target_s2);


#endif