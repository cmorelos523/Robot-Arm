#include "servos.h"


/**
 * @brief   Moves servo 1 to the specified angle
 * @details The Micro Servo 9g requires a PWM signal at 50 Hz as an input to control the angle of the motor. Each duty cycle value between 2.5% and 12.5%
 *          maps to an angle between 0° and 180° respectively. A simple linear function was used to map this values, its slope and intercept are defined as
 *          SLOPE_SERVO and INTERCEPT_SERVO
 * @param   angle: The angle to turn to
 * @return  None
 */
void set_angle_servo_1(uint8_t angle)
{
    esp_err_t status;

    // Get the duty cycle associated with this angle
    uint16_t pos_duty = (SLOPE_SERVO * angle) + INTERCEPT_SERVO;

    // Update duty cycle
    status = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pos_duty);
    status = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    if (status != ESP_OK)
    {
        printf("LEDC servo 1 update error\n");
    }
}


/**
 * @brief   Moves servo 2 to the specified angle
 * @details The Micro Servo 9g requires a PWM signal at 50 Hz as an input to control the angle of the motor. Each duty cycle value between 2.5% and 12.5%
 *          maps to an angle between 0° and 180° respectively. A simple linear function was used to map this values, its slope and intercept are defined as
 *          SLOPE_SERVO and INTERCEPT_SERVO
 * @param   angle: The angle to turn to
 * @return  None
 */
void set_angle_servo_2(uint8_t angle)
{
    esp_err_t status;

    // Get the duty cycle associated with this angle
    uint16_t pos_duty = (SLOPE_SERVO * angle) + INTERCEPT_SERVO;

    // Update duty cycle
    status = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pos_duty);
    status = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    if (status != ESP_OK)
    {
        printf("LEDC servo 2 update error\n");
    }
}


/**
 * @brief   This functions coordinates the movement of both motors
 * @details Divides the target angle into steps and calls the set_angle_servo_x() functions every step to smoothtly transition into the target position 
 * @param   targer_s1: Target angle of servo 1
 * @param   target_s2: Target angle of servo 2
 * @return  None
 */
void move_arm_to_target(uint8_t target_s1, uint8_t target_s2)
{
    // Calculate the angle difference and total steps required to get there
    int diff_s1 = target_s1 - current_s1_angle;
    int diff_s2 = target_s2 - current_s2_angle;
    int abs_diff_s1 = abs(diff_s1);
    int abs_diff_s2 = abs(diff_s2);
    int total_steps = (abs_diff_s1 > abs_diff_s2) ? (abs_diff_s1) : (abs_diff_s2);

    if (total_steps == 0)
    {
        return;
    }

    // Calculate stride
    float step_s1 = (float)diff_s1 / total_steps;
    float step_s2 = (float)diff_s2 / total_steps;

    // Angle trackers
    float running_s1 = current_s1_angle;
    float running_s2 = current_s2_angle;
    
    // Update servos one step at a time
    for (size_t i = 0; i < total_steps; i++)
    {
        running_s1 += step_s1;
        running_s2 += step_s2;
        set_angle_servo_1((uint8_t)round(running_s1));
        set_angle_servo_2((uint8_t)round(running_s2));

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    // Set to target angle, just in case
    set_angle_servo_1(target_s1);
    set_angle_servo_2(target_s2);

    // Update global angle trackers
    current_s1_angle = target_s1;
    current_s2_angle = target_s2;
}
