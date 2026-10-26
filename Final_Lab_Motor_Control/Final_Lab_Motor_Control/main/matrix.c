#include "matrix.h"


/**
 * @brief   This function contains the main path finder algorithm used to traverse the received 2D matrix and draw the image
 * @details Uses a recursive depth first search approach to explore the 2D LED matrix received via BLE, setting the servos to their appropiate angle on every step.
 *          Points explored are marked and set to 2. Order of exploration follows the cardinal directions:
 *              1. Top
 *              2. Right
 *              3. Down
 *              4. Left
 * @param   led_matrix: 2D matrix currently being explored 
 * @param   curr_row: Index of current row the function is exploring
 * @param   curr_column: Index of current column the function is exploring 
 * @return  None
 */
void path_finder(uint8_t led_matrix[][8], int curr_row, int curr_column)
{
    // Base case check
    // Not valid coordinates or number is not a 1 (either a 0 or a 2)
    if ((curr_column < 0) || (curr_column > 7) || (curr_row < 0) || (curr_row > 7) || (led_matrix[curr_row][curr_column] != 1))
    {
        return;
    }
    
    // Mark spot as checked
    led_matrix[curr_row][curr_column] = 2;
    
    // Move motors
    uint8_t s1_angle = matrix[curr_row][curr_column].s1_angle;
    uint8_t s2_angle = matrix[curr_row][curr_column].s2_angle;
    move_arm_to_target(s1_angle, s2_angle);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Walk into new coordinates
    // Up
    path_finder(led_matrix, curr_row - 1, curr_column);
    // Right
    path_finder(led_matrix, curr_row, curr_column + 1);
    // Down
    path_finder(led_matrix, curr_row + 1, curr_column);
    // Left
    path_finder(led_matrix, curr_row, curr_column - 1);
}


/**
 * @brief   This function sets up the 2D matrix "coordinates" (angles)
 * @details Each point on the LED matrix has an associated pair of angles needed to get to it, one for servo 1, the other for servo 2, and an index. This functions sets
 *          it up
 * @param   None
 * @return  None
 */
void matrix_set_up(void)
{
    // Lookup table storing the raw (servo 1 angle, servo 2 angle) pairs
    static const uint8_t raw_angles[8][8][2] = {
        // Row 0
        {{51, 127}, {56, 121}, {61, 114}, {66, 108}, {70, 102}, {75,  96}, {79,  90}, {85,  83}},
        // Row 1
        {{47, 123}, {52, 117}, {58, 110}, {63, 105}, {67,  99}, {72,  93}, {76,  87}, {83,  80}},
        // Row 2
        {{44, 119}, {48, 114}, {54, 107}, {60, 101}, {64,  95}, {70,  90}, {74,  84}, {80,  77}},
        // Row 3
        {{40, 115}, {45, 110}, {51, 103}, {57,  98}, {61,  92}, {67,  87}, {72,  81}, {78,  74}},
        // Row 4
        {{36, 112}, {41, 106}, {48, 100}, {53,  94}, {59,  89}, {64,  83}, {70,  77}, {76,  72}},
        // Row 5
        {{32, 108}, {38, 102}, {45,  96}, {50,  91}, {56,  86}, {61,  80}, {68,  74}, {74,  69}},
        // Row 6
        {{29, 104}, {34,  98}, {41,  93}, {47,  87}, {53,  82}, {59,  77}, {65,  71}, {71,  66}},
        // Row 7
        {{25, 100}, {31,  95}, {38,  89}, {44,  84}, {50,  79}, {56,  74}, {63,  68}, {69,  63}}
    };

    uint8_t current_idx = 0;

    // Populate global matrix
    for (int row = 0; row < 8; row++) 
    {
        for (int col = 0; col < 8; col++) 
        {
            matrix[row][col].s1_angle = raw_angles[row][col][0];
            matrix[row][col].s2_angle = raw_angles[row][col][1];
            matrix[row][col].idx      = current_idx++;
        }
    }
}