#ifndef MATRIX_H
#define MATRIX_H


#include <inttypes.h>
#include "servos.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"


// Struct to hold each coordinate's angles and index
typedef struct {
    uint8_t s1_angle;
    uint8_t s2_angle;
    uint8_t idx;
} coord_info;

// 2D matrix of coordinates, each one holding servo 1 and 2's required angles and index.
extern coord_info matrix[8][8];


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
void path_finder(uint8_t led_matrix[][8], int curr_row, int curr_column);

/**
 * @brief   This function sets up the 2D matrix "coordinates" (angles)
 * @details Each point on the LED matrix has an associated pair of angles needed to get to it, one for servo 1, the other for servo 2, and an index. This functions sets
 *          it up
 * @param   None
 * @return  None
 */
void matrix_set_up(void);

#endif