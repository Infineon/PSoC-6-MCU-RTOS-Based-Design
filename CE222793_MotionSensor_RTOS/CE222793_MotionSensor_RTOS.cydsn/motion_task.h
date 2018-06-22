/******************************************************************************
* File Name: motion_task.h
*
* Version: 1.00
*
* Description: This file is the public interface of motion_task.c
*
* Related Document: CE222793_MotionSensor_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/

/* Include Guard */
#ifndef MOTION_SENSOR_TASK_H
#define MOTION_SENSOR_TASK_H

/* Header files include */
#include "project.h"

/* Orientation types:
 * Indicates which edge of the board is pointing towards the ceiling/sky */
typedef enum
{
    ORIENTATION_TOP_EDGE        = 0,    /* Top edge of the board points towards the ceiling */
    ORIENTATION_BOTTOM_EDGE     = 1,    /* Bottom edge of the board points towards the ceiling */
    ORIENTATION_LEFT_EDGE       = 2,    /* Left edge of the board (USB connector side) points towards the ceiling */
    ORIENTATION_RIGHT_EDGE      = 3,    /* Right edge of the board points towards the ceiling */
    ORIENTATION_DISP_UP         = 4,    /* Display faces up (towards the sky/ceiling) */
    ORIENTATION_DISP_DOWN       = 5     /* Display faces down (towards the ground) */
} orientation_t;

/* Data-type of motion sensor data */
typedef struct
{
    orientation_t orientation;
    uint16_t stepCount;
} motion_sensor_info_t;

/* Task_Motion takes care of the Motion Sensor module in this code example */    
void Task_Motion(void* pvParameters);

#endif
/* [] END OF FILE */
