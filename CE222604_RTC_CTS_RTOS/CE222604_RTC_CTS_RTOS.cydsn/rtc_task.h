/******************************************************************************
* File Name: task_rtc.h
*
* Version: 1.00
*
* Description: This file is the public interface of task_ rtc.c source file 
*
* Related Document: CE222604_RTC_CTS_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
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
/******************************************************************************
* This file contains the declaration of tasks and functions used for the RTC
* application
*******************************************************************************/

/* Include guard */    
#ifndef RTC_TASK_H
#define RTC_TASK_H

/* Header file includes */ 
#include "project.h"
#include "FreeRTOS.h"
#include "queue.h"

/* Reuse the BLE CTS time and date structure for RTC APIs */    
typedef cy_stc_ble_cts_current_time_t time_and_date_t;     

/* Lis if RTC commands */
typedef enum
{
    RTC_UPDATE_TIME,
    RTC_PROCESS_ALARM
} rtc_command_t;

/* Data-type of RTC commands and data */
typedef struct
{
    rtc_command_t command;
    cy_stc_rtc_config_t rtcDateTime;
} rtc_command_and_data_t;

/* Handle for the Queue that contains RTC command and data */
extern QueueHandle_t rtcCommandAndDataQ;

/* Task_Rtc takes care of the RTC module in this code example */
void Task_Rtc(void *pvParameters);

#endif /* TASK_RTC_H */

/* [] END OF FILE */
