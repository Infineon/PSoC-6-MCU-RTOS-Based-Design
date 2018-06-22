/******************************************************************************
* File Name: proximity_task.h
*
* Version: 1.00
*
* Description: This file is the public interface of proximity_task.c source file
*
* Related Document: CE218137_BLE_Proximity_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/
/********************************************************************************
* This file contains the declaration of tasks and functions used for proximity
* sensing
********************************************************************************/

/* Include guard */
#ifndef PROXIMITY_TASK_H
#define PROXIMITY_TASK_H

/* Header file includes */ 
#include "project.h"
#include "FreeRTOS.h"     
#include "queue.h"    

/* Data-type that's used to request periodic proximity data */    
typedef enum
{
    SEND_PROXIMITY, 
    SEND_NONE,
    PROXIMITY_TIMER_EXPIRED
}   proximity_command_t;    

/* Handles for the Queues that contain proximity command */ 
extern QueueHandle_t proximityCommandQ;

/* Task_Proximity scans the proximity sensor and sends data to Task_Ble when
   required */    
void Task_Proximity(void *pvParameters);    

#endif /* PROXIMITY_TASK_H */
/* [] END OF FILE */
