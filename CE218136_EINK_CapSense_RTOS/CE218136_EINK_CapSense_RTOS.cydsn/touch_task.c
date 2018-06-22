/******************************************************************************
* File Name: touch_task.c
*
* Version: 1.00
*
* Description: This file contains the task that handles touch sensing and
*              gesture detection
*
* Related Document: CE218136_EINK_CapSense_RTOS.pdf
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
/*******************************************************************************
* Task that handles touch sensing and gesture detection
*******************************************************************************/

/* Header file includes */
#include "touch_task.h"
#include "uart_debug.h"
#include "task.h" 
#include "timers.h"

/* Scanning interval of 10ms is used to get 100 scans per second */
#define TOUCH_SCAN_INTERVAL  (pdMS_TO_TICKS(10u))

/* Gesture time increment at each scan interval. Set at half the scan interval
   to detect slow flicks */
#define GESTURE_TIME_INCREMENT  (5u)

/* Queue handle used for touch data */
QueueHandle_t touchDataQ;

/*******************************************************************************
* Function Name: void Task_Touch(void *pvParameters)   
********************************************************************************
* Summary:
*  Task that reads touch data from CapSense button and slider widgets and
*  calculates gesture information
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/
void Task_Touch(void *pvParameters)    
{
    /* Variable that stores CapSense API results */
    cy_status capSenseApiResult;
   
    /* Local variables used for touch and gesture detection */
    touch_data_t previousTouchData = NO_TOUCH; 
    touch_data_t currentTouchData;
    uint32_t     sliderGesture;
    uint32_t     gestureTimestamp = 0;

    /* Remove warning for unused parameter */
    (void)pvParameters ;
    
    /* Start the CapSense component and initialize the baselines */
    capSenseApiResult = CapSense_Start();
    
    /* Check if the operation has been successful */
    if (capSenseApiResult== CY_RET_SUCCESS)
    {
        Task_DebugPrintf("Success  : Touch - CapSense initialization", 0u);
        CapSense_SetGestureTimestamp(0u);
    }
    else
    {
        Task_DebugPrintf("Failure! : Touch - CapSense initialization, Error Code:"
                    , capSenseApiResult);
    }
    
    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Check if CapSense is busy with a previous scan */
        if(CapSense_IsBusy() == CapSense_NOT_BUSY)
        {
            /* Initialize the local variable that stores the touch
               information */
            currentTouchData  = NO_TOUCH;
            
            /* Process all widgets and read touch information */
            CapSense_ProcessAllWidgets();
            
            /* Button0 is active */
            if(CapSense_IsWidgetActive (CapSense_BUTTON0_WDGT_ID))
            {
                if(previousTouchData != BUTTON0_TOUCHED)
                {
                    currentTouchData = BUTTON0_TOUCHED;
                }
            }
            /* Button1 is active */
            else if (CapSense_IsWidgetActive (CapSense_BUTTON1_WDGT_ID))
            {
                if(previousTouchData != BUTTON1_TOUCHED)
                {
                    currentTouchData = BUTTON1_TOUCHED;
                }
            }
            else
            {
                /* Read gesture information from slider */
                sliderGesture = CapSense_DecodeWidgetGestures
                                (CapSense_LINEARSLIDER0_WDGT_ID);
                                
                /* Check if left or right flick has been detected */    
                switch(sliderGesture)
                {    
                    /* Left flick */
                    case CapSense_ONE_FINGER_FLICK_LEFT:
                        currentTouchData = SLIDER_FLICK_LEFT;
                        break;
                    /* Right flick */
                    case CapSense_ONE_FINGER_FLICK_RIGHT:
                        currentTouchData = SLIDER_FLICK_RIGHT;
                        break;                        
                    /* No gesture */
                    default:
                    break;   
                }
            }
            
            /* Start the next CapSense scan */
            CapSense_ScanAllWidgets();
            
            /* Check if there is any data to be sent */
            if(currentTouchData != NO_TOUCH)
            {
                previousTouchData = currentTouchData;
                /* Send the processed touch data */
                xQueueOverwrite(touchDataQ, &currentTouchData);
            }
        }
        
        /* Increment the gesture time stamp */
        gestureTimestamp += GESTURE_TIME_INCREMENT;
        CapSense_SetGestureTimestamp(gestureTimestamp);

        /* Block until next scan interval */
        vTaskDelay(TOUCH_SCAN_INTERVAL);
    }
}

/* [] END OF FILE */
