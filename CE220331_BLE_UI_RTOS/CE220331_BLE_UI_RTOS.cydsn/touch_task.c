/******************************************************************************
* File Name: touch.c
*
* Version: 1.00
*
* Description: This file contains the task that handles touch sensing
*
* Related Document: CE220331_BLE_UI_RTOS.pdf
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
* This file contains the task that handles touch sensing 
*******************************************************************************/

/* Header file includes */
#include "touch_task.h"
#include "uart_debug.h"
#include "task.h" 
#include "timers.h"
#include "ble_task.h"

/* Scanning interval of 33ms is used when repeatedly scanning CapSense widgets 
   to get a scan rate of ~30 scans per second */
#define TOUCH_SCAN_INTERVAL  (pdMS_TO_TICKS(20u))
/* Idle interval is used for when no CapSense data is required */
#define TOUCH_IDLE_INTERVAL  (portMAX_DELAY)

/* Queue handles used for touch commands and data */
QueueHandle_t touchCommandQ;

/* Timer handles used to control touch scanning interval */
TimerHandle_t xTimer_Touch;

/* Functions that start and control the timer */
void static TouchTimerStart(void);
void static TouchTimerUpdate(TickType_t period);

/*******************************************************************************
* Function Name: void Task_Touch(void *pvParameters)   
********************************************************************************
* Summary:
*  Task that reads touch data from CapSense button and slider widgets   
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
    
    /* Variable that stores direct the commands received  */
    touch_command_t touchCommand;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
       
    /* Flags that indicate if slider / button data need to be sent */
    bool sendSliderData = false;
    bool sendButtonData = false;
    
    /* Variables that store the previous and the current touch data */
    ble_command_t bleCommandTouch =
    {   
        .touchData.dataButton0 = false,
        .touchData.dataButton1 = false,
        .touchData.dataSlider  = (uint8_t) CapSense_SLIDER_NO_TOUCH
    };
    bool    button0Data          = false;
    bool    button1Data          = false;
    uint8_t sliderData           = (uint8_t) CapSense_SLIDER_NO_TOUCH;

    /* Start the timer that controls the processing interval */
    TouchTimerStart();

    /* Remove warning for unused parameter */
    (void)pvParameters ;

    /* Start the CapSense component and initialize the baselines */
    capSenseApiResult = CapSense_Start();
    
    /* Check if the operation has been successful */
    if (capSenseApiResult== CY_RET_SUCCESS)
    {
        Task_DebugPrintf("Success  : Touch - CapSense initialization", 0u);
    }
    else
    {
        Task_DebugPrintf("Failure! : Touch - CapSense initialization, Error Code:"
                    , capSenseApiResult);
    }
    
    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a command has been received over touchCommandQ */
        rtosApiResult = xQueueReceive(touchCommandQ, &touchCommand,
                        portMAX_DELAY);
         /* Command has been received from touchCommandQ */ 
        if(rtosApiResult == pdTRUE)
        {   
            /* Take an action based on the command received */
            switch (touchCommand)
            {
                /* Both the slider and the button data need to be sent */
                case SEND_BOTH:
                    sendSliderData = true;
                    sendButtonData = true;
                    /* Enable periodic scan */
                    TouchTimerUpdate(TOUCH_SCAN_INTERVAL);
                    break;
                
                /* Only the slider data needs to be sent */
                case SEND_SLIDER:
                    sendSliderData = true;
                    sendButtonData = false;
                    /* Enable periodic scan */
                    TouchTimerUpdate(TOUCH_SCAN_INTERVAL);
                    break;
                
                /* Only the slider data needs to be sent */
                case SEND_BUTTON:
                    sendSliderData = false;
                    sendButtonData = true;
                    /* Enable periodic scan */
                    TouchTimerUpdate(TOUCH_SCAN_INTERVAL);
                    break;
                
                /* No touch data need to be sent */
                case SEND_NONE:
                    sendSliderData = false;
                    sendButtonData = false;
                    /* Disable periodic scan */
                    TouchTimerUpdate(TOUCH_IDLE_INTERVAL);
                    break;
                /* Process touch data from CapSense widgets */
                case TOUCH_TIMER_EXPIRED:

                    /* Do this only when CapSense isn't busy with an 
                       ongoing scan */
                    if (CapSense_IsBusy() == CapSense_NOT_BUSY)
                    {
                        /* Process data from the widgets */
                		CapSense_ProcessAllWidgets();
                        
                        /* Read touch data from the widgets */                                         
                        bleCommandTouch.touchData.dataButton0 
                            = (CapSense_IsWidgetActive
                               (CapSense_BUTTON0_WDGT_ID))? true : false;
                        bleCommandTouch.touchData.dataButton1
                            = (CapSense_IsWidgetActive
                               (CapSense_BUTTON1_WDGT_ID))? true : false;              
                        bleCommandTouch.touchData.dataSlider  
                            = (uint8_t)CapSense_GetCentroidPos
                                       (CapSense_LINEARSLIDER0_WDGT_ID);
                                            
                        /* Start the next CapSense scan */
                        CapSense_ScanAllWidgets();
                    }
                    
                    /* Check if button data needs to be sent and touch status of
                       any of the buttons have changed */
                    if (sendButtonData && 
                         ((bleCommandTouch.touchData.dataButton0
                          != button0Data)||
                         (bleCommandTouch.touchData.dataButton1 
                          != button1Data)))
                    {
                        /* Pack the button data, respective command and send 
                           to the queue */
                        button0Data = bleCommandTouch.touchData.dataButton0;
                        button1Data = bleCommandTouch.touchData.dataButton1;
                        bleCommandTouch.command = SEND_BUTTON_NOTIFICATION;
                        
                        xQueueSend(bleCommandQ, &bleCommandTouch,0u);   
                    }
                    
                    /* Check if slider data needs to be sent and the touch  
                       position on the slider has changed */
                    if ((bleCommandTouch.touchData.dataSlider != sliderData)
                        && sendSliderData)
                    {
                        /* Pack the slider data, respective command and send 
                           to the queue */
                        sliderData = bleCommandTouch.touchData.dataSlider;
                        bleCommandTouch.command = SEND_SLIDER_NOTIFICATION;
                        
                        xQueueSend(bleCommandQ, &bleCommandTouch,0u);
                    }

                    /* Periodic processing request received when no touch data 
                       needs to be sent */
                    if(!(sendSliderData || sendButtonData))
                    {
                        Task_DebugPrintf("Error!   : Touch - periodic process "\
                                    " request during invalid state" , 0u);  
                    }
                    break;
                        
                /* Invalid command received */    
                default:
                    Task_DebugPrintf("Error!   : Touch - Invalid command "\
                                     "received .Error Code:", touchCommand);
                    break;
            }
        }            
        /* Task has timed out and received no commands during an interval of 
        portMAXDELAY ticks */
        else
        {
            Task_DebugPrintf("Warning! : Touch - Task Timed out ", 0u);   
        }
    }
}

/*******************************************************************************
* Function Name: void static TouchTimerCallback(TimerHandle_t xTimer)                          
********************************************************************************
* Summary:
*  This function is called when the touch timer expires
*
* Parameters:
*  TimerHandle_t xTimer :  Current timer value (unused)
*
* Return:
*  void
*
*******************************************************************************/
void static TouchTimerCallback(TimerHandle_t xTimer)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Remove warning for unused parameter */
    (void)xTimer;
    
    /* Send command to process touch */
    touch_command_t touchCommand = TOUCH_TIMER_EXPIRED;
    rtosApiResult = xQueueSend(touchCommandQ, &touchCommand,0u);
    
    /* Check if the operation has been successful */
    if(rtosApiResult != pdTRUE)
    {
        Task_DebugPrintf("Failure! : Touch  - Sending data to touch queue", 0u);    
    }
}

/*******************************************************************************
* Function Name: void static TouchTimerStart(void)                  
********************************************************************************
* Summary:
*  This function starts the timer that provides timing to periodic
*  touch processing
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void static TouchTimerStart(void)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Create an RTOS timer */
    xTimer_Touch =  xTimerCreate ("Touch Timer", TOUCH_IDLE_INTERVAL, pdTRUE,  
                                  NULL, TouchTimerCallback);
    
    /* Make sure that timer handle is valid */
    if (xTimer_Touch != NULL)
    {
        /* Start the timer */
        rtosApiResult = xTimerStart(xTimer_Touch,0u);
        
        /* Check if the operation has been successful */
        if(rtosApiResult != pdPASS)
        {
            Task_DebugPrintf("Failure! : Touch  - Timer initialization", 0u);    
        }
    }
    else
    {
        Task_DebugPrintf("Failure! : Touch  - Timer creation", 0u); 
    }
}

/*******************************************************************************
* Function Name: void static TouchTimerUpdate(TickType_t period))                 
********************************************************************************
* Summary:
*  This function updates the timer period per the parameter
*
* Parameters:
*  TickType_t period :  Period of the timer in ticks
*
* Return:
*  void
*
*******************************************************************************/
void static TouchTimerUpdate(TickType_t period)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Change the timer period */
    rtosApiResult = xTimerChangePeriod(xTimer_Touch, period, 0u);

    /* Check if the operation has been successful */
    if(rtosApiResult != pdPASS)
    {
        Task_DebugPrintf("Failure! : Touch - Timer update ", 0u);   
    }
}

/* [] END OF FILE */
