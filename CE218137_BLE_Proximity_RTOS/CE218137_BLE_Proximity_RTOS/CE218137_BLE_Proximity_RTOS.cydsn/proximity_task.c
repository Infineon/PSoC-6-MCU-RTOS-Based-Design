/******************************************************************************
* File Name: proximity_task.c
*
* Version: 1.00
*
* Description: This file contains the task that handles proximity sensing
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
/*******************************************************************************
* This file contains the task that handles proximity sensing 
*******************************************************************************/

/* Header file includes */
#include "proximity_task.h"
#include "ble_task.h"
#include "uart_debug.h"
#include "task.h" 
#include "timers.h"

/* Scanning interval of 33ms is used when repeatedly scanning proximity widget 
   to get a scan rate of ~30 scans per second */
#define PROXIMITY_SCAN_INTERVAL  (pdMS_TO_TICKS(33u))
/* Idle interval is used for when no proximity data is required */
#define PROXIMITY_IDLE_INTERVAL  (portMAX_DELAY)

/* Queue handles used for proximity commands and data */
QueueHandle_t proximityCommandQ;

/* Timer handles used to control proximity scanning interval */
TimerHandle_t xTimer_Proximity;

/* Functions that start and control the timer */
void static ProximityTimerStart(void);
void static ProximityTimerUpdate(TickType_t period);

/*******************************************************************************
* Function Name: void Task_Proximity(void *pvParameters)   
********************************************************************************
* Summary:
*  Task that reads and processes proximity data    
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/
void Task_Proximity(void *pvParameters)    
{
    /* Variable that stores CapSense API results */
    cy_status capSenseApiResult;
    
    /* Variable that stores command values received  */
    proximity_command_t proximityCommand;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
       
    /* Flags that indicate if proximity data need to be sent */
    bool    sendProximityData = false;

    /* Variables that store the previous and the current proximity data */
    uint8_t  currentProxData = 0u;
    uint8_t  prevProxData = UINT8_MAX;
    
    /* Flags that indicate if the proximity data has been updated */
    bool proximityDataUpdated;

    /* Start the timer that controls the processing interval */
    ProximityTimerStart();

    /* Remove warning for unused parameter */
    (void)pvParameters ;

    /* Start the CapSense component and initialize the baselines */
    capSenseApiResult = CapSense_Start();
    
    /* Check if the operation has been successful */
    if (capSenseApiResult== CY_RET_SUCCESS)
    {
        Task_DebugPrintf("Success  : Proximity - CapSense initialization", 0u);
    }
    else
    {
        Task_DebugPrintf("Failure! : Proximity - CapSense initialization, Error Code:"
                    , capSenseApiResult);
    }
    
    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a command has been received over proximityCommandQ */
        rtosApiResult = xQueueReceive(proximityCommandQ, &proximityCommand,
                        portMAX_DELAY);
         /* Command has been received from proximityCommandQ */ 
        if(rtosApiResult == pdTRUE)
        {   
            /* Take an action based on the command received */
            switch (proximityCommand)
            {
                /* Both the slider and the button data need to be sent */
                case SEND_PROXIMITY:
                    sendProximityData = true;
                    /* Enable periodic scan */
                    ProximityTimerUpdate(PROXIMITY_SCAN_INTERVAL);
                    break;
                 
                /* No proximity data need to be sent */
                case SEND_NONE:
                    sendProximityData = false;
                    /* Disable periodic scan */
                    ProximityTimerUpdate(PROXIMITY_IDLE_INTERVAL);
                    break;
                /* Process proximity data from proximity sensor */
                case PROXIMITY_TIMER_EXPIRED:
                    proximityDataUpdated = false;
                    
                    /* Do this only when CapSense isn't busy with an 
                       ongoing scan */
                    if (CapSense_IsBusy() == CapSense_NOT_BUSY)
                    {
                        /* Process data from the widget */
                		CapSense_ProcessAllWidgets();
                        
                        /* Check if the signal is greater than the finger threshold */
                        if(CapSense_PROXIMITY0_SNS0_DIFF_VALUE > 
                           CapSense_PROXIMITY0_FINGER_TH_VALUE)
                        {
                            /*If proximity value is not within the range, cap the value */
                    		if((CapSense_PROXIMITY0_SNS0_DIFF_VALUE -
                                CapSense_PROXIMITY0_FINGER_TH_VALUE) 
                                <= UINT8_MAX)
                    		{
                                /* Store the proximity data */
                    	         currentProxData =  (uint8_t)(CapSense_PROXIMITY0_SNS0_DIFF_VALUE - 
                                                        CapSense_PROXIMITY0_FINGER_TH_VALUE);
                    		}
                            /* Store the maximum value otherwise */
                            else
                            {
                                currentProxData = (uint8_t)UINT8_MAX;
                            }
                        }
                        /* Clear the value if the signal is less than the finger threshold */
                        else
                        {
                           currentProxData = 0u; 
                        }
                                            
                        /* Start the next CapSense scan */
                        CapSense_ScanAllWidgets();
                    }
            
                    /* Check if slider data needs to be sent and the proximity  
                       position on the slider has changed */
                    if ((currentProxData != prevProxData)
                        && sendProximityData)
                    {
                        prevProxData = currentProxData;
                        
                        proximityDataUpdated = true;
                    }
                   
                    /* Check if there is any data to be sent, then create the  
                       data header based on the type of data that has been
                       updated */            
                    if(proximityDataUpdated)
                    {
                        ble_command_t bleCommandProximity =
                        {   
                            .command = SEND_PROXIMITY_NOTIFICATION,
                        };
                        
                        bleCommandProximity.proximityData = currentProxData;
                        
                        /* Send the processed proximity data */
                        xQueueSend(bleCommandQ, &bleCommandProximity, 0u);
                    }
                    
                    /* Periodic processing request received when no proximity data 
                       needs to be sent */
                    if(!sendProximityData)
                    {
                        Task_DebugPrintf("Error!   : Proximity - periodic process "\
                                    " request during invalid state" , 0u);  
                    }
                    break;
                        
                        
                /* Invalid task notification value received */    
                default:
                    Task_DebugPrintf("Error!   : Proximity - Invalid command received ."\
                                "Error Code:", proximityCommand);
                    break;
            }
        }            
        /* Task has timed out and received no commands during an interval of 
        portMAXDELAY ticks */
        else
        {
            Task_DebugPrintf("Warning! : Proximity - Task Timed out ", 0u);   
        }
    }
}

/*******************************************************************************
* Function Name: void static ProximityTimerCallback(TimerHandle_t xTimer)                          
********************************************************************************
* Summary:
*  This function is called when the proximity timer expires
*
* Parameters:
*  TimerHandle_t xTimer :  Current timer value (unused)
*
* Return:
*  void
*
*******************************************************************************/
void static ProximityTimerCallback(TimerHandle_t xTimer)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Remove warning for unused parameter */
    (void)xTimer;
    
    /* Send command to process proximity */
    proximity_command_t proximityCommand = PROXIMITY_TIMER_EXPIRED;
    rtosApiResult = xQueueSend(proximityCommandQ, &proximityCommand,0u);
    
    /* Check if the operation has been successful */
    if(rtosApiResult != pdTRUE)
    {
        Task_DebugPrintf("Failure! : Proximity  - Sending data to proximity queue", 0u);    
    }
}

/*******************************************************************************
* Function Name: void static ProximityTimerStart(void)                  
********************************************************************************
* Summary:
*  This function starts the timer that provides timing to periodic
*  proximity processing
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void static ProximityTimerStart(void)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Create an RTOS timer */
    xTimer_Proximity =  xTimerCreate ("Proximity Timer", PROXIMITY_IDLE_INTERVAL, pdTRUE,  
                                  NULL, ProximityTimerCallback);
    
    /* Make sure that timer handle is valid */
    if (xTimer_Proximity != NULL)
    {
        /* Start the timer */
        rtosApiResult = xTimerStart(xTimer_Proximity,0u);
        
        /* Check if the operation has been successful */
        if(rtosApiResult != pdPASS)
        {
            Task_DebugPrintf("Failure! : Proximity  - Timer initialization", 0u);    
        }
    }
    else
    {
        Task_DebugPrintf("Failure! : Proximity  - Timer creation", 0u); 
    }
}

/*******************************************************************************
* Function Name: void static ProximityTimerUpdate(TickType_t period))                 
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
void static ProximityTimerUpdate(TickType_t period)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Change the timer period */
    rtosApiResult = xTimerChangePeriod(xTimer_Proximity, period, 0u);

    /* Check if the operation has been successful */
    if(rtosApiResult != pdPASS)
    {
        Task_DebugPrintf("Failure! : Proximity - Timer update ", 0u);   
    }
}

/* [] END OF FILE */
