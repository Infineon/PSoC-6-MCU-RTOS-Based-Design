/******************************************************************************
* File Name: ble_task.c
*
* Version: 1.00
*
* Description: This file contains the task that handles custom BLE service
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
* This file contains the task that handles custom BLE proximity service
*******************************************************************************/

/* Header file includes */
#include "ble_task.h"
#include "proximity_task.h"
#include "status_led_task.h"
#include "ble_proximity_config.h"
#include "uart_debug.h"
#include "task.h"  

/*  These static functions are not available outside this file. 
    See the respective function definitions for more details */
void static BleControllerInterruptEventHandler(void);
void static RegisterGPIOInterruptHandler(void);
void static StackEventHandler(uint32_t eventType, void *eventParam);
void static DisconnectEventHandler(void);
void static AdvertisementEventHandler(void);
void static WriteRequestHandler
                    (cy_stc_ble_gatts_write_cmd_req_param_t *writeRequest);
void static SendBleNotification
                    (cy_ble_gatt_db_attr_handle_t charHandle, uint8_t* value);

/* Variable that stores the BLE connection parameters */
cy_stc_ble_conn_handle_t static connectionHandle;                

/* Queue handle used for commands to BLE Task */
QueueHandle_t bleCommandQ; 
                    
/* Variables used to store the previous proximity command */
proximity_command_t static prevProximityCommand = SEND_NONE;                    
                    
/*******************************************************************************
* Function Name: void Task_Ble(void *pvParameters)
********************************************************************************
* Summary:
*  Task that processes the BLE state and events, and then commands other tasks 
*  to take an action based on the current BLE state and data received over BLE  
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/
void Task_Ble(void *pvParameters)
{
    /* Variable that stores BLE commands that need to be processed */
    ble_command_t bleCommand;
    
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t bleApiResult;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Remove warning for unused parameter */
    (void)pvParameters;
  
    /* Start the BLE component and register the stack event handler */
    bleApiResult = Cy_BLE_Start(StackEventHandler);
    
    /* Check if the operation was successful */
    if(bleApiResult == CY_BLE_SUCCESS)
    {
        Task_DebugPrintf("Success  : BLE - Stack initialization", 0u);
    
        /* Register the BLE controller (Cortex-M0+) interrupt event handler  */
        Cy_BLE_RegisterAppHostCallback(BleControllerInterruptEventHandler);
        /* Register GPIO interrupt handler that restarts advertisement */
        RegisterGPIOInterruptHandler();
        /* Process BLE events to turn the stack on */
        Cy_BLE_ProcessEvents();
    }
    else
    {
        Task_DebugPrintf("Failure! : BLE  - Stack initialization. Error Code:",
                         bleApiResult);
    }

    /* Repeatedly running part of the task */
    for(;;)
    {       
        /* Block until a BLE command has been received over bleCommandQ */
        rtosApiResult = xQueueReceive(bleCommandQ, &bleCommand,
                                      portMAX_DELAY);
        /* Command has been received from bleCommandQ */
        if(rtosApiResult == pdTRUE)
        {
            /* Take an action based on the command received */
            switch (bleCommand.command)
            {         
            /*~~~~~~~~~~~~~~ Command to process BLE events ~~~~~~~~~~~~~~~~~~~*/
                case PROCESS_BLE_EVENTS:
                    /* Process event callback to handle BLE events. The events 
                    and associated actions taken by this application are inside 
                    the 'StackEventHandler' routine. Note that Cortex M4 only 
                    handles the BLE host portion of the stack, while Cortex M0+ 
                    handles the BLE controller portion */
                    Cy_BLE_ProcessEvents();
                    break;
                    
            /*~~~~~~~~~~~~~~ Command to send Proximity Notification ~~~~~~~~~~~~~*/            
                case SEND_PROXIMITY_NOTIFICATION:
                    /* Send data over BLE slider notification */
                    SendBleNotification(PROXIMITY_CHAR_HANDLE, 
                                        &bleCommand.proximityData);
                    break;    
                    
            /*~~~~~~~~~~~~~~ Command to process GPIO interrupt ~~~~~~~~~~~~~~~*/               
                case HANDLE_GPIO_INTERRUPT:
                    /* Make sure that BLE is neither connected nor advertising
                       already */
                    if((Cy_BLE_GetConnectionState(connectionHandle) 
                       != CY_BLE_CONN_STATE_CONNECTED)&& 
                       (Cy_BLE_GetAdvertisementState() == 
                        CY_BLE_ADV_STATE_STOPPED))
                    {                        
                        /* Start Advertisement and enter discoverable mode */
                		bleApiResult = Cy_BLE_GAPP_StartAdvertisement(
                                       CY_BLE_ADVERTISING_FAST,
                                       CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX); 
                        
                        /* Check if the operation was successful */
                        if(bleApiResult == CY_BLE_SUCCESS )
                        {
                            Cy_BLE_ProcessEvents();
                            Task_DebugPrintf("Success  : BLE - Advertisement"\
                                             "API", 0u);
                        }
                        else
                        {
                            Task_DebugPrintf("Failure! : BLE - Advertisement"\
                                             "API. Error Code:" , bleApiResult);
                        }
                    }
                    break;    
                        
                /*~~~~~~~~~~~~~~ Invalid Command Received ~~~~~~~~~~~~~~~~~~~~*/ 
                default:
                    Task_DebugPrintf("Error!   : BLE - Invalid Task command "\
                                     "received. Error Code:", 
                                      bleCommand.command);
                    break;
            }          
        }
        /* Task has timed out and received no commands during an interval of 
            portMAXDELAY ticks */
        else
        {               
            Task_DebugPrintf("Warning! : BLE - Task Timed out ", 0u);
        }  
    }
}

/*******************************************************************************
* Function Name: void static StackEventHandler(uint32_t event, void *eventParam)
********************************************************************************
* Summary:
*  Call back event function to handle various events from the BLE stack. Note 
*  that Cortex M4 only handles the BLE host portion of the stack, while 
*  Cortex M0+ handles the BLE controller portion. 
*
* Parameters:
*  event        :	BLE event occurred
*  eventParam   :	Pointer to the value of event specific parameters
*
* Return:
*  void
*
*******************************************************************************/
void static StackEventHandler(uint32_t eventType, void *eventParam)
{
    /* Variable used to store data received over write request */
    cy_stc_ble_gatts_write_cmd_req_param_t* writeReqParameter;
    
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t bleApiResult;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Take an action based on the current event */
    switch ((cy_en_ble_event_t)eventType)
    {
        /*~~~~~~~~~~~~~~~~~~~~~~ GENERAL  EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
        {
            Task_DebugPrintf("Info     : BLE - Stack on", 0u);
            
            /* Start Advertisement and enter discoverable mode */
    		bleApiResult = Cy_BLE_GAPP_StartAdvertisement(
                            CY_BLE_ADVERTISING_FAST,
                            CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX); 

            if(bleApiResult == CY_BLE_SUCCESS )
            {
                Task_DebugPrintf("Success  : BLE - Advertisement API", 0u);
            }
            else
            {
                Task_DebugPrintf("Failure! : BLE - Advertisement API, Error code:"
                            , bleApiResult);
            }  
            break;
        }
        
        /* This event is received when there is a timeout */
        case CY_BLE_EVT_TIMEOUT:
        {    
            Task_DebugPrintf("Info     : BLE - Event timeout", 0u);
            break;
        }
        
        /* This event indicates that some internal HW error has occurred */    
	    case CY_BLE_EVT_HARDWARE_ERROR:
        {    
            Task_DebugPrintf("Error!   : BLE - Internal hardware error", 0u);
		    break;
        }    
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~ GATT EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        
        /* This event is received when device is connected over GATT level */    
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {    
            /* Update attribute handle on GATT Connection */
            connectionHandle = *(cy_stc_ble_conn_handle_t *) eventParam;

            Task_DebugPrintf("Info     : BLE - GATT connection established", 0u);
            break;
        }
        
        /* This event is received when device is disconnected */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            Task_DebugPrintf("Info     : BLE - GATT disconnection occurred ", 0u);
            break;
        }
        
        /* This event is received when Central device sends a Write command
           on an Attribute */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
        {    
            /* Read the write request parameter */
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t*) 
                                eventParam;

            /* When this event is triggered, the peripheral has received a 
               write command on the custom  characteristic. Check if command
               fits any of the custom attributes and update the flag for
               sending notifications by the respective service */
            WriteRequestHandler(writeReqParameter);
            break;
        }
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~ GAP EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        
        /* This event indicates peripheral device has started/stopped
           advertising */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        {    
                Task_DebugPrintf("Info     : BLE - Advertisement start/stop event"
                            ,0u);
                AdvertisementEventHandler();
            break;                
        }
        
        /* This event is generated at the GAP Peripheral end after connection 
           is completed with peer Central device */
        case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
        {    
            Task_DebugPrintf("Info     : BLE - GAP device connected", 0u); 
            /* Toggle Orange LED periodically to indicate that BLE is in a 
               connected state */
			status_led_data_t statusLedData = 
                        			{.orangeLed = LED_TOGGLE_EN,
                        			 .redLed    = LED_TURN_OFF
                        			};

            rtosApiResult = xQueueSend(statusLedDataQ, &statusLedData,0u);
            
            /* Check if the operation has been successful */
            if(rtosApiResult != pdTRUE)
            {
                Task_DebugPrintf("Failure! : BLE - Sending data to Status LED"\
                                 "queue", 0u);   
            } 
            break;
        }
        
        /* This event is generated when disconnected from remote device or 
           failed to establish connection */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {        
                Task_DebugPrintf("Info     : BLE - GAP device disconnected", 0u);
                DisconnectEventHandler();
            break;            
        }
        
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~ OTHER EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
        
         /* See the data-type cy_en_ble_event_t to understand the event 
			occurred */
        default:
        {
            Task_DebugPrintf("Info     : BLE - Event code: ", eventType);
            break;
        }    
    }
} 

/*******************************************************************************
* Function Name: void static BleControllerInterruptEventHandler (void)
********************************************************************************
* Summary:
*  Call back event function to handle interrupts from BLE Controller
*  (Cortex M0+)
*
* Parameters:
*  None
*
* Return:
*  void
*
*******************************************************************************/
void static BleControllerInterruptEventHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    
    /* Send command to process BLE events  */
    ble_command_t bleCommand = {.command = PROCESS_BLE_EVENTS};
    
    xQueueSendFromISR(bleCommandQ, &bleCommand,&xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken );
}

/*******************************************************************************
* Function Name: void isrGPIO(void)
********************************************************************************
*
* Summary:
*  Interrupt service routine for the port interrupt triggered from isr_gpio.
*    
*  Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void isrGPIO(void)
{   
    /* Clear the GPIO interrupt*/
    Cy_GPIO_ClearInterrupt(Pin_Advertise_PORT,Pin_Advertise_NUM);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);  
    
    /* Notify the Task_Ble to restart advertisement */
    BaseType_t xHigherPriorityTaskWoken;
    
    /* Send command to process BLE events  */
    ble_command_t bleCommand = {.command = HANDLE_GPIO_INTERRUPT};
    
    xQueueSendFromISR(bleCommandQ, &bleCommand,&xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken );
}

/*******************************************************************************
* Function Name: void static RegisterGPIOInterruptHandler(void)
********************************************************************************
* Summary:
*  Register GPIO interrupts from pin
*
* Parameters:
*  None
*
* Return:
*  void
*
*******************************************************************************/
void static RegisterGPIOInterruptHandler(void)
{
    /* Initialize and enable the GPIO interrupt assigned to CM4 */
    Cy_SysInt_Init(&isr_gpio_cfg,isrGPIO);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)isr_gpio_cfg.intrSrc);

}

/*******************************************************************************
* Function Name: void static AdvertisementEventHandler(void)
********************************************************************************
* Summary:
*  This functions handles advertisement start/stop events 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void static AdvertisementEventHandler(void)
{
    /* Variable that stores status LED data */
    status_led_data_t statusLedData;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Check if BLE is advertising */
    if(Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_ADVERTISING)
    {
        
        /* Turn on the Orange LED to indicate that BLE is advertising */
        statusLedData.orangeLed = LED_TURN_ON;
        statusLedData.redLed    = LED_TURN_OFF;
        rtosApiResult = xQueueSend(statusLedDataQ, &statusLedData,0u);
        
        /* Check if the operation has been successful */
        if(rtosApiResult != pdTRUE)
        {
            Task_DebugPrintf("Failure! : BLE - Sending data to Status LED"\
                             "queue",0u);   
        }
    }
    /* Check if the advertisement has timed out */
    else if(Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_STOPPED)
    {
        /* Turn off both status LEDs to indicate the idle mode */
        statusLedData.orangeLed = LED_TURN_OFF;
        statusLedData.redLed    = LED_TURN_OFF;
        rtosApiResult = xQueueSend(statusLedDataQ, &statusLedData,0u);
        
        /* Check if the operation has been successful */
        if(rtosApiResult != pdTRUE)
        {
            Task_DebugPrintf("Failure! : BLE - Sending data to Status LED"\
                             "queue",0u);   
        }
    }          
}

/*******************************************************************************
* Function Name: void static DisconnectEventHandler(void)
********************************************************************************
* Summary:
*  This functions handles the disconnect event 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void static DisconnectEventHandler(void)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Variable used to turn off the Orange LED and blink the Red LED once to 
       indicate disconnection*/
    status_led_data_t statusLedData = 
    {
       .orangeLed = LED_TURN_OFF,
       .redLed    = LED_BLINK_ONCE
    };    
    
    /* Local variable to store the current CCCD value */
    uint8_t cccdDefaultValue[CY_BLE_CCCD_LEN] = {(uint8_t)CY_BLE_CCCD_DEFAULT,
                                                 (uint8_t)CY_BLE_CCCD_DEFAULT};
        
    /* Local variable that stores handle value pair */
    cy_stc_ble_gatt_handle_value_pair_t  handleValuePair = 
    {
        .attrHandle = PROXIMITY_CCCD_HANDLE,
        .value.len  = CY_BLE_CCCD_LEN,
        .value.val  = cccdDefaultValue
    };

    /* Write to GATT database to disable slider notifications */
    Cy_BLE_GATTS_WriteAttributeValueLocal(&handleValuePair);
    
    /* Turn off the Orange LED and blink the Red LED once to 
       indicate disconnection*/
    rtosApiResult = xQueueSend(statusLedDataQ, &statusLedData,0u);
    if(rtosApiResult != pdTRUE)
    {
        Task_DebugPrintf("Failure! : BLE - Sending data to Status LED queue",
                          0u);   
    }

    /* Check if SEND_NONE was sent previously */
    if (prevProximityCommand != SEND_NONE)
    {   
        prevProximityCommand = SEND_NONE;

        /* Send the command to Proximity Task via proximityCommandQ */
        xQueueOverwrite(proximityCommandQ, &prevProximityCommand);        
    }
}

/*******************************************************************************
* Function Name: static void WriteRequestHandler 
*                       (cy_stc_ble_gatts_write_cmd_req_param_t *writeRequest)
********************************************************************************
* Summary:
*  This functions handles write requests
*
* Parameters:
*  cy_stc_ble_gatts_write_cmd_req_param_t: write request parameter
*
* Return:
*  void
*
*******************************************************************************/
static void WriteRequestHandler (cy_stc_ble_gatts_write_cmd_req_param_t 
                                 *writeRequest)
{
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t bleApiResult;

    /* Variable used to store proximity command */
    proximity_command_t proximityCommand;
    
    /* Flag that stores the notification statuses of proximity service */
    bool    proximityNotificationEnabled;
    
    /* Extract the data and write to GATT database */
    Cy_BLE_GATTS_WriteAttributeValuePeer(&connectionHandle,
                                         &writeRequest->handleValPair);
    
    /* Send the response to the write request received */
    bleApiResult = Cy_BLE_GATTS_WriteRsp(connectionHandle);
    if(bleApiResult  == CY_BLE_SUCCESS)
    {
        Cy_BLE_ProcessEvents();
    }
    else
    {        
        Task_DebugPrintf("Failure! : BLE - Sending write response. Error Code:", 
                          bleApiResult);
    }
    
    /* Check if the proximity service notifications are enabled */
    proximityNotificationEnabled = Cy_BLE_GATTS_IsNotificationEnabled(
                                &connectionHandle,
                                PROXIMITY_CHAR_HANDLE);
    
    /* Find out the command to be sent to the proximity module based on the 
       notification value  */
    if(proximityNotificationEnabled)
    {
        proximityCommand = SEND_PROXIMITY;
    }
    else
    {
        /* Don't send any data */
        proximityCommand = SEND_NONE;
    }
    
    /* Check for a change in the proximity command */
    if (proximityCommand != prevProximityCommand)
    {   
        /* Send the command to Proximity Task via proximityCommandQ */
        xQueueOverwrite(proximityCommandQ, &proximityCommand);
        
        /* Store the proximity command value for future comparisons */
        prevProximityCommand = proximityCommand;
    }
}

/*******************************************************************************
* Function Name: void static SendBleNotification 
*                      (cy_ble_gatt_db_attr_handle_t charHandle, uint8_t* value)                           
********************************************************************************
* Summary:
*  This function sends BLE notifications
*
* Parameters:
*  cy_ble_gatt_db_attr_handle_t :  Characteristic handle of the service
*  uint8_t*                     :  Pointer to the notification value 
*
* Return:
*  void
*
*******************************************************************************/
void static SendBleNotification (cy_ble_gatt_db_attr_handle_t charHandle,
                                 uint8_t* value)
{
    /* Flag used to check if the characteristics handle is valid */
    bool handleValid = true;
    
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t bleApiResult;
    
    /* Local variable that stores notification data parameters */
    cy_stc_ble_gatt_handle_value_pair_t  handleValuePair = 
    {
        .attrHandle = charHandle,
        .value.val = value
    };
    
    /* Find out the characteristics value size from the characteristics
       handle */
    switch (charHandle)
    {
        case PROXIMITY_CHAR_HANDLE:
            handleValuePair.value.len = PROXIMITY_DATA_LEN;
        break;
        default:
            handleValid = false;
        break;
    }
    
    /* The characteristics handle is valid */
    if(handleValid)
    {     
        /* Check if the BLE stack is free */
        if (Cy_BLE_GATT_GetBusyStatus(connectionHandle.attId) 
             == CY_BLE_STACK_STATE_FREE)
        {
            /* Send BLE notification */
            bleApiResult = Cy_BLE_GATTS_SendNotification(&connectionHandle,
                           &handleValuePair);
            
            /* Check if the operation has been successful */
            if(bleApiResult != CY_BLE_SUCCESS)
            {
                Task_DebugPrintf("Failure! : BLE - Sending notification. "\
                            "Error Code:", bleApiResult);
            }
        }
        /* Stack is busy, the current notification data will be dropped */ 
        else
        {
            Task_DebugPrintf("Info     : BLE - Stack busy to send notification", 0u);
        }
    }
    /* The characteristics handle is invalid */
    else
    {
        Task_DebugPrintf("Error!   : BLE - Invalid handle to send notification. "\
                    "Error Code:", charHandle);
    }
}

/* [] END OF FILE */
