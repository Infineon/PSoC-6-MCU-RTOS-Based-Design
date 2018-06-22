/******************************************************************************
* File Name: ble_task.c
*
* Version: 1.00
*
* Description: This file contains the task and functions that handle BLE HTS 
*              service
*
* Related Document: CE218138_BLE_Thermometer_RTOS.pdf
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
* This file contains the task that handles BLE HTS service
*******************************************************************************/

/* Header file includes */
#include <math.h>
#include "ble_task.h"
#include "temperature_task.h"
#include "status_led_task.h"
#include "ble_thermometer_config.h"
#include "uart_debug.h"
#include "task.h"  
#include "timers.h"    

/* These static functions are used by the BLE Task. These are not available 
   outside this file. See the respective function definitions for more 
   details */
void static BleControllerInterruptEventHandler(void);
void static RegisterGPIOInterruptHandler(void);
void static StackEventHandler(uint32_t eventType, void *eventParam);
void static DisconnectEventHandler(void);
void static AdvertisementEventHandler(void);
void static SendTemperatureIndication (float temperature);
void CallBackHts(uint32 event, void *eventParam);

/* Variable that stores the BLE connection parameters */
cy_stc_ble_conn_handle_t static connectionHandle;  

/* Static flag used to request HTS indications */
bool static requestHtsIndication    =   false;

/* Queue handle used for commands to BLE Task */
QueueHandle_t bleCommandQ; 

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
        /* Register the Health Thermometer Service specific callback handler */
        Cy_BLE_HTS_RegisterAttrCallback(CallBackHts);
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
                    
            /*~~~~~~~~~~~~~~ Command to send temperature indication ~~~~~~~~~~*/            
                case SEND_TEMPERATURE_INDICATION:
                    /* Send temperature data over BLE HTS notification */
                    SendTemperatureIndication(bleCommand.temperatureData);
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
********************************************************************************
*  Following are the static functions used for BLE processing. These functions *
*  are not available outside this file.                                            *
********************************************************************************
*******************************************************************************/

/*******************************************************************************
* Function Name: void CallBackHts(uint32 event, void *eventParam)
********************************************************************************
*
* Summary:
*  This is an event callback function to receive events from the BLE Component,
*  which are specific to Health Thermometer Service.
*
* Parameters:  
*  event:       Event for Health Thermometer Service.
*  eventParams: Event parameter for Health Thermometer Service.
*
* Return: 
*  None
*
*******************************************************************************/
void CallBackHts(uint32 event, void *eventParam)
{
    /* Remove warning for unused parameter */
    (void)eventParam;

    temperature_command_t temperatureCommand;
    
    /* Handle the HTS events */
    switch(event)
    {
        /* This event is received when indication are enabled by the central */
        case CY_BLE_EVT_HTSS_INDICATION_ENABLED:
            /* Set the requestHtsIndication flag */
            requestHtsIndication = true;
             /* Request periodic temperature data */
            temperatureCommand = SEND_TEMPERATURE;
            xQueueOverwrite(temperatureCommandQ, &temperatureCommand);
            break;

        /* This event is received when indication are disabled by the central */
        case CY_BLE_EVT_HTSS_INDICATION_DISABLED:
            /* Reset the requestHtsIndication flag */
            requestHtsIndication = false;
            /* Request Temperature Task not to send any data */
            temperatureCommand  = SEND_NONE;
            xQueueOverwrite(temperatureCommandQ, &temperatureCommand);
            break;
        
        /* Do nothing for all other events */
        default:
            break;
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
        
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t bleApiResult;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    (void)eventParam;
    
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
                Task_DebugPrintf("Failure! : BLE - Advertisement API, "\
                                 "Error code:", bleApiResult);
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

            Task_DebugPrintf("Info     : BLE - GATT connection established",
                              0u);
            break;
        }
        
        /* This event is received when device is disconnected */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            Task_DebugPrintf("Info     : BLE - GATT disconnection occurred ",
                              0u);
            break;
        }
        
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~ GAP EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

        /* This event indicates peripheral device has started/stopped
           advertising */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        {    
            Task_DebugPrintf("Info     : BLE - Advertisement start/stop event",
                              0u);
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
    
    requestHtsIndication = false;
  
    /* Turn off the Orange LED and blink the Red LED once to 
       indicate disconnection*/
    status_led_data_t statusLedData = 
    {
       .orangeLed = LED_TURN_OFF,
       .redLed    = LED_BLINK_ONCE
    };
    
    /* Turn off the Orange LED and blink the Red LED once to 
       indicate disconnection*/
    rtosApiResult = xQueueSend(statusLedDataQ, &statusLedData,0u);
    if(rtosApiResult != pdTRUE)
    {
        Task_DebugPrintf("Failure! : BLE - Sending data to Status LED queue",
                          0u);   
    }

    /* Request Temperature Task not to send any data */
    temperature_command_t temperatureCommand = SEND_NONE;
    xQueueOverwrite(temperatureCommandQ, &temperatureCommand);
}

/*******************************************************************************
* Function Name: void static SendTemperatureIndication (float temperature)
********************************************************************************
* Summary:
*  This functions handles the HTS indication
*
* Parameters:
*  float temperature : temperature data 
*
* Return:
*  void
*
*******************************************************************************/
void static SendTemperatureIndication (float temperature)
{   
    /* Check if BLE is in connected state and the HTS indication is
       enabled */
	if((Cy_BLE_GetConnectionState(connectionHandle) 
        == CY_BLE_CONN_STATE_CONNECTED) && requestHtsIndication)
	{      
        /* Temporary array to hold Health Thermometer Characteristic 
           information */
        uint8 valueArray[HTS_CHARACTERISTIC_SIZE];
        temperature_data_t tempData;

        /* Convert from IEEE-754 single precision floating point format to
           IEEE-11073 FLOAT, which is mandated by the health thermometer
           characteristic */
        tempData.temeratureValue = (int32_t)(roundf(temperature*
                                            IEEE_11073_MANTISSA_SCALER));
        tempData.temperatureArray[IEEE_11073_EXPONENT_INDEX] = 
                                            IEEE_11073_EXPONENT_VALUE;         
        
        /* Read Health Thermometer Characteristic from GATT DB */
        if(CY_BLE_SUCCESS == Cy_BLE_HTSS_GetCharacteristicValue
                             (CY_BLE_HTS_TEMP_MEASURE,
                              HTS_CHARACTERISTIC_SIZE, valueArray))
        { 
            /* Update temperature value in the characteristic */
            memcpy(&valueArray[HTS_TEMPERATURE_DATA_INDEX],
                   tempData.temperatureArray, HTS_TEMPERATURE_DATA_SIZE);

            /* Send indication to the central */
            Cy_BLE_HTSS_SendIndication(connectionHandle, 
                                       CY_BLE_HTS_TEMP_MEASURE,
                                       HTS_CHARACTERISTIC_SIZE, valueArray);   
        }
    }
}

/* [] END OF FILE */
