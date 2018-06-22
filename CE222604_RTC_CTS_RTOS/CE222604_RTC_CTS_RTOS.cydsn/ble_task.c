/******************************************************************************
* File Name: ble_task.c
*
* Version: 1.00
*
* Description: This file contains the task that handles custom BLE services
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
* This file contains the task that handles custom BLE services, which includes 
* the Current Time Service
*******************************************************************************/

/* Header file includes */
#include "ble_task.h"
#include "status_led_task.h"
#include "rtc_task.h"
#include "uart_debug.h"
#include "task.h"  

/* Shift by 8 to move a data to the higher byte */
#define SHIFT_TO_HIGHER_BYTE    (8u)

/* These static functions are used by the BLE Task. These are not available 
   outside this file. See the respective function definitions for more 
   details */
void static CallBackCts(uint32 event, void *eventParam);
void static BleControllerInterruptEventHandler(void);
void static RegisterGPIOInterruptHandler(void);
void static StackEventHandler(uint32_t eventType, void *eventParam);
void static ProcessBleEvents(void);
void static DisconnectEventHandler(void);
void static AdvertisementEventHandler(void);
bool static IsDeviceInBondList(uint8_t);
void static IsrGPIO(void);
/* Variable that stores the BLE connection parameters */
cy_stc_ble_conn_handle_t static connectionHandle;  
                    
/* Queue handle used for commands to BLE Task */
QueueHandle_t bleCommandQ; 

/* Structures that store CTS data */
time_and_date_t                         currentTimeAndDate; 

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
*  None
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
        /* Registers the CTS specific callback handler */
        Cy_BLE_CTS_RegisterAttrCallback(CallBackCts);
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
                    ProcessBleEvents();
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
                        
                /*~~~~~~~~~~~~~~ Invalid Command Recieved ~~~~~~~~~~~~~~~~~~~~*/ 
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
* Function Name: void static ProcessBleEvents(void)
********************************************************************************
* Summary:
*  Function that processes BLE events and states
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void static ProcessBleEvents(void)
{    
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t bleApiResult;

    /* Process event callback to handle BLE events. The events and associated
       actions taken by this application are inside the 'StackEventHandler' 
       routine. Note that Cortex M4 only handles the BLE host portion of the 
       stack, while Cortex M0+ handles the BLE controller portion */
    Cy_BLE_ProcessEvents();

    /* Store bonding data to flash only when all debug information has been sent */    
    if((cy_ble_pendingFlashWrite != 0u) && (Cy_BLE_GetNumOfActiveConn() == 0u))
    {   
        do
        {
            bleApiResult = Cy_BLE_StoreBondingData();
            
        } while((bleApiResult != CY_BLE_SUCCESS));
        Task_DebugPrintf("Info     : Storing bonding data", 0u);
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
*  None
*
*******************************************************************************/
void static StackEventHandler(uint32_t eventType, void *eventParam)
{      
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t bleApiResult;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Variable used to store security key information*/
    static cy_stc_ble_gap_sec_key_info_t keyInfo =
    {
        .localKeysFlag    = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST | 
                            CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST | 
                            CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST,
        .exchangeKeysFlag = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST | 
                            CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST | 
                            CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST|
                            CY_BLE_GAP_SMP_RESP_ENC_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_IRK_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_CSRK_KEY_DIST,
    };
    
    /* Take an action based on the current event */
    switch(eventType)
    {
        /*~~~~~~~~~~~~~~~~~~~~~~ GENERAL  EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
        {
            Task_DebugPrintf("Info     : BLE - Stack on", 0u);
            
            /* Generate the security keys*/
            bleApiResult = Cy_BLE_GAP_GenerateKeys(&keyInfo);
            if(bleApiResult == CY_BLE_SUCCESS )
            {
                Task_DebugPrintf("Success  : BLE - Generate Keys API", 0u);
            }
            else
            {
                Task_DebugPrintf("Failure! : BLE - Generate Keys API, "\
                                 "Error code:", bleApiResult);
            }
            
            /* Start Advertisement and enter discoverable mode */
    		bleApiResult = Cy_BLE_GAPP_StartAdvertisement(
                            CY_BLE_ADVERTISING_FAST,
                            CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            
            /* Check if the operation was successful */
            if(bleApiResult == CY_BLE_SUCCESS )
            {
                Cy_BLE_ProcessEvents();
                Task_DebugPrintf("Success  : BLE - Advertisement API", 0u);
            }
            else
            {
                Task_DebugPrintf("Failure! : BLE - Advertisement API, "\
                                 "Error code:", bleApiResult);
            }  
            break;
        }
        /* This event is received when set device address command completed */
        case CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE:
        {
            Task_DebugPrintf("Info     : BLE - Set device address command completed", 0u);
            
            /* Reads the BD device address from BLE Controller's memory */
            bleApiResult = Cy_BLE_GAP_GetBdAddress();
            
            if(bleApiResult != CY_BLE_SUCCESS)
            {   
                Task_DebugPrintf("Failed!     : BLE - Cy_BLE_GAP_GetBdAddress API", 0u);
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
        /* This event is received when the discovery of a remote device completed */
        case CY_BLE_EVT_GATTC_DISCOVERY_COMPLETE:
        {    
            Task_DebugPrintf("Info     : BLE - discovery of a remote device " \
                             "completed", 0u);
            
            /* Get index of discovery structure */
            uint32_t discIdx = 
                Cy_BLE_GetDiscoveryIdx(*(cy_stc_ble_conn_handle_t *)eventParam);
            
            if(cy_ble_serverInfo[discIdx][CY_BLE_SRVI_CTS].range.startHandle != 0u)
            {
                /* Enable Notification for Current Time Characteristic. */
                uint16_t timeCCCD = CY_BLE_CCCD_NOTIFICATION;
                
                Cy_BLE_CTSC_SetCharacteristicDescriptor
                    (connectionHandle,
                     CY_BLE_CTS_CURRENT_TIME,
                     CY_BLE_CTS_CURRENT_TIME_CCCD,
                     CY_BLE_CCCD_LEN,
                     (uint8_t *)&timeCCCD);
            }
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
            
            bool deviceInBondlist = IsDeviceInBondList((*(cy_stc_ble_gap_connected_param_t *)eventParam).bdHandle);
                        
            if(deviceInBondlist == false)
            {
                /* Fetch the security keys */
                keyInfo.SecKeyParam.bdHandle = 
                    (*(cy_stc_ble_gap_connected_param_t *)eventParam).bdHandle;
                
                /* Set the security keys that are to be exchanged with a peer 
                   device during key exchange stage of the authentication procedure
                   and sets it in the BLE Stack */ 
                Cy_BLE_GAP_SetSecurityKeys(&keyInfo);
            }
            
            /* Send authentication request to peer device */
            cy_ble_configPtr-> authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].bdHandle 
                = connectionHandle.bdHandle;
            Cy_BLE_GAP_AuthReq(&cy_ble_configPtr->
                                authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX]);
            
            /* Toggle Orange LED periodically to indicate that BLE is in a 
               connected state */
			status_led_data_t statusLedData = 
                        			{
                                        .orangeLed = LED_TOGGLE_EN,
                        			    .redLed    = LED_TURN_OFF
                        			};

            rtosApiResult = xQueueSend(statusLedDataQ, &statusLedData, 0u);
            
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
        /* This event is generated when key generation complete */
        case CY_BLE_EVT_GAP_KEYS_GEN_COMPLETE:
        {    
            Task_DebugPrintf("Info     : BLE - Security Keys Generated", 0u);
            
            /* Set the device's identity address in the BLE Stack */
            keyInfo.SecKeyParam = (*(cy_stc_ble_gap_sec_key_param_t *)eventParam);
            Cy_BLE_GAP_SetIdAddress(&cy_ble_deviceAddress);
            break;
        }   
        /* This event is received when the authentication requested */
        case CY_BLE_EVT_GAP_AUTH_REQ:
        {
            Task_DebugPrintf("Info     : BLE - Authentication Req event", 0u);
            
            if(cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].security == 
                (CY_BLE_GAP_SEC_MODE_1 | CY_BLE_GAP_SEC_LEVEL_1))
            {
                cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].authErr = 
                    CY_BLE_GAP_AUTH_ERROR_PAIRING_NOT_SUPPORTED;
            }    
            
            cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].bdHandle = 
                ((cy_stc_ble_gap_auth_info_t *)eventParam)->bdHandle;
            
            /* Reply to the authentication request */
            bleApiResult = Cy_BLE_GAPP_AuthReqReply(
                    &cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX]);            
            if(bleApiResult != CY_BLE_SUCCESS)
            {
                Task_DebugPrintf("Failure! : BLE - Authentication reply API", \
                    bleApiResult);
            }
            break;
        }   
        /* This event is generated when GAP authentication failed */
        case CY_BLE_EVT_GAP_AUTH_FAILED:
        {    
            Task_DebugPrintf("Failure! : BLE - Authentication Failed"
                            , *(cy_en_ble_gap_auth_failed_reason_t*)eventParam);
            break;
        }   
        /* This event is received when the authenticatin completed */
        case CY_BLE_EVT_GAP_AUTH_COMPLETE:
        {
            Task_DebugPrintf("Info     : BLE - GAP authentication completed", 0u);
            
            /* Start the discovery if the authentication has been completed */
            Cy_BLE_GATTC_StartDiscovery(connectionHandle);
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
* Function Name: void IsrGPIO(void)
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
void IsrGPIO(void)
{   
    /* Clear the GPIO interrupt*/
    Cy_GPIO_ClearInterrupt(Pin_Advertise_PORT, Pin_Advertise_NUM);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);  
    
    /* Notify the Task_Ble to restart advertisement */
    BaseType_t xHigherPriorityTaskWoken;
    
    /* Send command to process BLE events  */
    ble_command_t bleCommand = {.command = HANDLE_GPIO_INTERRUPT};
    
    xQueueSendFromISR(bleCommandQ, &bleCommand,&xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken );
}

/*******************************************************************************
* Function Name: void static AdvertisementEventHandler(void)
********************************************************************************
* Summary:
*  This function handles advertisement start/stop events 
*
* Parameters:
*  None
*
* Return:
*  None
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
*  This function handles the disconnect event 
*
* Parameters:
*  None
*
* Return:
*  None
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
       .redLed    = LED_TURN_ON
    };    
    
    
    rtosApiResult = xQueueSend(statusLedDataQ, &statusLedData,0u);
    if(rtosApiResult != pdTRUE)
    {
        Task_DebugPrintf("Failure! : BLE - Sending data to Status LED queue",
                          0u);   
    }
}

/*******************************************************************************
* Function Name: void CallBackCts(uint32 event, void *eventParam)
********************************************************************************
*
* Summary:
*  This is an event callback function to receive events from the BLE Component,
*  which are specific to Current Time Service.
*
* Parameters:  
*  event:       Event for Current Time Service.
*  eventParams: Event parameter for Current Time Service.
*
* Return: 
*  None
*
*******************************************************************************/
void static CallBackCts(uint32 event, void *eventParam)
{
    /* Variables that store CTS parameters */
    cy_stc_ble_cts_char_value_t *timeAttribute;
    static rtc_command_and_data_t rtcUpdateTime;
    
    /* This is a CTS specific event triggered by the BLE component */
    switch(event)
    {
        /* Event to read CTS characteristics */
        case CY_BLE_EVT_CTSC_READ_CHAR_RESPONSE:
            
            Task_DebugPrintf("info     : BLE - Read CTS characteristics", 0u);
            /* Read the event parameter */
            timeAttribute = (cy_stc_ble_cts_char_value_t *) eventParam;
            
            /* Copy the received current time from the time server to local data structure
               and then write to the RTC */
            if(timeAttribute->charIndex == CY_BLE_CTS_CURRENT_TIME)
            {
                memcpy(&currentTimeAndDate, timeAttribute->value->val, timeAttribute->value->len);
                rtcUpdateTime.rtcDateTime.date   = currentTimeAndDate.day;
                rtcUpdateTime.rtcDateTime.month  = currentTimeAndDate.month;
                rtcUpdateTime.rtcDateTime.year   = ((uint32_t)currentTimeAndDate.yearHigh << SHIFT_TO_HIGHER_BYTE | 
                        currentTimeAndDate.yearLow) - CY_RTC_TWO_THOUSAND_YEARS;
                rtcUpdateTime.rtcDateTime.sec    = currentTimeAndDate.seconds;
                rtcUpdateTime.rtcDateTime.min    = currentTimeAndDate.minutes;
                rtcUpdateTime.rtcDateTime.hour   = currentTimeAndDate.hours;
                
                rtcUpdateTime.command           = RTC_UPDATE_TIME;
                xQueueOverwrite(rtcCommandAndDataQ, &rtcUpdateTime);
            }
            break;
         
        /* Event to send Read request for Current Time characteristic to the Time Server */       
        case CY_BLE_EVT_CTSC_WRITE_DESCR_RESPONSE:
            
            Task_DebugPrintf("info     : BLE - Send Read request for CTS", 0u);
            Cy_BLE_CTSC_GetCharacteristicValue(connectionHandle, CY_BLE_CTS_CURRENT_TIME);
            break;

        default:
            break;
    }
}


/*******************************************************************************
* Function Name: IsDeviceInBondList()
********************************************************************************
* Summary:
*  This function returns true when bdHandle exists in bond list
*
* Parameters:  
*  bdHandle
*
* Return: 
*  bool 
*******************************************************************************/
bool IsDeviceInBondList(uint8_t bdHandle)
{
    /* Variable used to store number of devuces in bond list */
    uint8_t deviceCount = 0;
    
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t bleApiResult;
        
    /* Variable used to store bonded device information */
    cy_stc_ble_gap_peer_addr_info_t bondedDeviceInfo[CY_BLE_GAP_MAX_BONDED_DEVICE];
    cy_stc_ble_gap_bonded_device_list_info_t bondedDeviceList =
    {
        .bdHandleAddrList = bondedDeviceInfo
    };
    
    bool deviceIsDetected = false;
    
    /* Find out whether the device has bonded information stored already or not */
    bleApiResult = Cy_BLE_GAP_GetBondList(&bondedDeviceList);
    
    if(bleApiResult != CY_BLE_SUCCESS)
    {
        Task_DebugPrintf("Error   : BLE - Failed to get Bond List", bleApiResult);
    }
    else
    {
        deviceCount = bondedDeviceList.noOfDevices;
        while(deviceCount)
        {
            deviceCount--;
            if(bdHandle == bondedDeviceList.bdHandleAddrList[deviceCount].bdHandle)
            {
                deviceIsDetected = 1u;
                break;
            }
        }
    }
    return(deviceIsDetected);
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
*  None
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
* Function Name: void static RegisterGPIOInterruptHandler(void)
********************************************************************************
* Summary:
*  Register GPIO interrupts from pin
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void static RegisterGPIOInterruptHandler(void)
{
    /* Initialize and enable the GPIO interrupt assigned to CM4 */
    Cy_SysInt_Init(&isr_gpio_cfg, IsrGPIO);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)isr_gpio_cfg.intrSrc);
}

/* [] END OF FILE */
