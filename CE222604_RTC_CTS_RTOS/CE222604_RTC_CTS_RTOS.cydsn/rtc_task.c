/******************************************************************************
* File Name: task_rtc.c
*
* Version: 1.00
*
* Description:  
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

/* Header file includes */
#include "rtc_task.h"
#include "task.h"
#include "display_task.h"
#include "uart_debug.h"

/* Default time written to RTC at power-up or reset */
#define TIME_AT_RESET           (00u),   /* Seconds    */\
                                (00u),   /* Minutes    */\
                                (00u),   /* Hours      */\
                                (01u),   /* Date       */\
                                (01u),   /* Month      */\
                                (18u)    /* Year 20xx  */
                                
/* Shift by 8 to move a data to the higher byte */
#define SHIFT_TO_HIGHER_BYTE    (8u)

/* Wait time between two RTC APIs */ 
#define RTC_API_WAIT_TIME       (3*1000/CYDEV_CLK_BAKCLK__KHZ)

/* Structure that enables alarm interrupts at 1 minute intervals, i.e when the 
   seconds field rolls over to "00". Note that only the seconds field is enabled. 
   Other fields are kept at the default values and not enabled */                                
cy_stc_rtc_alarm_t const alarm = 
{
    .sec            =   00u,
    .secEn          =   CY_RTC_ALARM_ENABLE,
    .min            =   00u,
    .minEn          =   CY_RTC_ALARM_DISABLE,
    .hour           =   00u,
    .hourEn         =   CY_RTC_ALARM_DISABLE,
    .dayOfWeek      =   01u,
    .dayOfWeekEn    =   CY_RTC_ALARM_DISABLE,
    .date           =   01u,
    .dateEn         =   CY_RTC_ALARM_DISABLE,
    .month          =   01u,
    .monthEn        =   CY_RTC_ALARM_DISABLE,
    .almEn          =   CY_RTC_ALARM_ENABLE
};

/* Queue handle used for commands and data of RTC Task */
QueueHandle_t rtcCommandAndDataQ;

/*******************************************************************************
* Function Name: void Task_Rtc(void *pvParameters)
********************************************************************************
* Summary:
*  Task that processes the RTC interrupt and Display tasks 
*  to take an action based on the RTC interrupt
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/
void Task_Rtc(void *pvParameters)
{
    /* Variable that stores RTC commands and data */
    rtc_command_and_data_t rtcCommandAndData;
    
    /* Variable used to store the return values of RTC APIs */
    cy_en_rtc_status_t rtcApiResult;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Remove warning for unused parameter */
    (void)pvParameters;
    
    /* Start the RTC */
    RTC_Start();
    
    /* Clear any pending interrupts */
    Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
    __NVIC_ClearPendingIRQ(RTC_RTC_IRQ_cfg.intrSrc);
    
    /*Configures the source (Alarm1) that trigger the interrupts */
    Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM1);    
    
    
    
    /* Set the default date and time */
    rtcApiResult = Cy_RTC_SetDateAndTimeDirect(TIME_AT_RESET);
    
    if(rtcApiResult == CY_RTC_SUCCESS)
    {
        Task_DebugPrintf("Success  : RTC initialization", 0u);
    }
    else
    {
        Task_DebugPrintf("Failure! : RTC initialization. Error Code:",
                    rtcApiResult);
        /* If operation fails, halt */
        CY_ASSERT(0u);
    }
    vTaskDelay(RTC_API_WAIT_TIME);
    rtcApiResult = Cy_RTC_SetAlarmDateAndTime(&alarm,CY_RTC_ALARM_1);
    
    if(rtcApiResult == CY_RTC_SUCCESS)
    {
        Task_DebugPrintf("Success  : RTC Alarm set", 0u);
    }
    else
    {
        Task_DebugPrintf("Failure! : RTC Alarm set. Error Code:",
                    rtcApiResult);
        /* If operation fails, halt */
        CY_ASSERT(0u);
    }
    
    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a RTC command has been received over xQueue_RtcCommandAndData */
        rtosApiResult = xQueueReceive(rtcCommandAndDataQ, &rtcCommandAndData, portMAX_DELAY);
        
        /* Command has been received from xQueue_RtcCommandAndData */
        if(rtosApiResult == pdTRUE)
        {
            switch(rtcCommandAndData.command)
            {
                /* Update RTC date and time */
                case RTC_UPDATE_TIME:
                {
                    Cy_RTC_SetDateAndTimeDirect(
                        rtcCommandAndData.rtcDateTime.sec,  \
                        rtcCommandAndData.rtcDateTime.min,  \
                        rtcCommandAndData.rtcDateTime.hour, \
                        rtcCommandAndData.rtcDateTime.date, \
                        rtcCommandAndData.rtcDateTime.month,\
                        rtcCommandAndData.rtcDateTime.year);
                    xQueueSend(displayDataQ, &rtcCommandAndData, 0u);
                    break;
                }
                /* Process RTC alarm */
                case RTC_PROCESS_ALARM:
                {
                    Cy_RTC_GetDateAndTime(&rtcCommandAndData.rtcDateTime);
                    xQueueSend(displayDataQ, &rtcCommandAndData, 0u);
                    break;
                }
                default:
                    break;
            }
        }
    }
}

/*******************************************************************************
* Function Name: void Cy_RTC_Alarm1Interrupt(void)
********************************************************************************
*
* Summary:
*  This function is the handler for the RTC Alarm1 Interrupt. It clears the 
*  interrupt and schedules a display refresh
*
* Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
void Cy_RTC_Alarm1Interrupt(void)
{   
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Clear any pending interrupts */
    Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
    __NVIC_ClearPendingIRQ(RTC_RTC_IRQ_cfg.intrSrc);
        
    rtc_command_and_data_t rtcCommandAndData;
    rtcCommandAndData.command = RTC_PROCESS_ALARM;
    
    xQueueSendFromISR(rtcCommandAndDataQ, &rtcCommandAndData, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken );
}

/* [] END OF FILE */
