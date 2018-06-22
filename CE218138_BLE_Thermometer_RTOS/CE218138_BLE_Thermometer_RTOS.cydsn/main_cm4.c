/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description: This code example demonstrates BLE Health Thermometer Service 
*
* Related Document: CE218138_BLE_Thermometer_RTOS.pdf
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
/******************************************************************************
* This code example demonstrates interfacing PSoC 6 MCU with BLE Connectivity 
* (PSoC 6 MCU) with a thermistor circuit to read temperature information and 
* send the temperature information using BLE Health Thermometer Service 
* indications to a mobile device running CySmart mobile application
*
* This code examples uses FreeRTOS. For a bare-metal version of this code
* example, see CE220567 – PSoC 6 MCU with BLE Connectivity: BLE Thermometer
*******************************************************************************/

/* Header file includes */
#include "ble_task.h"
#include "status_led_task.h"  
#include "temperature_task.h"
#include "display_task.h"
#include "uart_debug.h"  
#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Priorities of user tasks in this project */
#define TASK_BLE_PRIORITY           (20u)
#define TASK_TEMPERATURE_PRIORITY   (15u)
#define TASK_STATUS_LED_PRIORITY    (10u)
#define TASK_DISPLAY_PRIORITY       (5u)

/* Stack sizes of user tasks in this project */
#define TASK_BLE_STACK_SIZE         (1024u)
#define TASK_STATUS_LED_STACK_SIZE  (configMINIMAL_STACK_SIZE)
#define TASK_TEMPERATURE_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define TASK_DISPLAY_STACK_SIZE     (512u)
 
/* Queue lengths of message queues used in this project */
#define BLE_COMMAND_QUEUE_LEN       (10u)
#define TEMP_COMMAND_QUEUE_LEN      (1u)
#define TEMP_DATA_QUEUE_LEN         (1u)
#define STATUS_LED_QUEUE_LEN        (5u)

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function enables interrupts and then calls the
*  the function that sets up user tasks and then starts RTOS scheduler. 
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main()
{   
    /* Create the queues. See the respective data-types for details of queue
       contents */
    bleCommandQ         = xQueueCreate(BLE_COMMAND_QUEUE_LEN,
                                        sizeof(ble_command_t));
    temperatureCommandQ = xQueueCreate(TEMP_COMMAND_QUEUE_LEN,
                                        sizeof(temperature_command_t));
    temperatureDataQ    = xQueueCreate(TEMP_DATA_QUEUE_LEN,
                                        sizeof(float));
    statusLedDataQ      = xQueueCreate(STATUS_LED_QUEUE_LEN,
                                        sizeof(status_led_data_t));
         
    /* Create the user Tasks. See the respective Task definition for more
       details of these tasks */       
    xTaskCreate(Task_Ble, "BLE Task", TASK_BLE_STACK_SIZE,
                NULL, TASK_BLE_PRIORITY, NULL);
    xTaskCreate(Task_Temperature, "Temperature Task", TASK_TEMPERATURE_STACK_SIZE,
                NULL, TASK_TEMPERATURE_PRIORITY, NULL);
    xTaskCreate(Task_StatusLed, "Status LED Task", TASK_STATUS_LED_STACK_SIZE,
                NULL, TASK_STATUS_LED_PRIORITY, NULL);
    xTaskCreate(Task_Display, "Display task", TASK_DISPLAY_STACK_SIZE,
                NULL, TASK_DISPLAY_PRIORITY, NULL);

    /* Initialize thread-safe debug message printing. See uart_debug.h header 
       file to enable / disable this feature */
    Task_DebugInit();
    
    /* Start the RTOS scheduler. This function should never return */
    vTaskStartScheduler();
    
    /* Should never get here! */ 
    DebugPrintf("Error!   : RTOS - scheduler crashed \r\n");
    
    /* Halt the CPU if scheduler exits */
    CY_ASSERT(0);
    
    for(;;)
    {
    }	
}

/*******************************************************************************
* Function Name: void vApplicationIdleHook(void)
********************************************************************************
*
* Summary:
*  This function is called when the RTOS in idle mode
*    
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationIdleHook(void)
{
    /* Enter sleep-mode */
    Cy_SysPm_Sleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
}

/*******************************************************************************
* Function Name: void vApplicationStackOverflowHook(TaskHandle_t *pxTask, 
                                                    signed char *pcTaskName)
********************************************************************************
*
* Summary:
*  This function is called when a stack overflow has been detected by the RTOS
*    
* Parameters:
*  TaskHandle_t  : Handle to the task
*  signed char   : Name of the task
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationStackOverflowHook(TaskHandle_t *pxTask, 
                                   signed char *pcTaskName)
{
    /* Remove warning for unused parameters */
    (void)pxTask;
    (void)pcTaskName;
    
    /* Print the error message with task name if debug is enabled in 
       uart_debug.h file */
    DebugPrintf("Error!   : RTOS - stack overflow in %s \r\n", pcTaskName);
    
    /* Halt the CPU */
    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: void vApplicationMallocFailedHook(void)
********************************************************************************
*
* Summary:
*  This function is called when a memory allocation operation by the RTOS
*  has failed
*    
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationMallocFailedHook(void)
{
    /* Print the error message if debug is enabled in uart_debug.h file */
    DebugPrintf("Error!   : RTOS - Memory allocation failed \r\n");
    
    /* Halt the CPU */
    CY_ASSERT(0);
}

/* [] END OF FILE */
