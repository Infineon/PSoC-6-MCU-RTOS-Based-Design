/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description: This project demonstrates connectivity between the PSoC 6 BLE and 
*              CySmart BLE host Emulation tool or mobile device running the CySmart
*              mobile application, to transfer CapSense Proximity information.
*
* Related Document: CE218137_BLE_Proximity_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
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
/******************************************************************************
* This project demonstrates the capabilities of the PSoC 6 BLE to communicate 
* with a BLE Central device over a custom service, sending CapSense proximity
* sensing inputs. This CapSense custom service allows notifications to be sent  
* to the central device when notifications are enabled.  
*
* This code example uses FreeRTOS. For documentation and API references of 
* FreeRTOS, visit : https://www.freertos.org 
*
*******************************************************************************/

/* Header file includes */
#include "ble_task.h"
#include "status_led_task.h"  
#include "proximity_task.h"
#include "display_task.h"
#include "uart_debug.h"
#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Priorities of user tasks in this project */
#define TASK_BLE_PRIORITY           (20u)
#define TASK_PROXIMITY_PRIORITY     (15u)
#define TASK_STATUS_LED_PRIORITY    (10u)
#define TASK_DISPLAY_PRIORITY       (5u)

/* Stack sizes of user tasks in this project */
#define TASK_BLE_STACK_SIZE         (1024u)
#define TASK_STATUS_LED_STACK_SIZE  (configMINIMAL_STACK_SIZE)
#define TASK_PROXIMITY__STACK_SIZE  (configMINIMAL_STACK_SIZE)
#define TASK_DISPLAY_STACK_SIZE     (configMINIMAL_STACK_SIZE)

/* Queue lengths of message queues used in this project */
#define BLE_COMMAND_QUEUE_LEN       (5u)
#define STATUS_LED_QUEUE_LEN        (5u)
#define PROXIMITY_QUEUE_LEN         (1u)

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
    bleCommandQ       = xQueueCreate(BLE_COMMAND_QUEUE_LEN,
                                        sizeof(ble_command_t));
    proximityCommandQ = xQueueCreate(PROXIMITY_QUEUE_LEN,
                                        sizeof(proximity_command_t));
    statusLedDataQ    = xQueueCreate(STATUS_LED_QUEUE_LEN,
                                        sizeof(status_led_data_t));
         
    /* Create the user Tasks. See the respective Task definition for more
       details of these tasks */       
    xTaskCreate(Task_Ble, "BLE Task", TASK_BLE_STACK_SIZE,
                NULL, TASK_BLE_PRIORITY, NULL);
    xTaskCreate(Task_Proximity, "Proximity Task", TASK_PROXIMITY__STACK_SIZE,
                NULL, TASK_PROXIMITY_PRIORITY, NULL);
    xTaskCreate(Task_StatusLed, "Status LED Task", TASK_STATUS_LED_STACK_SIZE,
                NULL, TASK_STATUS_LED_PRIORITY, NULL);
    xTaskCreate(Task_Display, "Display task", TASK_DISPLAY_STACK_SIZE,
                NULL, TASK_DISPLAY_PRIORITY, NULL);

    /* Initialize thread-safe debug message printing. See uart_debug.h header file
       to enable / disable this feature */
    DebugPrintfInit();
    
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
