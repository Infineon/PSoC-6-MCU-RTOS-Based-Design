/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description: This project demonstrates a BLE Eddystone beacon that broadcasts
*              the core Eddystone beacon frames (URL, UID and TLM)
*
* Related Document: CE220331_BLE_UI_RTOS.pdf
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
* This project demonstrates a BLE Eddystone beacon that broadcasts the core 
* Eddystone beacon frames (URL, UID and TLM). 
* 
* The Eddystone-UID frame broadcasts a unique 16-byte Beacon ID composed of a 
* 10-byte name-space and a 6-byte instance.  
*
* The Eddystone-URL frame broadcasts a URL using a compressed encoding format 
* in order to fit more within the limited advertisement packet.
* 
* The Eddystone-TLM frame broadcasts telemetry information about the beacon 
* itself such as battery voltage, device temperature, and counts of broadcast 
* packets.
*
* This project broadcasts the URL frames by default, with interleaved TLM 
* frames. To change the Eddystone packet settings, see the header file
* eddystone_config.h
*
* This code example uses FreeRTOS. For documentation and API references of 
* FreeRTOS, visit : https://www.freertos.org 
*
*******************************************************************************/

/* Header file includes */
#include <limits.h>
#include "ble_task.h"
#include "display_task.h"
#include "status_led_task.h"
#include "uart_debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Priorities of user tasks in this project */
#define TASK_BLE_PRIORITY           (15u)
#define TASK_STATUS_LED_PRIORITY    (10u)
#define TASK_DISPLAY_PRIORITY       (5u)

/* Stack sizes of user tasks in this project */
#define TASK_BLE_STACK_SIZE         (1024u)
#define TASK_STATUS_LED_STACK_SIZE  (configMINIMAL_STACK_SIZE)
#define TASK_DISPLAY_STACK_SIZE     (configMINIMAL_STACK_SIZE)

/* Queue lengths of message queues used in this project */
#define STATUS_LED_QUEUE_LEN       (5u)

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
     /* Create the queues and semaphores. See the respective data-types for 
       details of queue contents */
    bleSemaphore    = xSemaphoreCreateCounting(ULONG_MAX,0u);
    statusLedDataQ  = xQueueCreate(STATUS_LED_QUEUE_LEN, sizeof(status_led_data_t));
 
    /* Create the user Tasks. See the respective Task definition for more
       details of these tasks */       
    xTaskCreate(Task_Ble, "BLE Task", TASK_BLE_STACK_SIZE,
                NULL, TASK_BLE_PRIORITY, NULL);
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
