/******************************************************************************
* File Name: main_cm0p.c
*
* Version: 1.00
*
* Description: This file contains the Cortex-M0+ BLE controller code
*
* Related Document: CE220331_BLE_UI_RTOS.pdf
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
/******************************************************************************
* Cortex-M0+ handles the controller portion of BLE. For the host functions, 
* see main_cm4.c
*******************************************************************************/

/* Header file includes */
#include "project.h"
#include "uart_debug.h"

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function enables the Cortex-M4 and continuously 
*  processes BLE controller events
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    /* Enable global interrupts */
    __enable_irq();
    
    /* Initialize the hardware used to send debug messages, if debug is enabled
       in uart_debug.h header file */
    DebugPrintfInit();
    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    DebugPrintf("\x1b[2J\x1b[;H");

    /* Start the controller portion of BLE. Host runs on the CM4 */
    if(Cy_BLE_Start(NULL) == CY_BLE_SUCCESS)
    {
        DebugPrintf("Success  : Cortex-M0+ - BLE controller initialization\r\n");
        
        /* Enable CM4 only if BLE Controller started successfully. 
           CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout 
           is changed. */
        Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);
    }
    else
    {
        DebugPrintf("Failure! : Cortex-M0+ - BLE controller initialization\r\n");
        
        /* Halt the CPU */
        CY_ASSERT(0u);
    }
    
    for (;;)
    {
        /* Process the controller portion of the BLE events and wake up the host
           (CM4) and send data to the host via IPC if necessary */
        Cy_BLE_ProcessEvents();
        
        /* Put CM0+ to Deep Sleep mode. The BLE hardware automatically wakes
            up the CPU if processing is required */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}

/* [] END OF FILE */
