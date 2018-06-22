/******************************************************************************
* File Name: main_cm0p.c
*
* Version: 1.00
*
* Description: This file contains the Cortex-M0+ code that starts Cortex-M4
*
* Related Document: CE218136_EINK_CapSense_RTOS.pdf
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
* Cortex-M0+ starts UART based debug and enables Cortex-M4. See main_cm4.c
* for the main firmware.
*******************************************************************************/

/* Header file includes */
#include "project.h"
#include "uart_debug.h"

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. Starts UART based debug and enables Cortex-M4
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

    /* Enable CM4 only if BLE Controller started successfully. 
       CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout 
       is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);
 
    for (;;)
    {
        /* Put CM0+ to Deep Sleep mode */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}

/* [] END OF FILE */
