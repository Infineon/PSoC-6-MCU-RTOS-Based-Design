/******************************************************************************
* File Name: display_task.c
*
* Version: 1.00
*
* Description: This file contains the task that initializes the E-INK display
*              and show the instructions to use this code example at startup
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
/*******************************************************************************
* This file contains the task that initialize the E-INK display and show the
* instructions to use this code example at startup
*
* For the details of the E-INK display and library functions, see the code 
* example CE218133 - PSoC 6 MCU E-INK Display with CapSense
*******************************************************************************/

/* Header file includes */
#include "display_task.h"
#include "screen_contents.h"
#include "cy_eink_library.h"
#include "temperature_eink.h"
#include "display_task.h"
#include "uart_debug.h"
#include "FreeRTOS.h"
#include "task.h"  

/* Origin coordinates of text (instructions) */
#define INSTRUCTION_TEXT_ORIGIN {0x00u, 0x02u}

/* Function used to register the E-INK delay call back */
void static DelayMs(uint32_t delayInMs)  {vTaskDelay(pdMS_TO_TICKS(delayInMs));}

/* Frame buffer used for combining text and images into one frame */
cy_eink_frame_t frameBuffer[CY_EINK_FRAME_SIZE];

/*******************************************************************************
* Task Name  : Task_Display
********************************************************************************
*
* Summary:
*  This Task combines text and logo/heading images using a frame buffer and then
*  writes the frame buffer to the E-INK display to show the instructions to use
*  this code example. After displaying the instructions, the Task suspends
*  itself.
*
* Task Handle: None
*                                                    
*******************************************************************************/
void Task_Display (void *pvParameters)
{  
    /* Flag that indicates if the E-INK display has been detected */
    cy_eink_api_result displayDetected = CY_EINK_FAILURE;
    
    /* Local variable that stores the value of ambient temperature */
    int8_t ambientTemperature;

    /* Remove warning for unused parameter */
    (void)pvParameters ;
    
    /* Initialize the components used for temperature sensing */
    InitTemperature();
    
    /* Read the value of ambient temperature */
    ambientTemperature = (int8_t)GetTemperature();
    
    /* Initialize the E-INK display hardware with the ambient temperature 
       value and the delay function pointer */
    if(Cy_EINK_Start(ambientTemperature,DelayMs) == CY_EINK_SUCCESS)
    {
        DebugPrintf("Success  : Display - Cy_EINK_Start API", 0u);

        /* Power on the display and check if the operation was successful */
        displayDetected = Cy_EINK_Power(CY_EINK_ON);
    }
    else
    {
        DebugPrintf("Failure! : Display - Cy_EINK_Start API", 0u);
    }

    for(;;)
    {
        /* Do this only if the E-INK display has been detected */
        if(displayDetected == CY_EINK_SUCCESS)
        {           
            /* Coordinates of a full-image and text origin */
            uint8_t   imageCoordinates[]  = CY_EINK_COMPLETE_IMAGE;
            uint8_t   textOrigin[]        = INSTRUCTION_TEXT_ORIGIN;
            
            DebugPrintf("Success  : Display - E-INK display power on", 0u); 
        
            /* Clear the display to white background */
            Cy_EINK_Clear(CY_EINK_WHITE_BACKGROUND, CY_EINK_POWER_MANUAL);
            
            /* Load the image that has the logo and the heading to the 
               frame buffer */
            Cy_EINK_ImageToFrameBuffer(frameBuffer, 
                                      (cy_eink_frame_t*)logoAndHeading,
                                       imageCoordinates);
            /* Load the text that stores instructions to the frame buffer */
            Cy_EINK_TextToFrameBuffer(frameBuffer, (char*)instructions,
                                      CY_EINK_FONT_8X12BLACK, textOrigin);
            
            /* Write the frame buffer to the E-INK display */
            Cy_EINK_ShowFrame(CY_EINK_WHITE_FRAME, frameBuffer,
                              CY_EINK_FULL_2STAGE, CY_EINK_POWER_MANUAL);
        }
        else
        {   
            DebugPrintf("Failure! : Display - E-INK display power on ", 0u); 
        }
        
        /* Turn off the E-INK power */
        Cy_EINK_Power(CY_EINK_OFF);
        
        DebugPrintf("Info     : Display - Task completed and suspended", 0u);

        /* Suspend the task */
        vTaskSuspend(NULL);
    }
}

/* [] END OF FILE */
