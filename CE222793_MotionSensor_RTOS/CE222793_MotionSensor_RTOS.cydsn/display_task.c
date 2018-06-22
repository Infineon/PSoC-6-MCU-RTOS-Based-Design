/******************************************************************************
* File Name: display_task.c
*
* Version: 1.00
*
* Description: This file contains the task that initializes the E-INK display
*              and show the instructions to use this code example at startup
*
* Related Document: CE222793_MotionSensor_RTOS.pdf
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
* This file contains the task that initialize the E-INK display and show the
* instructions to use this code example at startup
*
* For the details of the E-INK display and library functions, see the code 
* example CE218133 - PSoC 6 MCU E-INK Display with CapSense
*******************************************************************************/

/* Header file includes */
#include "display_task.h"
#include "motion_task.h"
#include "screen_contents.h"
#include "cy_eink_library.h"
#include "uart_debug.h"
#include "FreeRTOS.h"
#include "task.h"  

/* Origin coordinates of text (instructions) */
#define INSTRUCTION_TEXT_ORIGIN {0x00u, 0x02u}

/* Ambient temperature used in this project */
#define AMBIENT_TEMPRATURE  (25u)

/* Function used to register the E-INK delay call back */
void static DelayMs(uint32_t delayInMs)  {vTaskDelay(pdMS_TO_TICKS(delayInMs));}

/* Frame buffer used for combining text and images into one frame */
cy_eink_frame_t frameBuffer[CY_EINK_FRAME_SIZE];

/* Variable that stores the pointer to the current frame being displayed */
cy_eink_frame_t* currentFrame = CY_EINK_WHITE_FRAME;

/* Handle for the Queue that contains display data */
QueueHandle_t displayDataQ;

/* This static function is used by the display task. This function is not 
   available outside this file. See the function definition for more details */
void static RefreshDisplay (motion_sensor_info_t* );

/*******************************************************************************
* Task Name  : Task_Display
********************************************************************************
*
* Summary:
*  This Task combines text and logo/heading images using a frame buffer and then
*  writes the frame buffer to the E-INK display to show the instructions to use
*  this code example. 
*
* Task Handle: None
*                                                    
*******************************************************************************/
void Task_Display (void *pvParameters)
{  
    /* Flag that indicates if the E-INK display has been detected */
    cy_eink_api_result displayDetected = CY_EINK_FAILURE;
    
    /* Variable that stores Motion Sensor information */
    motion_sensor_info_t displayData =
    {
        .orientation    = ORIENTATION_DISP_UP,
        .stepCount      = 0u 
    };
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
        
    /* Remove warning for unused parameter */
    (void)pvParameters ;
        
    /* Initialize the E-INK display hardware with the ambient temperature 
       value and the delay function pointer */
    if(Cy_EINK_Start(AMBIENT_TEMPRATURE, DelayMs) != CY_EINK_SUCCESS)
    {
        Task_DebugPrintf("Failure! : Display - Cy_EINK_Start API", 0u);
    }

    for(;;)
    {
        /* Block until a display data has been received over displayDataQ */
        rtosApiResult = xQueueReceive(displayDataQ, &displayData,
                         portMAX_DELAY);
        
        /* Data has been received from displayDataQ */
        if(rtosApiResult == pdTRUE)
        {
            /* Power on the display and check if the operation was successful */
            displayDetected = Cy_EINK_Power(CY_EINK_ON);
            
            if(displayDetected == CY_EINK_SUCCESS)
            {
                /* Refresh E-INK display */
                RefreshDisplay(&displayData);
                                
                /* Turn off E-INK display */
                Cy_EINK_Power(CY_EINK_OFF);
            } 
            else
            {
                Task_DebugPrintf("Failure! : Display - E-INK display power on ", 0u); 
            }
        }
    }
}

/*******************************************************************************
* Function Name: void RefreshDisplay (void)
********************************************************************************
*
* Summary:
*  Updates the EINK display with current time and date if conditions permit
*
* Parameters:
*  motion_sensor_info_t*
*
* Return:
*  None
*
* Side Effects:
*  This function is hard-coded to work with the images, text, and font used in 
*  this code example. Since this is NOT A GENERIC FUNCTION, it NOT RECOMMENDED 
*  to copy/paste this function into a different project to display text and 
*  images on the EINK display. It is recommended to use the EINK library functions 
*  for such use cases. See Appendix A of the CE218133_PSoC6_EINK_CapSense code 
*  example document for the details of EINK library functions.
*
*******************************************************************************/
void RefreshDisplay (motion_sensor_info_t* currentDisplayInfo)
{   
    /* Text fields used in this project start printing from these coordinates. See
       Appendix A of the CE218133_PSoC6_EINK_CapSense code example document for the 
       details of font size and text coordinate details */
    uint8 const  stepOrigin[]           =   {0x0Au, 0x03u};
    
    /* Coordinates at which the background image is cropped before loading to the 
       frame buffers. In this project, the entire frame is copied to initialize the
       frame buffers */
    uint8 const  fullFrameCoordinates[]     =   {00, 33, 00, 175};
    uint8 const  orientationCoordinates[]   =   {02, 14, 42, 144}; 
    
    /* Variable that stores step count */
    char  static stepCountString[6];
    
    /* Load the frame buffer with the background */
    Cy_EINK_ImageToFrameBuffer(frameBuffer, (uint8*)background,
                                (uint8*)fullFrameCoordinates);
    
    /* Load the frame buffer with the current orientation and stepcount */
    Cy_EINK_ImageToFrameBuffer(frameBuffer , 
        (cy_eink_image_t*)orientationImages[currentDisplayInfo->orientation],
        (uint8_t*)orientationCoordinates);
    sprintf (stepCountString,"%05u",currentDisplayInfo->stepCount);
            
    Cy_EINK_TextToFrameBuffer(frameBuffer, 
            stepCountString,
            CY_EINK_FONT_16X16BLACK,
            (uint8_t*)stepOrigin);

    /* Update the display with the current date and time */
    Cy_EINK_ShowFrame(currentFrame, frameBuffer,
                   CY_EINK_FULL_4STAGE, CY_EINK_POWER_MANUAL);
}

/* [] END OF FILE */
