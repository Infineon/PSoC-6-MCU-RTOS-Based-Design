/******************************************************************************
* File Name: display_task.c
*
* Version: 1.00
*
* Description: This file contains the task that initializes the E-INK display, 
*              shows the instructions to use this code example at startup and
*              frequenly refreshes the display to show temperature
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
* This file contains the task that initialize the E-INK display, shows the 
* instructions to use this code example at startup and frequenly refreshes the
* display to show temperature
*
* For the details of the E-INK display and library functions, see the code 
* example CE218133 - PSoC 6 MCU E-INK Display with CapSense
*******************************************************************************/

/* Header file includes */
#include <math.h>
#include <string.h>
#include "display_task.h"
#include "screen_contents.h"
#include "cy_eink_library.h"
#include "temperature_task.h"
#include "uart_debug.h"
#include "task.h" 
#include "timers.h"

/* Two frame buffers are used in this project since the E-INK display requires
   the storage of the previous as well as the current frame. See Appendix A
   of the the code example document of CE218133 - PSoC 6 MCU E-INK Display with 
   CapSense for details */
#define NUMBER_OF_FRAME_BUFFERS        (uint8_t) (2u)
/* Character string size used to store temperature value */
#define TEMPERATURE_STRING_SIZE        (uint8_t) (10u)
/* Macro used to convert the temperature to two digit decimal values */
#define DECIMAL_2DIGIT_SCALER          (uint8_t) (100u)

/* Enumerated data type used to identify the frame buffers */
typedef enum 
{
    BUFFER0 = (0x00u),
    BUFFER1 = (0x01u)
}   frame_buffer_t;

/* Variable that stores the current frame buffer being used */
frame_buffer_t      currentFrameBuffer  = BUFFER0;

/* Frame buffers used by the display update functions. See Appendix A of the 
   the code example document of CE218133 - PSoC 6 MCU E-INK Display with 
   CapSense for details of frame buffers */
cy_eink_frame_t     frameBuffer[NUMBER_OF_FRAME_BUFFERS]
                               [CY_EINK_FRAME_SIZE];
/* Variable that stores the pointer to the current frame being displayed */
cy_eink_frame_t*    currentFrame =  CY_EINK_WHITE_FRAME;

/* Ambient temperature is defined at 25c, since the thermistor circuit is
   used for health themometer */
#define AMBIENT_TEMPERATURE     (int8_t)(25u)

/* Function used to register the E-INK delay call back */
void static DelayMs(uint32_t delayInMs)  {vTaskDelay(pdMS_TO_TICKS(delayInMs));}


/*******************************************************************************
* Task Name  : Task_Display
********************************************************************************
*
* Summary:
*  This Task combines text and logo/heading images using a frame buffer and then
*  writes the frame buffer to the E-INK display to show the instructions to use
*  this code example. In addition, this task frequenly refreshes the display to
*  show temperature
*
* Task Handle: None
*                                                    
*******************************************************************************/
void Task_Display (void *pvParameters)
{  
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Flag that indicates if the E-INK display has been detected */
    cy_eink_api_result displayDetected = CY_EINK_FAILURE;

    /* Variable that store text to be printed */
    char static tempSting[TEMPERATURE_STRING_SIZE];
    
    /* Variables used to convert the string to printable format */
    int8  decimal;
    int8  fraction;
    int16 temp;

    /* Coordinates at which the background image is cropped before loading to 
       the frame buffers. In this project, the entire frame is copied to 
       initialize the frame buffers with calender and clock icons and to clear 
       the text area with white pixels */
    uint8 const  fullFrameCoordinates[] =   {0, 33, 0, 175};

    /* Variable that counts the number of frame buffers initialized */
    uint8 static buffersFilled  = 0u;
    
    /* Variable that stores temperature value */
    float temperature;

    /* Remove warning for unused parameter */
    (void)pvParameters ;
    
    /* Initialize the E-INK display hardware with the ambient temperature 
       value and the delay function pointer */
    if(Cy_EINK_Start(AMBIENT_TEMPERATURE, DelayMs) == CY_EINK_SUCCESS)
    {
        Task_DebugPrintf("Success  : Display - Cy_EINK_Start API", 0u);

        /* Power on the display and check if the operation was successful */
        displayDetected = Cy_EINK_Power(CY_EINK_ON);
        
         /* Do this only if the E-INK display has been detected */
        if(displayDetected == CY_EINK_SUCCESS)
        { 
            Cy_EINK_Clear(CY_EINK_WHITE_BACKGROUND,CY_EINK_POWER_MANUAL);
            Cy_EINK_Power(CY_EINK_OFF);
        }
        else
        {
            Task_DebugPrintf("Failure! : Display - E-INK display power on ", 0u);
        }                
    }
    else
    {
        Task_DebugPrintf("Failure! : Display - Cy_EINK_Start API", 0u);
    }
    
    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a command has been received over 
           temperatureDataQ */
        rtosApiResult = xQueueReceive(temperatureDataQ, &temperature,
                        portMAX_DELAY);
        
         /* Command has been received from temperatureDataQ */ 
        if(rtosApiResult == pdTRUE)   
        {
            /* Two frame buffers are used in this project since the E-INK 
               display requires the retention of the previous as well as the 
               current frame. The code below finds the frame buffer used in the 
               previous update and selects the other frame buffer to be 
               overwritten by the current operation */
            if (currentFrameBuffer == BUFFER0)
            {
                currentFrameBuffer = BUFFER1;
            }
            else
            {
                currentFrameBuffer = BUFFER0;
            }
            
            /* Initialize the current frame buffer being used with the 
               background image and the instructions (text), if both the frame 
               buffers are not initialized already */
            if (buffersFilled < NUMBER_OF_FRAME_BUFFERS)
            {
                /* Text fields used in this project start printing from these 
                   coordinates. See Appendix A of the code example document of 
                   CE218133 - PSoC 6 MCU E-INK Display with CapSense for the  
                   details of font size and text coordinate formats */
                uint8 const  note1Origin[]          =   {0x15u, 0x00u};
                uint8 const  note2Origin[]          =   {0x15u, 0x01u};
                uint8 const  note3Origin[]          =   {0x15u, 0x02u};
                uint8 const  instructionOrigin[]    =   {0x00u, 0x04u};
                
                /* Load the images and text fields into the framebuffer */
                Cy_EINK_ImageToFrameBuffer(frameBuffer[currentFrameBuffer],
                                           (uint8*)background,
                                           (uint8*)fullFrameCoordinates); 
                Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], 
                                          (char*)note1, CY_EINK_FONT_8X12BLACK,
                                          (uint8*)note1Origin); 
                Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer],
                                          (char*)note2, CY_EINK_FONT_8X12BLACK,
                                          (uint8*)note2Origin);
                Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer],
                                          (char*)note3, CY_EINK_FONT_8X12BLACK,
                                          (uint8*)note3Origin); 
                Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer],
                                          (char*)instructions, 
                                          CY_EINK_FONT_8X12BLACK,
                                          (uint8*)instructionOrigin); 
                buffersFilled++;
            }    

            /* Covert temperature data to string */
            temp     = roundf(temperature*DECIMAL_2DIGIT_SCALER);
            decimal  = temp/DECIMAL_2DIGIT_SCALER;
            fraction = (int8_t)(abs(temp%DECIMAL_2DIGIT_SCALER));
            sprintf (tempSting,"%3d.%02d",decimal,fraction);
            
            /* Coordinate at which the temperature value is printed */
            uint8 const  tempOrigin[] =   {0x02u, 0x01u};
            
            /* Load the frame buffer with the string that stores temperature 
               data */
            Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], 
                                      tempSting, CY_EINK_FONT_16X16BLACK,
                                      (uint8_t*)tempOrigin);

            /* Update the display with the current frame */
            Cy_EINK_ShowFrame(currentFrame, frameBuffer[currentFrameBuffer],
                              CY_EINK_FULL_2STAGE, CY_EINK_POWER_AUTO);
            
            /* Store the pointer to the current frame, which is required for 
               subsequent updates */
            currentFrame = frameBuffer[currentFrameBuffer];
        }
    }    
}

/* [] END OF FILE */
