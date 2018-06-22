/******************************************************************************
* File Name: display_task.c
*
* Version: 1.00
*
* Description: This file contains the task that initializes the E-INK display
*              and show the instructions to use this code example at startup
*
* Related Document: CE222604_RTC_CTS_RTOS.pdf
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
#include "rtc_task.h"
#include "screen_contents.h"
#include "cy_eink_library.h"
#include "uart_debug.h"
#include "FreeRTOS.h"
#include "task.h"  

/* Ambient temprature used in thin project */
#define AMBIENT_TEMPRATURE  (25u)

/* Character size of a time string in HH:MM format */    
#define TIME_STRING_SIZE    (06u)
/* Character size of a date string in MM/DD/YYYY format */  
#define DATE_STRING_SIZE    (11u)    
    
/* Data-type that stores time and date as strings */
typedef struct 
{
    char timeSting[TIME_STRING_SIZE];
    char dateString[DATE_STRING_SIZE];
}   time_date_strings_t;

/* Origin coordinates of text (instructions) */
#define INSTRUCTION_TEXT_ORIGIN {0x00u, 0x06u}

/* Origin coordinates of date and time */
#define DATE_TEXT_ORIGIN        {0x05u, 0x01u}
#define TIME_TEXT_ORIGIN        {0x05u, 0x03u}

/* Queue handle used for display data  */
QueueHandle_t displayDataQ;

/* Frame buffer used for combining text and images into one frame */
cy_eink_frame_t frameBuffer[CY_EINK_FRAME_SIZE];

/* Function used to register the E-INK delay call back */
void static DelayMs(uint32_t delayInMs)  {vTaskDelay(pdMS_TO_TICKS(delayInMs));}


/* This static functions is used by the Task_Display. See the function definitions for 
   more details */
time_date_strings_t static GetRtcStrings(cy_stc_rtc_config_t *dateTime);

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
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Flag that indicates if the E-INK display has been detected */
    cy_eink_api_result displayDetected = CY_EINK_FAILURE;

    /* Variable used to store RTC data*/   
    rtc_command_and_data_t  timeAndDate;
    
    /* Remove warning for unused parameter */
    (void)pvParameters ;
    
    /* Initialize the E-INK display hardware with the ambient temperature 
       value and the delay function pointer */
    if(Cy_EINK_Start(AMBIENT_TEMPRATURE, DelayMs) == CY_EINK_SUCCESS)
    {
        Task_DebugPrintf("Success  : Display - Cy_EINK_Start API", 0u);
    }
    else
    {
        Task_DebugPrintf("Failure! : Display - Cy_EINK_Start API", 0u);
    }

    for(;;)
    {
        /* Block until a display data been received over displayDataQ */
        rtosApiResult = xQueueReceive(displayDataQ, &timeAndDate, portMAX_DELAY);
        
        /* Data has been received from displayDataQ */
        if(rtosApiResult == pdTRUE)
        {
            /* Power on the display and check if the operation was successful */
            displayDetected = Cy_EINK_Power(CY_EINK_ON);
            
            if(displayDetected == CY_EINK_SUCCESS)
            {
                /* Coordinates of a full-image and text origin */
                uint8_t   imageCoordinates[]    = CY_EINK_COMPLETE_IMAGE;
                uint8_t   textOrigin[]          = INSTRUCTION_TEXT_ORIGIN;
                uint8_t const  timeOrigin[]     = DATE_TEXT_ORIGIN;
                uint8_t const  dateOrigin[]     = TIME_TEXT_ORIGIN;
                
                /* Variable that stores the time and date strings */
                time_date_strings_t     timeAndDateBuffer;            
                
                /* Load the image that has the logo and the heading to the 
                   frame buffer */
                Cy_EINK_ImageToFrameBuffer(frameBuffer, 
                                          (cy_eink_frame_t*)background,
                                           imageCoordinates);
                
                /* Load the text that stores instructions to the frame buffer */
                Cy_EINK_TextToFrameBuffer(frameBuffer, (char*)instructions,
                                          CY_EINK_FONT_8X12BLACK, textOrigin);
                
                timeAndDateBuffer = GetRtcStrings(&timeAndDate.rtcDateTime);
                
                /* Load the frame buffer with the strings that store current time and date */
                Cy_EINK_TextToFrameBuffer(frameBuffer, 
                                      timeAndDateBuffer.timeSting,
                                      CY_EINK_FONT_16X16BLACK, (uint8_t*)timeOrigin);
                Cy_EINK_TextToFrameBuffer(frameBuffer, 
                                      timeAndDateBuffer.dateString,
                                      CY_EINK_FONT_16X16BLACK, (uint8_t*)dateOrigin);
                
                Cy_EINK_Power(CY_EINK_ON);
                Task_DebugPrintf("Success  : Display - E-INK display power on", 0u);
                
                /* Write the frame buffer to the E-INK display */
                Cy_EINK_ShowFrame(CY_EINK_WHITE_FRAME, frameBuffer,
                                  CY_EINK_FULL_4STAGE, CY_EINK_POWER_MANUAL);
                
                /* Turn off the E-INK power */
                Cy_EINK_Power(CY_EINK_OFF);
                Task_DebugPrintf("Info     : Display - Task completed", 0u);
            }
            else
            {   
                Task_DebugPrintf("Failure! : Display - E-INK display power on ",
                                  0u); 
            }
        }
    }
}

/*******************************************************************************
* Function Name: time_date_strings_t* GetRtcStrings(void)
********************************************************************************
*
* Summary:
*  This function returns a structure that contains current time and date strings.
*  Date is returned in "mm/dd/yyyy" format and time is in "hh:mm" format
*
* Parameters:
*  time_and_date_t* : structure that contains time and date strings
*
* Return:
*  None
*
**********************************************************************************/
time_date_strings_t static GetRtcStrings(cy_stc_rtc_config_t *dateTime)
{
    /* Variable that stores converted strings */
    time_date_strings_t dateTimeStrings;


    /* Print Date info in mm/dd/yyyy format */
    sprintf(dateTimeStrings.dateString, "%02u/%02u/20%02u", (uint16_t) dateTime->month,
           (uint16_t) dateTime->date, (uint16_t) dateTime->year);
    
    /* Print Time in hh:mm:ss format */
    sprintf(dateTimeStrings.timeSting, "%02u:%02u", (uint16_t) dateTime->hour, 
            (uint16_t) dateTime->min);
    
    /* Return the converted strings */
    return dateTimeStrings;
}

/* [] END OF FILE */
