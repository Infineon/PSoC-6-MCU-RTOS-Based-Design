/******************************************************************************
* File Name: cy_eink_library.c
*
* Version: 1.00
*
* Description: This file contains the library functions for controlling
*              the E-INK display.
*
* Hardware Dependency: CY8CKIT-028-EPD E-INK Display Shield
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
* This file contains the library functions for controlling the E-INK display.
*
* For the details of the E-INK display and library functions, see the code  
* example document of CE218133 - PSoC 6 MCU E-INK Display with CapSense
*
* For the details of E-INK display hardware and driver interface, see the 
* documents available at the following website:
* http://www.pervasivedisplays.com/products/271
 *******************************************************************************/

/* Header file includes */
#include "cy_eink_library.h"

/* Range of ASCII printable characters: "!" to "~" */
#define CY_EINK_ASCII_MIN   (uint8_t) (0x21)
#define CY_EINK_ASCII_MAX   (uint8_t) (0x7E)

/* Range of image byte coordinates */
#define CY_EINK_IMAGE_X_MAX (uint8_t) (33u)
#define CY_EINK_IMAGE_Y_MAX (uint8_t) (175u)

/*******************************************************************************
* Function Name: cy_eink_api_result Cy_EINK_Start(int8_t temperature,
*                                       cy_eink_delay_function_t delayFunction)
********************************************************************************
*
* Summary: Initialize the E-INK display hardware, starts the required PSoC 
*  components, and also performs temperature compensation of E-INK parameters.
*
*  Note: This function does not turn on the E-INK display.
*
* Parameters:
*  int8_t temperature       : Ambient temperature in degree Celsius
*  cy_eink_delay_function_t : Pointer to a delay function that accepts
*                             delay in milliseconds (uint32_t type)                         
*
* Return:
*  None
*
* Side Effects:
*  Lower ambient temperature results in higher refresh times
*
*******************************************************************************/
cy_eink_api_result Cy_EINK_Start(int8_t temperature,
                                 cy_eink_delay_function_t delayFunction)
{
    /* Variable to store the return value */
    cy_eink_api_result returnValue;
    
    /* Make sure that the function pointer parameter is not NULL */
    if(delayFunction != NULL)
    {
        /* Register the callback function for EINK delay in milliseconds */
        Cy_EINK_RegisterDelayFunction(delayFunction);
        
        /* Initialize the E-INK display hardware and associated PSoC
           components */
        Pv_EINK_Init();
        /* Perform temperature compensation of E-INK parameters */
        Pv_EINK_SetTempFactor(temperature);
        
        /* Return success */
        returnValue = CY_EINK_SUCCESS;
    }
    else
    {
        /* Return failure */
        returnValue = CY_EINK_FAILURE;
    }
    
    return returnValue;
}

/*******************************************************************************
* Function Name: cy_eink_api_result Cy_EINK_Power(bool powerCtrl)
********************************************************************************
*
* Summary: This function is used to turn on/off the E-INK display power.
*
*  Note: This function can not be used to clear the E-INK display. The display 
*  will retain the previously written frame even when it's turned off.
*
* Parameters:
*  bool powerCtrl       : "False" turns off and "True" turns on the display.
*
* Return:
*  cy_eink_api_result   : "CY_EINK_SUCCESS" if operation was successful;
*                         "CY_EINK_FAILURE" otherwise
*
* Side Effects:
*  None
*
*******************************************************************************/
cy_eink_api_result Cy_EINK_Power(bool powerCtrl)
{
    /* Variable to store operation status */
    pv_eink_status_t pwrStatus;
    cy_eink_api_result returnValue;
    
    /* Turn on the E-INK power if powerCtrl is "true" */
    if (powerCtrl == CY_EINK_ON)
    {
        pwrStatus = Pv_EINK_HardwarePowerOn();
    }
    /* Turn off the E-INK power if powerCtrl is "false" */
    else
    {
        pwrStatus = Pv_EINK_HardwarePowerOff();
    }
    /* If the operation was successful, return "true" */
    if (pwrStatus == PV_EINK_RES_OK)
    {
        returnValue = CY_EINK_SUCCESS;
    }
    /* If the operation was not successful, return "false" */
    else
    {
        returnValue = CY_EINK_FAILURE;
    }
    
    /* Return the outcome of the power control operation */
    return (returnValue);
}

/*******************************************************************************
* Function Name: void Cy_EINK_Clear(bool background, bool powerCycle)
********************************************************************************
*
* Summary: This function is used to clear the display to all white or all black
*  pixels.
*
*  Note1: The E-INK display should be powered on (using Cy_EINK_Power function) 
*  before calling this function if "powerCycle" is false. Otherwise the display 
*  won't be cleared.
*
*  Note2: This function is intended to be called only after a reset/power up.
*  Use Cy_EINK_ShowFrame() function to clear the display if you know the frame
*  that has been written to the display.
*
* Parameters:
*  bool background   : False for black background and True for white background.
*  bool powerCycle   : True for automatic power cycle. False otherwise
* 
* Return:
*  None
*
* Side Effects:
*  This is a blocking function that can take as many as 2 seconds 
*
*******************************************************************************/
void Cy_EINK_Clear(bool background, bool powerCycle)
{
    /* If power cycle operation requested, turn on E-INK power */
    if (powerCycle)
    {
        Cy_EINK_Power(CY_EINK_ON);
    }
    /* Clear the display to white background */
    if (background == CY_EINK_WHITE_BACKGROUND)
    {
        /* Two consecutive display updates to reduce ghosting */
        Pv_EINK_FullStageHandler(CY_EINK_WHITE_FRAME, PV_EINK_STAGE4);
        Pv_EINK_FullStageHandler(CY_EINK_WHITE_FRAME, PV_EINK_STAGE4);
    }
    /* Clear the display to black background */
    else if (background == CY_EINK_BLACK_BACKGROUND)
    {
        /* Two consecutive display updates to reduce ghosting */
        Pv_EINK_FullStageHandler(CY_EINK_BLACK_FRAME, PV_EINK_STAGE4);
        Pv_EINK_FullStageHandler(CY_EINK_BLACK_FRAME, PV_EINK_STAGE4);
    }
    else
    {
    }
    
    /* If power cycle operation requested, turn off E-INK power */
    if (powerCycle)
    {
        Cy_EINK_Power(CY_EINK_OFF);
    }
}

/*******************************************************************************
* Function Name: void Cy_EINK_ShowFrame(cy_eink_frame_t* prevFrame, 
*      cy_eink_frame_t* newFrame,CY_EINK_UpdateType updateType, bool powerCycle)
********************************************************************************
*
* Summary: Updates the E-INK display with a frame/image stored in the flash
*          or RAM.
*
*  Notes: This function requires the previous frame data as well as the new 
*  frame data. If the previous frame data changes from the actual frame 
*  previously written to the display, considerable ghosting may occur.
*
*  The E-INK display should be powered on (using Cy_EINK_Power function) before 
*  calling this function, if "powerCycle" parameter is false. Otherwise the 
*  display won't be updated.
*
* Parameters:
*  cy_eink_frame_t* prevFrame    : Pointer to the previous frame written on the 
*                                  display
*  cy_eink_frame_t* newFrame     : Pointer to the new frame that need to be
*                                  written
*  cy_eink_update_t              : Full update (2/4 stages) or  Partial update
*  bool powerCycle               : "true" for automatic power cycle, "false" 
*                                  for manual
*  
* Return:
*  None
*
* Side Effects:
*  This is a blocking function that can take as many as 2 seconds 
*
*******************************************************************************/
void Cy_EINK_ShowFrame(cy_eink_frame_t* prevFrame, cy_eink_frame_t* newFrame,
                       cy_eink_update_t updateType, bool powerCycle)
{
    /* If power cycle operation requested, turn on E-INK power */
    if (powerCycle)
    {
        Cy_EINK_Power(CY_EINK_ON);
    }
    /* Partial update stage */
    if (updateType == CY_EINK_PARTIAL)
    {
        /* Update the display with changes from previous frame */
        Pv_EINK_PartialStageHandler(prevFrame, newFrame);
    }
    /* Full update stages */
    else if ((updateType == CY_EINK_FULL_4STAGE) || 
             (updateType == CY_EINK_FULL_2STAGE))
    {
        /* Stage 1: update the display with the inverted version of the previous 
           frame */
        Pv_EINK_FullStageHandler(prevFrame, PV_EINK_STAGE1);
        
        /* Additional stages that reduce ghosting for a 4 stage full update */
        if (updateType == CY_EINK_FULL_4STAGE)
        {
            /* Stage 2: update the display with an all white frame */
            Pv_EINK_FullStageHandler(prevFrame, PV_EINK_STAGE2);
            /* Stage 3: update the display with the inverted version of the new 
               frame */
            Pv_EINK_FullStageHandler(newFrame, PV_EINK_STAGE3);
        }
        
        /* Stage 4: update the display with the new frame */
        Pv_EINK_FullStageHandler(newFrame, PV_EINK_STAGE4);
    }
    else
    {
    }
    
    /* If power cycle operation requested, turn off E-INK power */
    if (powerCycle)
    {
        Cy_EINK_Power(CY_EINK_OFF);
    }
}

/*******************************************************************************
* Function Name: void Cy_EINK_ImageToFrameBuffer(cy_eink_frame_t* frameBuffer,
*                               cy_eink_frame_t* image, uint8_t* imgCoordinates)
********************************************************************************
*
* Summary: Copies pixel block data between the specified x and y coordinates 
*  from an image (typically stored in the flash) to a frame buffer in RAM.
*
*  Notes: Image array should be of the same size as frame buffer.
*
*  Copying is done at byte level. Pixel level operations are not supported.
*
*  This function does not update the E-INK display. After frame buffer update,
*  use Cy_EINK_ShowFrame() function to update the display if required.
*
*  This function can be used to clear the frame buffer by passing an all-white 
*  or an all-black image.
*
* Parameters:
*  cy_eink_frame_t* frameBuffer   : Pointer to the frame buffer array in RAM
*  cy_eink_frame_t* image         : Pointer to the image array (typically in 
*                                   flash)
*  uint8_t* imgCoordinates        : Pointer to a 4-byte array of image byte
*                                   coordinates
*
* Return:
*  cy_eink_api_result   : "CY_EINK_SUCCESS" if operation was successful;
*                         "CY_EINK_FAILURE" otherwise
*
* Side Effects:
*  This is a blocking function that can take as many as 5808 RAM write cycles
*  to complete.
*
*******************************************************************************/
cy_eink_api_result Cy_EINK_ImageToFrameBuffer(cy_eink_frame_t* frameBuffer, 
                        cy_eink_frame_t* image, uint8_t* imgCoordinates)
{
    /* Counter variable for horizontal pixel lines */
    uint16_t pixelLineCounter;
    
    /* Variable that stores the starting location of the current pixel line */
    uint16_t startLocation;
    
    /* Variable that stores the length of the pixel line */
    uint16_t lineLength;
    
    /* Variable that stores the return value */
    cy_eink_api_result returnValue;
    
    /* Check if the coordinates are within limits */
    if((imgCoordinates[CY_EINK_IMG_X1] < imgCoordinates[CY_EINK_IMG_X2])&& 
       (imgCoordinates[CY_EINK_IMG_Y1] < imgCoordinates[CY_EINK_IMG_Y2])&& 
       (imgCoordinates[CY_EINK_IMG_X2] <= CY_EINK_IMAGE_X_MAX)&&
       (imgCoordinates[CY_EINK_IMG_Y2] <= CY_EINK_IMAGE_Y_MAX))
    {
        /* Do this for all horizontal pixel lines between Y1 and Y2 
           coordinates */
        for (pixelLineCounter  = imgCoordinates[CY_EINK_IMG_Y1];
             pixelLineCounter <= imgCoordinates[CY_EINK_IMG_Y2];
             pixelLineCounter++)
        {
            
            /* Find the starting location of the pixel line */
            startLocation = (pixelLineCounter * PV_EINK_HORIZONTAL_SIZE) + 
                             imgCoordinates[CY_EINK_IMG_X1];
            
           /* Find the length of the pixel line to be copied */
            lineLength = imgCoordinates[CY_EINK_IMG_X2] - 
                         imgCoordinates[CY_EINK_IMG_X1];
            
            /* Copy the pixel line from the image to the same location in the 
               frame buffer */
            memcpy (&frameBuffer[startLocation], &image[startLocation],
                    lineLength);
        }
        
        returnValue = CY_EINK_SUCCESS;
    }
    else
    {   
        /* Operation failed. Check the input parameters */
        returnValue = CY_EINK_FAILURE; 
    }
    
    return returnValue;
}

/*******************************************************************************
* Function Name: void Cy_EINK_TextToFrameBuffer(cy_eink_frame_t* frameBuffer, 
*                char* string, cy_eink_font_t* fontInfo , uint8_t* fontCor)
********************************************************************************
*
* Summary: Converts a text input (string) to pixel data and stores it at the
*  specified coordinates of a frame buffer.
*
*  Notes: This function only supports printable ASCII characters from "!" to "~"
*
*  X and Y locations used in this function are font coordinates, rather than the
*  pixel coordinates used in Cy_EINK_ImageToFrameBuffer() function. See the 
*  CY_EINK_fonts.h for details.
*
*  This function does not update the E-INK display. After frame buffer update,
*  use CY_EINK_ShowFrame() function to update the display if required.
*
* Parameters:
*  cy_eink_frame_t* frameBuffer : Pointer to the frame buffer array in RAM
*  char* string                 : Pointer to the string
*  cy_eink_font_t* fontInfo     : Structure that stores font information
*  uint8_t* fontCor             : Pointer to the 2-byte array containing X and
*                                 Y locations
*
* Return:
*  cy_eink_api_result   : "CY_EINK_SUCCESS" if operation was successful;
*                         "CY_EINK_FAILURE" otherwise
*
* Side Effects:
*  This is a blocking function that can take as many as 5808 RAM write cycles
*  to complete.
*
*******************************************************************************/
cy_eink_api_result  Cy_EINK_TextToFrameBuffer(cy_eink_frame_t* frameBuffer, 
                       char* string, cy_eink_font_t* fontInfo, uint8_t* fontCor)
{ 
    /* Pointer variable for current character under conversion */
    char* currentChar;
    
    /* Variable for storing the location in the frame buffer to which 
       the font data is printed */
    uint16_t printLocation;
    /* Variable for storing the character location in the font */
    uint16_t charLocation;
    
    /* Variable that stores the pixel data byte location in the frame 
       buffer */
    uint16_t frameByte;
    /* Variable that stores the pixel data byte location in the font array */
    uint16_t fontByte;
    
    /* Counter variable for the vertical loop */
    uint8_t verticalCounter;
    /* Counter variable for the horizontal loop */
    uint8_t horizontalCounter;
    
    /* Variable that stores the return value */
    cy_eink_api_result returnValue = CY_EINK_SUCCESS;
    
    /* Repeat this process until the null character of the string is reached */
    for (currentChar = string;  *currentChar != '\0'; currentChar++)
    {
        /* Check if X and Y coordinates are between the font's max limits */
        if ((fontCor[CY_EINK_FONT_X] < (fontInfo->xSpan)) && 
            (fontCor[CY_EINK_FONT_Y] < (fontInfo->ySpan)))
        {
            /* Find the actual pixel location in the frame buffer where the
               pixel data should be copied */
            printLocation = (fontInfo->xOffset) + (fontCor[CY_EINK_FONT_X]   * 
                            (fontInfo->xSize))  + (((fontCor[CY_EINK_FONT_Y] * 
                            (fontInfo->ySize))  + (fontInfo->yOffset))    * 
                             PV_EINK_HORIZONTAL_SIZE);
            
            /* Check if the current character is a printable ASCII character 
               between "!" and "~" */
            if ((*currentChar <= CY_EINK_ASCII_MAX) &&
                (*currentChar >= CY_EINK_ASCII_MIN))
            {
                /* Find the location of pixel data corresponding to the 
                   current character from the font array */
                charLocation = (*currentChar - CY_EINK_ASCII_MIN) *
                               ((fontInfo->ySize) * (fontInfo->xSize));
                
                /* Vertical copying loop */
                for (verticalCounter = 0; verticalCounter < (fontInfo->ySize); 
                     verticalCounter++)
                {
                    /* Horizontal copying loop */
                    for (horizontalCounter = 0; 
                         horizontalCounter < (fontInfo->xSize); 
                         horizontalCounter++)
                    {
                        /* Calculate the current pixel data byte location of 
                           the frame buffer */
                        frameByte = horizontalCounter + (printLocation +
                                    (PV_EINK_HORIZONTAL_SIZE 
                                     * verticalCounter));
                        
                        /* Calculate the current pixel data byte location of 
                           the font */
                        fontByte = charLocation + (verticalCounter * 
                                   (fontInfo->xSize)) + horizontalCounter;
                        
                        /* Copy the pixel data as is for black font on white 
                           background */
                        if ((fontInfo->color) == CY_EINK_WHITE_BACKGROUND)
                        {
                            frameBuffer[frameByte] = ~(fontInfo->fontData
                                                                 [fontByte]);
                        }
                        /* Copy the inverted pixel data for white font on black
                           background */
                        else if ((fontInfo->color) == CY_EINK_BLACK_BACKGROUND)
                        {
                            frameBuffer[frameByte] = fontInfo->fontData
                                                              [fontByte];
                        }
                        else
                        {
                            returnValue = CY_EINK_FAILURE;
                        }
                    }
                }
            }
            /* For space and non-printable ASCII characters */
            else
            {
                /* Vertical copying loop */
                for (verticalCounter = 0; 
                     verticalCounter < (fontInfo->ySize); 
                     verticalCounter++)
                {
                    /* Horizontal copying loop */
                    for (horizontalCounter = 0; 
                         horizontalCounter < (fontInfo->xSize);
                         horizontalCounter++)
                    {
                        /* Calculate the current pixel data byte location of 
                           the frame buffer */
                        frameByte = horizontalCounter + (printLocation +
                                    (PV_EINK_HORIZONTAL_SIZE 
                                    * verticalCounter));
                        
                        /* Copy white pixels to create a space for fonts with 
                           white background */
                        if ((fontInfo->color) == CY_EINK_WHITE_BACKGROUND)
                        {
                            frameBuffer[frameByte] = CY_EINK_CLEAR_TO_WHITE;
                        }
                        /* Copy black pixels to create a space for fonts with 
                           black background */
                        else
                        {
                            frameBuffer[frameByte] = CY_EINK_CLEAR_TO_BLACK;
                        }
                    }
                }
            }
        }
        /* Move on to the next character by incrementing the horizontal
           location */
        fontCor[CY_EINK_FONT_X]++;
        
        /* Reset the X location and increment the Y location if the X span 
           is reached */
        if (fontCor[CY_EINK_FONT_X] >= (fontInfo->xSpan))
        {
            fontCor[CY_EINK_FONT_X] = 0;
            fontCor[CY_EINK_FONT_Y]++;
        }
        /* End the function if the Y span is reached */
        if (fontCor[CY_EINK_FONT_Y] > (fontInfo->ySpan))
        {
            break;
        }
    }
    
    return returnValue;
}

/* [] END OF FILE */
