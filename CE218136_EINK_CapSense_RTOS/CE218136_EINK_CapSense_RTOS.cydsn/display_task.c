/******************************************************************************
* File Name: display_task.c
*
* Version: 1.00
*
* Description: This file contains the task that shows an interactive menu and
*              text pages based on the commands recieved
*
* Related Document: CE218136_EINK_CapSense_RTOS.pdf
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
* This file contains the task that shows an interactive menu and text pages 
* based on the commands recieved
*
* For the details of the E-INK display and library functions, see the code 
* example CE218133 - PSoC 6 MCU E-INK Display with CapSense
*******************************************************************************/

/* Header file includes */
#include "cy_eink_library.h"
#include "display_task.h"
#include "screen_contents.h"
#include "menu_configuration.h"
#include "touch_task.h"
#include "temperature_eink.h"
#include "uart_debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Two frame buffers are used in this project since the E-INK display requires
   the storage of the previous as well as the current frame. See Appendix A
   of the code example document for the details of frame buffers */
#define NUMBER_OF_FRAME_BUFFERS     (uint8_t) (2u)

/* Enumerated data type used to identify the frame buffers */
typedef enum
{
    BUFFER0 = 0x00,
    BUFFER1 = 0x01
}   frame_buffers_t;

/* Variable that stores the current screen content being displayed */
screen_content_t    currentScreenContent = STARTUP_CONTENT;

/* Variable that stores the current frame buffer being used */
frame_buffers_t     currentFrameBuffer   = BUFFER0;

/* Frame buffers used by the display update functions. See Appendix A of the 
   code example document for details of frame buffers */
cy_eink_frame_t     frameBuffer[NUMBER_OF_FRAME_BUFFERS][CY_EINK_FRAME_SIZE];

/* Variable that stores the pointer to the current frame being displayed */
cy_eink_frame_t*    currentFrame = CY_EINK_WHITE_FRAME;

/* Variable that stores the details of the current screen */
screen_t  currentScreen =
{
    .screen     = MAIN_MENU,
    .menuItem   = MAIN_MENU_INDEX_START,
    .textPage   = TEXT_PAGE_INDEX_START
};

/*  These static functions are not available outside this file. 
    See the respective function definitions for more details */
void static NoScreenChange(void);
void static MoveArrowUp(void);
void static MoveArrowDown(void);
void static GoToTextPage(void);
void static BackToMainMenu(void);
void static PreviousTextPage(void);
void static NextTextPage(void);
void static InitializeFrameBuffers(cy_eink_image_t* imagePointer);
void static DisplayImage(cy_eink_image_t* imagePointer);
void static DisplayImageAndText(char* text, cy_eink_image_t* backgroundImage);

/* Function used to register the E-INK delay call back */
void static DelayMs(uint32_t delayInMs)  {vTaskDelay(pdMS_TO_TICKS(delayInMs));}

/*******************************************************************************
* Function Name: void Task_Display (void *pvParameters)
********************************************************************************
* Summary:
*  Task that processes the touch command recieved and the updates the display 
*  with the corresponding menu / text page
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/
void Task_Display (void *pvParameters)
{  
    /* Flag that indicates if the E-INK display has been detected */
    cy_eink_api_result displayDetected = CY_EINK_FAILURE;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Local variable that stores the value of ambient temperature */
    int8_t ambientTemperature;
    
    touch_data_t touchInput;

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
        Task_DebugPrintf("Success  : Display - Cy_EINK_Start API", 0u);

        /* Power on the display and check if the operation was successful */
        displayDetected = Cy_EINK_Power(CY_EINK_ON);

        if(displayDetected == CY_EINK_SUCCESS)
        {
            Task_DebugPrintf("Success  : Display - E-INK display power on", 0u);   
        } 
        else
        {
            Task_DebugPrintf("Failure! : Display - E-INK display power on ", 0u); 
            
            /* Turn the red LED on to indicate that the E-INK display is not 
            detected. Check the connection between the E-INK shield and the 
            Pioneer Baseboard if this happens, and then reset the PSoC 6 BLE */
            Cy_GPIO_Write(Pin_LED_Red_0_PORT, Pin_LED_Red_0_NUM, 0u);
        }
    }
    else
    {
        Task_DebugPrintf("Failure! : Display - Cy_EINK_Start API", 0u);
    }

    /* Clear the display to white background */
    Cy_EINK_Clear(CY_EINK_WHITE_BACKGROUND, CY_EINK_POWER_MANUAL);
            
    /* Show the logo */
    DisplayImage((cy_eink_image_t*) logo);

    /* Keep the logo on for a specific time*/
    vTaskDelay(STARTUP_SCREEN_DELAY);

    /* Show the default main menu image*/
    DisplayImage((cy_eink_image_t*) mainMenuImage[MAIN_MENU_INDEX_START]);
    
    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a touch input has been received over touchDataQ */
        rtosApiResult = xQueueReceive(touchDataQ, &touchInput,
                         portMAX_DELAY);
        
        /* Touch input has been received over touchDataQ */
        if(rtosApiResult == pdTRUE)
        {
       
        /* Function pointer that selects a screen update function based on the 
          current screen type and the touch input:
        _______________________________________________________________
        |                                     |                       |
        |          Parameters                 |   Selected Function   |
        |_____________________________________|_______________________|
        |                                     |                       |
        |   [MAIN_MENU][BUTTON0_TOUCHED])     |   GoToTextPage        |
        |   [MAIN_MENU][BUTTON1_TOUCHED])     |   NoScreenChange      |
        |   [MAIN_MENU][SLIDER_FLICK_LEFT])   |   MoveArrowUp         |
        |   [MAIN_MENU][SLIDER_FLICK_RIGHT])  |   MoveArrowDown       |
        |                                     |                       |
        |   [TEXT_PAGE][BUTTON0_TOUCHED])     |   NoScreenChange      |
        |   [TEXT_PAGE][BUTTON1_TOUCHED])     |   BackToMainMenu      |
        |   [TEXT_PAGE][SLIDER_FLICK_LEFT])   |   PreviousTextPage    |
        |   [TEXT_PAGE][SLIDER_FLICK_RIGHT])  |   NextTextPage        |
        |_____________________________________|_______________________|*/
            
        static void (*ScreenUpdatePointer[NUMBER_OF_SCREEN_TYPES]
                                         [NUMBER_OF_INPUT_TYPES]) (void) =
        {
            { GoToTextPage, NoScreenChange, MoveArrowUp, MoveArrowDown },
            { NoScreenChange, BackToMainMenu, PreviousTextPage,NextTextPage }
        };

            /* Call the function pointer that selects a screen update function
            according to the touch input */     
            (*ScreenUpdatePointer[currentScreen.screen][touchInput])();

        }
        /* Task has timed out and received no inputs during an interval of 
           portMAXDELAY ticks */
        else
        {
            Task_DebugPrintf("Warning! : Display - Task Timed out ", 0u);   
        }
    }
}

/*******************************************************************************
********************************************************************************
*  Following are the static functions used for display updates. These          *
*  functions are not available outside this file.                                        *
********************************************************************************
*******************************************************************************/

/*******************************************************************************
* Function Name: void static NoScreenChange(void)
********************************************************************************
*
* Summary:
*  Null function that is called by the "ScreenUpdatePointer" function pointer  
*  when no screen change is required
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static NoScreenChange(void)
{
}

/*******************************************************************************
* Function Name: void static GoToTextPage(void)
********************************************************************************
*
* Summary:
*  Performs transition from the main menu to the text page
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static GoToTextPage(void)
{
    /* Variable that stores the index of the character array, which in turn 
       stores the current text page as a string */
    uint8_t currentPageIndex;
    
    /* Change the current screen type to text page */
    currentScreen.screen = TEXT_PAGE;

    /* Re-initialize the text page index to point to the start page */
    currentScreen.textPage = TEXT_PAGE_INDEX_START;
    
    /* Access the index array and fetch the index of the character array that 
      stores the current text page as a string */
    currentPageIndex = textPageIndex[currentScreen.menuItem]
                                    [currentScreen.textPage];
    
    /* Check if the fetched index is valid */
    if (currentPageIndex != INVALID_PAGE_INDEX)
    {
        /* Display the text page with background image */
        DisplayImageAndText((char*) textPage[currentPageIndex],
                            (cy_eink_image_t*) textPageBackground);
    }
}

/*******************************************************************************
* Function Name: void static BackToMainMenu(void)
********************************************************************************
*
* Summary:
*  Returns to the main menu from a text page
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static BackToMainMenu(void)
{
    /* Change the current screen type to main menu */
    currentScreen.screen = MAIN_MENU;
    
    /* Display the current main menu image */
    DisplayImage((cy_eink_image_t*) mainMenuImage[currentScreen.menuItem]);
}

/*******************************************************************************
* Function Name: void static MoveArrowUp(void)
********************************************************************************
*
* Summary:
*  Moves the selection arrow of the main menu upwards
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static MoveArrowUp(void)
{
    
    /* If the beginning of the main menu is reached, then move the arrow to the
       final item of the main menu by selecting the maximum index */
    if (currentScreen.menuItem == MAIN_MENU_INDEX_START)
    {
        currentScreen.menuItem = MAIN_MENU_MAX_INDEX;
    }
    /* Otherwise, decrement the index to move the arrow up */
    else
    {
        currentScreen.menuItem--;
    }
    
    /* Display the current main menu image */
    DisplayImage((cy_eink_image_t*) mainMenuImage[currentScreen.menuItem]);
}

/*******************************************************************************
* Function Name: void static MoveArrowDown(void)
********************************************************************************
*
* Summary:
*  Moves the selection arrow of the main menu downwards
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static MoveArrowDown(void)
{ 
    /* If the final item of the main menu is reached, then move the arrow to the
       beginning of the main menu by selecting the starting index */
    if (currentScreen.menuItem >= MAIN_MENU_MAX_INDEX)
    {
        currentScreen.menuItem = MAIN_MENU_INDEX_START;
    }
    /* Otherwise, increment the index to move the arrow down */
    else
    {
        currentScreen.menuItem++;
    }
    
    /* Display the current main menu image */
    DisplayImage((cy_eink_image_t*) mainMenuImage[currentScreen.menuItem]);
}

/*******************************************************************************
* Function Name: void static PreviousTextPage(void)
********************************************************************************
*
* Summary:
*  Performs the transition to the previous text page
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static PreviousTextPage(void)
{
    
    /* Variable that stores the index of the character array, which in turn 
       stores the current text page as a string */
    uint8_t currentPageIndex;
    
    /* If the start page is reached, then go to the final page by selecting the 
       maximum index of the text pages */
    if (currentScreen.textPage == TEXT_PAGE_INDEX_START)
    {
        currentScreen.textPage = maxTextPageIndexes[currentScreen.menuItem];
    }
    /* Otherwise, select the previous page by decrementing the index */
    else
    {
        currentScreen.textPage--;
    }

    /* Access the index array and fetch the index of the character array that
        stores the current text page as a string */
    currentPageIndex = textPageIndex[currentScreen.menuItem]
                                    [currentScreen.textPage];
    
    /* Check if the fetched index is valid */
    if (currentPageIndex != INVALID_PAGE_INDEX)
    {
        /* Display the text page with background image */
        DisplayImageAndText((char*) textPage[currentPageIndex],
                            (cy_eink_image_t*) textPageBackground);
    }
}

/*******************************************************************************
* Function Name: void static NextTextPage(void)
********************************************************************************
*
* Summary:
*  Performs the transition to the next text page
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static NextTextPage(void)
{
    /* Variable that stores the index of the character array, which in turn 
       stores the current text page as a string */
    uint8_t currentPageIndex;
    
    /* If the final page is reached, then go to the start page by selecting 
       the starting index of the text pages */
    if (currentScreen.textPage >= maxTextPageIndexes[currentScreen.menuItem])
    {
        currentScreen.textPage = TEXT_PAGE_INDEX_START;
    }
     /* Otherwise, select the next page by incrementing the index */
    else
    {
        currentScreen.textPage++;
    }

    /* Access the index array and fetch the index of the character array that
        stores the current text page as a string */
    currentPageIndex = textPageIndex[currentScreen.menuItem]
                                    [currentScreen.textPage];
    
    /* Check if the fetched index is valid */
    if (currentPageIndex != INVALID_PAGE_INDEX)
    {
        /* Display the text page with background image */
        DisplayImageAndText((char*) textPage[currentPageIndex],
                            (cy_eink_image_t*) textPageBackground);
    }
}


/*******************************************************************************
* Function Name: void static InitializeFrameBuffers(cy_eink_image_t* 
*                                                   imagePointer)
********************************************************************************
*
* Summary:
*  Initialize the frame buffers with a background image
*
* Parameters:
*  cy_eink_image_t* imagePointer : pointer to the image
*
* Return:
*  None
*
* Side Effects:
*  This function is hard-coded to work with the images and text used in this 
*  code example. The background image is loaded only once to the frame buffer
*  since only one type of background is used in this project. Similarly, the 
*  images are cropped at predesignated coordinates before loading to the frame 
*  buffers. This cropping speeds up the initialization of frame buffers by not 
*  initializing the area to which the text is written. Since this is NOT A 
*  GENERIC FUNCTION, it NOT RECOMMENDED to copy/paste this function into a 
*  different project to initialize the frame buffer with an image. It is 
*  recommended to use the E-INK library functions for such use cases. See 
*  Appendix A of the code example document for the details of E-INK library 
*  functions.
*
*******************************************************************************/
void static InitializeFrameBuffers(cy_eink_image_t* imagePointer)
{
    /* Coordinates at which the text menu images cropped before loading to the
       frame buffers. This cropping speeds up the initialization of the frame 
       buffers by not initializing the area to which the text is written */
    const uint8_t   cropTop[]    = CROP_BACKGROUND_TOP;
    const uint8_t   cropBottom[] = CROP_BACKGROUND_BOTTOM;
    
    /* Flag used to determine when this function is called for the first time */
    bool static     firstTime = true;
    
    /* The background image is loaded only once to the frame buffer since only
       one type of background is used in this project. */
    if (firstTime)
    {
        /* Crop the background image at the predesignated coordinates and  then 
           load to the frame buffers. This cropping speeds up the initialization 
           of the frame buffers by not initializing the area to which the text 
           is written. */
        Cy_EINK_ImageToFrameBuffer(frameBuffer[BUFFER0], imagePointer,
                                   (uint8_t*) cropTop);
        Cy_EINK_ImageToFrameBuffer(frameBuffer[BUFFER0], imagePointer,
                                   (uint8_t*) cropBottom);
        Cy_EINK_ImageToFrameBuffer(frameBuffer[BUFFER1], imagePointer,
                                   (uint8_t*) cropTop);
        Cy_EINK_ImageToFrameBuffer(frameBuffer[BUFFER1], imagePointer,
                                   (uint8_t*) cropBottom);
                                
        /* Clear the first time flag */
        firstTime = false;
    }
}

/*******************************************************************************
* Function Name: void static DisplayImage(cy_eink_image_t* imagePointer)
********************************************************************************
*
* Summary:
*  Displays an image on the E-INK display.
*
* Parameters:
*  image* cy_eink_image_t : pointer to the image to be displayed
*
* Return:
*  None
*
* Side Effects:
*  This function selects an update type (partial/full) based on the current
*  screen type. This function also stores the pointer to the previous frame,
*  which is used for subsequent updates. This function is hard-coded to work 
*  with the menu types and image types used in this code example. Since this is 
*  NOT A GENERIC FUNCTION, it NOT RECOMMENDED to copy/paste this function into a 
*  different project to display an image on the E-INK display. It is recommended 
*  to use the Cy_EINK_ShowFrame() library function for such use cases. See 
*  Appendix A of the code example document for the details of E-INK library 
*  functions.
*
*******************************************************************************/
void static DisplayImage(cy_eink_image_t* imagePointer)
{
    /* Counter variable for the startup screen */
    uint8_t static  startupScreenCounter = 0u;
    
    /* Turn on the Orange LED to indicate a display update in progress */
    Cy_GPIO_Write(Pin_LED_Orange_0_PORT, Pin_LED_Orange_0_NUM, 0u);
    
    /* If the existing image on the display belong to the startup type */
    if (currentScreenContent == STARTUP_CONTENT)
    {
        /* Perform a full update to avoid ghosting as the startup images differ
           significantly from one another */
        Cy_EINK_ShowFrame(currentFrame, imagePointer, CY_EINK_FULL_2STAGE,
                          CY_EINK_POWER_AUTO);
        
        /* Check if the end of startup screen is reached */
        if (startupScreenCounter >= NUMBER_OF_STARTUP_SCREENS)
        {
            /* Change the current screen content to main menu type */
            currentScreenContent = MAIN_MENU_CONTENT;
        }
        else
        {
            /* Increment the startup counter */
            startupScreenCounter++;
        }
    }
    /* If the existing image on the display belong to the startup type */
    else if (currentScreenContent == TEXT_PAGE_CONTENT)
    {
        /* Perform a full update to avoid ghosting as the text pages differ
           significantly from the main menu images */
        Cy_EINK_ShowFrame(currentFrame, imagePointer, CY_EINK_FULL_2STAGE,
                          CY_EINK_POWER_AUTO);
        
        /* Change the current screen content to main menu type */
        currentScreenContent = MAIN_MENU_CONTENT;
    }
    /* If the existing image on the display belong to the main menu type */
    else
    {
        /* Perform a partial update for a fast refresh as the main menu images 
           are similar */
        Cy_EINK_ShowFrame(currentFrame, imagePointer, CY_EINK_PARTIAL, 
                          CY_EINK_POWER_AUTO);
        
        /* Change the current screen content to main menu type */
        currentScreenContent = MAIN_MENU_CONTENT;
    }
    /* Store the pointer to the current image, which is required for subsequent 
       updates */
    currentFrame = imagePointer;
    
    /* Turn off the Orange LED to indicate that display update is finished */
    Cy_GPIO_Write(Pin_LED_Orange_0_PORT, Pin_LED_Orange_0_NUM, 1u);
}

/*******************************************************************************
* Function Name: void static DisplayImageAndText(char* text, cy_eink_image_t* 
*                                                            backgroundImage)
********************************************************************************
*
* Summary:
*  Displays a string of text with a background image on the E-INK display
*
* Parameters:
*  char* text                             : pointer to the character string
*  cy_eink_image_t* backgroundImage       : pointer to the background image
*
* Return:
*  None
*
* Side Effects:
*  This function is hard-coded to work with the images, text, and font used in 
*  this code example. Since this is NOT A GENERIC FUNCTION, it NOT RECOMMENDED 
*  to copy/paste this function into a different project to display text on the 
*  E-INK display. It is recommended to use the E-INK library functions for such 
*  use cases. See Appendix A of the code example document for the details of 
*  E-INK library functions.
*
*******************************************************************************/
void static DisplayImageAndText(char* text, cy_eink_image_t* backgroundImage)
{
    /* Text pages used in this project start printing from these coordinates.
       See Appendix A of the code example document for the details of text and 
       font */
    const uint8_t   textOrigin[] = TEXT_PAGE_ORIGIN;
    
    /* Turn on the Orange LED to indicate a display update in progress */
    Cy_GPIO_Write(Pin_LED_Orange_0_PORT, Pin_LED_Orange_0_NUM, 0u);
    
    /* Initialize the frame buffers with the background image. See the function
       definition for more details */
    InitializeFrameBuffers(backgroundImage);
    
    /* Two frame buffers are used in this project since the E-INK display 
       requires the retention of the previous as well as the current frame. The 
       code below finds the frame buffer used in the previous update and selects
       the other frame buffer to be overwritten by the current operation */
    if (currentFrameBuffer == BUFFER0)
    {
        currentFrameBuffer = BUFFER1;
    }
    else
    {
        currentFrameBuffer = BUFFER0;
    }
    /* Load the frame buffer with the current text Page content */
    Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], text,
                              CY_EINK_FONT_8X12BLACK, (uint8_t*) textOrigin);
    
    /* Perform a full update to avoid ghosting as the text pages differ
       significantly from one another and also from the main menu images */
    Cy_EINK_ShowFrame(currentFrame, frameBuffer[currentFrameBuffer],
                      CY_EINK_FULL_2STAGE, CY_EINK_POWER_AUTO);
    
    /* Store the pointer to the current frame, which is required for subsequent 
       updates */
    currentFrame = frameBuffer[currentFrameBuffer];
    
    /* Change the current screen content to text page type */
    currentScreenContent = TEXT_PAGE_CONTENT;
    
    /* Turn off the Orange LED to indicate that display update is finished */
    Cy_GPIO_Write(Pin_LED_Orange_0_PORT, Pin_LED_Orange_0_NUM, 1u);
}

/* [] END OF FILE */
