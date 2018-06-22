/******************************************************************************
* File Name: screen_contents.h
*
* Version: 1.00
*
* Description: This file contains the and variable declarations that can be used 
*              to access the image and text stored in screen_contents.c 
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
/********************************************************************************
* This file contains the and variable declarations that can be used to access 
* the image and text stored in screen_contents.c 
*
* For the details of the E-INK display and library functions, see the code 
* example CE218133 - PSoC 6 MCU E-INK Display with CapSense
********************************************************************************/

/* Include Guard */
#ifndef SCREEN_CONTENTS_H
#define SCREEN_CONTENTS_H
    
/* Header file includes */
#include "cy_eink_library.h"

/* Variables from screen_contents.c that store images and text in flash */
extern cy_eink_image_t const logoAndHeading[CY_EINK_IMAGE_SIZE];
extern char const instructions[];

#endif /* SCREEN_CONTENTS_H */
/* [] END OF FILE */
