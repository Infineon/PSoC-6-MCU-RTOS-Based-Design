/******************************************************************************
* File Name: cy_eink_fonts.h
*
* Version: 1.00
*
* Description: This file contains macros, structures and constant variable
*              declarations that can be used to access the font information 
*              stored in the cy_eink_fonts.c file.
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
* This file contains macros, structures and constant variable declarations that
* can be used to access the font information stored in the cy_eink_fonts.c file.
* 
* For the details of the E-INK display and library functions, see the code  
* example document of CE218133 - PSoC 6 MCU E-INK Display with CapSense
*******************************************************************************/

/* Include Guard */
#ifndef CY_EINK_FONTS_H
#define CY_EINK_FONTS_H 

/* Header file includes */
#include <project.h>

/* Structure that contains font information */  
typedef struct
{
    /* Pointer to the font pixel data array */
    uint8_t* fontData;
    /* X offset of the font in pixels */
    uint8_t  xOffset;
    /* Y offset of the font in pixels */
    uint8_t  yOffset;
    /* X size of one font data in bytes */
    uint8_t  xSize;
    /* Y size of one font data in bytes */
    uint8_t  ySize;
    /* Number of characters that fit the screen horizontally */
    uint8_t  xSpan;
    /* Number of characters that fit the screen vertically */
    uint8_t  ySpan;
    /* Color of the font : true  = black characters in white background
                           false = white characters in black background */
    bool color;
}   cy_eink_font_t;

/* Two predefined fonts provided by the library */
extern cy_eink_font_t cy_eink_font8By12_blackInWhite;
extern cy_eink_font_t cy_eink_font16By16_blackInWhite; 

#endif  /* CY_EINK_FONTS_H */
/* [] END OF FILE */
