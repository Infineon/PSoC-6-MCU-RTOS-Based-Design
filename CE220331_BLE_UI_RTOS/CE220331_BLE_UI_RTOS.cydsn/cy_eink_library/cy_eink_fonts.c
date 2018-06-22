/******************************************************************************
* File Name: cy_eink_fonts.c
*
* Version: 1.00
*
* Description: This file stores the font data in flash.
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
* This file contains no executable code. This file just stores the font data 
* in flash.
*
* For the details of the E-INK display and library functions, see the code  
* example document of CE218133 - PSoC 6 MCU E-INK Display with CapSense
*******************************************************************************/

/* Header file includes */
#include "cy_eink_fonts.h"

/* Definitions of Font attributes for 8 pixel x 12 pixel font (1 byte x 12 bytes
   in storage ), with equal offsets and black in white background */
#define CY_EINK_FONT8x12_X_OFFSET           (uint8_t)(0x00)
#define CY_EINK_FONT8x12_Y_OFFSET           (uint8_t)(0x04)
#define CY_EINK_FONT8x12_X_SIZE             (uint8_t)(0x01)
#define CY_EINK_FONT8x12_Y_SIZE             (uint8_t)(0x0c)
#define CY_EINK_FONT8x12_X_SPAN             (uint8_t)(0x21)
#define CY_EINK_FONT8x12_Y_SPAN             (uint8_t)(0x0E)
#define CY_EINK_FONT8x12_COLOR              true
    
/* Definitions of Font attributes for 16 pixel x 16 pixel font (1 byte x 12 
   bytes in storage), with equal offsets and black in white background */
#define CY_EINK_FONT16x16_X_OFFSET          (uint8_t)(0x00)
#define CY_EINK_FONT16x16_Y_OFFSET          (uint8_t)(0x00)
#define CY_EINK_FONT16x16_X_SIZE            (uint8_t)(0x02)
#define CY_EINK_FONT16x16_Y_SIZE            (uint8_t)(0x10)
#define CY_EINK_FONT16x16_X_SPAN            (uint8_t)(0x10)
#define CY_EINK_FONT16x16_Y_SPAN            (uint8_t)(0x0B)
#define CY_EINK_FONT16x16_COLOR             true

/* Character bitmaps for an 8 pixel x 12 pixel font (1 byte x 12 bytes
   in storage) */
uint8_t const cy_eink_fontData8By12[] = 
{
    /* Starting address:0 '!' */
    0x00, //        
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x00, //        
    0x00, //        
    0x10, //    #   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:12 '"' */
    0x00, //        
    0x6C, //  ## ## 
    0x48, //  #  #  
    0x48, //  #  #  
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:24 '#' */
    0x00, //        
    0x14, //    # # 
    0x14, //    # # 
    0x28, //   # #  
    0x7C, //  ##### 
    0x28, //   # #  
    0x7C, //  ##### 
    0x28, //   # #  
    0x50, //  # #   
    0x50, //  # #   
    0x00, //        
    0x00, //        

    /* Starting address:36 '$' */
    0x00, //        
    0x10, //    #   
    0x38, //   ###  
    0x40, //  #     
    0x40, //  #     
    0x38, //   ###  
    0x48, //  #  #  
    0x70, //  ###   
    0x10, //    #   
    0x10, //    #   
    0x00, //        
    0x00, //        

    /* Starting address:48 '%' */
    0x00, //        
    0x20, //   #    
    0x50, //  # #   
    0x20, //   #    
    0x0C, //     ## 
    0x70, //  ###   
    0x08, //     #  
    0x14, //    # # 
    0x08, //     #  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:60 '&' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x18, //    ##  
    0x20, //   #    
    0x20, //   #    
    0x54, //  # # # 
    0x48, //  #  #  
    0x34, //   ## # 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:72 ''' */
    0x00, //        
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:84 '(' */
    0x00, //        
    0x08, //     #  
    0x08, //     #  
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x08, //     #  
    0x08, //     #  
    0x00, //        

    /* Starting address:96 ')' */
    0x00, //        
    0x20, //   #    
    0x20, //   #    
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x20, //   #    
    0x20, //   #    
    0x00, //        

    /* Starting address:108 '*' */
    0x00, //        
    0x10, //    #   
    0x7C, //  ##### 
    0x10, //    #   
    0x28, //   # #  
    0x28, //   # #  
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:120 '+' */
    0x00, //        
    0x00, //        
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0xFE, // #######
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:132 ',' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x18, //    ##  
    0x10, //    #   
    0x30, //   ##   
    0x20, //   #    
    0x00, //        

    /* Starting address:144 '-' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:156 '.' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x30, //   ##   
    0x30, //   ##   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:168 '/' */
    0x00, //        
    0x04, //      # 
    0x04, //      # 
    0x08, //     #  
    0x08, //     #  
    0x10, //    #   
    0x10, //    #   
    0x20, //   #    
    0x20, //   #    
    0x40, //  #     
    0x00, //        
    0x00, //        

    /* Starting address:180 '0' */
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:192 '1' */
    0x00, //        
    0x30, //   ##   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:204 '2' */
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x04, //      # 
    0x08, //     #  
    0x10, //    #   
    0x20, //   #    
    0x44, //  #   # 
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:216 '3' */
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x04, //      # 
    0x18, //    ##  
    0x04, //      # 
    0x04, //      # 
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:228 '4' */
    0x00, //        
    0x0C, //     ## 
    0x14, //    # # 
    0x14, //    # # 
    0x24, //   #  # 
    0x44, //  #   # 
    0x7E, //  ######
    0x04, //      # 
    0x0E, //     ###
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:240 '5' */
    0x00, //        
    0x3C, //   #### 
    0x20, //   #    
    0x20, //   #    
    0x38, //   ###  
    0x04, //      # 
    0x04, //      # 
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:252 '6' */
    0x00, //        
    0x1C, //    ### 
    0x20, //   #    
    0x40, //  #     
    0x78, //  ####  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:264 '7' */
    0x00, //        
    0x7C, //  ##### 
    0x44, //  #   # 
    0x04, //      # 
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x10, //    #   
    0x10, //    #   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:276 '8' */
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x44, //  #   # 
    0x38, //   ###  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:288 '9' */
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x3C, //   #### 
    0x04, //      # 
    0x08, //     #  
    0x70, //  ###   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:300 ':' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x30, //   ##   
    0x30, //   ##   
    0x00, //        
    0x00, //        
    0x30, //   ##   
    0x30, //   ##   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:312 ';' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x18, //    ##  
    0x18, //    ##  
    0x00, //        
    0x00, //        
    0x18, //    ##  
    0x30, //   ##   
    0x20, //   #    
    0x00, //        
    0x00, //        

    /* Starting address:324 '<' */
    0x00, //        
    0x00, //        
    0x0C, //     ## 
    0x10, //    #   
    0x60, //  ##    
    0x80, // #      
    0x60, //  ##    
    0x10, //    #   
    0x0C, //     ## 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:336 '=' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x7C, //  ##### 
    0x00, //        
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:348 '>' */
    0x00, //        
    0x00, //        
    0xC0, // ##     
    0x20, //   #    
    0x18, //    ##  
    0x04, //      # 
    0x18, //    ##  
    0x20, //   #    
    0xC0, // ##     
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:360 '?' */
    0x00, //        
    0x00, //        
    0x18, //    ##  
    0x24, //   #  # 
    0x04, //      # 
    0x08, //     #  
    0x10, //    #   
    0x00, //        
    0x30, //   ##   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:372 '@' */
    0x38, //   ###  
    0x44, //  #   # 
    0x44, //  #   # 
    0x4C, //  #  ## 
    0x54, //  # # # 
    0x54, //  # # # 
    0x4C, //  #  ## 
    0x40, //  #     
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        

    /* Starting address:384 'A' */
    0x00, //        
    0x30, //   ##   
    0x10, //    #   
    0x28, //   # #  
    0x28, //   # #  
    0x28, //   # #  
    0x7C, //  ##### 
    0x44, //  #   # 
    0xEE, // ### ###
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:396 'B' */
    0x00, //        
    0xF8, // #####  
    0x44, //  #   # 
    0x44, //  #   # 
    0x78, //  ####  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0xF8, // #####  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:408 'C' */
    0x00, //        
    0x3C, //   #### 
    0x44, //  #   # 
    0x40, //  #     
    0x40, //  #     
    0x40, //  #     
    0x40, //  #     
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:420 'D' */
    0x00, //        
    0xF0, // ####   
    0x48, //  #  #  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x48, //  #  #  
    0xF0, // ####   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:432 'E' */
    0x00, //        
    0xFC, // ###### 
    0x44, //  #   # 
    0x50, //  # #   
    0x70, //  ###   
    0x50, //  # #   
    0x40, //  #     
    0x44, //  #   # 
    0xFC, // ###### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:444 'F' */
    0x00, //        
    0x7E, //  ######
    0x22, //   #   #
    0x28, //   # #  
    0x38, //   ###  
    0x28, //   # #  
    0x20, //   #    
    0x20, //   #    
    0x70, //  ###   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:456 'G' */
    0x00, //        
    0x3C, //   #### 
    0x44, //  #   # 
    0x40, //  #     
    0x40, //  #     
    0x4E, //  #  ###
    0x44, //  #   # 
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:468 'H' */
    0x00, //        
    0xEE, // ### ###
    0x44, //  #   # 
    0x44, //  #   # 
    0x7C, //  ##### 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0xEE, // ### ###
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:480 'I' */
    0x00, //        
    0x7C, //  ##### 
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:492 'J' */
    0x00, //        
    0x3C, //   #### 
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x48, //  #  #  
    0x48, //  #  #  
    0x48, //  #  #  
    0x30, //   ##   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:504 'K' */
    0x00, //        
    0xEE, // ### ###
    0x44, //  #   # 
    0x48, //  #  #  
    0x50, //  # #   
    0x70, //  ###   
    0x48, //  #  #  
    0x44, //  #   # 
    0xE6, // ###  ##
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:516 'L' */
    0x00, //        
    0x70, //  ###   
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x24, //   #  # 
    0x24, //   #  # 
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:528 'M' */
    0x00, //        
    0xEE, // ### ###
    0x6C, //  ## ## 
    0x6C, //  ## ## 
    0x54, //  # # # 
    0x54, //  # # # 
    0x44, //  #   # 
    0x44, //  #   # 
    0xEE, // ### ###
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:540 'N' */
    0x00, //        
    0xEE, // ### ###
    0x64, //  ##  # 
    0x64, //  ##  # 
    0x54, //  # # # 
    0x54, //  # # # 
    0x54, //  # # # 
    0x4C, //  #  ## 
    0xEC, // ### ## 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:552 'O' */
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:564 'P' */
    0x00, //        
    0x78, //  ####  
    0x24, //   #  # 
    0x24, //   #  # 
    0x24, //   #  # 
    0x38, //   ###  
    0x20, //   #    
    0x20, //   #    
    0x70, //  ###   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:576 'Q' */
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x38, //   ###  
    0x1C, //    ### 
    0x00, //        
    0x00, //        

    /* Starting address:588 'R' */
    0x00, //        
    0xF8, // #####  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x78, //  ####  
    0x48, //  #  #  
    0x44, //  #   # 
    0xE2, // ###   #
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:600 'S' */
    0x00, //        
    0x34, //   ## # 
    0x4C, //  #  ## 
    0x40, //  #     
    0x38, //   ###  
    0x04, //      # 
    0x04, //      # 
    0x64, //  ##  # 
    0x58, //  # ##  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:612 'T' */
    0x00, //        
    0xFE, // #######
    0x92, // #  #  #
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:624 'U' */
    0x00, //        
    0xEE, // ### ###
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:636 'V' */
    0x00, //        
    0xEE, // ### ###
    0x44, //  #   # 
    0x44, //  #   # 
    0x28, //   # #  
    0x28, //   # #  
    0x28, //   # #  
    0x10, //    #   
    0x10, //    #   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:648 'W' */
    0x00, //        
    0xEE, // ### ###
    0x44, //  #   # 
    0x44, //  #   # 
    0x54, //  # # # 
    0x54, //  # # # 
    0x54, //  # # # 
    0x54, //  # # # 
    0x28, //   # #  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:660 'X' */
    0x00, //        
    0xC6, // ##   ##
    0x44, //  #   # 
    0x28, //   # #  
    0x10, //    #   
    0x10, //    #   
    0x28, //   # #  
    0x44, //  #   # 
    0xC6, // ##   ##
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:672 'Y' */
    0x00, //        
    0xEE, // ### ###
    0x44, //  #   # 
    0x28, //   # #  
    0x28, //   # #  
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:684 'Z' */
    0x00, //        
    0x7C, //  ##### 
    0x44, //  #   # 
    0x08, //     #  
    0x10, //    #   
    0x10, //    #   
    0x20, //   #    
    0x44, //  #   # 
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:696 '[' */
    0x00, //        
    0x38, //   ###  
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x38, //   ###  
    0x00, //        

    /* Starting address:708 '\' */
    0x00, //        
    0x40, //  #     
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x10, //    #   
    0x10, //    #   
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x00, //        
    0x00, //        

    /* Starting address:720 ']' */
    0x00, //        
    0x38, //   ###  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x38, //   ###  
    0x00, //        

    /* Starting address:732 '^' */
    0x00, //        
    0x10, //    #   
    0x10, //    #   
    0x28, //   # #  
    0x44, //  #   # 
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:744 '_' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0xFE, // #######

    /* Starting address:756 '`' */
    0x00, //        
    0x10, //    #   
    0x08, //     #  
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:768 'a' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x3C, //   #### 
    0x44, //  #   # 
    0x44, //  #   # 
    0x3E, //   #####
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:780 'b' */
    0x00, //        
    0xC0, // ##     
    0x40, //  #     
    0x58, //  # ##  
    0x64, //  ##  # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0xF8, // #####  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:792 'c' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x3C, //   #### 
    0x44, //  #   # 
    0x40, //  #     
    0x40, //  #     
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:804 'd' */
    0x00, //        
    0x0C, //     ## 
    0x04, //      # 
    0x34, //   ## # 
    0x4C, //  #  ## 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x3E, //   #####
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:816 'e' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x7C, //  ##### 
    0x40, //  #     
    0x40, //  #     
    0x3C, //   #### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:828 'f' */
    0x00, //        
    0x1C, //    ### 
    0x20, //   #    
    0x7C, //  ##### 
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:840 'g' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x36, //   ## ##
    0x4C, //  #  ## 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x3C, //   #### 
    0x04, //      # 
    0x38, //   ###  
    0x00, //        

    /* Starting address:852 'h' */
    0x00, //        
    0xC0, // ##     
    0x40, //  #     
    0x58, //  # ##  
    0x64, //  ##  # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0xEE, // ### ###
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:864 'i' */
    0x00, //        
    0x10, //    #   
    0x00, //        
    0x70, //  ###   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:876 'j' */
    0x00, //        
    0x10, //    #   
    0x00, //        
    0x78, //  ####  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x08, //     #  
    0x70, //  ###   
    0x00, //        

    /* Starting address:888 'k' */
    0x00, //        
    0xC0, // ##     
    0x40, //  #     
    0x5C, //  # ### 
    0x48, //  #  #  
    0x70, //  ###   
    0x50, //  # #   
    0x48, //  #  #  
    0xDC, // ## ### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:900 'l' */
    0x00, //        
    0x30, //   ##   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:912 'm' */
    0x00, //        
    0x00, //        
    0x00, //        
    0xE8, // ### #  
    0x54, //  # # # 
    0x54, //  # # # 
    0x54, //  # # # 
    0x54, //  # # # 
    0xFE, // #######
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:924 'n' */
    0x00, //        
    0x00, //        
    0x00, //        
    0xD8, // ## ##  
    0x64, //  ##  # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0xEE, // ### ###
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:936 'o' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x38, //   ###  
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x38, //   ###  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:948 'p' */
    0x00, //        
    0x00, //        
    0x00, //        
    0xD8, // ## ##  
    0x64, //  ##  # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x78, //  ####  
    0x40, //  #     
    0xE0, // ###    
    0x00, //        

    /* Starting address:960 'q' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x36, //   ## ##
    0x4C, //  #  ## 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x3C, //   #### 
    0x04, //      # 
    0x0E, //     ###
    0x00, //        

    /* Starting address:972 'r' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x6C, //  ## ## 
    0x30, //   ##   
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:984 's' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x3C, //   #### 
    0x44, //  #   # 
    0x38, //   ###  
    0x04, //      # 
    0x44, //  #   # 
    0x78, //  ####  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:996 't' */
    0x00, //        
    0x00, //        
    0x20, //   #    
    0x7C, //  ##### 
    0x20, //   #    
    0x20, //   #    
    0x20, //   #    
    0x22, //   #   #
    0x1C, //    ### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:1008 'u' */
    0x00, //        
    0x00, //        
    0x00, //        
    0xCC, // ##  ## 
    0x44, //  #   # 
    0x44, //  #   # 
    0x44, //  #   # 
    0x4C, //  #  ## 
    0x36, //   ## ##
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:1020 'v' */
    0x00, //        
    0x00, //        
    0x00, //        
    0xEE, // ### ###
    0x44, //  #   # 
    0x44, //  #   # 
    0x28, //   # #  
    0x28, //   # #  
    0x10, //    #   
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:1032 'w' */
    0x00, //        
    0x00, //        
    0x00, //        
    0xEE, // ### ###
    0x44, //  #   # 
    0x54, //  # # # 
    0x54, //  # # # 
    0x54, //  # # # 
    0x28, //   # #  
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:1044 'x' */
    0x00, //        
    0x00, //        
    0x00, //        
    0xCC, // ##  ## 
    0x48, //  #  #  
    0x30, //   ##   
    0x30, //   ##   
    0x48, //  #  #  
    0xCC, // ##  ## 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:1056 'y' */
    0x00, //        
    0x00, //        
    0x00, //        
    0xEE, // ### ###
    0x44, //  #   # 
    0x24, //   #  # 
    0x28, //   # #  
    0x18, //    ##  
    0x10, //    #   
    0x10, //    #   
    0x78, //  ####  
    0x00, //        

    /* Starting address:1068 'z' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x7C, //  ##### 
    0x48, //  #  #  
    0x10, //    #   
    0x20, //   #    
    0x44, //  #   # 
    0x7C, //  ##### 
    0x00, //        
    0x00, //        
    0x00, //        

    /* Starting address:1080 '{' */
    0x00, //        
    0x08, //     #  
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x20, //   #    
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x08, //     #  
    0x00, //        

    /* Starting address:1092 '|' */
    0x00, //        
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x00, //        
    0x00, //        

    /* Starting address:1104 '}' */
    0x00, //        
    0x20, //   #    
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x08, //     #  
    0x10, //    #   
    0x10, //    #   
    0x10, //    #   
    0x20, //   #    
    0x00, //        

    /* Starting address:1116 '~' */
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x24, //   #  # 
    0x58, //  # ##  
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
    0x00, //        
};

/* Character bitmaps for a 16 pixel x 16 pixel font (2 byte x 16 bytes in storage ) */
uint8_t const cy_eink_fontData16By16[] = 
{
    // @0 '!' (16 pixels wide)
    0x00, 0x00, //                 
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x01, 0x80, //        ##       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @32 '"' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xE0, //      ######     
    0x07, 0xE0, //      ######     
    0x07, 0xE0, //      ######     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @64 '#' (16 pixels wide)
    0x00, 0x00, //                 
    0x03, 0x60, //       ## ##     
    0x03, 0x60, //       ## ##     
    0x03, 0x60, //       ## ##     
    0x1F, 0xF8, //    ##########   
    0x06, 0xC0, //      ## ##      
    0x06, 0xC0, //      ## ##      
    0x06, 0xC0, //      ## ##      
    0x06, 0xC0, //      ## ##      
    0x3F, 0xF0, //   ##########    
    0x0D, 0x80, //     ## ##       
    0x0D, 0x80, //     ## ##       
    0x0D, 0x80, //     ## ##       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @96 '$' (16 pixels wide)
    0x01, 0x80, //        ##       
    0x07, 0xF0, //      #######    
    0x0F, 0xB0, //     ##### ##    
    0x0D, 0x80, //     ## ##       
    0x0D, 0x80, //     ## ##       
    0x0F, 0x80, //     #####       
    0x07, 0x80, //      ####       
    0x03, 0xC0, //       ####      
    0x01, 0xE0, //        ####     
    0x01, 0xF0, //        #####    
    0x01, 0xB0, //        ## ##    
    0x0D, 0xF0, //     ## #####    
    0x0F, 0xC0, //     ######      
    0x01, 0x80, //        ##       
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @128 '%' (16 pixels wide)
    0x00, 0x00, //                 
    0x3C, 0x30, //   ####    ##    
    0x66, 0x30, //  ##  ##   ##    
    0x66, 0x60, //  ##  ##  ##     
    0x66, 0xC0, //  ##  ## ##      
    0x67, 0x80, //  ##  ####       
    0x3F, 0x00, //   ######        
    0x06, 0xF0, //      ## ####    
    0x07, 0x98, //      ####  ##   
    0x0D, 0x98, //     ## ##  ##   
    0x19, 0x98, //    ##  ##  ##   
    0x31, 0x98, //   ##   ##  ##   
    0x60, 0xF0, //  ##     ####    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @160 '&' (16 pixels wide)
    0x00, 0x00, //                 
    0x0F, 0x80, //     #####       
    0x18, 0xC0, //    ##   ##      
    0x18, 0xC0, //    ##   ##      
    0x18, 0xC0, //    ##   ##      
    0x1F, 0x80, //    ######       
    0x1E, 0x00, //    ####         
    0x3F, 0x78, //   ###### ####   
    0x67, 0xB0, //  ##  #### ##    
    0x63, 0xB0, //  ##   ### ##    
    0x61, 0xE0, //  ##    ####     
    0x38, 0xE0, //   ###   ###     
    0x1F, 0xF0, //    #########    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @192 ''' (16 pixels wide)
    0x00, 0x00, //                 
    0x03, 0x80, //       ###       
    0x03, 0x80, //       ###       
    0x03, 0x80, //       ###       
    0x03, 0x80, //       ###       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @224 '(' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0xC0, //         ##      
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x01, 0x80, //        ##       
    0x00, 0xC0, //         ##      
    0x00, 0x00, //                 

    // @256 ')' (16 pixels wide)
    0x00, 0x00, //                 
    0x06, 0x00, //      ##         
    0x03, 0x00, //       ##        
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x00, 0x00, //                 

    // @288 '*' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x0F, 0xC0, //     ######      
    0x0F, 0xC0, //     ######      
    0x07, 0x80, //      ####       
    0x0F, 0xC0, //     ######      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @320 '+' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x1F, 0xF8, //    ##########   
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @352 ',' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x00, 0x00, //                 

    // @384 '-' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x07, 0xC0, //      #####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @416 '.' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @448 '/' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x30, //           ##    
    0x00, 0x30, //           ##    
    0x00, 0x60, //          ##     
    0x00, 0x60, //          ##     
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x00, 0x00, //                 

    // @480 '0' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xC0, //      #####      
    0x0C, 0x60, //     ##   ##     
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x0C, 0x60, //     ##   ##     
    0x07, 0xC0, //      #####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @512 '1' (16 pixels wide)
    0x00, 0x00, //                 
    0x01, 0x80, //        ##       
    0x07, 0x80, //      ####       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x07, 0xE0, //      ######     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @544 '2' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0x80, //      ####       
    0x0C, 0xC0, //     ##  ##      
    0x18, 0x60, //    ##    ##     
    0x00, 0x60, //          ##     
    0x00, 0x60, //          ##     
    0x00, 0xC0, //         ##      
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x0C, 0x00, //     ##          
    0x18, 0x00, //    ##           
    0x1F, 0xE0, //    ########     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @576 '3' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xC0, //      #####      
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x30, //     ##    ##    
    0x00, 0x30, //           ##    
    0x00, 0x60, //          ##     
    0x07, 0xC0, //      #####      
    0x00, 0x60, //          ##     
    0x00, 0x30, //           ##    
    0x00, 0x30, //           ##    
    0x00, 0x30, //           ##    
    0x0C, 0x60, //     ##   ##     
    0x07, 0xC0, //      #####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @608 '4' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0xC0, //         ##      
    0x01, 0xC0, //        ###      
    0x01, 0xC0, //        ###      
    0x03, 0xC0, //       ####      
    0x06, 0xC0, //      ## ##      
    0x0C, 0xC0, //     ##  ##      
    0x18, 0xC0, //    ##   ##      
    0x1F, 0xF0, //    #########    
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x01, 0xE0, //        ####     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @640 '5' (16 pixels wide)
    0x00, 0x00, //                 
    0x0F, 0xE0, //     #######     
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0F, 0x80, //     #####       
    0x00, 0xC0, //         ##      
    0x00, 0x60, //          ##     
    0x00, 0x60, //          ##     
    0x00, 0x60, //          ##     
    0x00, 0x60, //          ##     
    0x0C, 0xC0, //     ##  ##      
    0x07, 0x80, //      ####       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @672 '6' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xE0, //      ######     
    0x0C, 0x30, //     ##    ##    
    0x18, 0x00, //    ##           
    0x18, 0x00, //    ##           
    0x1F, 0xC0, //    #######      
    0x1C, 0x60, //    ###   ##     
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x0C, 0x60, //     ##   ##     
    0x07, 0xC0, //      #####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @704 '7' (16 pixels wide)
    0x00, 0x00, //                 
    0x0F, 0xF0, //     ########    
    0x00, 0x60, //          ##     
    0x00, 0x60, //          ##     
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @736 '8' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xE0, //      ######     
    0x0E, 0x70, //     ###  ###    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x07, 0x60, //      ### ##     
    0x07, 0xC0, //      #####      
    0x0D, 0xE0, //     ## ####     
    0x18, 0x70, //    ##    ###    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x1C, 0x60, //    ###   ##     
    0x07, 0xC0, //      #####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @768 '9' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xC0, //      #####      
    0x0C, 0x60, //     ##   ##     
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x0C, 0x30, //     ##    ##    
    0x07, 0xF0, //      #######    
    0x00, 0x30, //           ##    
    0x00, 0x30, //           ##    
    0x18, 0x60, //    ##    ##     
    0x1F, 0xC0, //    #######      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @800 ':' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @832 ';' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x00, 0x00, //                 

    // @864 '<' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x30, //           ##    
    0x00, 0xE0, //         ###     
    0x01, 0x80, //        ##       
    0x07, 0x00, //      ###        
    0x1C, 0x00, //    ###          
    0x07, 0x00, //      ###        
    0x01, 0x80, //        ##       
    0x00, 0xE0, //         ###     
    0x00, 0x30, //           ##    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @896 '=' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x1F, 0xF0, //    #########    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x1F, 0xF0, //    #########    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @928 '>' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x18, 0x00, //    ##           
    0x0E, 0x00, //     ###         
    0x03, 0x00, //       ##        
    0x01, 0xC0, //        ###      
    0x00, 0x60, //          ##     
    0x01, 0xC0, //        ###      
    0x07, 0x00, //      ###        
    0x0C, 0x00, //     ##          
    0x18, 0x00, //    ##           
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @960 '?' (16 pixels wide)
    0x00, 0x00, //                 
    0x0F, 0xC0, //     ######      
    0x0C, 0x60, //     ##   ##     
    0x00, 0x60, //          ##     
    0x00, 0x60, //          ##     
    0x00, 0xC0, //         ##      
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x06, 0x00, //      ##         
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @992 '@' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xE0, //      ######     
    0x0C, 0x30, //     ##    ##    
    0x18, 0x18, //    ##      ##   
    0x33, 0xEC, //   ##  ##### ##  
    0x66, 0x6C, //  ##  ##  ## ##  
    0x6C, 0xEC, //  ## ##  ### ##  
    0x6C, 0xCC, //  ## ##  ##  ##  
    0x6F, 0xD8, //  ## ###### ##   
    0x6F, 0xF0, //  ## ########    
    0x30, 0x00, //   ##            
    0x18, 0x00, //    ##           
    0x0F, 0xC0, //     ######      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1024 'A' (16 pixels wide)
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x07, 0x00, //      ###        
    0x07, 0x80, //      ####       
    0x0D, 0x80, //     ## ##       
    0x0D, 0x80, //     ## ##       
    0x0C, 0xC0, //     ##  ##      
    0x18, 0xC0, //    ##   ##      
    0x1F, 0xC0, //    #######      
    0x18, 0x60, //    ##    ##     
    0x30, 0x60, //   ##     ##     
    0x30, 0x30, //   ##      ##    
    0x78, 0x78, //  ####    ####   
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1056 'B' (16 pixels wide)
    0x00, 0x00, //                 
    0x1F, 0xC0, //    #######      
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x60, //     ##   ##     
    0x0F, 0xC0, //     ######      
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x60, //     ##   ##     
    0x1F, 0xC0, //    #######      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1088 'C' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xE0, //      ######     
    0x0C, 0x70, //     ##   ###    
    0x18, 0x30, //    ##     ##    
    0x30, 0x00, //   ##            
    0x30, 0x00, //   ##            
    0x30, 0x00, //   ##            
    0x30, 0x00, //   ##            
    0x30, 0x00, //   ##            
    0x30, 0x00, //   ##            
    0x18, 0x00, //    ##           
    0x0C, 0x30, //     ##    ##    
    0x07, 0xE0, //      ######     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1120 'D' (16 pixels wide)
    0x00, 0x00, //                 
    0x3F, 0xE0, //   #########     
    0x18, 0x30, //    ##     ##    
    0x18, 0x18, //    ##      ##   
    0x18, 0x0C, //    ##       ##  
    0x18, 0x0C, //    ##       ##  
    0x18, 0x0C, //    ##       ##  
    0x18, 0x0C, //    ##       ##  
    0x18, 0x0C, //    ##       ##  
    0x18, 0x0C, //    ##       ##  
    0x18, 0x18, //    ##      ##   
    0x18, 0x30, //    ##     ##    
    0x3F, 0xE0, //   #########     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1152 'E' (16 pixels wide)
    0x00, 0x00, //                 
    0x1F, 0xE0, //    ########     
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0xC0, //     ##  ##      
    0x0F, 0xC0, //     ######      
    0x0C, 0xC0, //     ##  ##      
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x60, //     ##   ##     
    0x1F, 0xE0, //    ########     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1184 'F' (16 pixels wide)
    0x00, 0x00, //                 
    0x1F, 0xE0, //    ########     
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0xC0, //     ##  ##      
    0x0F, 0xC0, //     ######      
    0x0C, 0xC0, //     ##  ##      
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x1E, 0x00, //    ####         
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1216 'G' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xE0, //      ######     
    0x0C, 0x70, //     ##   ###    
    0x18, 0x30, //    ##     ##    
    0x30, 0x00, //   ##            
    0x30, 0x00, //   ##            
    0x30, 0x00, //   ##            
    0x30, 0xF8, //   ##    #####   
    0x30, 0x30, //   ##      ##    
    0x30, 0x30, //   ##      ##    
    0x18, 0x30, //    ##     ##    
    0x0C, 0x30, //     ##    ##    
    0x07, 0xE0, //      ######     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1248 'H' (16 pixels wide)
    0x00, 0x00, //                 
    0x3C, 0x3C, //   ####    ####  
    0x18, 0x18, //    ##      ##   
    0x18, 0x18, //    ##      ##   
    0x18, 0x18, //    ##      ##   
    0x18, 0x18, //    ##      ##   
    0x1F, 0xF8, //    ##########   
    0x18, 0x18, //    ##      ##   
    0x18, 0x18, //    ##      ##   
    0x18, 0x18, //    ##      ##   
    0x18, 0x18, //    ##      ##   
    0x18, 0x18, //    ##      ##   
    0x3C, 0x3C, //   ####    ####  
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1280 'I' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0x80, //      ####       
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x07, 0x80, //      ####       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1312 'J' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xC0, //      #####      
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x1B, 0x00, //    ## ##        
    0x1E, 0x00, //    ####         
    0x00, 0x00, //                 

    // @1344 'K' (16 pixels wide)
    0x00, 0x00, //                 
    0x3C, 0xF0, //   ####  ####    
    0x18, 0x60, //    ##    ##     
    0x18, 0xC0, //    ##   ##      
    0x19, 0x80, //    ##  ##       
    0x1B, 0x00, //    ## ##        
    0x1E, 0x00, //    ####         
    0x1F, 0x00, //    #####        
    0x19, 0x80, //    ##  ##       
    0x18, 0xC0, //    ##   ##      
    0x18, 0x60, //    ##    ##     
    0x18, 0x30, //    ##     ##    
    0x3C, 0xF8, //   ####  #####   
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1376 'L' (16 pixels wide)
    0x00, 0x00, //                 
    0x1E, 0x00, //    ####         
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x1F, 0xF0, //    #########    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1408 'M' (16 pixels wide)
    0x00, 0x00, //                 
    0x78, 0x1C, //  ####      ###  
    0x38, 0x38, //   ###     ###   
    0x3C, 0x38, //   ####    ###   
    0x3C, 0x38, //   ####    ###   
    0x3C, 0x78, //   ####   ####   
    0x36, 0x78, //   ## ##  ####   
    0x36, 0x78, //   ## ##  ####   
    0x33, 0xD8, //   ##  #### ##   
    0x33, 0xD8, //   ##  #### ##   
    0x33, 0x98, //   ##  ###  ##   
    0x31, 0x98, //   ##   ##  ##   
    0x78, 0x3C, //  ####     ####  
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1440 'N' (16 pixels wide)
    0x00, 0x00, //                 
    0x38, 0x78, //   ###    ####   
    0x1C, 0x30, //    ###    ##    
    0x1E, 0x30, //    ####   ##    
    0x1E, 0x30, //    ####   ##    
    0x1B, 0x30, //    ## ##  ##    
    0x1B, 0x30, //    ## ##  ##    
    0x19, 0xB0, //    ##  ## ##    
    0x19, 0xB0, //    ##  ## ##    
    0x18, 0xF0, //    ##   ####    
    0x18, 0x70, //    ##    ###    
    0x18, 0x70, //    ##    ###    
    0x3C, 0x30, //   ####    ##    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1472 'O' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xE0, //      ######     
    0x0C, 0x30, //     ##    ##    
    0x18, 0x18, //    ##      ##   
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x18, 0x18, //    ##      ##   
    0x0C, 0x30, //     ##    ##    
    0x07, 0xE0, //      ######     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1504 'P' (16 pixels wide)
    0x00, 0x00, //                 
    0x1F, 0xC0, //    #######      
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x60, //     ##   ##     
    0x0F, 0xC0, //     ######      
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x1E, 0x00, //    ####         
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1536 'Q' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xE0, //      ######     
    0x0C, 0x30, //     ##    ##    
    0x18, 0x18, //    ##      ##   
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x30, 0x0C, //   ##        ##  
    0x18, 0x18, //    ##      ##   
    0x0C, 0x30, //     ##    ##    
    0x07, 0xE0, //      ######     
    0x00, 0x3C, //           ####  
    0x00, 0x0F, //             ####
    0x00, 0x00, //                 

    // @1568 'R' (16 pixels wide)
    0x00, 0x00, //                 
    0x3F, 0x80, //   #######       
    0x18, 0xC0, //    ##   ##      
    0x18, 0x60, //    ##    ##     
    0x18, 0x60, //    ##    ##     
    0x18, 0x60, //    ##    ##     
    0x18, 0xC0, //    ##   ##      
    0x1F, 0x80, //    ######       
    0x19, 0x80, //    ##  ##       
    0x18, 0xC0, //    ##   ##      
    0x18, 0x60, //    ##    ##     
    0x18, 0x30, //    ##     ##    
    0x3C, 0x38, //   ####    ###   
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1600 'S' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0xC0, //      #####      
    0x1C, 0x60, //    ###   ##     
    0x18, 0x60, //    ##    ##     
    0x18, 0x00, //    ##           
    0x0C, 0x00, //     ##          
    0x07, 0x00, //      ###        
    0x01, 0xC0, //        ###      
    0x00, 0xE0, //         ###     
    0x00, 0x60, //          ##     
    0x18, 0x60, //    ##    ##     
    0x1C, 0xC0, //    ###  ##      
    0x0F, 0x80, //     #####       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1632 'T' (16 pixels wide)
    0x00, 0x00, //                 
    0x7F, 0xE0, //  ##########     
    0x66, 0x60, //  ##  ##  ##     
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x0F, 0x00, //     ####        
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1664 'U' (16 pixels wide)
    0x00, 0x00, //                 
    0x3C, 0x78, //   ####   ####   
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x0C, 0x60, //     ##   ##     
    0x07, 0xC0, //      #####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1696 'V' (16 pixels wide)
    0x00, 0x00, //                 
    0x78, 0xF0, //  ####   ####    
    0x30, 0x60, //   ##     ##     
    0x18, 0x60, //    ##    ##     
    0x18, 0xC0, //    ##   ##      
    0x18, 0xC0, //    ##   ##      
    0x18, 0xC0, //    ##   ##      
    0x0D, 0x80, //     ## ##       
    0x0D, 0x80, //     ## ##       
    0x0D, 0x80, //     ## ##       
    0x07, 0x00, //      ###        
    0x07, 0x00, //      ###        
    0x07, 0x00, //      ###        
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1728 'W' (16 pixels wide)
    0x00, 0x00, //                 
    0xF3, 0x1E, // ####  ##   #### 
    0x63, 0x0C, //  ##   ##    ##  
    0x67, 0x8C, //  ##  ####   ##  
    0x37, 0x8C, //   ## ####   ##  
    0x37, 0x98, //   ## ####  ##   
    0x37, 0xD8, //   ## ##### ##   
    0x3C, 0xD8, //   ####  ## ##   
    0x3C, 0xD8, //   ####  ## ##   
    0x1C, 0xF0, //    ###  ####    
    0x1C, 0x70, //    ###   ###    
    0x1C, 0x70, //    ###   ###    
    0x18, 0x70, //    ##    ###    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1760 'X' (16 pixels wide)
    0x00, 0x00, //                 
    0x3E, 0x78, //   #####  ####   
    0x18, 0x30, //    ##     ##    
    0x0C, 0x60, //     ##   ##     
    0x06, 0xC0, //      ## ##      
    0x06, 0xC0, //      ## ##      
    0x03, 0x80, //       ###       
    0x03, 0x80, //       ###       
    0x06, 0xC0, //      ## ##      
    0x06, 0xC0, //      ## ##      
    0x0C, 0x60, //     ##   ##     
    0x18, 0x30, //    ##     ##    
    0x3C, 0xF8, //   ####  #####   
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1792 'Y' (16 pixels wide)
    0x00, 0x00, //                 
    0x3E, 0xF0, //   ##### ####    
    0x18, 0x60, //    ##    ##     
    0x0C, 0xC0, //     ##  ##      
    0x0C, 0xC0, //     ##  ##      
    0x07, 0x80, //      ####       
    0x07, 0x80, //      ####       
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x07, 0x80, //      ####       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1824 'Z' (16 pixels wide)
    0x00, 0x00, //                 
    0x1F, 0xF0, //    #########    
    0x18, 0x30, //    ##     ##    
    0x00, 0x60, //          ##     
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x0C, 0x00, //     ##          
    0x18, 0x30, //    ##     ##    
    0x1F, 0xF0, //    #########    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1856 '[' (16 pixels wide)
    0x00, 0x00, //                 
    0x03, 0xC0, //       ####      
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0xC0, //       ####      
    0x00, 0x00, //                 

    // @1888 '\' (16 pixels wide)
    0x00, 0x00, //                 
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0x60, //          ##     
    0x00, 0x60, //          ##     
    0x00, 0x30, //           ##    
    0x00, 0x30, //           ##    
    0x00, 0x00, //                 

    // @1920 ']' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0x80, //      ####       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x07, 0x80, //      ####       
    0x00, 0x00, //                 

    // @1952 '^' (16 pixels wide)
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x07, 0x80, //      ####       
    0x07, 0x80, //      ####       
    0x0F, 0xC0, //     ######      
    0x0C, 0xC0, //     ##  ##      
    0x0C, 0xC0, //     ##  ##      
    0x18, 0x60, //    ##    ##     
    0x18, 0x60, //    ##    ##     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @1984 '_' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x0F, 0xE0, //     #######     
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2016 '`' (16 pixels wide)
    0x00, 0x00, //                 
    0x03, 0x80, //       ###       
    0x01, 0xC0, //        ###      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2048 'a' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x07, 0x80, //      ####       
    0x0C, 0xC0, //     ##  ##      
    0x0C, 0xC0, //     ##  ##      
    0x00, 0xC0, //         ##      
    0x07, 0xC0, //      #####      
    0x1C, 0xC0, //    ###  ##      
    0x18, 0xC0, //    ##   ##      
    0x18, 0xC0, //    ##   ##      
    0x0F, 0xE0, //     #######     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2080 'b' (16 pixels wide)
    0x00, 0x00, //                 
    0x1C, 0x00, //    ###          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0F, 0xE0, //     #######     
    0x0E, 0x30, //     ###   ##    
    0x0C, 0x18, //     ##     ##   
    0x0C, 0x18, //     ##     ##   
    0x0C, 0x18, //     ##     ##   
    0x0C, 0x18, //     ##     ##   
    0x0C, 0x18, //     ##     ##   
    0x0E, 0x30, //     ###   ##    
    0x0F, 0xE0, //     #######     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2112 'c' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x03, 0xC0, //       ####      
    0x06, 0x60, //      ##  ##     
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x06, 0x60, //      ##  ##     
    0x03, 0xC0, //       ####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2144 'd' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0xF0, //         ####    
    0x00, 0x30, //           ##    
    0x00, 0x30, //           ##    
    0x07, 0xF0, //      #######    
    0x0C, 0x70, //     ##   ###    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x0C, 0x70, //     ##   ###    
    0x07, 0xF8, //      ########   
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2176 'e' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x07, 0xC0, //      #####      
    0x0C, 0x60, //     ##   ##     
    0x18, 0x60, //    ##    ##     
    0x1F, 0xE0, //    ########     
    0x18, 0x00, //    ##           
    0x18, 0x00, //    ##           
    0x18, 0x00, //    ##           
    0x0C, 0x60, //     ##   ##     
    0x07, 0xC0, //      #####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2208 'f' (16 pixels wide)
    0x00, 0x00, //                 
    0x01, 0xF0, //        #####    
    0x03, 0x30, //       ##  ##    
    0x03, 0x00, //       ##        
    0x07, 0xC0, //      #####      
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x07, 0x80, //      ####       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2240 'g' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x0F, 0xF0, //     ########    
    0x1C, 0xC0, //    ###  ##      
    0x18, 0xC0, //    ##   ##      
    0x18, 0xC0, //    ##   ##      
    0x1C, 0xC0, //    ###  ##      
    0x0F, 0x80, //     #####       
    0x18, 0x00, //    ##           
    0x1F, 0xC0, //    #######      
    0x18, 0x60, //    ##    ##     
    0x30, 0x60, //   ##     ##     
    0x30, 0xE0, //   ##    ###     
    0x1F, 0x80, //    ######       

    // @2272 'h' (16 pixels wide)
    0x00, 0x00, //                 
    0x1C, 0x00, //    ###          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0F, 0xE0, //     #######     
    0x0E, 0x30, //     ###   ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x1E, 0x78, //    ####  ####   
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2304 'i' (16 pixels wide)
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x07, 0x00, //      ###        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x07, 0x80, //      ####       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2336 'j' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0xC0, //         ##      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x01, 0xC0, //        ###      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x00, 0xC0, //         ##      
    0x0C, 0xC0, //     ##  ##      
    0x0F, 0x80, //     #####       

    // @2368 'k' (16 pixels wide)
    0x00, 0x00, //                 
    0x1C, 0x00, //    ###          
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x0C, 0xF0, //     ##  ####    
    0x0C, 0xC0, //     ##  ##      
    0x0D, 0x80, //     ## ##       
    0x0F, 0x00, //     ####        
    0x0E, 0x00, //     ###         
    0x0F, 0x00, //     ####        
    0x0D, 0x80, //     ## ##       
    0x0C, 0xC0, //     ##  ##      
    0x1E, 0x78, //    ####  ####   
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2400 'l' (16 pixels wide)
    0x00, 0x00, //                 
    0x07, 0x00, //      ###        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x07, 0x80, //      ####       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2432 'm' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0xFE, 0xF0, // ####### ####    
    0x73, 0x98, //  ###  ###  ##   
    0x63, 0x18, //  ##   ##   ##   
    0x63, 0x18, //  ##   ##   ##   
    0x63, 0x18, //  ##   ##   ##   
    0x63, 0x18, //  ##   ##   ##   
    0x63, 0x18, //  ##   ##   ##   
    0x63, 0x18, //  ##   ##   ##   
    0xF7, 0xBC, // #### #### ####  
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2464 'n' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x1F, 0xE0, //    ########     
    0x0E, 0x30, //     ###   ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x1E, 0x78, //    ####  ####   
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2496 'o' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x07, 0xC0, //      #####      
    0x0C, 0x60, //     ##   ##     
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x0C, 0x60, //     ##   ##     
    0x07, 0xC0, //      #####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2528 'p' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x1F, 0xE0, //    ########     
    0x0E, 0x30, //     ###   ##    
    0x0C, 0x18, //     ##     ##   
    0x0C, 0x18, //     ##     ##   
    0x0C, 0x18, //     ##     ##   
    0x0C, 0x18, //     ##     ##   
    0x0C, 0x18, //     ##     ##   
    0x0E, 0x30, //     ###   ##    
    0x0F, 0xE0, //     #######     
    0x0C, 0x00, //     ##          
    0x0C, 0x00, //     ##          
    0x1E, 0x00, //    ####         

    // @2560 'q' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x07, 0xF0, //      #######    
    0x0C, 0x70, //     ##   ###    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x18, 0x30, //    ##     ##    
    0x0C, 0x70, //     ##   ###    
    0x07, 0xF0, //      #######    
    0x00, 0x30, //           ##    
    0x00, 0x30, //           ##    
    0x00, 0x78, //          ####   

    // @2592 'r' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x0F, 0xE0, //     #######     
    0x07, 0x60, //      ### ##     
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x06, 0x00, //      ##         
    0x0F, 0x00, //     ####        
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2624 's' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x07, 0xC0, //      #####      
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x60, //     ##   ##     
    0x0E, 0x00, //     ###         
    0x03, 0xC0, //       ####      
    0x00, 0xE0, //         ###     
    0x0C, 0x60, //     ##   ##     
    0x0E, 0x60, //     ###  ##     
    0x07, 0xC0, //      #####      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2656 't' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x07, 0xC0, //      #####      
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x01, 0xC0, //        ###      
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2688 'u' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x1C, 0x70, //    ###   ###    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x30, //     ##    ##    
    0x0C, 0x70, //     ##   ###    
    0x07, 0xF8, //      ########   
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2720 'v' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x3E, 0xF0, //   ##### ####    
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x60, //     ##   ##     
    0x06, 0xC0, //      ## ##      
    0x06, 0xC0, //      ## ##      
    0x06, 0xC0, //      ## ##      
    0x03, 0x80, //       ###       
    0x03, 0x80, //       ###       
    0x03, 0x80, //       ###       
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2752 'w' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x7B, 0x3C, //  #### ##  ####  
    0x33, 0x18, //   ##  ##   ##   
    0x37, 0x98, //   ## ####  ##   
    0x37, 0x98, //   ## ####  ##   
    0x1F, 0xB0, //    ###### ##    
    0x1E, 0xF0, //    #### ####    
    0x1E, 0xF0, //    #### ####    
    0x0C, 0xE0, //     ##  ###     
    0x0C, 0x60, //     ##   ##     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2784 'x' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x1F, 0x70, //    ##### ###    
    0x0C, 0x60, //     ##   ##     
    0x06, 0xC0, //      ## ##      
    0x03, 0x80, //       ###       
    0x01, 0x80, //        ##       
    0x03, 0xC0, //       ####      
    0x06, 0xC0, //      ## ##      
    0x0C, 0x60, //     ##   ##     
    0x1D, 0xF0, //    ### #####    
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2816 'y' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x3E, 0xF0, //   ##### ####    
    0x0C, 0x60, //     ##   ##     
    0x0C, 0x60, //     ##   ##     
    0x06, 0xC0, //      ## ##      
    0x06, 0xC0, //      ## ##      
    0x06, 0xC0, //      ## ##      
    0x03, 0x80, //       ###       
    0x03, 0x80, //       ###       
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x0E, 0x00, //     ###         

    // @2848 'z' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x1F, 0xE0, //    ########     
    0x18, 0x60, //    ##    ##     
    0x00, 0xC0, //         ##      
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x0C, 0x00, //     ##          
    0x18, 0x60, //    ##    ##     
    0x1F, 0xE0, //    ########     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 

    // @2880 '{' (16 pixels wide)
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x01, 0x80, //        ##       
    0x00, 0x00, //                 

    // @2912 '|' (16 pixels wide)
    0x00, 0x00, //                 
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x01, 0x80, //        ##       
    0x00, 0x00, //                 

    // @2944 '}' (16 pixels wide)
    0x06, 0x00, //      ##         
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x01, 0x80, //        ##       
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x03, 0x00, //       ##        
    0x06, 0x00, //      ##         
    0x00, 0x00, //                 

    // @2976 '~' (16 pixels wide)
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x0E, 0x30, //     ###   ##    
    0x1B, 0xB0, //    ## ### ##    
    0x18, 0xE0, //    ##   ###     
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
    0x00, 0x00, //                 
};

/* Font with a constant character size of 8 pixels by 12 pixels, black in white 
   background */
cy_eink_font_t cy_eink_font8By12_blackInWhite =
{
    (uint8_t*)&cy_eink_fontData8By12,
    CY_EINK_FONT8x12_X_OFFSET,
    CY_EINK_FONT8x12_Y_OFFSET,
    CY_EINK_FONT8x12_X_SIZE,
    CY_EINK_FONT8x12_Y_SIZE,
    CY_EINK_FONT8x12_X_SPAN,
    CY_EINK_FONT8x12_Y_SPAN,
    CY_EINK_FONT8x12_COLOR
};

/* Font with a constant character size of 16 pixels by 16 pixels, black in white 
   background*/
cy_eink_font_t cy_eink_font16By16_blackInWhite =
{
    (uint8_t*)&cy_eink_fontData16By16,
    CY_EINK_FONT16x16_X_OFFSET,
    CY_EINK_FONT16x16_Y_OFFSET,
    CY_EINK_FONT16x16_X_SIZE,
    CY_EINK_FONT16x16_Y_SIZE,
    CY_EINK_FONT16x16_X_SPAN,
    CY_EINK_FONT16x16_Y_SPAN,
    CY_EINK_FONT16x16_COLOR    
};

/* [] END OF FILE */
