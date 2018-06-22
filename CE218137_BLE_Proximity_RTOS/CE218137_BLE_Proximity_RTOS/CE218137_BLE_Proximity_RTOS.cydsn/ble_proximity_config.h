/******************************************************************************
* File Name: ble_proximity_config.h
*
* Version: 1.00
*
* Description: This file contains the macros that enable the use of three custom
*              BLE profiles used in this code example
*
* Related Document: CE218137_BLE_Proximity_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
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
* This file contains the macros that enable the use of CapSense Slider, 
* CapSense Button and RGB LED custom BLE profiles used in this code example
********************************************************************************/

/* Include guard */
#ifndef BLE_CUSTOM_SERVICES_H
#define BLE_CUSTOM_SERVICES_H

/* Header file includes */
#include "project.h"    
    
/* Redefinition of long custom service macros for better readability of the
   code */
#define PROXIMITY_CCCD_HANDLE      \
(CY_BLE_CAPSENSE_PROXIMITY_CAPSENSE_PROXIMITY_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE)
#define PROXIMITY_CHAR_HANDLE      \
(CY_BLE_CAPSENSE_PROXIMITY_CAPSENSE_PROXIMITY_CHAR_HANDLE)    

/* For more details on the data formats of CapSense button, CapSense slider and 
   RGB LED Custom BLE Profiles provided by Cypress, see Cypress custom profile
   specifications available at:
   http://www.cypress.com/documentation/software-and-drivers/cypresss-custom-ble-profiles-and-services
*/
    
/* Size of Proximity characteristics data in bytes */
#define PROXIMITY_DATA_LEN    (uint8_t) (0x01u)

#endif /* BLE_CUSTOM_SERVICES_H */   
/* [] END OF FILE */
