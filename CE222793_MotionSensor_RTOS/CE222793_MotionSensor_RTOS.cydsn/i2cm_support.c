/*******************************************************************************
* File Name: i2cm_support.c
*
* Version 1.0
*
* Description: This file contains the functions for I2C read and write 
               operations for interfacing with the bmi160 motion sensor.
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

/* Header files include */ 
#include "i2cm_support.h"
#include "FreeRTOS.h"
#include "task.h"

/* Time Out */
#define I2CM_TIMEOUT        10 /* 10 msec */

/*******************************************************************************
* Function Name: I2C_WriteBytes()
********************************************************************************
* Summary:
*  Function that writes data to an I2C slave device 
*
* Parameters:
*  unsigned char Address        - I2C slave address
*  unsigned char RegisterAddr   - register address to select gyro, accelerometer
*                                 data
*  unsigned char *RegisterValue - pointer to data to be written to I2C slave
*  unsigned char  RegisterLen   - length of sensor data in bytes
* Return:
*  unsigned long status - status of I2C write operation
*
*******************************************************************************/
unsigned int I2C_WriteBytes(uint8_t Address, uint8_t RegisterAddr, uint8_t *RegisterValue, uint8_t RegisterLen)
{
    
    /* Variable used for status of I2C operation */
    static unsigned int status;
    /* Variable used to store the register length */
    uint8 i = RegisterLen;
    
    /* Send Start with Address + Write */
    status = I2Cm_MasterSendStart((uint32)Address, CY_SCB_I2C_WRITE_XFER, I2CM_TIMEOUT);
    
    /* Send register address */
    if(status == CY_SCB_I2C_SUCCESS)
    {
        status |= I2Cm_MasterWriteByte((uint32)RegisterAddr, I2CM_TIMEOUT);
    }
    if(status == CY_SCB_I2C_SUCCESS)
    {
        /* Send data one byte at a time */
        while(i-- && (status == CY_SCB_I2C_SUCCESS))
        {
            status |= I2Cm_MasterWriteByte((uint32)*RegisterValue++, I2CM_TIMEOUT);
            /* Delay to allow DMP time to setle */
            vTaskDelay(10u);
        }
    }
    /* Send Stop */
    status |= I2Cm_MasterSendStop(I2CM_TIMEOUT);
    
    return status;
}

/*******************************************************************************
* Function Name: I2C_ReadBytes()
********************************************************************************
* Summary:
*  Function that reads data from an I2C slave device 
*
* Parameters:
*  unsigned char Address        - I2C slave address
*  unsigned char RegisterAddr   - register address to select gyro, accelerometer
*                                 data
*  unsigned char *RegisterValue - pointer to sensor data
*  unsigned char  RegisterLen   - length of sensor data in bytes
* Return:
*  unsigned long status - status of I2C write operation
*
*******************************************************************************/ 
unsigned int I2C_ReadBytes(uint8_t Address, uint8_t RegisterAddr, uint8_t *RegisterValue, uint8_t RegisterLen)
{
    /* Variable used for status of I2C operation */
    static unsigned int status;
    
    uint8 i = 0;

    /* Send Start with Address + Write */
    status = I2Cm_MasterSendStart((uint32)Address, CY_SCB_I2C_WRITE_XFER, I2CM_TIMEOUT);
    if(status == CY_SCB_I2C_SUCCESS)
    {
        /* Send register address */
        status |= I2Cm_MasterWriteByte((uint32)RegisterAddr, I2CM_TIMEOUT);
    }
    if(status == CY_SCB_I2C_SUCCESS)
    {
        /* Send Start with Address + Read */
        status |= I2Cm_MasterSendReStart((uint32)Address, CY_SCB_I2C_READ_XFER, I2CM_TIMEOUT);
    }
    if(status == CY_SCB_I2C_SUCCESS)
    {
        /* Read in data bytes */
        while(i < (RegisterLen - 1))
        {
            status = I2Cm_MasterReadByte(CY_SCB_I2C_ACK, RegisterValue+i, I2CM_TIMEOUT);
            /* Delay to allow DMP time to setle */
            vTaskDelay(10u);
            i++;
        }
        /* Read and NAK the final byte */
        I2Cm_MasterReadByte(CY_SCB_I2C_NAK, RegisterValue+i, I2CM_TIMEOUT);
    }

    /* Send Stop to indicate end of communciation */
    status |= I2Cm_MasterSendStop(I2CM_TIMEOUT);
    
    return status;
}

/* [] END OF FILE */
