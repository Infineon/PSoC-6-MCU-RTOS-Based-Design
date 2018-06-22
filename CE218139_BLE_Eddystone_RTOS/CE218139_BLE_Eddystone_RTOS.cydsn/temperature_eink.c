/******************************************************************************
* File Name: temperature.c
*
* Version: 1.00
*
* Description: This file contains functions that handle the initialization and
*              the measurement of temperature
*
* Related Document: CE220331_BLE_UI_RTOS.pdf
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
* This file contains functions that handle the initialization and measurement of 
* temperature using a thermistor circuit
*******************************************************************************/

/* Header file includes */
#include "temperature_eink.h"
#include <math.h>

/* ADC channels used to measure reference and thermistor voltages */
#define REFERENCE_CHANNEL   (uint32_t)(0x00u)
#define THERMISTOR_CHANNEL  (uint32_t)(0x01u)

/* Reference resistor in series with the thermistor is of 10 KOhm */
#define R_REFERENCE         (float)(10000)

/* Beta constant of this thermistor is 3380 Kelvin. See the thermistor
   (NCP18XH103F03RB) data sheet for more details. */
#define B_CONSTANT          (float)(3380)

/* Resistance of the thermistor is 10K at 25 degrees C (from data sheet)
   Therefore R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
   R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855 */
#define R_INFINITY          (float)(0.1192855)

/* Zero Kelvin in degree C */
#define ABSOLUTE_ZERO       (float)(-273.15)

/*******************************************************************************
* Function Name: float GetTemperature(void)
********************************************************************************
* Summary:
*  This function measures temperature from a thermistor circuit using SAR ADC
*
* Parameters:
*  void
*
* Return:
*  float    : Temperature in degree Celsius
*
*******************************************************************************/
float GetTemperature(void)
{   
    /* Variables used to store ADC counts, thermistor resistance and
       the temperature */
    int16_t countThermistor, countReference;
    float rThermistor, temperature;
    
    /* Set the GPIO that drives the thermistor circuit */
    Cy_GPIO_Set(THER_VDD_0_PORT,THER_VDD_0_NUM);
    
    /* Wake up the ADC and start conversion */
    ADC_Wakeup();
    ADC_StartConvert();
    ADC_IsEndConversion(CY_SAR_WAIT_FOR_RESULT);
    
    /* Read the ADC count values */
    countReference  = ADC_GetResult16(REFERENCE_CHANNEL);
    countThermistor = ADC_GetResult16(THERMISTOR_CHANNEL);
   
    /* Put the ADC to sleep so that entering low power modes won't affect
       the ADC operation */
    ADC_Sleep();
    
    /* Clear the GPIO that drives the thermistor circuit, to save power */
    Cy_GPIO_Clr(THER_VDD_0_PORT,THER_VDD_0_NUM);
    
    /* Calculate the thermistor resistance and the corresponding temperature */
    rThermistor = (R_REFERENCE*countThermistor)/countReference;    
    temperature = (B_CONSTANT/(logf(rThermistor/R_INFINITY)))+ABSOLUTE_ZERO;
    
    /* Return the temperature value */
    return temperature;
}

/*******************************************************************************
* Function Name: void InitTemperature(void)
********************************************************************************
* Summary:
*  This function initializes the components used for temperature sensing
*
* Parameters:
*  void
*
* Return:
*  None
*
*******************************************************************************/
void InitTemperature(void)
{
    /* Initialize the ADC and put it to sleep so that entering low
       power modes won't affect the ADC operation */
    ADC_Start();
    ADC_Sleep();
}

/* [] END OF FILE */
