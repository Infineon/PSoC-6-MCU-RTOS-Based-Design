/******************************************************************************
* File Name: temperature_task.c
*
* Version: 1.00
*
* Description: This file contains the task that handles temperature sensing
*
* Related Document: CE218138_BLE_Thermometer_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
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
* This file contains the task that handles temperature sensing 
*******************************************************************************/

/* Header file includes */
#include <math.h>
#include "temperature_task.h"
#include "uart_debug.h"
#include "ble_task.h"
#include "task.h" 
#include "timers.h"

/* Scanning interval of 100ms is used when repeatedly scanning temperature for 
   sending via BLE. 10s is used for E-INK refresh */
#define FAST_SCAN_INTERVAL  (pdMS_TO_TICKS(100u))
#define SLOW_SCAN_INTERVAL  (pdMS_TO_TICKS(10000u))

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

/* Queue handles used for temperature commands and data */
QueueHandle_t temperatureCommandQ;
QueueHandle_t temperatureDataQ;

/* Timer handles used to control temperature scanning interval */
TimerHandle_t xTimer_Temperature;

/* ADC interrupt handler */
void isrADC(void);
bool static processingComplete = false;

/* Functions that start and control the timer */
void static TemperatureTimerStart(void);
void static TemperatureTimerUpdate(TickType_t period);

/*******************************************************************************
* Function Name: void Task_Temperature(void *pvParameters)   
********************************************************************************
* Summary:
*  Task that reads temperature data from thermistor circuit  
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/
void Task_Temperature(void *pvParameters)    
{ 
    /* Variable that stores commands received  */
    temperature_command_t temperatureCommand;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
       
    /* Flag that indicate temperature data need to be sent at 
       fast intervals */
    bool sendTemperatureDataFast = false;
    
    /* Variables used to calculate temperature */
    int16_t countThermistor, countReference;
    float rThermistor, temperature;
    
    /* Variable used to send commands and data to BLE task */
    ble_command_t bleCommandAndData;

    /* Remove warning for unused parameter */
    (void)pvParameters ;

    /* Initialize the ADC  */
    ADC_StartEx(isrADC);
    ADC_IRQ_Enable();
    ADC_StartConvert();
    
    /* Start the timer that controls the processing interval */
    TemperatureTimerStart();                    
    
    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a command has been received over 
           temperatureCommandQ */
        rtosApiResult = xQueueReceive(temperatureCommandQ, 
                                      &temperatureCommand, portMAX_DELAY);
        
         /* Command has been received from temperatureCommandQ */ 
        if(rtosApiResult == pdTRUE)
        {   
            /* Take an action based on the command received */
            switch (temperatureCommand)
            {
                /* Temperature data need to be sent */
                case SEND_TEMPERATURE:
                    sendTemperatureDataFast = true;
                    /* Enable periodic scan */
                    TemperatureTimerUpdate(FAST_SCAN_INTERVAL);
                    break;
                 
                /* No temperature data need to be sent */
                case SEND_NONE:
                    sendTemperatureDataFast = false;
                    /* Disable periodic scan */
                    TemperatureTimerUpdate(SLOW_SCAN_INTERVAL);
                    break;
                /* Process temperature data from CapSense widgets */
                case HANDLE_ADC_INTERRUPT:
                    /* Read the ADC count values */
                    countReference  = ADC_GetResult16(REFERENCE_CHANNEL);
                    countThermistor = ADC_GetResult16(THERMISTOR_CHANNEL);
                   
                    /* Put the ADC to sleep so that entering low power modes 
                       won't affect the ADC operation */
                    ADC_Sleep();
                    
                    /* Clear the GPIO that drives the thermistor circuit, to 
                       save power */
                    Cy_GPIO_Clr(THER_VDD_0_PORT,THER_VDD_0_NUM);
                    
                    /* Calculate the thermistor resistance and the corresponding
                       temperature */
                    rThermistor = (R_REFERENCE*countThermistor)/countReference;    
                    temperature = (B_CONSTANT/(logf(rThermistor/R_INFINITY)))+
                                                                ABSOLUTE_ZERO;
                    
                    processingComplete = true;

                    /* Send the processed temperature data */
                    if(sendTemperatureDataFast)
                    {
                        /* Pack the temperature data, respective command and send 
                           to the queue */
                        bleCommandAndData.command = SEND_TEMPERATURE_INDICATION;
                        bleCommandAndData.temperatureData = temperature;
                        xQueueSend(bleCommandQ, &bleCommandAndData,0u);
                    }
                    else
                    {
                        xQueueOverwrite(temperatureDataQ, &temperature);
                    }
                    break;
                /* Start the next scan */                    
                case TEMPERATURE_TIMER_EXPIRED:    

                        if(processingComplete)
                        {
                            Cy_GPIO_Set(THER_VDD_0_PORT,THER_VDD_0_NUM);    
                        
                            /* Wake up the ADC and start conversion */
                            ADC_Wakeup();
                            ADC_StartConvert();
                            processingComplete = false;
                        }    

                    break;

                /* Invalid task notification value received */    
                default:
                    Task_DebugPrintf("Error!   : Temperature - Invalid command "\
                                "received .Error Code:", temperatureCommand);
                    break;
            }
        }            
        /* Task has timed out and received no commands during an interval of 
        portMAXDELAY ticks */
        else
        {
            Task_DebugPrintf("Warning! : Temperature - Task Timed out ", 0u);   
        }
    }
}

/*******************************************************************************
* Function Name: void isrADC(void)                         
********************************************************************************
* Summary:
*  Interrupt service routine of the Scanning SAR ADC
*
* Parameters:
*  None
*
* Return:
*  void
*
*******************************************************************************/
void isrADC(void)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Variable that stored interrupt status */
    uint32_t intr_status;

    /* Read interrupt status register */
    intr_status = Cy_SAR_GetInterruptStatus(ADC_SAR__HW);
    
     /* Clear handled interrupt */
    Cy_SAR_ClearInterrupt(ADC_SAR__HW, intr_status);
    
    /* Read interrupt status register to ensure write completed due to 
       buffered writes */
    (void)Cy_SAR_GetInterruptStatus(ADC_SAR__HW);
    
    /* Send command to process temperature */
    temperature_command_t temperatureCommand = HANDLE_ADC_INTERRUPT;
    rtosApiResult = xQueueSendFromISR(temperatureCommandQ,
                                      &temperatureCommand,0u);
    
    /* Check if the operation has been successful */
    if(rtosApiResult != pdTRUE)
    {
        Task_DebugPrintf("Failure! : Temperature  - Sending data to temperature"\
                    "queue", 0u);    
    }  
}

/*******************************************************************************
* Function Name: void static TemperatureTimerCallback(TimerHandle_t xTimer)                          
********************************************************************************
* Summary:
*  This function is called when the temperature timer expires
*
* Parameters:
*  TimerHandle_t xTimer :  Current timer value (unused)
*
* Return:
*  void
*
*******************************************************************************/
void static TemperatureTimerCallback(TimerHandle_t xTimer)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Remove warning for unused parameter */
    (void)xTimer;
    
    /* Send command to process temperature */
    temperature_command_t temperatureCommand = TEMPERATURE_TIMER_EXPIRED;
    rtosApiResult = xQueueSend(temperatureCommandQ, 
                               &temperatureCommand,0u);
    
    /* Check if the operation has been successful */
    if(rtosApiResult != pdTRUE)
    {
        Task_DebugPrintf("Failure! : Temperature  - Sending data to temperature"\
                         "queue", 0u);    
    }
}

/*******************************************************************************
* Function Name: void static TemperatureTimerStart(void)                  
********************************************************************************
* Summary:
*  This function starts the timer that provides timing to periodic
*  temperature processing
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void static TemperatureTimerStart(void)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Create an RTOS timer */
    xTimer_Temperature =  xTimerCreate ("Temperature Timer",
                                        SLOW_SCAN_INTERVAL, pdTRUE,  
                                        NULL, TemperatureTimerCallback);
    
    /* Make sure that timer handle is valid */
    if (xTimer_Temperature != NULL)
    {
        /* Start the timer */
        rtosApiResult = xTimerStart(xTimer_Temperature,0u);
        
        /* Check if the operation has been successful */
        if(rtosApiResult != pdPASS)
        {
            Task_DebugPrintf("Failure! : Temperature  - Timer initialization", 0u);    
        }
    }
    else
    {
        Task_DebugPrintf("Failure! : Temperature  - Timer creation", 0u); 
    }
}

/*******************************************************************************
* Function Name: void static TemperatureTimerUpdate(TickType_t period))                 
********************************************************************************
* Summary:
*  This function updates the timer period per the parameter
*
* Parameters:
*  TickType_t period :  Period of the timer in ticks
*
* Return:
*  void
*
*******************************************************************************/
void static TemperatureTimerUpdate(TickType_t period)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Change the timer period */
    rtosApiResult = xTimerChangePeriod(xTimer_Temperature, period, 0u);

    /* Check if the operation has been successful */
    if(rtosApiResult != pdPASS)
    {
        Task_DebugPrintf("Failure! : Temperature - Timer update ", 0u);   
    }
}

/* [] END OF FILE */
