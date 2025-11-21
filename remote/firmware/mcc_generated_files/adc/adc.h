/**
 * ADC Generated Driver API Header File
 * 
 * @file adc.h
 * 
 * @defgroup  adc ADC
 * 
 * @brief This file provides API prototypes for the ADC driver.
 *
 * @version ADC Driver Version 3.0.0
 * 
 * @version ADC Package Version 4.0.0
*/
/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef ADC_H
#define ADC_H


#include <stdint.h>
#include <stdbool.h>
#include "./adc_types.h"



#define ACQ_US_DELAY 5

/**
 * @ingroup adc
 * @brief Initializes the registers based on the configurable options in the MPLAB® Code Configurator (MCC) Melody UI for the Analog-to-Digital Converter (ADC) operation.
 * @param None.
 * @return None.
*/
void ADC_Initialize(void);

/**
 * @ingroup adc
 * @brief Deinitializes the registers to Power-on Reset (POR) values.
 * @param None.
 * @return None.
*/
void ADC_Deinitialize(void);

/**
 * @ingroup adc
 * @brief Sets the ADC Enable (ADON) bit to '1'.
 * @param None.
 * @return None.
*/
void ADC_Enable(void);

/**
 * @ingroup adc
 * @brief Sets the ADC Enable (ADON) bit to '0'.
 * @param None.
 * @return None.
*/
void ADC_Disable(void);

/**
 * @ingroup adc
 * @brief Sets the channel to use for the ADC conversion.
 * @param channel Desired analog channel. Refer to the @ref adc_channel_t enum for the list of available analog channels.
 * @return None.
*/
void ADC_ChannelSelect(adc_channel_t channel);

/**
 * @ingroup adc
 * @brief Starts the conversion on a selected channel. 
 * @pre Select the channel(s) using @ref ADC_ChannelSelect
 * and call @ref ADC_Initialize(void) to initialize the Analog-to-Digital Converter (ADC) module before using this API.
 * @param None.
 * @return None.
*/
void ADC_ConversionStart(void);

/**
 * @ingroup adc
 * @brief Checks if the ongoing conversion is complete.
 * @pre Call @ref ADC_ConversionStart(void) before using this API.
 * @param None.
 * @retval True Conversion is complete
 * @retval False Conversion is ongoing
*/
bool ADC_IsConversionDone(void);

/**
 * @ingroup adc
 * @brief Stops the ongoing conversion.
 * @param None.
 * @return None.
*/
void ADC_ConversionStop(void);

/**
 * @ingroup adc
 * @brief Retrieves the result of the latest conversion.
 * @pre Call @ref ADC_ConversionStart(void) before using this API.
 * @param None.
 * @return adc_result_t - The result of the conversion
*/
adc_result_t ADC_ConversionResultGet(void);

/**
 * @ingroup adc
 * @brief Starts the conversion and retrieves the result of the single conversion on the selected channel.
 * @param channel Desired analog channel. Refer to the @ref adc_channel_t enum for the list of available analog channels.
 * @return adc_result_t - The result of the conversion
*/
adc_result_t ADC_ChannelSelectAndConvert(adc_channel_t channel); 

/**
 * @ingroup adc
 * @brief Sets the auto conversion trigger source.
 * @param triggerSource Desired auto conversion trigger source. Refer to the @ref adc_trigger_source_t for the list of available trigger sources.
 * @return None.
*/
void ADC_AutoTriggerSourceSet(adc_trigger_source_t triggerSource);

/**
 * @ingroup adc
 * @brief Returns the resolution of the Analog-to-Digital Converter (ADC) module.
 * @param None.
 * @return uint8_t - Resolution value
*/
uint8_t ADC_ResolutionGet(void);

/**
 * @ingroup adc
 * @brief Clears the ADC Conversion Done Interrupt Flag bit to 0.
 * @param None.
 * @return None.
*/
void ADC_ConversionDoneInterruptFlagClear(void);

/**
 * @ingroup adc
 * @brief Checks the ADC Conversion Done Interrupt Flag (ADIF) status.
 * @param None.
 * @retval True ADI flag is set
 * @retval False ADI flag is not set
*/
bool ADC_IsConversionDoneInterruptFlagSet(void);

/**
 * @ingroup adc
 * @brief Sets the callback function for the ADC Conversion Done Interrupt (ADI).
 * @param Pointer The pointer to the function to be executed
 * @return None.
 */
void ADC_ConversionDoneCallbackRegister(void (*callback)(void));

/**
 * @ingroup adc
 * @brief Implements the Tasks routine for the polling implementations.
 * @param None.
 * @return None.
*/
void ADC_Tasks(void);

#endif // ADC_H
