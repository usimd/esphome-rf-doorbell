 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.2
 *
 * @version Package Version: 3.1.2
*/

/*
� [2025] Microchip Technology Inc. and its subsidiaries.

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
#include "mcc_generated_files/fvr/fvr.h"
#include "mcc_generated_files/system/system.h"
#include "RFM69.h"
#include <pic.h>
#include <pic12lf1552.h>

#pragma config IDLOC0 = 0xA
#pragma config IDLOC1 = 0xB
#pragma config IDLOC2 = 0xC
#pragma config IDLOC3 = 0xD

/*
    Main application
*/
typedef enum {
    BUTTON_PRESSED = 0x0,
    ADC_MEASUREMENT = 0x1,
    ADC_MEASUREMENT_DONE = 0x2,
    TRANSMIT_PACKAGE = 0x3,
    TRANSMISSION_DONE = 0x4,
    SLEEP = 0x5
} app_state_t;

typedef struct {
    uint16_t device_uid;
    uint16_t battery_voltage;
} package_t;

static volatile app_state_t state = SLEEP;
package_t buffer;

void ButtonInterrupt(void) {
    state = BUTTON_PRESSED;
}


#define NUMERATOR (int32_t)(1.024 * 1023 * 100) // = 104755


int main(void)
{
    SYSTEM_Initialize();
    RFM69_initialize(RF69_433MHZ, 24, 119);
    INTERRUPT_GlobalInterruptEnable(); 
    INTERRUPT_PeripheralInterruptEnable();
    BTN_SetInterruptHandler(&ButtonInterrupt);
    buffer.device_uid = (((CONFIGURATION_Read(0x8000) << 12) | CONFIGURATION_Read(0x8001) << 8) | CONFIGURATION_Read(0x8002) << 4) | CONFIGURATION_Read(0x8003);
    
    adc_result_t adc_result;
    
    while(1)
    {
        switch(state) {
            case BUTTON_PRESSED:
                state = ADC_MEASUREMENT;
                break;
                
            case ADC_MEASUREMENT:
                FVRCONbits.FVREN = 1;
                while (!FVR_IsOutputReady());
                adc_result = ADC_ChannelSelectAndConvert(ADC_CHANNEL_FVR);
                buffer.battery_voltage = 1048576L / adc_result;
                FVRCONbits.FVREN = 0;
                state = SLEEP;
                break;
                
            case TRANSMIT_PACKAGE:
                //RFM69_setMode(RF69_MODE_TX);
                RFM69_send(1, &buffer, sizeof(buffer), false);
                state = SLEEP;
                break;
            case SLEEP:
            default:
                RFM69_sleep();
                SLEEP();
                NOP();
                break;
        }
    }    
}