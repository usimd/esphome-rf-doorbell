/**
 * Generated Driver File
 * 
 * @file pins.c
 * 
 * @ingroup  pinsdriver
 * 
 * @brief This is generated driver implementation for pins. 
 *        This file provides implementations for pin APIs for all pins selected in the GUI.
 *
 * @version Driver Version 3.0.0
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

#include "../pins.h"

void (*BTN_InterruptHandler)(void);
void (*RFINT_InterruptHandler)(void);

void PIN_MANAGER_Initialize(void)
{
   /**
    LATx registers
    */
    LATA = 0x0;

    /**
    TRISx registers
    */
    TRISA = 0x2C;

    /**
    ANSELx registers
    */
    ANSELA = 0x0;

    /**
    WPUx registers
    */
    WPUA = 0x7;
    OPTION_REGbits.nWPUEN = 0x0;
  
    /**
    ODx registers
    */
   
    /**
    SLRCONx registers
    */
    /**
    INLVLx registers
    */

    /**
    PPS registers
    */

    /**
    APFCON registers
    */
    APFCON = 0x0; //RA0->MSSP:SDO;

   /**
    IOCx registers 
    */
    IOCAP = 0x20;
    IOCAN = 0x8;
    IOCAF = 0x0;

    BTN_SetInterruptHandler(BTN_DefaultInterruptHandler);
    RFINT_SetInterruptHandler(RFINT_DefaultInterruptHandler);

    // Enable INTCONbits.IOCIE interrupt 
    INTCONbits.IOCIE = 1; 
}
  
void PIN_MANAGER_IOC(void)
{
    // interrupt on change for pin BTN}
    if(IOCAFbits.IOCAF3 == 1)
    {
        BTN_ISR();  
    }
    // interrupt on change for pin RFINT}
    if(IOCAFbits.IOCAF5 == 1)
    {
        RFINT_ISR();  
    }
}
   
/**
   BTN Interrupt Service Routine
*/
void BTN_ISR(void) {

    // Add custom IOCAF3 code

    // Call the interrupt handler for the callback registered at runtime
    if(BTN_InterruptHandler)
    {
        BTN_InterruptHandler();
    }
    IOCAFbits.IOCAF3 = 0;
}

/**
  Allows selecting an interrupt handler for IOCAF3 at application runtime
*/
void BTN_SetInterruptHandler(void (* InterruptHandler)(void)){
    BTN_InterruptHandler = InterruptHandler;
}

/**
  Default interrupt handler for IOCAF3
*/
void BTN_DefaultInterruptHandler(void){
    // add your BTN interrupt custom code
    // or set custom function using BTN_SetInterruptHandler()
}
   
/**
   RFINT Interrupt Service Routine
*/
void RFINT_ISR(void) {

    // Add custom IOCAF5 code

    // Call the interrupt handler for the callback registered at runtime
    if(RFINT_InterruptHandler)
    {
        RFINT_InterruptHandler();
    }
    IOCAFbits.IOCAF5 = 0;
}

/**
  Allows selecting an interrupt handler for IOCAF5 at application runtime
*/
void RFINT_SetInterruptHandler(void (* InterruptHandler)(void)){
    RFINT_InterruptHandler = InterruptHandler;
}

/**
  Default interrupt handler for IOCAF5
*/
void RFINT_DefaultInterruptHandler(void){
    // add your RFINT interrupt custom code
    // or set custom function using RFINT_SetInterruptHandler()
}
/**
 End of File
*/