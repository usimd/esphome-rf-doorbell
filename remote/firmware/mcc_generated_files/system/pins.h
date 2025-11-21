/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.0.0
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

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set IO_RA0 aliases
#define IO_RA0_TRIS                 TRISAbits.TRISA0
#define IO_RA0_LAT                  LATAbits.LATA0
#define IO_RA0_PORT                 PORTAbits.RA0
#define IO_RA0_WPU                  WPUAbits.WPUA0
#define IO_RA0_OD                   ODCONAbits.
#define IO_RA0_ANS                  ANSELAbits.ANSA0
#define IO_RA0_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define IO_RA0_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define IO_RA0_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define IO_RA0_GetValue()           PORTAbits.RA0
#define IO_RA0_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define IO_RA0_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define IO_RA0_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define IO_RA0_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define IO_RA0_SetPushPull()        do { ODCONAbits. = 0; } while(0)
#define IO_RA0_SetOpenDrain()       do { ODCONAbits. = 1; } while(0)
#define IO_RA0_SetAnalogMode()      do { ANSELAbits.ANSA0 = 1; } while(0)
#define IO_RA0_SetDigitalMode()     do { ANSELAbits.ANSA0 = 0; } while(0)
// get/set IO_RA1 aliases
#define IO_RA1_TRIS                 TRISAbits.TRISA1
#define IO_RA1_LAT                  LATAbits.LATA1
#define IO_RA1_PORT                 PORTAbits.RA1
#define IO_RA1_WPU                  WPUAbits.WPUA1
#define IO_RA1_OD                   ODCONAbits.
#define IO_RA1_ANS                  ANSELAbits.ANSA1
#define IO_RA1_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define IO_RA1_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define IO_RA1_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define IO_RA1_GetValue()           PORTAbits.RA1
#define IO_RA1_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define IO_RA1_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define IO_RA1_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define IO_RA1_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define IO_RA1_SetPushPull()        do { ODCONAbits. = 0; } while(0)
#define IO_RA1_SetOpenDrain()       do { ODCONAbits. = 1; } while(0)
#define IO_RA1_SetAnalogMode()      do { ANSELAbits.ANSA1 = 1; } while(0)
#define IO_RA1_SetDigitalMode()     do { ANSELAbits.ANSA1 = 0; } while(0)
// get/set IO_RA2 aliases
#define IO_RA2_TRIS                 TRISAbits.TRISA2
#define IO_RA2_LAT                  LATAbits.LATA2
#define IO_RA2_PORT                 PORTAbits.RA2
#define IO_RA2_WPU                  WPUAbits.WPUA2
#define IO_RA2_OD                   ODCONAbits.
#define IO_RA2_ANS                  ANSELAbits.ANSA2
#define IO_RA2_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define IO_RA2_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define IO_RA2_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define IO_RA2_GetValue()           PORTAbits.RA2
#define IO_RA2_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define IO_RA2_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define IO_RA2_SetPullup()          do { WPUAbits.WPUA2 = 1; } while(0)
#define IO_RA2_ResetPullup()        do { WPUAbits.WPUA2 = 0; } while(0)
#define IO_RA2_SetPushPull()        do { ODCONAbits. = 0; } while(0)
#define IO_RA2_SetOpenDrain()       do { ODCONAbits. = 1; } while(0)
#define IO_RA2_SetAnalogMode()      do { ANSELAbits.ANSA2 = 1; } while(0)
#define IO_RA2_SetDigitalMode()     do { ANSELAbits.ANSA2 = 0; } while(0)
// get/set IO_RA3 aliases
#define BTN_TRIS                 TRISAbits.TRISA3
#define BTN_LAT                  LATAbits.
#define BTN_PORT                 PORTAbits.RA3
#define BTN_WPU                  WPUAbits.WPUA3
#define BTN_OD                   ODCONAbits.
#define BTN_ANS                  ANSELAbits.
#define BTN_SetHigh()            do { LATAbits. = 1; } while(0)
#define BTN_SetLow()             do { LATAbits. = 0; } while(0)
#define BTN_Toggle()             do { LATAbits. = ~LATAbits.; } while(0)
#define BTN_GetValue()           PORTAbits.RA3
#define BTN_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define BTN_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define BTN_SetPullup()          do { WPUAbits.WPUA3 = 1; } while(0)
#define BTN_ResetPullup()        do { WPUAbits.WPUA3 = 0; } while(0)
#define BTN_SetPushPull()        do { ODCONAbits. = 0; } while(0)
#define BTN_SetOpenDrain()       do { ODCONAbits. = 1; } while(0)
#define BTN_SetAnalogMode()      do { ANSELAbits. = 1; } while(0)
#define BTN_SetDigitalMode()     do { ANSELAbits. = 0; } while(0)
#define RA3_SetInterruptHandler  BTN_SetInterruptHandler
// get/set IO_RA4 aliases
#define CS_TRIS                 TRISAbits.TRISA4
#define CS_LAT                  LATAbits.LATA4
#define CS_PORT                 PORTAbits.RA4
#define CS_WPU                  WPUAbits.WPUA4
#define CS_OD                   ODCONAbits.
#define CS_ANS                  ANSELAbits.ANSA4
#define CS_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define CS_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define CS_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define CS_GetValue()           PORTAbits.RA4
#define CS_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define CS_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define CS_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define CS_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define CS_SetPushPull()        do { ODCONAbits. = 0; } while(0)
#define CS_SetOpenDrain()       do { ODCONAbits. = 1; } while(0)
#define CS_SetAnalogMode()      do { ANSELAbits.ANSA4 = 1; } while(0)
#define CS_SetDigitalMode()     do { ANSELAbits.ANSA4 = 0; } while(0)
// get/set IO_RA5 aliases
#define RFINT_TRIS                 TRISAbits.TRISA5
#define RFINT_LAT                  LATAbits.LATA5
#define RFINT_PORT                 PORTAbits.RA5
#define RFINT_WPU                  WPUAbits.WPUA5
#define RFINT_OD                   ODCONAbits.
#define RFINT_ANS                  ANSELAbits.ANSA5
#define RFINT_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define RFINT_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define RFINT_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define RFINT_GetValue()           PORTAbits.RA5
#define RFINT_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define RFINT_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define RFINT_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define RFINT_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define RFINT_SetPushPull()        do { ODCONAbits. = 0; } while(0)
#define RFINT_SetOpenDrain()       do { ODCONAbits. = 1; } while(0)
#define RFINT_SetAnalogMode()      do { ANSELAbits.ANSA5 = 1; } while(0)
#define RFINT_SetDigitalMode()     do { ANSELAbits.ANSA5 = 0; } while(0)
#define RA5_SetInterruptHandler  RFINT_SetInterruptHandler
/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handler for the BTN pin functionality
 * @param none
 * @return none
 */
void BTN_ISR(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for BTN pin interrupt-on-change functionality.
 *        Allows selecting an interrupt handler for BTN at application runtime.
 * @pre Pins intializer called
 * @param InterruptHandler function pointer.
 * @return none
 */
void BTN_SetInterruptHandler(void (* InterruptHandler)(void));

/**
 * @ingroup  pinsdriver
 * @brief Dynamic Interrupt Handler for BTN pin.
 *        This is a dynamic interrupt handler to be used together with the BTN_SetInterruptHandler() method.
 *        This handler is called every time the BTN ISR is executed and allows any function to be registered at runtime.
 * @pre Pins intializer called
 * @param none
 * @return none
 */
extern void (*BTN_InterruptHandler)(void);

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for BTN pin. 
 *        This is a predefined interrupt handler to be used together with the BTN_SetInterruptHandler() method.
 *        This handler is called every time the BTN ISR is executed. 
 * @pre Pins intializer called
 * @param none
 * @return none
 */
void BTN_DefaultInterruptHandler(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handler for the RFINT pin functionality
 * @param none
 * @return none
 */
void RFINT_ISR(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for RFINT pin interrupt-on-change functionality.
 *        Allows selecting an interrupt handler for RFINT at application runtime.
 * @pre Pins intializer called
 * @param InterruptHandler function pointer.
 * @return none
 */
void RFINT_SetInterruptHandler(void (* InterruptHandler)(void));

/**
 * @ingroup  pinsdriver
 * @brief Dynamic Interrupt Handler for RFINT pin.
 *        This is a dynamic interrupt handler to be used together with the RFINT_SetInterruptHandler() method.
 *        This handler is called every time the RFINT ISR is executed and allows any function to be registered at runtime.
 * @pre Pins intializer called
 * @param none
 * @return none
 */
extern void (*RFINT_InterruptHandler)(void);

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for RFINT pin. 
 *        This is a predefined interrupt handler to be used together with the RFINT_SetInterruptHandler() method.
 *        This handler is called every time the RFINT ISR is executed. 
 * @pre Pins intializer called
 * @param none
 * @return none
 */
void RFINT_DefaultInterruptHandler(void);


#endif // PINS_H
/**
 End of File
*/