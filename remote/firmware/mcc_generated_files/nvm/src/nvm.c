/**
 *
 * @file nvm.c
 *
 * @ingroup nvm_driver
 *
 * @brief Contains the implementation file for the NVM driver.
 *
 * @version NVM Driver Version 1.0.0
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

#include "../nvm.h"

/*
 * AoU: These variables are placed in the Access RAM using the "__near" qualifier.
 *      Additionally, "Address Qualifiers" must be configured to the "Require" under
 *      "Project Properties -> XC8 Compiler -> Optimizations".
 */
__near volatile static uint8_t unlockKeyLow;
__near volatile static uint8_t unlockKeyHigh;

void NVM_Initialize(void)
{
    NVM_StatusClear();
}

bool NVM_IsBusy(void)
{
    return (PMCON1bits.WR || PMCON1bits.RD);
}

nvm_status_t NVM_StatusGet(void)
{
    if (PMCON1bits.WRERR == 1)
    {
        return NVM_ERROR;
    }
    else
    {
        return NVM_OK;
    }
}

void NVM_StatusClear(void)
{
    PMCON1bits.WRERR = 0;
}

void NVM_UnlockKeySet(uint16_t unlockKey)
{
    unlockKeyHigh = (uint8_t) (unlockKey >> 8);
    unlockKeyLow = (uint8_t) unlockKey;
}

void NVM_UnlockKeyClear(void)
{
    unlockKeyHigh = 0x00;
    unlockKeyLow = 0x00;
}

configuration_data_t CONFIGURATION_Read(configuration_address_t address)
{   
    uint8_t GIEBitValue = INTCONbits.GIE;   

    //Load PMADR with the Device ID address
    PMADRH = (uint8_t) (address >> 8);
    PMADRL = (uint8_t) address;

    //Access configuration word
    PMCON1bits.CFGS = 1; 

    //Disable global interrupt
    INTCONbits.GIE = 0;
    
    //Initiate Read
    PMCON1bits.RD = 1;

    NOP();
    NOP();
    INTCONbits.GIE = GIEBitValue;

    return ((configuration_data_t) ((PMDATH << 8) | PMDATL));
}
