// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <stdint.h>
#include <stdbool.h>

#ifndef RFM69_h
#define RFM69_h




#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)

#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

// 
#define ISRFM69HW  0

// module interface, platform specific
extern void noInterrupts();                 // function to disable interrupts
extern void interrupts();                   // function to enable interrupts
extern void RFM69_SetCSPin(bool state);     // function to control the GPIO tied to RFM69 chip select (parameter HIGH or LOW)
extern bool RFM69_ReadDIO0Pin(void);        // function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)
extern uint8_t SPI_transfer8(uint8_t data); // function to transfer 1byte on SPI with readback
extern bool RFM69_IsTimeout(void);          // function for timeout handling, checks if previously set timeout expired
extern void RFM69_SetTimeout(uint16_t ms);  // function for timeout handling, sets a timeout, parameter is in milliseconds (ms)
extern void RFM69_TimerTick(void);
    

bool RFM69_initialize(uint8_t freqBand, uint8_t nodeID, uint16_t networkID);
uint8_t RFM69_receiveDone(uint8_t *p_buf);
void RFM69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK);
void RFM69_interruptHandler(void);
void RFM69_encrypt(const char* key);
void RFM69_sleep(void);

void RFM69_setHighPower(bool onOff);
bool RFM69_ACKReceived(uint8_t fromNodeID);
void RFM69_sendACK(const void* buffer, uint8_t bufferSize);

bool RFM69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime);
bool RFM69_ACKRequested(void);


#endif