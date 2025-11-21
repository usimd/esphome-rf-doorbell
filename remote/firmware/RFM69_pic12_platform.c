#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "mcc_generated_files/system/system.h"


volatile uint16_t RFM69_ms_timer = 0;
// function to disable interrupts
void noInterrupts(){
  INTERRUPT_GlobalInterruptDisable(); 
}

// function to enable interrupts
void interrupts(){
  INTERRUPT_GlobalInterruptEnable(); 
}

// function to control the GPIO tied to RFM69 chip select (parameter HIGH or LOW)
void RFM69_SetCSPin(bool state){
  if(state) {
      CS_SetHigh();
  } else {
      CS_SetLow();
  }
}          

// function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)
bool RFM69_ReadDIO0Pin(void){
  return RFINT_GetValue() == 1;
}

// function to transfer 1byte on SPI with readback
uint8_t SPI_transfer8(uint8_t data){
  uint8_t retvalue = 0;
  
  retvalue = SPI1_Host_ByteExchange(data);
  
  return retvalue;
}       

 // function for timeout handling, checks if previously set timeout expired
bool RFM69_IsTimeout(void){
  return (bool)(RFM69_ms_timer == 0);
}

// function for timeout handling, set timeout value
void RFM69_SetTimeout(uint16_t ms){
  RFM69_ms_timer = ms;
}

// function supply timeout variable decrement
void RFM69_TimerTick(void){
  if(RFM69_ms_timer){
    RFM69_ms_timer--;
  }
}