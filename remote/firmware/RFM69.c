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

#include "RFM69.h"
#include "RFM69registers.h"

void RFM69_setHighPower(bool onOff);

static volatile uint8_t data[RF69_MAX_DATA_LEN]; // recv/xmit buf, including header & crc bytes
static volatile uint8_t datalen;
static volatile uint8_t senderID;
static volatile uint8_t targetID;                // should match _address
static volatile uint8_t payloadLen;
static volatile uint8_t ACK_Requested;
static volatile uint8_t ACK_RECEIVED;           // should be polled immediately after sending a packet with ACK request
static volatile uint8_t _mode = RF69_MODE_STANDBY;
static volatile int16_t rssi;                   // most accurate RSSI during reception (closest to the reception)

static uint8_t _address;
static uint8_t _powerLevel = 31;
bool _promiscuousMode = false;

// used function prototypes
void RFM69_writeReg(uint8_t addr, uint8_t val);
uint8_t RFM69_readReg(uint8_t addr);
void RFM69_setAddress(uint8_t addr);
void setMode(uint8_t mode);
uint32_t RFM69_getFrequency();
void RFM69_setFrequency(uint32_t freqHz);
void RFM69_setHighPower(bool onOff);
void RFM69_setHighPowerRegs(bool onOff);
void RFM69_setMode(uint8_t newMode);
void RFM69_sleep(void);
void RFM69_setNetwork(uint16_t networkID);
void RFM69_setPowerLevel(uint8_t level); // reduce/increase transmit power level
bool RFM69_canSend(void);
void RFM69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK);
bool RFM69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime); // 40ms roundtrip req for 61byte packets
bool RFM69_ACKReceived(uint8_t fromNodeID);
uint8_t RFM69_receiveDone(uint8_t *p_buf);
bool RFM69_ACKRequested(void);
void RFM69_sendACK(const void* buffer, uint8_t bufferSize);
void RFM69_unselect(void);
void RFM69_interruptHandler(void);
void RFM69_receiveBegin(void);
void RFM69_promiscuous(bool onOff);
void RFM69_readAllRegs(void);
uint8_t RFM69_readTemperature(uint8_t calFactor); // get CMOS temperature (8bit)
void RFM69_rcCalibration(void); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]
void RFM69_encrypt(const char* key);
int16_t RFM69_readRSSI(bool forceTrigger); 
void RFM69_select(void);

// extern functions
void noInterrupts();                    // function to disable interrupts
void interrupts();                      // function to enable interrupts
void RFM69_SetCSPin(bool state);        // function to control the GPIO tied to RFM69 chip select (parameter HIGH or LOW)
bool RFM69_ReadDIO0Pin(void);           // function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)
uint8_t SPI_transfer8(uint8_t data);    // function to transfer 1byte on SPI with readback
void Serialprint(char* data);           // function to print to serial port a string
bool RFM69_IsTimeout(void);             // function for timeout handling, checks if previously set timeout expired
void RFM69_SetTimeout(uint16_t ms);     // function for timeout handling, sets a timeout, parameter is in milliseconds (ms)

// internal
static void RFM69_sendFrame(uint8_t toAddress, const void* buffer, uint8_t size, bool requestACK, bool sendACK);


bool RFM69_initialize(uint8_t freqBand, uint8_t nodeID, uint16_t networkID)
{
  uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_19200}, // 19200bps
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATEMSB_19200},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
    /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
    /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    /* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    /* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
    /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };
  uint8_t i;
  
  RFM69_SetCSPin((bool)true);
  RFM69_SetTimeout(50);
  do
  {
    RFM69_writeReg(REG_SYNCVALUE1, 0xAA); 
  }
  while (RFM69_readReg(REG_SYNCVALUE1) != 0xaa && !RFM69_IsTimeout());
  
  RFM69_SetTimeout(50);
  do
  {
    RFM69_writeReg(REG_SYNCVALUE1, 0x55); 
  }
  while (RFM69_readReg(REG_SYNCVALUE1) != 0x55 && !RFM69_IsTimeout());

  for (i = 0; CONFIG[i][0] != 255; i++)
  {
    RFM69_writeReg(CONFIG[i][0], CONFIG[i][1]);
  }

  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  RFM69_encrypt(0);

  RFM69_setHighPower((bool)ISRFM69HW); // called regardless if it's a RFM69W or RFM69HW
  RFM69_setMode(RF69_MODE_STANDBY);
  RFM69_SetTimeout(50);
  while(((RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && RFM69_IsTimeout()); // wait for ModeReady
  if(RFM69_IsTimeout())
  {
    return false;
  }

  _address = nodeID;
  return true;
}

// return the frequency (in Hz)
uint32_t RFM69_getFrequency()
{
  return RF69_FSTEP * (((uint32_t) RFM69_readReg(REG_FRFMSB) << 16) + ((uint16_t) RFM69_readReg(REG_FRFMID) << 8) + RFM69_readReg(REG_FRFLSB));
}

// set the frequency (in Hz)
void RFM69_setFrequency(uint32_t freqHz)
{
  uint8_t oldMode = _mode;
  if (oldMode == RF69_MODE_TX) {
    RFM69_setMode(RF69_MODE_RX);
  }
  freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
  RFM69_writeReg(REG_FRFMSB, freqHz >> 16);
  RFM69_writeReg(REG_FRFMID, freqHz >> 8);
  RFM69_writeReg(REG_FRFLSB, freqHz);
  if (oldMode == RF69_MODE_RX) {
    RFM69_setMode(RF69_MODE_SYNTH);
  }
  RFM69_setMode(oldMode);
}

void RFM69_setMode(uint8_t newMode)
{
  if (newMode == _mode)
    return;

  switch (newMode) {
    case RF69_MODE_TX:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      if (ISRFM69HW) RFM69_setHighPowerRegs(true);
      break;
    case RF69_MODE_RX:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      if (ISRFM69HW) RFM69_setHighPowerRegs(false);
      break;
    case RF69_MODE_SYNTH:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default:
      return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  _mode = newMode;
}

//put transceiver in sleep mode to save battery - to wake or resume receiving just call RFM69_receiveDone()
void RFM69_sleep() {
  RFM69_setMode(RF69_MODE_SLEEP);
}

//set this node's address
void RFM69_setAddress(uint8_t addr)
{
  _address = addr;
  RFM69_writeReg(REG_NODEADRS, _address);
}

//set this node's network id
void RFM69_setNetwork(uint16_t networkID)
{
  RFM69_writeReg(REG_SYNCVALUE1, (uint8_t)(networkID & 0x00FF));
  RFM69_writeReg(REG_SYNCVALUE2, (uint8_t)(networkID >> 8));
}

// set *transmit/TX* output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
void RFM69_setPowerLevel(uint8_t powerLevel)
{
  _powerLevel = (powerLevel > 31 ? 31 : powerLevel);
  if (ISRFM69HW)
  {
    _powerLevel /= 2;
  }
  RFM69_writeReg(REG_PALEVEL, (RFM69_readReg(REG_PALEVEL) & 0xE0) | _powerLevel);
}

bool RFM69_canSend()
{
  if (_mode == RF69_MODE_RX && payloadLen == 0 && RFM69_readRSSI(false) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    RFM69_setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

void RFM69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK)
{
  RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  //uint32_t now = millis();
  while (!RFM69_canSend() /*&& millis() - now < RF69_CSMA_LIMIT_MS*/) RFM69_receiveDone(0);
  RFM69_sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

// to increase the chance of getting a packet across, call this function instead of send
// and it handles all the ACK requesting/retrying for you :)
// The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
// The reason for the semi-automaton is that the lib is interrupt driven and
// requires user action to read the received data and decide what to do with it
// replies usually take only 5..8ms at 50kbps@915MHz
bool RFM69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) 
{
  for (uint8_t i = 0; i <= retries; i++)
  {
    RFM69_send(toAddress, buffer, bufferSize, true);
    RFM69_SetTimeout(retryWaitTime);
    while (!RFM69_IsTimeout())
    {
      if (RFM69_ACKReceived(toAddress))
      {
        return true;
      }
    }
  }
  return false;
}

// should be polled immediately after sending a packet with ACK request
bool RFM69_ACKReceived(uint8_t fromNodeID) 
{
  if (RFM69_receiveDone(0)){
    return (senderID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && (bool)ACK_RECEIVED;
  }
  else{
    return false;
  }
}

// check whether an ACK was requested in the last received packet (non-broadcasted packet)
bool RFM69_ACKRequested(void) 
{
  return (bool)ACK_Requested && (bool)(targetID != RF69_BROADCAST_ADDR);
}

// should be called immediately after reception in case sender wants ACK
void RFM69_sendACK(const void* buffer, uint8_t bufferSize) 
{
  ACK_Requested = 0;   // TWS added to make sure we don't end up in a timing race and infinite loop sending Acks
  uint8_t sender = senderID;
  int16_t l_rssi = rssi; // save payload received RSSI value
  RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  RFM69_SetTimeout(RF69_CSMA_LIMIT_MS);
  while (!RFM69_canSend() && !RFM69_IsTimeout())
  {
    RFM69_receiveDone(0);
  }
  senderID = sender;    // TWS: Restore senderID after it gets wiped out by RFM69_receiveDone()
  RFM69_sendFrame(sender, buffer, bufferSize, false, true);
  rssi = l_rssi; // restore payload RSSI
}

// internal function
static void RFM69_sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
  RFM69_setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  // control byte
  uint8_t CTLbyte = 0x00;
  if (sendACK)
    CTLbyte = RFM69_CTL_SENDACK;
  else if (requestACK)
    CTLbyte = RFM69_CTL_REQACK;

  // write to FIFO
  RFM69_select();
  SPI_transfer8(REG_FIFO | 0x80);
  SPI_transfer8(bufferSize + 3);
  SPI_transfer8(toAddress);
  SPI_transfer8(_address);
  SPI_transfer8(CTLbyte);

  for (uint8_t i = 0; i < bufferSize; i++)
    SPI_transfer8(((uint8_t*) buffer)[i]);
  RFM69_unselect();

  // no need to wait for transmit mode to be ready since its handled by the radio
  RFM69_setMode(RF69_MODE_TX);
  RFM69_SetTimeout(RF69_TX_LIMIT_MS);
  while (RFM69_ReadDIO0Pin() == 0 && !RFM69_IsTimeout()); // wait for DIO0 to turn HIGH signalling transmission finish
  //while (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
  RFM69_setMode(RF69_MODE_STANDBY);
}

// internal function - interrupt gets called when a packet is received
void RFM69_interruptHandler(void) {
  
  if (_mode == RF69_MODE_RX && (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    uint8_t CTLbyte;
    //rssi = RFM69_readRSSI();
    RFM69_setMode(RF69_MODE_STANDBY);
    RFM69_select();
    SPI_transfer8(REG_FIFO & 0x7F);
    payloadLen = SPI_transfer8(0);
    payloadLen = payloadLen > 66 ? 66 : payloadLen; // precaution
    targetID = SPI_transfer8(0);
    if(!(_promiscuousMode || targetID == _address || targetID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in promiscuous mode
       || payloadLen < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
    {
      payloadLen = 0;
      RFM69_unselect();
      RFM69_receiveBegin();
      return;
    }

    datalen = payloadLen - 3;
    senderID = SPI_transfer8(0);
    CTLbyte = SPI_transfer8(0);

    ACK_RECEIVED = CTLbyte & RFM69_CTL_SENDACK; // extract ACK-received flag
    ACK_Requested = CTLbyte & RFM69_CTL_REQACK; // extract ACK-requested flag

    for (uint8_t i = 0; i < datalen; i++)
    {
      data[i] = SPI_transfer8(0);
    }
    if (datalen < RF69_MAX_DATA_LEN) data[datalen] = 0; // add null at end of string
    RFM69_unselect();
    RFM69_setMode(RF69_MODE_RX);
  }
  rssi = RFM69_readRSSI(false);
}

// internal function
void RFM69_receiveBegin() 
{
  datalen = 0;
  senderID = 0;
  targetID = 0;
  payloadLen = 0;
  ACK_Requested = 0;
  ACK_RECEIVED = 0;
  rssi = 0;
  if (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  RFM69_setMode(RF69_MODE_RX);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
uint8_t RFM69_receiveDone(uint8_t *p_buf) 
{
  noInterrupts(); // re-enabled in RFM69_unselect() via setMode() or via RFM69_receiveBegin()
  if (_mode == RF69_MODE_RX && payloadLen > 0)
  {
    RFM69_setMode(RF69_MODE_STANDBY); // enables interrupts
    
    if(p_buf){
        for(uint8_t i = 0; i < payloadLen; i++){
            *(p_buf++) = data[i];
        }
    }
    
    return payloadLen;
  }
  else if (_mode == RF69_MODE_RX) // already in RX no payload yet
  {
    interrupts(); // explicitly re-enable interrupts
    
    return 0;
  }
  
  RFM69_receiveBegin();
  
  return 0;
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void RFM69_encrypt(const char* key) 
{
  RFM69_setMode(RF69_MODE_STANDBY);
  if (key != 0)
  {
    RFM69_select();
    SPI_transfer8(REG_AESKEY1 | 0x80);
    for (uint8_t i = 0; i < 16; i++)
      SPI_transfer8(key[i]);
    RFM69_unselect();
  }
  RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
}

// get the received signal strength indicator (RSSI)
int16_t RFM69_readRSSI(bool forceTrigger) 
{
  int16_t rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    RFM69_writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((RFM69_readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  rssi = -RFM69_readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

// true  = disable filtering to capture all frames on network
// false = enable node/broadcast filtering to capture only frames sent to this/broadcast address
void RFM69_promiscuous(bool onOff) 
{
  _promiscuousMode = onOff;
  //RFM69_writeReg(REG_PACKETCONFIG1, (RFM69_readReg(REG_PACKETCONFIG1) & 0xF9) | (onOff ? RF_PACKET1_ADRSFILTERING_OFF : RF_PACKET1_ADRSFILTERING_NODEBROADCAST));
}

// for RFM69HW only: you must call RFM69_setHighPower(true) after initialize() or else transmission won't work
void RFM69_setHighPower(bool onOff) {
  RFM69_writeReg(REG_OCP, ISRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (ISRFM69HW) // turning ON
    RFM69_writeReg(REG_PALEVEL, (RFM69_readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    RFM69_writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}

// internal function
void RFM69_setHighPowerRegs(bool onOff) 
{
  RFM69_writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  RFM69_writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}


uint8_t RFM69_readTemperature(uint8_t calFactor) // returns centigrade
{
  RFM69_setMode(RF69_MODE_STANDBY);
  RFM69_writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((RFM69_readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~RFM69_readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void RFM69_rcCalibration()
{
  RFM69_writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((RFM69_readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}

uint8_t RFM69_readReg(uint8_t addr)
{
  uint8_t regval;
  RFM69_select();
  SPI_transfer8(addr & 0x7F);
  regval = SPI_transfer8(0);
  RFM69_unselect();
  return regval;
}

void RFM69_writeReg(uint8_t addr, uint8_t value)
{
  RFM69_select();
  SPI_transfer8(addr | 0x80);
  SPI_transfer8(value);
  RFM69_unselect();
}

// select the RFM69 transceiver (save SPI settings, set CS low)
void RFM69_select() 
{
  noInterrupts();
  RFM69_SetCSPin(false);
}

// unselect the RFM69 transceiver (set CS high, restore SPI settings)
void RFM69_unselect() 
{
  RFM69_SetCSPin(true);
  interrupts();
}