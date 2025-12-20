#include "rfm69.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rfm69 {

static const char *const TAG = "rfm69";

void RFM69Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RFM69...");
  
  // Setup pins
  if (this->cs_pin_ != nullptr) {
    this->cs_pin_->setup();
    this->cs_pin_->digital_write(true);
  }
  
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(10);
  }
  
  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();
  }
  
  // Initialize SPI
  this->spi_setup();
  
  // Verify communication by reading version register
  uint8_t version = this->read_register_(RFM69_REG_VERSION);
  if (version == 0 || version == 0xFF) {
    ESP_LOGE(TAG, "Failed to communicate with RFM69, version: 0x%02X", version);
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "RFM69 version: 0x%02X", version);
  
  // Set standby mode
  if (!this->set_mode_(RFM69_MODE_STANDBY)) {
    ESP_LOGE(TAG, "Failed to set standby mode");
    this->mark_failed();
    return;
  }
  
  // Configure frequency
  if (!this->set_frequency_()) {
    ESP_LOGE(TAG, "Failed to set frequency");
    this->mark_failed();
    return;
  }
  
  // Configure data mode: Packet mode, FSK modulation (datasheet defaults)
  this->write_register_(RFM69_REG_DATA_MODUL, 0x00);
  
  // Configure sync words (network ID)
  this->write_register_(RFM69_REG_SYNC_CONFIG, 0x88);  // Sync on, 2 bytes
  this->write_register_(RFM69_REG_SYNC_VALUE1, this->network_id_);
  this->write_register_(RFM69_REG_SYNC_VALUE1 + 1, this->network_id_);
  
  // Set node address
  this->write_register_(RFM69_REG_NODE_ADDR, this->node_id_);
  this->write_register_(RFM69_REG_BROADCAST_ADDR, 0xFF);
  
  // Configure power level (datasheet: PA0 for RFM69W)
  if (this->is_high_power_) {
    this->write_register_(RFM69_REG_PALEVEL, 0x9F);  // PA1+PA2, max power
  } else {
    this->write_register_(RFM69_REG_PALEVEL, 0x9F);  // PA0, max power for RFM69W
  }
  
  // Set RX mode
  if (!this->set_mode_(RFM69_MODE_RX)) {
    ESP_LOGE(TAG, "Failed to set RX mode");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "RFM69 setup complete");
}

void RFM69Component::dump_config() {
  ESP_LOGCONFIG(TAG, "RFM69:");
  LOG_PIN("  CS Pin: ", this->cs_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  Interrupt Pin: ", this->interrupt_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %.1f MHz", this->frequency_);
  ESP_LOGCONFIG(TAG, "  Node ID: %d", this->node_id_);
  ESP_LOGCONFIG(TAG, "  Network ID: %d", this->network_id_);
  ESP_LOGCONFIG(TAG, "  High Power: %s", YESNO(this->is_high_power_));
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with RFM69 failed!");
  }
}

void RFM69Component::loop() {
  // Check for incoming packets via interrupt pin
  if (this->interrupt_pin_ != nullptr && this->interrupt_pin_->digital_read()) {
    this->handle_interrupt_();
  }
}

bool RFM69Component::write_register_(uint8_t reg, uint8_t value) {
  this->enable();
  this->write_byte(reg | 0x80);  // Set write bit
  this->write_byte(value);
  this->disable();
  return true;
}

uint8_t RFM69Component::read_register_(uint8_t reg) {
  this->enable();
  this->write_byte(reg & 0x7F);  // Clear write bit
  uint8_t value = this->read_byte();
  this->disable();
  return value;
}

bool RFM69Component::set_mode_(uint8_t mode) {
  this->write_register_(RFM69_REG_OPMODE, mode);
  
  // Wait for mode ready
  uint32_t start = millis();
  while ((millis() - start) < 100) {
    if (this->read_register_(RFM69_REG_IRQ_FLAGS1) & RFM69_IRQ1_MODE_READY) {
      return true;
    }
    delay(1);
  }
  
  ESP_LOGW(TAG, "Timeout waiting for mode 0x%02X", mode);
  return false;
}

bool RFM69Component::set_frequency_() {
  // Calculate frequency register value
  // Datasheet: Freq = (FRF * FXOSC) / 2^19, where FXOSC = 32MHz
  uint32_t frf = (uint32_t)((this->frequency_ * 1000000.0f) / RFM69_FSTEP);
  
  this->write_register_(RFM69_REG_FRF_MSB, (frf >> 16) & 0xFF);
  this->write_register_(RFM69_REG_FRF_MID, (frf >> 8) & 0xFF);
  this->write_register_(RFM69_REG_FRF_LSB, frf & 0xFF);
  
  ESP_LOGD(TAG, "Frequency set to %.3f MHz (FRF: 0x%06X)", this->frequency_, frf);
  return true;
}

void RFM69Component::handle_interrupt_() {
  uint8_t irq_flags2 = this->read_register_(RFM69_REG_IRQ_FLAGS2);
  
  // Check if payload ready
  if (irq_flags2 & RFM69_IRQ2_PAYLOAD_READY) {
    // Read FIFO (simple implementation - just clear the flag)
    uint8_t dummy = this->read_register_(RFM69_REG_FIFO);
    
    ESP_LOGD(TAG, "Packet received");
    this->packet_received_ = true;
    this->packet_received_callbacks_.call();
  }
}

}  // namespace rfm69
}  // namespace esphome
