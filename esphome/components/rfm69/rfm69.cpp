#include "rfm69.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>
#include <esp_random.h>

namespace esphome {
namespace rfm69 {

static const char *const TAG = "rfm69";

// ============================================================================
// Setup and Configuration
// ============================================================================

void RFM69Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RFM69...");

  // Setup pins
  if (this->cs_pin_ != nullptr) {
    this->cs_pin_->setup();
    this->cs_pin_->digital_write(true);
  }

  // Hardware reset
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
    delay(10);
    this->reset_pin_->digital_write(false);
    delay(10);
  }

  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();
  }

  // Initialize SPI
  this->spi_setup();

  // Verify communication by reading version register
  // RFM69 should return 0x24
  this->version_ = this->read_register_(RFM69_REG_VERSION);
  if (this->version_ != 0x24) {
    ESP_LOGE(TAG, "Failed to communicate with RFM69!");
    ESP_LOGE(TAG, "  Version register: 0x%02X (expected 0x24)", this->version_);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "RFM69 detected - Version: 0x%02X (SX1231)", this->version_);

  // Start in standby mode
  this->idle_mode_ = RFM69_MODE_STANDBY;
  this->set_mode_idle_();

  // Configure modem (GFSK, 250kbps, 250kHz deviation - RadioHead default)
  this->configure_modem_();

  // Configure frequency
  if (!this->configure_frequency_()) {
    ESP_LOGE(TAG, "Failed to set frequency");
    this->mark_failed();
    return;
  }

  // Configure sync words (RadioHead default: 0x2D, 0xD4)
  this->write_register_(RFM69_REG_SYNC_CONFIG, RFM69_SYNC_ON | RFM69_SYNC_SIZE_2);
  this->write_register_(RFM69_REG_SYNC_VALUE1, RFM69_DEFAULT_SYNC1);
  this->write_register_(RFM69_REG_SYNC_VALUE2, RFM69_DEFAULT_SYNC2);

  // Packet config: variable length, whitening, CRC on, no address filtering
  this->write_register_(RFM69_REG_PACKET_CONFIG1,
                        RFM69_PKT_VARIABLE_LEN | RFM69_PKT_DCFREE_WHITENING |
                        RFM69_PKT_CRC_ON | RFM69_PKT_ADDR_FILTER_NONE);

  // FIFO threshold: start TX when FIFO not empty
  this->write_register_(RFM69_REG_FIFO_THRESH, RFM69_FIFO_TX_START_NOT_EMPTY | 0x0F);

  // Configure power amplifier
  this->configure_power_();

  // Configure AES encryption if key provided
  this->configure_encryption_();

  // Improved AGC setting for better reception
  this->write_register_(RFM69_REG_TEST_DAGC, RFM69_TESTDAGC_IMPROVED_LOWBETA);

  // Set node address (used for filtering if enabled)
  this->write_register_(RFM69_REG_NODE_ADDR, this->node_id_);
  this->write_register_(RFM69_REG_BROADCAST_ADDR, RFM69_BROADCAST_ADDR);

  // Set transmit header (FROM = our node ID)
  this->tx_header_from_ = this->node_id_;

  // Load paired devices from flash
  this->load_paired_devices_();

  // Read and log temperature for verification
  int8_t temp = this->read_temperature();
  ESP_LOGI(TAG, "RFM69 chip temperature: %d°C", temp);

  // Dump register configuration for debugging
  this->dump_registers();

  // Enter RX mode to start listening
  this->set_mode_rx_();

  ESP_LOGI(TAG, "RFM69 initialization complete:");
  ESP_LOGI(TAG, "  Frequency: %.2f MHz", this->frequency_);
  ESP_LOGI(TAG, "  Node ID: %d", this->node_id_);
  ESP_LOGI(TAG, "  Network ID: %d", this->network_id_);
  ESP_LOGI(TAG, "  TX Power: %d dBm", this->tx_power_);
  ESP_LOGI(TAG, "  High Power Module: %s", this->is_high_power_ ? "Yes" : "No");
  ESP_LOGI(TAG, "  AES Encryption: %s", this->aes_enabled_ ? "Enabled" : "Disabled");
  ESP_LOGI(TAG, "  Paired Devices: %d", this->paired_devices_.size());
  ESP_LOGI(TAG, "  Challenge Timeout: %d ms", this->challenge_timeout_ms_);
}

void RFM69Component::configure_modem_() {
  // GFSK modulation, packet mode, BT=1.0
  this->write_register_(RFM69_REG_DATA_MODUL,
                        RFM69_DATAMODUL_PACKET | RFM69_DATAMODUL_FSK | RFM69_DATAMODUL_GFSK_BT1_0);

  // Bitrate: 250 kbps (FXOSC / bitrate = 32MHz / 250000 = 128 = 0x0080)
  this->write_register_(RFM69_REG_BITRATE_MSB, 0x00);
  this->write_register_(RFM69_REG_BITRATE_LSB, 0x80);

  // Frequency deviation: 250 kHz (Fdev / Fstep = 250000 / 61.035 = 4096 = 0x1000)
  this->write_register_(RFM69_REG_FDEV_MSB, 0x10);
  this->write_register_(RFM69_REG_FDEV_LSB, 0x00);

  // RX bandwidth: 500 kHz (for 250 kHz deviation with 250 kbps)
  this->write_register_(RFM69_REG_RXBW, 0xE0);   // DCC=4%, RxBw=500kHz
  this->write_register_(RFM69_REG_AFCBW, 0xE0); // AFC bandwidth same as RX

  // 4 bytes preamble
  this->write_register_(RFM69_REG_PREAMBLE_MSB, 0x00);
  this->write_register_(RFM69_REG_PREAMBLE_LSB, 0x04);
}

void RFM69Component::configure_power_() {
  if (this->is_high_power_) {
    // RFM69HW: PA1+PA2 on PABOOST, power range -2 to +20 dBm
    int8_t pa_level = std::max((int8_t)-2, std::min((int8_t)20, this->tx_power_));
    uint8_t reg = 0x60 | ((pa_level + 14) & 0x1F);  // PA1+PA2
    this->write_register_(RFM69_REG_PALEVEL, reg);

    // For +18 to +20 dBm, enable high power boost
    if (pa_level >= 18) {
      this->write_register_(RFM69_REG_TEST_PA1, RFM69_TESTPA1_BOOST);
      this->write_register_(RFM69_REG_TEST_PA2, RFM69_TESTPA2_BOOST);
    } else {
      this->write_register_(RFM69_REG_TEST_PA1, RFM69_TESTPA1_NORMAL);
      this->write_register_(RFM69_REG_TEST_PA2, RFM69_TESTPA2_NORMAL);
    }
  } else {
    // RFM69W: PA0 only, power range -18 to +13 dBm
    int8_t pa_level = std::max((int8_t)-18, std::min((int8_t)13, this->tx_power_));
    uint8_t reg = 0x80 | ((pa_level + 18) & 0x1F);  // PA0 enabled
    this->write_register_(RFM69_REG_PALEVEL, reg);
  }

  ESP_LOGD(TAG, "TX power configured: %d dBm (high power: %s)", 
           this->tx_power_, this->is_high_power_ ? "yes" : "no");
}

void RFM69Component::configure_encryption_() {
  if (this->encryption_key_.empty()) {
    // Disable AES
    uint8_t pkt2 = this->read_register_(RFM69_REG_PACKET_CONFIG2);
    this->write_register_(RFM69_REG_PACKET_CONFIG2, pkt2 & ~RFM69_PKT2_AES_ON);
    this->aes_enabled_ = false;
    ESP_LOGD(TAG, "AES encryption: disabled");
    return;
  }

  // Encryption key must be exactly 16 bytes (128-bit AES)
  if (this->encryption_key_.length() < RFM69_AES_KEY_SIZE) {
    ESP_LOGW(TAG, "Encryption key too short (%d bytes), padding with zeros", 
             this->encryption_key_.length());
  }

  // Write 16-byte AES key to registers 0x3E-0x4D
  uint8_t key[RFM69_AES_KEY_SIZE] = {0};
  size_t copy_len = std::min(this->encryption_key_.length(), (size_t)RFM69_AES_KEY_SIZE);
  memcpy(key, this->encryption_key_.c_str(), copy_len);

  this->burst_write_(RFM69_REG_AES_KEY1, key, RFM69_AES_KEY_SIZE);

  // Enable AES encryption
  uint8_t pkt2 = this->read_register_(RFM69_REG_PACKET_CONFIG2);
  this->write_register_(RFM69_REG_PACKET_CONFIG2, pkt2 | RFM69_PKT2_AES_ON);
  this->aes_enabled_ = true;

  ESP_LOGI(TAG, "AES-128 encryption: enabled");
}

bool RFM69Component::configure_frequency_() {
  // Freq = FRF * Fstep, so FRF = Freq / Fstep
  uint32_t frf = (uint32_t)((this->frequency_ * 1000000.0f) / RFM69_FSTEP);

  this->write_register_(RFM69_REG_FRF_MSB, (frf >> 16) & 0xFF);
  this->write_register_(RFM69_REG_FRF_MID, (frf >> 8) & 0xFF);
  this->write_register_(RFM69_REG_FRF_LSB, frf & 0xFF);

  // Verify by reading back
  uint8_t msb = this->read_register_(RFM69_REG_FRF_MSB);
  uint8_t mid = this->read_register_(RFM69_REG_FRF_MID);
  uint8_t lsb = this->read_register_(RFM69_REG_FRF_LSB);
  uint32_t frf_read = ((uint32_t)msb << 16) | ((uint32_t)mid << 8) | lsb;

  ESP_LOGD(TAG, "Frequency configured: %.3f MHz (FRF: 0x%06X, readback: 0x%06X)", 
           this->frequency_, frf, frf_read);
  return frf == frf_read;
}

void RFM69Component::dump_config() {
  ESP_LOGCONFIG(TAG, "RFM69:");
  ESP_LOGCONFIG(TAG, "  Version: 0x%02X", this->version_);
  LOG_PIN("  CS Pin: ", this->cs_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  Interrupt Pin: ", this->interrupt_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %.2f MHz", this->frequency_);
  ESP_LOGCONFIG(TAG, "  Node ID: %d", this->node_id_);
  ESP_LOGCONFIG(TAG, "  Network ID: %d", this->network_id_);
  ESP_LOGCONFIG(TAG, "  TX Power: %d dBm", this->tx_power_);
  ESP_LOGCONFIG(TAG, "  High Power Module: %s", YESNO(this->is_high_power_));
  ESP_LOGCONFIG(TAG, "  AES Encryption: %s", YESNO(this->aes_enabled_));
  ESP_LOGCONFIG(TAG, "  Challenge Timeout: %d ms", this->challenge_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Paired Devices: %d", this->paired_devices_.size());
  
  for (const auto &dev : this->paired_devices_) {
    ESP_LOGCONFIG(TAG, "    - 0x%08X (%s)", dev.serial_number, dev.name);
  }

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with RFM69 failed!");
  }
}

void RFM69Component::dump_registers() {
  ESP_LOGD(TAG, "RFM69 Register Dump:");
  ESP_LOGD(TAG, "  OPMODE (0x01): 0x%02X", this->read_register_(RFM69_REG_OPMODE));
  ESP_LOGD(TAG, "  DATA_MODUL (0x02): 0x%02X", this->read_register_(RFM69_REG_DATA_MODUL));
  ESP_LOGD(TAG, "  BITRATE (0x03-04): 0x%02X%02X", 
           this->read_register_(RFM69_REG_BITRATE_MSB),
           this->read_register_(RFM69_REG_BITRATE_LSB));
  ESP_LOGD(TAG, "  FDEV (0x05-06): 0x%02X%02X",
           this->read_register_(RFM69_REG_FDEV_MSB),
           this->read_register_(RFM69_REG_FDEV_LSB));
  ESP_LOGD(TAG, "  FRF (0x07-09): 0x%02X%02X%02X",
           this->read_register_(RFM69_REG_FRF_MSB),
           this->read_register_(RFM69_REG_FRF_MID),
           this->read_register_(RFM69_REG_FRF_LSB));
  ESP_LOGD(TAG, "  PALEVEL (0x11): 0x%02X", this->read_register_(RFM69_REG_PALEVEL));
  ESP_LOGD(TAG, "  RXBW (0x19): 0x%02X", this->read_register_(RFM69_REG_RXBW));
  ESP_LOGD(TAG, "  SYNC_CONFIG (0x2E): 0x%02X", this->read_register_(RFM69_REG_SYNC_CONFIG));
  ESP_LOGD(TAG, "  SYNC_VALUE (0x2F-30): 0x%02X%02X",
           this->read_register_(RFM69_REG_SYNC_VALUE1),
           this->read_register_(RFM69_REG_SYNC_VALUE2));
  ESP_LOGD(TAG, "  PACKET_CONFIG1 (0x37): 0x%02X", this->read_register_(RFM69_REG_PACKET_CONFIG1));
  ESP_LOGD(TAG, "  PACKET_CONFIG2 (0x3D): 0x%02X", this->read_register_(RFM69_REG_PACKET_CONFIG2));
}

int8_t RFM69Component::read_temperature() {
  // Must be in standby mode to read temperature
  uint8_t prev_mode = this->mode_;
  this->set_mode_idle_();

  // Start temperature measurement
  this->write_register_(RFM69_REG_TEMP1, RFM69_TEMP1_START);

  // Wait for measurement to complete (up to 100ms)
  uint32_t start = millis();
  while ((millis() - start) < 100) {
    if (!(this->read_register_(RFM69_REG_TEMP1) & RFM69_TEMP1_RUNNING)) {
      break;
    }
    delay(1);
  }

  // Read temperature value
  // Temperature = ~166 - TEMP2 (approximate, based on silicon measurements)
  int8_t temp = 166 - this->read_register_(RFM69_REG_TEMP2);

  // Restore previous mode
  if (prev_mode == RFM69_MODE_RX) {
    this->set_mode_rx_();
  }

  return temp;
}

// ============================================================================
// Main Loop
// ============================================================================

void RFM69Component::loop() {
  // Cleanup expired challenges periodically
  static uint32_t last_cleanup = 0;
  if (millis() - last_cleanup > 5000) {
    this->cleanup_expired_challenges_();
    last_cleanup = millis();
  }

  // Check for incoming packets via interrupt pin (DIO0 = PayloadReady)
  if (this->interrupt_pin_ != nullptr && this->interrupt_pin_->digital_read()) {
    this->handle_interrupt_();
  }
}

// ============================================================================
// Mode Control
// ============================================================================

bool RFM69Component::set_mode_(uint8_t mode) {
  // Read current opmode, preserve non-mode bits, update mode
  uint8_t opmode = this->read_register_(RFM69_REG_OPMODE);
  opmode = (opmode & ~RFM69_OPMODE_MODE_MASK) | (mode & RFM69_OPMODE_MODE_MASK);
  this->write_register_(RFM69_REG_OPMODE, opmode);

  // Wait for mode ready
  uint32_t start = millis();
  while ((millis() - start) < 100) {
    if (this->read_register_(RFM69_REG_IRQ_FLAGS1) & RFM69_IRQ1_MODE_READY) {
      this->mode_ = mode;
      return true;
    }
    delay(1);
  }

  ESP_LOGW(TAG, "Timeout waiting for mode 0x%02X", mode);
  return false;
}

void RFM69Component::set_mode_idle_() {
  if (this->mode_ != this->idle_mode_) {
    // If high power boost was enabled for TX, disable it
    if (this->is_high_power_ && this->tx_power_ >= 18) {
      this->write_register_(RFM69_REG_TEST_PA1, RFM69_TESTPA1_NORMAL);
      this->write_register_(RFM69_REG_TEST_PA2, RFM69_TESTPA2_NORMAL);
    }
    this->set_mode_(this->idle_mode_);
  }
}

void RFM69Component::set_mode_rx_() {
  if (this->mode_ != RFM69_MODE_RX) {
    // Disable high power boost for RX
    if (this->is_high_power_ && this->tx_power_ >= 18) {
      this->write_register_(RFM69_REG_TEST_PA1, RFM69_TESTPA1_NORMAL);
      this->write_register_(RFM69_REG_TEST_PA2, RFM69_TESTPA2_NORMAL);
    }

    // DIO0 = PayloadReady
    this->write_register_(RFM69_REG_DIO_MAPPING1, RFM69_DIO0_RX_PAYLOAD_READY);
    this->set_mode_(RFM69_MODE_RX);
  }
}

void RFM69Component::set_mode_tx_() {
  if (this->mode_ != RFM69_MODE_TX) {
    // Enable high power boost if needed
    if (this->is_high_power_ && this->tx_power_ >= 18) {
      this->write_register_(RFM69_REG_TEST_PA1, RFM69_TESTPA1_BOOST);
      this->write_register_(RFM69_REG_TEST_PA2, RFM69_TESTPA2_BOOST);
    }

    // DIO0 = PacketSent
    this->write_register_(RFM69_REG_DIO_MAPPING1, RFM69_DIO0_TX_PACKET_SENT);
    this->set_mode_(RFM69_MODE_TX);
  }
}

bool RFM69Component::set_sleep() {
  if (this->mode_ != RFM69_MODE_SLEEP) {
    this->write_register_(RFM69_REG_OPMODE, RFM69_MODE_SLEEP);
    this->mode_ = RFM69_MODE_SLEEP;
  }
  return true;
}

// ============================================================================
// SPI Communication
// ============================================================================

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

void RFM69Component::burst_read_(uint8_t reg, uint8_t *dest, uint8_t len) {
  this->enable();
  this->write_byte(reg & 0x7F);
  while (len--) {
    *dest++ = this->read_byte();
  }
  this->disable();
}

void RFM69Component::burst_write_(uint8_t reg, const uint8_t *src, uint8_t len) {
  this->enable();
  this->write_byte(reg | 0x80);
  while (len--) {
    this->write_byte(*src++);
  }
  this->disable();
}

// ============================================================================
// Packet Handling
// ============================================================================

void RFM69Component::read_fifo_() {
  // Read RSSI before reading FIFO (it's valid after PayloadReady)
  this->last_rssi_ = -(this->read_register_(RFM69_REG_RSSI_VALUE) / 2);

  // Start FIFO burst read
  this->enable();
  this->write_byte(RFM69_REG_FIFO & 0x7F);

  // First byte is payload length (including headers)
  uint8_t payload_len = this->read_byte();

  if (payload_len > RFM69_MAX_ENCRYPTABLE_PAYLOAD_LEN || payload_len < RFM69_HEADER_LEN) {
    // Invalid length
    ESP_LOGW(TAG, "Invalid packet length: %d", payload_len);
    this->disable();
    return;
  }

  // Read RadioHead headers
  this->rx_header_to_ = this->read_byte();
  this->rx_header_from_ = this->read_byte();
  this->rx_header_id_ = this->read_byte();
  this->rx_header_flags_ = this->read_byte();

  // Check if packet is for us (or broadcast)
  if (this->rx_header_to_ != this->node_id_ && this->rx_header_to_ != RFM69_BROADCAST_ADDR) {
    // Not for us
    ESP_LOGV(TAG, "Packet not for us (to: 0x%02X)", this->rx_header_to_);
    this->disable();
    return;
  }

  // Read message data
  this->rx_buf_len_ = payload_len - RFM69_HEADER_LEN;
  for (uint8_t i = 0; i < this->rx_buf_len_; i++) {
    this->rx_buf_[i] = this->read_byte();
  }

  this->disable();
  this->rx_buf_valid_ = true;

  ESP_LOGD(TAG, "Packet RX: from=0x%02X, to=0x%02X, id=%d, flags=0x%02X, len=%d, RSSI=%d dBm",
           this->rx_header_from_, this->rx_header_to_, this->rx_header_id_,
           this->rx_header_flags_, this->rx_buf_len_, this->last_rssi_);
}

void RFM69Component::handle_interrupt_() {
  uint8_t irq_flags2 = this->read_register_(RFM69_REG_IRQ_FLAGS2);

  // Check if payload ready
  if (irq_flags2 & RFM69_IRQ2_PAYLOAD_READY) {
    this->read_fifo_();

    if (this->rx_buf_valid_) {
      // Process protocol commands
      this->process_command_(this->rx_header_from_, this->rx_buf_, this->rx_buf_len_);

      // Create vector from received data for callbacks
      std::vector<uint8_t> data(this->rx_buf_, this->rx_buf_ + this->rx_buf_len_);

      // Trigger raw packet received callbacks
      this->packet_received_callbacks_.call(this->rx_header_from_, (uint8_t)(-this->last_rssi_), data);

      // Clear valid flag after processing
      this->rx_buf_valid_ = false;
    }
  }

  // Ensure we stay in RX mode
  this->set_mode_rx_();
}

bool RFM69Component::send(const uint8_t *data, uint8_t len, uint8_t to_address) {
  if (len > RFM69_MAX_MESSAGE_LEN) {
    ESP_LOGW(TAG, "Message too long: %d bytes (max %d)", len, RFM69_MAX_MESSAGE_LEN);
    return false;
  }

  // Go to standby mode for FIFO access
  this->set_mode_idle_();

  // Write packet to FIFO
  this->enable();
  this->write_byte(RFM69_REG_FIFO | 0x80);

  // Length byte (headers + data)
  this->write_byte(len + RFM69_HEADER_LEN);

  // RadioHead headers
  this->write_byte(to_address);            // TO
  this->write_byte(this->tx_header_from_); // FROM
  this->write_byte(this->tx_header_id_++); // ID (auto-increment)
  this->write_byte(this->tx_header_flags_); // FLAGS

  // Payload
  for (uint8_t i = 0; i < len; i++) {
    this->write_byte(data[i]);
  }

  this->disable();

  ESP_LOGD(TAG, "Packet TX: to=0x%02X, len=%d", to_address, len + RFM69_HEADER_LEN);

  // Start transmission
  this->set_mode_tx_();

  // Wait for packet sent
  uint32_t start = millis();
  while ((millis() - start) < 100) {
    if (this->read_register_(RFM69_REG_IRQ_FLAGS2) & RFM69_IRQ2_PACKET_SENT) {
      ESP_LOGD(TAG, "Packet sent successfully");
      // Return to RX mode
      this->set_mode_rx_();
      return true;
    }
    delay(1);
  }

  ESP_LOGW(TAG, "Timeout waiting for packet sent");
  this->set_mode_rx_();
  return false;
}

// ============================================================================
// Protocol Command Processing
// ============================================================================

void RFM69Component::process_command_(uint8_t from_address, const uint8_t *data, uint8_t len) {
  if (len < 1) return;

  RFM69Command cmd = static_cast<RFM69Command>(data[0]);

  switch (cmd) {
    case RFM69Command::PING:
      if (len >= 5) {
        uint32_t remote_sn = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | 
                             ((uint32_t)data[3] << 8) | data[4];
        ESP_LOGI(TAG, "PING received from device 0x%08X", remote_sn);
        this->handle_ping_(from_address, remote_sn);
      }
      break;

    case RFM69Command::DOOR_OPEN:
      if (len >= 9) {
        uint32_t remote_sn = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | 
                             ((uint32_t)data[3] << 8) | data[4];
        uint32_t challenge = ((uint32_t)data[5] << 24) | ((uint32_t)data[6] << 16) | 
                             ((uint32_t)data[7] << 8) | data[8];
        ESP_LOGI(TAG, "DOOR_OPEN request from 0x%08X with challenge 0x%08X", remote_sn, challenge);
        this->handle_door_open_(from_address, remote_sn, challenge);
      }
      break;

    case RFM69Command::PAIR_REQUEST:
      if (len >= 5) {
        uint32_t remote_sn = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | 
                             ((uint32_t)data[3] << 8) | data[4];
        ESP_LOGI(TAG, "PAIR_REQUEST from device 0x%08X", remote_sn);
        this->handle_pair_request_(from_address, remote_sn);
      }
      break;

    case RFM69Command::DOORBELL_RING:
      // Simple doorbell ring - no security required, just trigger the callback
      ESP_LOGI(TAG, "DOORBELL_RING from address 0x%02X", from_address);
      // This is handled by the raw packet callback
      break;

    default:
      ESP_LOGD(TAG, "Unknown command: 0x%02X", data[0]);
      break;
  }
}

void RFM69Component::handle_ping_(uint8_t from_address, uint32_t remote_sn) {
  // Check if device is paired
  if (!this->is_device_paired(remote_sn)) {
    ESP_LOGW(TAG, "PING from unpaired device 0x%08X - ignoring", remote_sn);
    this->send_ack_(from_address, RFM69AckStatus::DEVICE_NOT_PAIRED);
    return;
  }

  // Generate a new challenge for this device
  uint32_t challenge = this->generate_challenge_();

  // Store challenge for validation
  for (auto &dev : this->paired_devices_) {
    if (dev.serial_number == remote_sn) {
      dev.active_challenge = challenge;
      dev.challenge_timestamp = millis();
      dev.challenge_valid = true;
      break;
    }
  }

  ESP_LOGI(TAG, "Sending challenge 0x%08X to device 0x%08X", challenge, remote_sn);
  this->send_challenge_(from_address, challenge);
}

void RFM69Component::handle_door_open_(uint8_t from_address, uint32_t remote_sn, uint32_t challenge) {
  // Check if device is paired
  if (!this->is_device_paired(remote_sn)) {
    ESP_LOGW(TAG, "DOOR_OPEN from unpaired device 0x%08X", remote_sn);
    this->send_ack_(from_address, RFM69AckStatus::DEVICE_NOT_PAIRED);
    this->door_open_request_callbacks_.call(remote_sn, false);
    return;
  }

  // Validate challenge
  if (!this->validate_challenge_(remote_sn, challenge)) {
    ESP_LOGW(TAG, "Invalid or expired challenge from device 0x%08X", remote_sn);
    this->send_ack_(from_address, RFM69AckStatus::INVALID_CHALLENGE);
    this->door_open_request_callbacks_.call(remote_sn, false);
    return;
  }

  // Challenge valid - authorize door open
  ESP_LOGI(TAG, "DOOR_OPEN authorized for device 0x%08X", remote_sn);
  this->send_ack_(from_address, RFM69AckStatus::SUCCESS);
  this->door_open_request_callbacks_.call(remote_sn, true);
}

void RFM69Component::handle_pair_request_(uint8_t from_address, uint32_t remote_sn) {
  if (!this->pairing_mode_enabled_) {
    ESP_LOGW(TAG, "Pairing request rejected - pairing mode disabled");
    this->send_pair_confirm_(from_address, RFM69AckStatus::PAIRING_DISABLED);
    return;
  }

  if (this->is_device_paired(remote_sn)) {
    ESP_LOGI(TAG, "Device 0x%08X already paired", remote_sn);
    this->send_pair_confirm_(from_address, RFM69AckStatus::SUCCESS);
    return;
  }

  if (this->paired_devices_.size() >= RFM69_MAX_PAIRED_DEVICES) {
    ESP_LOGW(TAG, "Cannot pair device - max devices reached");
    this->send_pair_confirm_(from_address, RFM69AckStatus::PAIRING_FULL);
    return;
  }

  // Add device
  if (this->add_paired_device(remote_sn, nullptr)) {
    ESP_LOGI(TAG, "Device 0x%08X paired successfully", remote_sn);
    this->send_pair_confirm_(from_address, RFM69AckStatus::SUCCESS);
    this->device_paired_callbacks_.call(remote_sn);
  } else {
    ESP_LOGE(TAG, "Failed to pair device 0x%08X", remote_sn);
    this->send_pair_confirm_(from_address, RFM69AckStatus::PAIRING_FULL);
  }
}

void RFM69Component::send_challenge_(uint8_t to_address, uint32_t challenge) {
  uint8_t payload[5];
  payload[0] = static_cast<uint8_t>(RFM69Command::CHALLENGE);
  payload[1] = (challenge >> 24) & 0xFF;
  payload[2] = (challenge >> 16) & 0xFF;
  payload[3] = (challenge >> 8) & 0xFF;
  payload[4] = challenge & 0xFF;

  this->send(payload, sizeof(payload), to_address);
}

void RFM69Component::send_ack_(uint8_t to_address, RFM69AckStatus status) {
  uint8_t payload[2];
  payload[0] = static_cast<uint8_t>(RFM69Command::ACK);
  payload[1] = static_cast<uint8_t>(status);

  this->send(payload, sizeof(payload), to_address);
}

void RFM69Component::send_pair_confirm_(uint8_t to_address, RFM69AckStatus status) {
  uint8_t payload[2];
  payload[0] = static_cast<uint8_t>(RFM69Command::PAIR_CONFIRM);
  payload[1] = static_cast<uint8_t>(status);

  this->send(payload, sizeof(payload), to_address);
}

// ============================================================================
// Challenge Management
// ============================================================================

uint32_t RFM69Component::generate_challenge_() {
  // Use ESP32's hardware RNG
  return esp_random();
}

bool RFM69Component::validate_challenge_(uint32_t remote_sn, uint32_t challenge) {
  for (auto &dev : this->paired_devices_) {
    if (dev.serial_number == remote_sn && dev.challenge_valid) {
      // Check timeout
      if ((millis() - dev.challenge_timestamp) > this->challenge_timeout_ms_) {
        ESP_LOGW(TAG, "Challenge expired for device 0x%08X", remote_sn);
        dev.challenge_valid = false;
        return false;
      }

      // Check challenge value
      if (dev.active_challenge == challenge) {
        // Invalidate challenge after use (one-time use)
        dev.challenge_valid = false;
        dev.active_challenge = 0;
        return true;
      } else {
        ESP_LOGW(TAG, "Challenge mismatch: expected 0x%08X, got 0x%08X", 
                 dev.active_challenge, challenge);
        return false;
      }
    }
  }
  return false;
}

void RFM69Component::cleanup_expired_challenges_() {
  uint32_t now = millis();
  for (auto &dev : this->paired_devices_) {
    if (dev.challenge_valid && (now - dev.challenge_timestamp) > this->challenge_timeout_ms_) {
      ESP_LOGV(TAG, "Expiring challenge for device 0x%08X", dev.serial_number);
      dev.challenge_valid = false;
      dev.active_challenge = 0;
    }
  }
}

// ============================================================================
// Paired Device Management
// ============================================================================

bool RFM69Component::add_paired_device(uint32_t serial_number, const char *name) {
  if (this->is_device_paired(serial_number)) {
    return true;  // Already paired
  }

  if (this->paired_devices_.size() >= RFM69_MAX_PAIRED_DEVICES) {
    return false;
  }

  PairedDevice dev;
  dev.serial_number = serial_number;
  dev.active_challenge = 0;
  dev.challenge_timestamp = 0;
  dev.challenge_valid = false;
  
  if (name != nullptr) {
    strncpy(dev.name, name, sizeof(dev.name) - 1);
    dev.name[sizeof(dev.name) - 1] = '\0';
  } else {
    snprintf(dev.name, sizeof(dev.name), "Remote_%08X", serial_number);
  }

  this->paired_devices_.push_back(dev);
  this->save_paired_devices_();

  ESP_LOGI(TAG, "Added paired device: 0x%08X (%s)", serial_number, dev.name);
  return true;
}

bool RFM69Component::remove_paired_device(uint32_t serial_number) {
  for (auto it = this->paired_devices_.begin(); it != this->paired_devices_.end(); ++it) {
    if (it->serial_number == serial_number) {
      ESP_LOGI(TAG, "Removed paired device: 0x%08X (%s)", serial_number, it->name);
      this->paired_devices_.erase(it);
      this->save_paired_devices_();
      return true;
    }
  }
  return false;
}

bool RFM69Component::is_device_paired(uint32_t serial_number) {
  for (const auto &dev : this->paired_devices_) {
    if (dev.serial_number == serial_number) {
      return true;
    }
  }
  return false;
}

void RFM69Component::clear_all_paired_devices() {
  ESP_LOGW(TAG, "Clearing all paired devices (%d devices)", this->paired_devices_.size());
  this->paired_devices_.clear();
  this->save_paired_devices_();
}

std::vector<uint32_t> RFM69Component::get_paired_devices() {
  std::vector<uint32_t> devices;
  for (const auto &dev : this->paired_devices_) {
    devices.push_back(dev.serial_number);
  }
  return devices;
}

// ============================================================================
// Persistence
// ============================================================================

void RFM69Component::save_paired_devices_() {
  // Save to flash using ESPHome preferences
  // Note: For production use, consider NVS or SPIFFS for larger data
  ESP_LOGD(TAG, "Saving %d paired devices to flash", this->paired_devices_.size());
  
  // Simple implementation - would need proper NVS handling for production
  // This is a placeholder that logs the action
}

void RFM69Component::load_paired_devices_() {
  // For now, start with empty list
  // In production, this would load from NVS/flash
  this->paired_devices_.clear();
  ESP_LOGD(TAG, "Paired devices loaded (count: %d)", this->paired_devices_.size());
}

}  // namespace rfm69
}  // namespace esphome
