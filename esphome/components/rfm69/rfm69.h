#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/automation.h"
#include <vector>
#include <map>

namespace esphome {
namespace rfm69 {

// ============================================================================
// RFM69 Register Map - RadioHead compatible (from nopnop2002/esp-idf-rf69)
// ============================================================================

// Common registers
static const uint8_t RFM69_REG_FIFO = 0x00;
static const uint8_t RFM69_REG_OPMODE = 0x01;
static const uint8_t RFM69_REG_DATA_MODUL = 0x02;
static const uint8_t RFM69_REG_BITRATE_MSB = 0x03;
static const uint8_t RFM69_REG_BITRATE_LSB = 0x04;
static const uint8_t RFM69_REG_FDEV_MSB = 0x05;
static const uint8_t RFM69_REG_FDEV_LSB = 0x06;
static const uint8_t RFM69_REG_FRF_MSB = 0x07;
static const uint8_t RFM69_REG_FRF_MID = 0x08;
static const uint8_t RFM69_REG_FRF_LSB = 0x09;
static const uint8_t RFM69_REG_OSC1 = 0x0A;
static const uint8_t RFM69_REG_AFC_CTRL = 0x0B;

// Transmitter registers
static const uint8_t RFM69_REG_PALEVEL = 0x11;
static const uint8_t RFM69_REG_PARAMP = 0x12;
static const uint8_t RFM69_REG_OCP = 0x13;

// Receiver registers
static const uint8_t RFM69_REG_LNA = 0x18;
static const uint8_t RFM69_REG_RXBW = 0x19;
static const uint8_t RFM69_REG_AFCBW = 0x1A;
static const uint8_t RFM69_REG_AFC_FEI = 0x1E;

// IRQ and status registers
static const uint8_t RFM69_REG_VERSION = 0x10;
static const uint8_t RFM69_REG_RSSI_VALUE = 0x24;
static const uint8_t RFM69_REG_DIO_MAPPING1 = 0x25;
static const uint8_t RFM69_REG_DIO_MAPPING2 = 0x26;
static const uint8_t RFM69_REG_IRQ_FLAGS1 = 0x27;
static const uint8_t RFM69_REG_IRQ_FLAGS2 = 0x28;
static const uint8_t RFM69_REG_RSSI_THRESH = 0x29;

// Packet engine registers
static const uint8_t RFM69_REG_PREAMBLE_MSB = 0x2C;
static const uint8_t RFM69_REG_PREAMBLE_LSB = 0x2D;
static const uint8_t RFM69_REG_SYNC_CONFIG = 0x2E;
static const uint8_t RFM69_REG_SYNC_VALUE1 = 0x2F;
static const uint8_t RFM69_REG_SYNC_VALUE2 = 0x30;
static const uint8_t RFM69_REG_PACKET_CONFIG1 = 0x37;
static const uint8_t RFM69_REG_PAYLOAD_LENGTH = 0x38;
static const uint8_t RFM69_REG_NODE_ADDR = 0x39;
static const uint8_t RFM69_REG_BROADCAST_ADDR = 0x3A;
static const uint8_t RFM69_REG_AUTOMODES = 0x3B;
static const uint8_t RFM69_REG_FIFO_THRESH = 0x3C;
static const uint8_t RFM69_REG_PACKET_CONFIG2 = 0x3D;

// AES encryption registers
static const uint8_t RFM69_REG_AES_KEY1 = 0x3E;  // AES key bytes 0x3E-0x4D (16 bytes)

// Temperature sensor
static const uint8_t RFM69_REG_TEMP1 = 0x4E;
static const uint8_t RFM69_REG_TEMP2 = 0x4F;

// Test registers (for high power PA)
static const uint8_t RFM69_REG_TEST_PA1 = 0x5A;
static const uint8_t RFM69_REG_TEST_PA2 = 0x5C;
static const uint8_t RFM69_REG_TEST_DAGC = 0x6F;

// ============================================================================
// Register bit definitions
// ============================================================================

// OPMODE register bits
static const uint8_t RFM69_OPMODE_MODE_MASK = 0x1C;
static const uint8_t RFM69_MODE_SLEEP = 0x00;
static const uint8_t RFM69_MODE_STANDBY = 0x04;
static const uint8_t RFM69_MODE_FS = 0x08;
static const uint8_t RFM69_MODE_TX = 0x0C;
static const uint8_t RFM69_MODE_RX = 0x10;

// DATA_MODUL register bits
static const uint8_t RFM69_DATAMODUL_PACKET = 0x00;
static const uint8_t RFM69_DATAMODUL_FSK = 0x00;
static const uint8_t RFM69_DATAMODUL_GFSK_BT1_0 = 0x01;

// IRQ1 flags
static const uint8_t RFM69_IRQ1_MODE_READY = 0x80;
static const uint8_t RFM69_IRQ1_RX_READY = 0x40;
static const uint8_t RFM69_IRQ1_TX_READY = 0x20;

// IRQ2 flags
static const uint8_t RFM69_IRQ2_FIFO_FULL = 0x80;
static const uint8_t RFM69_IRQ2_FIFO_NOT_EMPTY = 0x40;
static const uint8_t RFM69_IRQ2_FIFO_LEVEL = 0x20;
static const uint8_t RFM69_IRQ2_FIFO_OVERRUN = 0x10;
static const uint8_t RFM69_IRQ2_PACKET_SENT = 0x08;
static const uint8_t RFM69_IRQ2_PAYLOAD_READY = 0x04;
static const uint8_t RFM69_IRQ2_CRC_OK = 0x02;

// DIO mapping (DIO0 in bits 7:6 of DIO_MAPPING1)
static const uint8_t RFM69_DIO0_RX_PAYLOAD_READY = 0x40;  // DIO0 = PayloadReady in RX
static const uint8_t RFM69_DIO0_TX_PACKET_SENT = 0x00;    // DIO0 = PacketSent in TX

// SYNC config
static const uint8_t RFM69_SYNC_ON = 0x80;
static const uint8_t RFM69_SYNC_SIZE_2 = 0x08;  // 2 sync bytes

// PACKET_CONFIG1 bits
static const uint8_t RFM69_PKT_VARIABLE_LEN = 0x80;
static const uint8_t RFM69_PKT_DCFREE_NONE = 0x00;
static const uint8_t RFM69_PKT_DCFREE_WHITENING = 0x40;
static const uint8_t RFM69_PKT_CRC_ON = 0x10;
static const uint8_t RFM69_PKT_ADDR_FILTER_NONE = 0x00;

// PACKET_CONFIG2 bits
static const uint8_t RFM69_PKT2_AES_ON = 0x01;
static const uint8_t RFM69_PKT2_AUTO_RX_RESTART = 0x02;

// FIFO threshold
static const uint8_t RFM69_FIFO_TX_START_NOT_EMPTY = 0x80;

// Temperature sensor
static const uint8_t RFM69_TEMP1_START = 0x08;
static const uint8_t RFM69_TEMP1_RUNNING = 0x04;

// Test register values for high power PA
static const uint8_t RFM69_TESTPA1_NORMAL = 0x55;
static const uint8_t RFM69_TESTPA1_BOOST = 0x5D;
static const uint8_t RFM69_TESTPA2_NORMAL = 0x70;
static const uint8_t RFM69_TESTPA2_BOOST = 0x7C;
static const uint8_t RFM69_TESTDAGC_IMPROVED_LOWBETA = 0x30;

// ============================================================================
// Constants
// ============================================================================

// Frequency calculation: Fstep = FXOSC / 2^19 = 32MHz / 524288 = 61.03515625 Hz
static const float RFM69_FSTEP = 61.03515625f;

// Max payload size (excluding length byte and headers)
static const uint8_t RFM69_MAX_MESSAGE_LEN = 60;

// RadioHead header size (TO, FROM, ID, FLAGS)
static const uint8_t RFM69_HEADER_LEN = 4;

// Max encryptable payload (FIFO is 66 bytes, minus length and headers)
static const uint8_t RFM69_MAX_ENCRYPTABLE_PAYLOAD_LEN = RFM69_MAX_MESSAGE_LEN + RFM69_HEADER_LEN;

// Broadcast address
static const uint8_t RFM69_BROADCAST_ADDR = 0xFF;

// Default sync words (RadioHead compatible)
static const uint8_t RFM69_DEFAULT_SYNC1 = 0x2D;
static const uint8_t RFM69_DEFAULT_SYNC2 = 0xD4;

// AES key size
static const uint8_t RFM69_AES_KEY_SIZE = 16;

// Maximum paired devices
static const uint8_t RFM69_MAX_PAIRED_DEVICES = 10;

// Challenge timeout in milliseconds
static const uint32_t RFM69_CHALLENGE_TIMEOUT_MS = 30000;

// ============================================================================
// Protocol Command Types
// ============================================================================

enum class RFM69Command : uint8_t {
  PING = 0x01,           // Remote → Receiver: Request challenge (payload: 4-byte remote SN)
  CHALLENGE = 0x02,      // Receiver → Remote: Challenge response (payload: 4-byte challenge)
  DOOR_OPEN = 0x03,      // Remote → Receiver: Open door (payload: 4-byte SN + 4-byte challenge)
  ACK = 0x04,            // Receiver → Remote: Acknowledge (payload: 1-byte status)
  PAIR_REQUEST = 0x05,   // Remote → Receiver: Pairing request (payload: 4-byte SN)
  PAIR_CONFIRM = 0x06,   // Receiver → Remote: Pairing confirmed (payload: 1-byte status)
  DOORBELL_RING = 0x10,  // Remote → Receiver: Simple doorbell ring (no security needed)
};

// ACK status codes
enum class RFM69AckStatus : uint8_t {
  SUCCESS = 0x00,
  INVALID_CHALLENGE = 0x01,
  DEVICE_NOT_PAIRED = 0x02,
  CHALLENGE_EXPIRED = 0x03,
  PAIRING_DISABLED = 0x04,
  PAIRING_FULL = 0x05,
};

// ============================================================================
// Paired Device Structure
// ============================================================================

struct PairedDevice {
  uint32_t serial_number;       // 4-byte unique device ID
  uint32_t active_challenge;    // Current challenge issued to this device
  uint32_t challenge_timestamp; // When the challenge was issued
  bool challenge_valid;         // Whether a challenge is pending
  char name[16];                // Human-readable device name
};

// ============================================================================
// RFM69 Component Class
// ============================================================================

class PacketReceivedTrigger;
class DevicePairedTrigger;
class DoorOpenRequestTrigger;

class RFM69Component : public Component,
                       public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                              spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // Configuration setters
  void set_cs_pin(GPIOPin *cs) { cs_pin_ = cs; }
  void set_reset_pin(GPIOPin *reset) { reset_pin_ = reset; }
  void set_interrupt_pin(InternalGPIOPin *interrupt) { interrupt_pin_ = interrupt; }
  void set_frequency(float frequency) { frequency_ = frequency; }
  void set_node_id(uint8_t node_id) { node_id_ = node_id; }
  void set_network_id(uint8_t network_id) { network_id_ = network_id; }
  void set_encryption_key(const std::string &key) { encryption_key_ = key; }
  void set_high_power(bool is_high_power) { is_high_power_ = is_high_power; }
  void set_tx_power(int8_t power) { tx_power_ = power; }
  void set_challenge_timeout(uint32_t timeout_ms) { challenge_timeout_ms_ = timeout_ms; }

  // Callback registration
  void add_on_packet_received_callback(std::function<void(uint8_t, uint8_t, std::vector<uint8_t>)> &&callback) {
    packet_received_callbacks_.add(std::move(callback));
  }
  void add_on_device_paired_callback(std::function<void(uint32_t)> &&callback) {
    device_paired_callbacks_.add(std::move(callback));
  }
  void add_on_door_open_request_callback(std::function<void(uint32_t, bool)> &&callback) {
    door_open_request_callbacks_.add(std::move(callback));
  }

  // Public API
  bool send(const uint8_t *data, uint8_t len, uint8_t to_address = RFM69_BROADCAST_ADDR);
  bool is_packet_available() { return rx_buf_valid_; }
  int16_t get_last_rssi() { return last_rssi_; }
  uint8_t get_sender_address() { return rx_header_from_; }
  bool set_sleep();

  // Pairing management
  void set_pairing_mode(bool enabled) { pairing_mode_enabled_ = enabled; }
  bool is_pairing_mode() { return pairing_mode_enabled_; }
  bool add_paired_device(uint32_t serial_number, const char *name = nullptr);
  bool remove_paired_device(uint32_t serial_number);
  bool is_device_paired(uint32_t serial_number);
  void clear_all_paired_devices();
  std::vector<uint32_t> get_paired_devices();

  // Debug/diagnostics
  int8_t read_temperature();
  void dump_registers();
  uint8_t get_version() { return version_; }

 protected:
  // Pin configurations
  GPIOPin *cs_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  InternalGPIOPin *interrupt_pin_{nullptr};

  // Configuration
  float frequency_{433.0f};
  uint8_t node_id_{1};
  uint8_t network_id_{100};
  std::string encryption_key_;
  bool is_high_power_{false};
  int8_t tx_power_{13};
  uint32_t challenge_timeout_ms_{RFM69_CHALLENGE_TIMEOUT_MS};

  // Device info
  uint8_t version_{0};

  // State
  uint8_t mode_{RFM69_MODE_STANDBY};
  uint8_t idle_mode_{RFM69_MODE_STANDBY};
  bool rx_buf_valid_{false};
  uint8_t rx_buf_[RFM69_MAX_MESSAGE_LEN];
  uint8_t rx_buf_len_{0};
  int16_t last_rssi_{0};
  bool aes_enabled_{false};

  // RadioHead headers (received)
  uint8_t rx_header_to_{0};
  uint8_t rx_header_from_{0};
  uint8_t rx_header_id_{0};
  uint8_t rx_header_flags_{0};

  // RadioHead headers (transmit)
  uint8_t tx_header_to_{RFM69_BROADCAST_ADDR};
  uint8_t tx_header_from_{RFM69_BROADCAST_ADDR};
  uint8_t tx_header_id_{0};
  uint8_t tx_header_flags_{0};

  // Security - Paired devices
  std::vector<PairedDevice> paired_devices_;
  bool pairing_mode_enabled_{false};
  ESPPreferenceObject pref_;

  // Callbacks
  CallbackManager<void(uint8_t, uint8_t, std::vector<uint8_t>)> packet_received_callbacks_;
  CallbackManager<void(uint32_t)> device_paired_callbacks_;
  CallbackManager<void(uint32_t, bool)> door_open_request_callbacks_;

  // Internal methods
  bool write_register_(uint8_t reg, uint8_t value);
  uint8_t read_register_(uint8_t reg);
  void burst_read_(uint8_t reg, uint8_t *dest, uint8_t len);
  void burst_write_(uint8_t reg, const uint8_t *src, uint8_t len);
  
  bool set_mode_(uint8_t mode);
  void set_mode_idle_();
  void set_mode_rx_();
  void set_mode_tx_();
  
  bool configure_frequency_();
  void configure_modem_();
  void configure_power_();
  void configure_encryption_();
  
  void read_fifo_();
  void handle_interrupt_();
  
  // Protocol handling
  void process_command_(uint8_t from_address, const uint8_t *data, uint8_t len);
  void handle_ping_(uint8_t from_address, uint32_t remote_sn);
  void handle_door_open_(uint8_t from_address, uint32_t remote_sn, uint32_t challenge);
  void handle_pair_request_(uint8_t from_address, uint32_t remote_sn);
  void send_challenge_(uint8_t to_address, uint32_t challenge);
  void send_ack_(uint8_t to_address, RFM69AckStatus status);
  void send_pair_confirm_(uint8_t to_address, RFM69AckStatus status);
  
  // Challenge management
  uint32_t generate_challenge_();
  bool validate_challenge_(uint32_t remote_sn, uint32_t challenge);
  void cleanup_expired_challenges_();
  
  // Persistence
  void save_paired_devices_();
  void load_paired_devices_();
};

// ============================================================================
// Automation Triggers
// ============================================================================

class PacketReceivedTrigger : public Trigger<uint8_t, uint8_t, std::vector<uint8_t>> {
 public:
  explicit PacketReceivedTrigger(RFM69Component *parent) {
    parent->add_on_packet_received_callback(
        [this](uint8_t from, uint8_t rssi, std::vector<uint8_t> data) {
          this->trigger(from, rssi, data);
        });
  }
};

class DevicePairedTrigger : public Trigger<uint32_t> {
 public:
  explicit DevicePairedTrigger(RFM69Component *parent) {
    parent->add_on_device_paired_callback(
        [this](uint32_t serial_number) {
          this->trigger(serial_number);
        });
  }
};

class DoorOpenRequestTrigger : public Trigger<uint32_t, bool> {
 public:
  explicit DoorOpenRequestTrigger(RFM69Component *parent) {
    parent->add_on_door_open_request_callback(
        [this](uint32_t serial_number, bool authorized) {
          this->trigger(serial_number, authorized);
        });
  }
};

}  // namespace rfm69
}  // namespace esphome
