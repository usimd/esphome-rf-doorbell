#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/automation.h"
#include "esphome/components/spi/spi.h"
#include <RFM69.h>  // LowPowerLab RFM69 library
#include <vector>
#include <map>
#include <deque>

namespace esphome {
namespace rfm69 {

// Packet types for challenge-response protocol
static const uint8_t PKT_TYPE_CHALLENGE_REQUEST = 0x01;
static const uint8_t PKT_TYPE_CHALLENGE_RESPONSE = 0x02;
static const uint8_t PKT_TYPE_AUTHENTICATED = 0x03;
static const uint8_t PKT_TYPE_PAIRING_REQUEST = 0x04;

// Maximum nonces to track for replay protection
static const size_t MAX_NONCE_HISTORY = 100;

struct Challenge {
  uint32_t nonce;
  uint32_t timestamp;
  uint8_t sender_id;
  bool used;
};

struct SecurePacket {
  uint8_t type;
  uint32_t nonce;
  uint32_t rolling_code;  // Incrementing counter per device
  uint8_t sender_id;
  uint8_t payload_length;
  uint8_t payload[56];  // Reduced from 61 to accommodate security header
} __attribute__((packed));

class RFM69Component : public Component,
                       public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                             spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  RFM69Component() : radio_(nullptr) {}
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_cs_pin(GPIOPin *pin) { cs_pin_ = pin; }
  void set_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }
  void set_interrupt_pin(GPIOPin *pin) { interrupt_pin_ = pin; }
  void set_frequency(float freq_mhz) { frequency_mhz_ = freq_mhz; }
  void set_node_id(uint8_t id) { node_id_ = id; }
  void set_network_id(uint8_t id) { network_id_ = id; }
  void set_encryption_key(const std::string &key) { encryption_key_ = key; }
  void set_high_power(bool hp) { is_high_power_ = hp; }
  void set_use_challenge_response(bool use) { use_challenge_response_ = use; }
  void set_challenge_timeout(uint32_t timeout) { challenge_timeout_ = timeout; }

  // Public API methods
  bool send_packet(uint8_t target_id, const uint8_t *data, uint8_t length);
  bool is_valid_remote();
  void add_paired_device(uint8_t device_id, uint32_t rolling_code = 0);
  void remove_paired_device(uint8_t device_id);
  void enter_pairing_mode();
  void exit_pairing_mode();

  // Trigger for automation
  void add_on_packet_received_callback(std::function<void()> callback) {
    packet_received_callbacks_.push_back(callback);
  }

 protected:
  GPIOPin *cs_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  GPIOPin *interrupt_pin_{nullptr};

  RFM69 *radio_{nullptr};  // LowPowerLab RFM69 instance
  
  float frequency_mhz_;
  uint8_t node_id_;
  uint8_t network_id_;
  std::string encryption_key_;
  bool is_high_power_;
  bool use_challenge_response_{true};
  uint32_t challenge_timeout_{30};  // seconds

  // Paired devices with rolling codes
  struct PairedDevice {
    uint8_t device_id;
    uint32_t last_rolling_code;
    uint32_t rolling_code_window;  // Allow codes within this window
  };
  std::vector<PairedDevice> paired_devices_;
  std::vector<std::function<void()>> packet_received_callbacks_;
  bool pairing_mode_{false};

  // Challenge-response state
  std::map<uint8_t, Challenge> active_challenges_;
  std::deque<uint32_t> used_nonces_;
  uint32_t next_nonce_{1};

  // Last received packet info
  uint8_t last_sender_id_{0};
  int16_t last_rssi_{0};
  
  // Helper methods
  bool initialize_radio_();
  void process_packet_();
  bool handle_challenge_request_(uint8_t sender_id, uint32_t nonce);
  bool handle_challenge_response_(uint8_t sender_id, uint32_t nonce, uint32_t rolling_code);
  bool handle_authenticated_packet_(uint8_t sender_id, uint32_t rolling_code, const uint8_t *payload, uint8_t length);
  bool verify_rolling_code_(uint8_t sender_id, uint32_t rolling_code);
  void update_rolling_code_(uint8_t sender_id, uint32_t rolling_code);
  uint32_t generate_nonce_();
  bool is_nonce_used_(uint32_t nonce);
  void mark_nonce_used_(uint32_t nonce);
  void cleanup_expired_challenges_();
  PairedDevice *find_paired_device_(uint8_t device_id);
};

class PacketReceivedTrigger : public Trigger<> {
 public:
  explicit PacketReceivedTrigger(RFM69Component *parent) {
    parent->add_on_packet_received_callback([this]() { this->trigger(); });
  }
};

}  // namespace rfm69
}  // namespace esphome
static const uint8_t RFM69_REG_OPMODE = 0x01;
static const uint8_t RFM69_REG_DATAMODUL = 0x02;
static const uint8_t RFM69_REG_BITRATEMSB = 0x03;
static const uint8_t RFM69_REG_BITRATELSB = 0x04;
static const uint8_t RFM69_REG_FDEVMSB = 0x05;
static const uint8_t RFM69_REG_FDEVLSB = 0x06;
static const uint8_t RFM69_REG_FRFMSB = 0x07;
static const uint8_t RFM69_REG_FRFMID = 0x08;
static const uint8_t RFM69_REG_FRFLSB = 0x09;
static const uint8_t RFM69_REG_OSC1 = 0x0A;
static const uint8_t RFM69_REG_AFCCTRL = 0x0B;
static const uint8_t RFM69_REG_LISTEN1 = 0x0D;
static const uint8_t RFM69_REG_LISTEN2 = 0x0E;
static const uint8_t RFM69_REG_LISTEN3 = 0x0F;
static const uint8_t RFM69_REG_VERSION = 0x10;
static const uint8_t RFM69_REG_PALEVEL = 0x11;
static const uint8_t RFM69_REG_PARAMP = 0x12;
static const uint8_t RFM69_REG_OCP = 0x13;
static const uint8_t RFM69_REG_LNA = 0x18;
static const uint8_t RFM69_REG_RXBW = 0x19;
static const uint8_t RFM69_REG_AFCBW = 0x1A;
static const uint8_t RFM69_REG_OOKPEAK = 0x1B;
static const uint8_t RFM69_REG_OOKAVG = 0x1C;
static const uint8_t RFM69_REG_OOKFIX = 0x1D;
static const uint8_t RFM69_REG_AFCFEI = 0x1E;
static const uint8_t RFM69_REG_AFCMSB = 0x1F;
static const uint8_t RFM69_REG_AFCLSB = 0x20;
static const uint8_t RFM69_REG_FEIMSB = 0x21;
static const uint8_t RFM69_REG_FEILSB = 0x22;
static const uint8_t RFM69_REG_RSSICONFIG = 0x23;
static const uint8_t RFM69_REG_RSSIVALUE = 0x24;
static const uint8_t RFM69_REG_DIOMAPPING1 = 0x25;
static const uint8_t RFM69_REG_DIOMAPPING2 = 0x26;
static const uint8_t RFM69_REG_IRQFLAGS1 = 0x27;
static const uint8_t RFM69_REG_IRQFLAGS2 = 0x28;
static const uint8_t RFM69_REG_RSSITHRESH = 0x29;
static const uint8_t RFM69_REG_RXTIMEOUT1 = 0x2A;
static const uint8_t RFM69_REG_RXTIMEOUT2 = 0x2B;
static const uint8_t RFM69_REG_PREAMBLEMSB = 0x2C;
static const uint8_t RFM69_REG_PREAMBLELSB = 0x2D;
static const uint8_t RFM69_REG_SYNCCONFIG = 0x2E;
static const uint8_t RFM69_REG_SYNCVALUE1 = 0x2F;
static const uint8_t RFM69_REG_PACKETCONFIG1 = 0x37;
static const uint8_t RFM69_REG_PAYLOADLENGTH = 0x38;
static const uint8_t RFM69_REG_NODEADRS = 0x39;
static const uint8_t RFM69_REG_BROADCASTADRS = 0x3A;
static const uint8_t RFM69_REG_AUTOMODES = 0x3B;
static const uint8_t RFM69_REG_FIFOTHRESH = 0x3C;
static const uint8_t RFM69_REG_PACKETCONFIG2 = 0x3D;
static const uint8_t RFM69_REG_AESKEY1 = 0x3E;
static const uint8_t RFM69_REG_TEMP1 = 0x4E;
static const uint8_t RFM69_REG_TEMP2 = 0x4F;
static const uint8_t RFM69_REG_TESTLNA = 0x58;
static const uint8_t RFM69_REG_TESTPA1 = 0x5A;
static const uint8_t RFM69_REG_TESTPA2 = 0x5C;
static const uint8_t RFM69_REG_TESTDAGC = 0x6F;
static const uint8_t RFM69_REG_TESTAFC = 0x71;

// Operating modes
static const uint8_t RFM69_MODE_SLEEP = 0x00;
static const uint8_t RFM69_MODE_STANDBY = 0x04;
static const uint8_t RFM69_MODE_FS = 0x08;
static const uint8_t RFM69_MODE_TX = 0x0C;
static const uint8_t RFM69_MODE_RX = 0x10;

// IRQ Flags
static const uint8_t RFM69_IRQ1_MODEREADY = 0x80;
static const uint8_t RFM69_IRQ2_FIFONOTEMPTY = 0x40;
static const uint8_t RFM69_IRQ2_PACKETSENT = 0x08;
static const uint8_t RFM69_IRQ2_PAYLOADREADY = 0x04;

struct RFM69Packet {
  uint8_t sender_id;
  uint8_t target_id;
  uint8_t payload_length;
  uint8_t payload[61];  // Max payload size
  int16_t rssi;
};

class RFM69Component : public Component,
                       public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                             spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_cs_pin(GPIOPin *pin) { cs_pin_ = pin; }
  void set_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }
  void set_interrupt_pin(GPIOPin *pin) { interrupt_pin_ = pin; }
  void set_frequency(float freq_mhz) { frequency_mhz_ = freq_mhz; }
  void set_node_id(uint8_t id) { node_id_ = id; }
  void set_network_id(uint8_t id) { network_id_ = id; }
  void set_encryption_key(const std::string &key) { encryption_key_ = key; }
  void set_high_power(bool hp) { is_high_power_ = hp; }

  // Public API methods
  bool send_packet(uint8_t target_id, const uint8_t *data, uint8_t length);
  bool is_valid_remote();
  void add_paired_device(uint8_t device_id);
  void remove_paired_device(uint8_t device_id);
  void enter_pairing_mode();
  void exit_pairing_mode();

  // Trigger for automation
  void add_on_packet_received_callback(std::function<void()> callback) {
    packet_received_callbacks_.push_back(callback);
  }

 protected:
  GPIOPin *cs_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  GPIOPin *interrupt_pin_{nullptr};

  float frequency_mhz_;
  uint8_t node_id_;
  uint8_t network_id_;
  std::string encryption_key_;
  bool is_high_power_;

  std::vector<uint8_t> paired_devices_;
  std::vector<std::function<void()>> packet_received_callbacks_;
  bool pairing_mode_{false};
  RFM69Packet last_packet_;

  bool reset_radio_();
  bool configure_radio_();
  void set_mode_(uint8_t mode);
  uint8_t read_register_(uint8_t reg);
  void write_register_(uint8_t reg, uint8_t value);
  void read_fifo_(uint8_t *buffer, uint8_t length);
  void write_fifo_(const uint8_t *buffer, uint8_t length);
  bool receive_packet_();
  int16_t read_rssi_();
  void set_encryption_(bool enable);
};

class PacketReceivedTrigger : public Trigger<> {
 public:
  explicit PacketReceivedTrigger(RFM69Component *parent) {
    parent->add_on_packet_received_callback([this]() { this->trigger(); });
  }
};

}  // namespace rfm69
}  // namespace esphome
