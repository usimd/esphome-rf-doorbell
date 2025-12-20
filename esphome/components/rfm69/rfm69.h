#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace rfm69 {

// RFM69 Register Map - from datasheet
static const uint8_t RFM69_REG_OPMODE = 0x01;
static const uint8_t RFM69_REG_DATA_MODUL = 0x02;
static const uint8_t RFM69_REG_FRF_MSB = 0x07;
static const uint8_t RFM69_REG_FRF_MID = 0x08;
static const uint8_t RFM69_REG_FRF_LSB = 0x09;
static const uint8_t RFM69_REG_VERSION = 0x10;
static const uint8_t RFM69_REG_PALEVEL = 0x11;
static const uint8_t RFM69_REG_IRQ_FLAGS1 = 0x27;
static const uint8_t RFM69_REG_IRQ_FLAGS2 = 0x28;
static const uint8_t RFM69_REG_SYNC_CONFIG = 0x2E;
static const uint8_t RFM69_REG_SYNC_VALUE1 = 0x2F;
static const uint8_t RFM69_REG_NODE_ADDR = 0x39;
static const uint8_t RFM69_REG_BROADCAST_ADDR = 0x3A;
static const uint8_t RFM69_REG_FIFO = 0x00;

// Operating modes
static const uint8_t RFM69_MODE_SLEEP = 0x00;
static const uint8_t RFM69_MODE_STANDBY = 0x04;
static const uint8_t RFM69_MODE_RX = 0x10;
static const uint8_t RFM69_MODE_TX = 0x0C;

// IRQ flags
static const uint8_t RFM69_IRQ1_MODE_READY = 0x80;
static const uint8_t RFM69_IRQ2_FIFO_NOT_EMPTY = 0x40;
static const uint8_t RFM69_IRQ2_PAYLOAD_READY = 0x04;
static const uint8_t RFM69_IRQ2_PACKET_SENT = 0x08;

// Frequency calculation: Fstep = FXOSC / 2^19 = 32MHz / 524288 = 61.03515625 Hz
static const float RFM69_FSTEP = 61.03515625f;

class PacketReceivedTrigger;

class RFM69Component : public Component,
                       public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                              spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_cs_pin(GPIOPin *cs) { cs_pin_ = cs; }
  void set_reset_pin(GPIOPin *reset) { reset_pin_ = reset; }
  void set_interrupt_pin(InternalGPIOPin *interrupt) { interrupt_pin_ = interrupt; }
  void set_frequency(float frequency) { frequency_ = frequency; }
  void set_node_id(uint8_t node_id) { node_id_ = node_id; }
  void set_network_id(uint8_t network_id) { network_id_ = network_id; }
  void set_encryption_key(const std::string &key) { encryption_key_ = key; }
  void set_is_high_power(bool is_high_power) { is_high_power_ = is_high_power; }

  void add_on_packet_received_callback(std::function<void()> &&callback) {
    packet_received_callbacks_.add(std::move(callback));
  }

  bool is_valid_remote() { return packet_received_; }

 protected:
  GPIOPin *cs_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  InternalGPIOPin *interrupt_pin_{nullptr};
  float frequency_;
  uint8_t node_id_;
  uint8_t network_id_;
  std::string encryption_key_;
  bool is_high_power_;
  bool packet_received_{false};

  CallbackManager<void()> packet_received_callbacks_;

  bool write_register_(uint8_t reg, uint8_t value);
  uint8_t read_register_(uint8_t reg);
  bool set_mode_(uint8_t mode);
  bool set_frequency_();
  void handle_interrupt_();
};

class PacketReceivedTrigger : public Trigger<> {
 public:
  explicit PacketReceivedTrigger(RFM69Component *parent) {
    parent->add_on_packet_received_callback([this]() { this->trigger(); });
  }
};

}  // namespace rfm69
}  // namespace esphome
