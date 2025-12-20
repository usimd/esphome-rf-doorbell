#include "rfm69.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace rfm69 {

static const char *const TAG = "rfm69";

void RFM69Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RFM69...");

  // Initialize CS pin
  if (this->cs_pin_ != nullptr) {
    this->cs_pin_->setup();
    this->cs_pin_->digital_write(true);
  }

  // Initialize reset pin if provided
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(10);
  }

  // Initialize interrupt pin if provided
  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();
  }

  // Initialize the radio
  if (!this->initialize_radio_()) {
    ESP_LOGE(TAG, "Failed to initialize RFM69 radio");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "RFM69 setup complete");
}

bool RFM69Component::initialize_radio_() {
  // Create RFM69 instance (LowPowerLab library)
  this->radio_ = new RFM69();
  
  // Initialize with node ID, network ID, and high power flag
  bool success = this->radio_->initialize(
      this->is_high_power_ ? RF69_433MHZ : RF69_433MHZ,  // Frequency band
      this->node_id_,
      this->network_id_
  );
  
  if (!success) {
    ESP_LOGE(TAG, "RFM69 initialization failed");
    return false;
  }

  // Set exact frequency if different from default
  this->radio_->setFrequency(this->frequency_mhz_ * 1000000);
  
  // Configure high power mode if needed
  if (this->is_high_power_) {
    this->radio_->setHighPower(true);
  }

  // Set encryption key if provided
  if (!this->encryption_key_.empty() && this->encryption_key_.length() == 16) {
    this->radio_->encrypt(this->encryption_key_.c_str());
    ESP_LOGD(TAG, "AES encryption enabled");
  } else if (!this->encryption_key_.empty()) {
    ESP_LOGW(TAG, "Encryption key must be exactly 16 characters, encryption disabled");
  }

  // Put radio in receive mode
  this->radio_->receiveDone();  // Clear any pending packets
  
  ESP_LOGI(TAG, "RFM69 initialized: NodeID=%d, NetworkID=%d, Freq=%.2fMHz", 
           this->node_id_, this->network_id_, this->frequency_mhz_);
  
  return true;
}

void RFM69Component::loop() {
  if (this->radio_ == nullptr) {
    return;
  }

  // Clean up expired challenges periodically
  static uint32_t last_cleanup = 0;
  uint32_t now = millis();
  if (now - last_cleanup > 10000) {  // Every 10 seconds
    this->cleanup_expired_challenges_();
    last_cleanup = now;
  }

  // Check for received packets
  if (this->radio_->receiveDone()) {
    this->process_packet_();
  }
}

void RFM69Component::process_packet_() {
  if (this->radio_->DATALEN == 0) {
    return;
  }

  uint8_t sender_id = this->radio_->SENDERID;
  int16_t rssi = this->radio_->RSSI;
  
  ESP_LOGD(TAG, "Received packet from %d, RSSI=%d, Length=%d", sender_id, rssi, this->radio_->DATALEN);

  // Store for later access
  this->last_sender_id_ = sender_id;
  this->last_rssi_ = rssi;

  // If challenge-response is disabled, accept all packets from paired devices
  if (!this->use_challenge_response_) {
    if (this->find_paired_device_(sender_id) != nullptr) {
      ESP_LOGI(TAG, "Authenticated packet from device %d (challenge-response disabled)", sender_id);
      for (auto &callback : this->packet_received_callbacks_) {
        callback();
      }
    } else {
      ESP_LOGW(TAG, "Packet from unpaired device %d", sender_id);
    }
    return;
  }

  // Parse secure packet structure
  if (this->radio_->DATALEN < sizeof(SecurePacket)) {
    ESP_LOGW(TAG, "Packet too small for secure protocol");
    return;
  }

  SecurePacket *packet = (SecurePacket *)this->radio_->DATA;

  switch (packet->type) {
    case PKT_TYPE_CHALLENGE_REQUEST:
      this->handle_challenge_request_(sender_id, packet->nonce);
      break;
    
    case PKT_TYPE_CHALLENGE_RESPONSE:
      this->handle_challenge_response_(sender_id, packet->nonce, packet->rolling_code);
      break;
    
    case PKT_TYPE_AUTHENTICATED:
      if (this->handle_authenticated_packet_(sender_id, packet->rolling_code, 
                                             packet->payload, packet->payload_length)) {
        // Trigger automation
        for (auto &callback : this->packet_received_callbacks_) {
          callback();
        }
      }
      break;
    
    case PKT_TYPE_PAIRING_REQUEST:
      if (this->pairing_mode_) {
        ESP_LOGI(TAG, "Pairing request from device %d", sender_id);
        this->add_paired_device(sender_id, packet->rolling_code);
        
        // Send acknowledgment
        SecurePacket ack_packet;
        ack_packet.type = PKT_TYPE_AUTHENTICATED;
        ack_packet.nonce = 0;
        ack_packet.rolling_code = 0;
        ack_packet.sender_id = this->node_id_;
        ack_packet.payload_length = 0;
        
        this->radio_->send(sender_id, (const void *)&ack_packet, sizeof(ack_packet));
      } else {
        ESP_LOGW(TAG, "Pairing request from %d but not in pairing mode", sender_id);
      }
      break;
    
    default:
      ESP_LOGW(TAG, "Unknown packet type: %d", packet->type);
      break;
  }
}

bool RFM69Component::handle_challenge_request_(uint8_t sender_id, uint32_t nonce) {
  // Only respond to paired devices
  if (this->find_paired_device_(sender_id) == nullptr) {
    ESP_LOGW(TAG, "Challenge request from unpaired device %d", sender_id);
    return false;
  }

  // Check if nonce was already used (replay protection)
  if (this->is_nonce_used_(nonce)) {
    ESP_LOGW(TAG, "Replay attack detected: nonce %u already used", nonce);
    return false;
  }

  // Generate our own challenge
  uint32_t our_nonce = this->generate_nonce_();
  
  // Store challenge
  Challenge challenge;
  challenge.nonce = our_nonce;
  challenge.timestamp = millis();
  challenge.sender_id = sender_id;
  challenge.used = false;
  this->active_challenges_[sender_id] = challenge;

  // Send challenge response
  SecurePacket response;
  response.type = PKT_TYPE_CHALLENGE_RESPONSE;
  response.nonce = our_nonce;
  response.rolling_code = 0;
  response.sender_id = this->node_id_;
  response.payload_length = 0;

  this->radio_->send(sender_id, (const void *)&response, sizeof(response));
  
  ESP_LOGD(TAG, "Sent challenge response to device %d with nonce %u", sender_id, our_nonce);
  return true;
}

bool RFM69Component::handle_challenge_response_(uint8_t sender_id, uint32_t nonce, uint32_t rolling_code) {
  // Verify this challenge exists and hasn't been used
  auto it = this->active_challenges_.find(sender_id);
  if (it == this->active_challenges_.end()) {
    ESP_LOGW(TAG, "No active challenge for device %d", sender_id);
    return false;
  }

  Challenge &challenge = it->second;
  
  // Check if challenge has expired
  if (millis() - challenge.timestamp > this->challenge_timeout_ * 1000) {
    ESP_LOGW(TAG, "Challenge from device %d has expired", sender_id);
    this->active_challenges_.erase(it);
    return false;
  }

  // Verify nonce matches
  if (challenge.nonce != nonce) {
    ESP_LOGW(TAG, "Nonce mismatch for device %d", sender_id);
    return false;
  }

  // Verify rolling code
  if (!this->verify_rolling_code_(sender_id, rolling_code)) {
    ESP_LOGW(TAG, "Invalid rolling code from device %d", sender_id);
    return false;
  }

  // Mark nonce as used
  this->mark_nonce_used_(nonce);
  challenge.used = true;
  
  // Update rolling code
  this->update_rolling_code_(sender_id, rolling_code);

  ESP_LOGI(TAG, "Challenge-response authenticated for device %d", sender_id);
  return true;
}

bool RFM69Component::handle_authenticated_packet_(uint8_t sender_id, uint32_t rolling_code, 
                                                   const uint8_t *payload, uint8_t length) {
  // Verify rolling code
  if (!this->verify_rolling_code_(sender_id, rolling_code)) {
    ESP_LOGW(TAG, "Invalid rolling code in authenticated packet from device %d", sender_id);
    return false;
  }

  // Update rolling code
  this->update_rolling_code_(sender_id, rolling_code);

  ESP_LOGI(TAG, "Authenticated packet from device %d (rolling code %u)", sender_id, rolling_code);
  return true;
}

bool RFM69Component::verify_rolling_code_(uint8_t sender_id, uint32_t rolling_code) {
  PairedDevice *device = this->find_paired_device_(sender_id);
  if (device == nullptr) {
    return false;
  }

  // Accept rolling codes within a window (to handle lost packets)
  // Must be greater than last known code but within reasonable window
  if (rolling_code > device->last_rolling_code && 
      rolling_code <= device->last_rolling_code + device->rolling_code_window) {
    return true;
  }

  return false;
}

void RFM69Component::update_rolling_code_(uint8_t sender_id, uint32_t rolling_code) {
  PairedDevice *device = this->find_paired_device_(sender_id);
  if (device != nullptr) {
    device->last_rolling_code = rolling_code;
  }
}

uint32_t RFM69Component::generate_nonce_() {
  // Generate cryptographically random nonce
  // In production, use hardware RNG if available
  uint32_t nonce = esp_random();
  
  // Ensure it's not zero and hasn't been used recently
  while (nonce == 0 || this->is_nonce_used_(nonce)) {
    nonce = esp_random();
  }
  
  return nonce;
}

bool RFM69Component::is_nonce_used_(uint32_t nonce) {
  for (const auto &used : this->used_nonces_) {
    if (used == nonce) {
      return true;
    }
  }
  return false;
}

void RFM69Component::mark_nonce_used_(uint32_t nonce) {
  this->used_nonces_.push_back(nonce);
  
  // Limit history size
  while (this->used_nonces_.size() > MAX_NONCE_HISTORY) {
    this->used_nonces_.pop_front();
  }
}

void RFM69Component::cleanup_expired_challenges_() {
  uint32_t now = millis();
  auto it = this->active_challenges_.begin();
  
  while (it != this->active_challenges_.end()) {
    if (now - it->second.timestamp > this->challenge_timeout_ * 1000) {
      ESP_LOGD(TAG, "Removing expired challenge for device %d", it->first);
      it = this->active_challenges_.erase(it);
    } else {
      ++it;
    }
  }
}

RFM69Component::PairedDevice *RFM69Component::find_paired_device_(uint8_t device_id) {
  for (auto &device : this->paired_devices_) {
    if (device.device_id == device_id) {
      return &device;
    }
  }
  return nullptr;
}

void RFM69Component::add_paired_device(uint8_t device_id, uint32_t rolling_code) {
  // Check if already paired
  PairedDevice *existing = this->find_paired_device_(device_id);
  if (existing != nullptr) {
    ESP_LOGW(TAG, "Device %d already paired, updating rolling code", device_id);
    existing->last_rolling_code = rolling_code;
    return;
  }

  PairedDevice device;
  device.device_id = device_id;
  device.last_rolling_code = rolling_code;
  device.rolling_code_window = 100;  // Allow 100 codes ahead for lost packets
  
  this->paired_devices_.push_back(device);
  ESP_LOGI(TAG, "Paired device %d with rolling code %u", device_id, rolling_code);
}

void RFM69Component::remove_paired_device(uint8_t device_id) {
  auto it = std::find_if(this->paired_devices_.begin(), this->paired_devices_.end(),
                         [device_id](const PairedDevice &d) { return d.device_id == device_id; });
  
  if (it != this->paired_devices_.end()) {
    this->paired_devices_.erase(it);
    ESP_LOGI(TAG, "Removed paired device %d", device_id);
  }
}

void RFM69Component::enter_pairing_mode() {
  this->pairing_mode_ = true;
  ESP_LOGI(TAG, "Entered pairing mode");
}

void RFM69Component::exit_pairing_mode() {
  this->pairing_mode_ = false;
  ESP_LOGI(TAG, "Exited pairing mode");
}

bool RFM69Component::send_packet(uint8_t target_id, const uint8_t *data, uint8_t length) {
  if (this->radio_ == nullptr) {
    return false;
  }

  if (length > 56) {  // Max payload in SecurePacket
    ESP_LOGW(TAG, "Packet too large: %d bytes (max 56)", length);
    return false;
  }

  if (this->use_challenge_response_) {
    // Build secure packet
    SecurePacket packet;
    packet.type = PKT_TYPE_AUTHENTICATED;
    packet.nonce = this->generate_nonce_();
    
    // Get rolling code for this device
    PairedDevice *device = this->find_paired_device_(target_id);
    packet.rolling_code = (device != nullptr) ? device->last_rolling_code + 1 : 1;
    
    packet.sender_id = this->node_id_;
    packet.payload_length = length;
    memcpy(packet.payload, data, length);

    bool success = this->radio_->send(target_id, (const void *)&packet, sizeof(packet));
    
    if (success && device != nullptr) {
      device->last_rolling_code = packet.rolling_code;
    }
    
    return success;
  } else {
    // Direct send without security
    return this->radio_->send(target_id, data, length);
  }
}

bool RFM69Component::is_valid_remote() {
  // Check if last received packet was from a paired device
  return this->find_paired_device_(this->last_sender_id_) != nullptr;
}

void RFM69Component::dump_config() {
  ESP_LOGCONFIG(TAG, "RFM69:");
  ESP_LOGCONFIG(TAG, "  Node ID: %d", this->node_id_);
  ESP_LOGCONFIG(TAG, "  Network ID: %d", this->network_id_);
  ESP_LOGCONFIG(TAG, "  Frequency: %.2f MHz", this->frequency_mhz_);
  ESP_LOGCONFIG(TAG, "  High Power: %s", this->is_high_power_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Encryption: %s", this->encryption_key_.empty() ? "NO" : "YES");
  ESP_LOGCONFIG(TAG, "  Challenge-Response: %s", this->use_challenge_response_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Challenge Timeout: %u seconds", this->challenge_timeout_);
  ESP_LOGCONFIG(TAG, "  Paired Devices: %d", this->paired_devices_.size());
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Setup failed!");
  }
}

}  // namespace rfm69
}  // namespace esphome
