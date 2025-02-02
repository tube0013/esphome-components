#include "cc2652_flasher.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"  // Provides delay() and millis()
#include <cstring>

namespace esphome {
namespace cc2652_flasher {

static const char *TAG = "cc2652_flasher";

uint16_t CC2652FlasherComponent::compute_crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

int CC2652FlasherComponent::read_byte(uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (this->available())
      return this->read();
    delay(1);
  }
  return -1;
}

bool CC2652FlasherComponent::enter_bootloader_mode() {
  if (!bsl_output_ || !reset_output_) {
    ESP_LOGE(TAG, "BSL and/or reset output not set");
    return false;
  }
  ESP_LOGI(TAG, "Entering bootloader mode: driving BSL low and pulsing reset");
  // Activate bootloader mode.
  bsl_output_->turn_off();
  reset_output_->turn_off();
  delay(100);
  reset_output_->turn_on();
  delay(500);
  return true;
}

bool CC2652FlasherComponent::sync_bootloader() {
  ESP_LOGI(TAG, "Syncing with bootloader...");
  uint8_t sync_byte = CMD_SYNC;
  for (int attempt = 0; attempt < 10; attempt++) {
    ESP_LOGD(TAG, "Sync attempt %d", attempt + 1);
    this->write_array(&sync_byte, 1);
    int response = read_byte(100);
    if (response == ACK) {
      ESP_LOGI(TAG, "Bootloader sync succeeded");
      return true;
    }
    delay(50);
  }
  ESP_LOGE(TAG, "Failed to sync with bootloader");
  return false;
}

bool CC2652FlasherComponent::erase_flash() {
  uint8_t cmd = CMD_ERASE;
  ESP_LOGI(TAG, "Sending flash erase command");
  this->write_array(&cmd, 1);
  int response = read_byte(1000);
  if (response == ACK) {
    ESP_LOGI(TAG, "Flash erase acknowledged");
    return true;
  }
  ESP_LOGE(TAG, "Flash erase failed, response: 0x%02X", response);
  return false;
}

bool CC2652FlasherComponent::write_flash_block(uint32_t address, const uint8_t* data, size_t length) {
  size_t packet_len = 1 + 4 + 1 + length;  // Without CRC bytes.
  uint8_t packet[packet_len + 2];
  size_t pos = 0;
  packet[pos++] = CMD_WRITE;
  packet[pos++] = (address >> 24) & 0xFF;
  packet[pos++] = (address >> 16) & 0xFF;
  packet[pos++] = (address >> 8) & 0xFF;
  packet[pos++] = address & 0xFF;
  packet[pos++] = static_cast<uint8_t>(length);
  memcpy(&packet[pos], data, length);
  pos += length;
  uint16_t crc = compute_crc16(packet, pos);
  packet[pos++] = (crc >> 8) & 0xFF;
  packet[pos++] = crc & 0xFF;

  ESP_LOGI(TAG, "Writing block at 0x%08X, length %d", address, length);
  this->write_array(packet, pos);
  int response = read_byte(500);
  if (response == ACK) {
    ESP_LOGD(TAG, "Block written successfully");
    return true;
  }
  ESP_LOGE(TAG, "Block write failed at 0x%08X, response: 0x%02X", address, response);
  return false;
}

bool CC2652FlasherComponent::exit_bootloader() {
  uint8_t cmd = CMD_EXIT;
  ESP_LOGI(TAG, "Sending bootloader exit command");
  this->write_array(&cmd, 1);
  int response = read_byte(500);
  if (response == ACK) {
    ESP_LOGI(TAG, "Bootloader exit acknowledged");
    return true;
  }
  ESP_LOGE(TAG, "Bootloader exit failed, response: 0x%02X", response);
  return false;
}

void CC2652FlasherComponent::flash_firmware() {
  ESP_LOGI(TAG, "Starting firmware flash process");

  if (!enter_bootloader_mode()) {
    ESP_LOGE(TAG, "Unable to enter bootloader mode");
    return;
  }

  uint32_t original_baud_rate = this->get_baud_rate();
  ESP_LOGI(TAG, "Changing UART baud rate from %u to %u for flashing", original_baud_rate, flashing_baud_rate_);
  this->set_baud_rate(flashing_baud_rate_);
  delay(50);

  if (!sync_bootloader()) {
    ESP_LOGE(TAG, "Bootloader sync failed");
    return;
  }

  if (!erase_flash()) {
    ESP_LOGE(TAG, "Flash erase failed");
    return;
  }

  // Firmware download and flashing logic would be here...
  // (For brevity, that part of the code is omitted.)

  exit_bootloader();

  ESP_LOGI(TAG, "Reverting UART baud rate back to %u", original_baud_rate);
  this->set_baud_rate(original_baud_rate);
  delay(50);

  ESP_LOGI(TAG, "Firmware flashing completed");
}

void CC2652FlasherComponent::setup() {
  ESP_LOGI(TAG, "Setting up CC2652 Flasher Component");
  // For demonstration, flash_firmware() is called during setup.
  // In production, consider registering a service to trigger flashing.
  flash_firmware();
}

void CC2652FlasherComponent::loop() {
  // No recurring tasks in this example.
}

}  // namespace cc2652_flasher
}  // namespace esphome
