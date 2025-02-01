#include "cc2652_flasher.h"
#include "esphome/core/log.h"
#include "esphome/core/delay.h"
#include <cstring>

namespace esphome {
namespace cc2652_flasher {

static const char *TAG = "cc2652_flasher";

///////////////////////
// Helper Functions  //
///////////////////////

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

////////////////////////////
// Bootloader Procedures  //
////////////////////////////

bool CC2652FlasherComponent::enter_bootloader_mode() {
  if (!bsl_output_ || !reset_output_) {
    ESP_LOGE(TAG, "BSL and/or reset output not set");
    return false;
  }
  ESP_LOGI(TAG, "Entering bootloader mode: driving BSL low and pulsing reset");
  // To enter bootloader mode, drive the BSL pin LOW.
  bsl_output_->turn_off();
  // Pulse reset: drive reset LOW, then HIGH.
  reset_output_->turn_off();
  delay(100);
  reset_output_->turn_on();
  // Allow time for the bootloader to initialize.
  delay(500);
  return true;
}

bool CC2652FlasherComponent::sync_bootloader() {
  ESP_LOGI(TAG, "Syncing with bootloader...");
  uint8_t sync_byte = CMD_SYNC;
  // Try up to 10 attempts
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
  // Packet structure:
  // [CMD_WRITE][address (4 bytes, big-endian)][length (1 byte)][data ...][CRC16 (2 bytes, big-endian)]
  size_t packet_len = 1 + 4 + 1 + length;  // without CRC bytes
  uint8_t packet[packet_len + 2];           // allocate space for CRC

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

////////////////////////////
// Main Flash Procedure   //
////////////////////////////

void CC2652FlasherComponent::flash_firmware() {
  ESP_LOGI(TAG, "Starting firmware flash process");

  // --- Step 0: Enter bootloader mode ---
  if (!enter_bootloader_mode()) {
    ESP_LOGE(TAG, "Unable to enter bootloader mode");
    return;
  }

  // --- Step 1: Change baud rate for flashing ---
  // Store the current baud rate; if get_baud_rate() is not available, use 115200 as default.
  uint32_t original_baud_rate = this->get_baud_rate();
  ESP_LOGI(TAG, "Changing UART baud rate from %u to %u for flashing", original_baud_rate, flashing_baud_rate_);
  this->set_baud_rate(flashing_baud_rate_);
  delay(50);  // Allow time for the change to take effect

  // --- Step 2: Sync with bootloader ---
  if (!sync_bootloader()) {
    ESP_LOGE(TAG, "Bootloader sync failed");
    return;
  }
  
  // --- Step 3: Erase flash ---
  if (!erase_flash()) {
    ESP_LOGE(TAG, "Flash erase failed");
    return;
  }

  // --- Step 4: Download manifest and firmware ---
  HTTPClient http;
  ESP_LOGI(TAG, "Downloading manifest from %s", manifest_url_.c_str());
  http.begin(manifest_url_.c_str());
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    ESP_LOGE(TAG, "Manifest download failed, HTTP code: %d", httpCode);
    http.end();
    return;
  }
  String manifest_payload = http.getString();
  http.end();
  ESP_LOGD(TAG, "Manifest: %s", manifest_payload.c_str());

  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, manifest_payload);
  if (error) {
    ESP_LOGE(TAG, "Manifest JSON parse failed: %s", error.c_str());
    return;
  }
  const char* version = doc["version"];
  const char* firmware_url = doc["firmware_url"];
  size_t firmware_size = doc["size"];
  ESP_LOGI(TAG, "Manifest details - version: %s, firmware_url: %s, size: %d", version, firmware_url, firmware_size);

  HTTPClient firmware_http;
  ESP_LOGI(TAG, "Downloading firmware from %s", firmware_url);
  firmware_http.begin(firmware_url);
  int firmware_httpCode = firmware_http.GET();
  if (firmware_httpCode != HTTP_CODE_OK) {
    ESP_LOGE(TAG, "Firmware download failed, HTTP code: %d", firmware_httpCode);
    firmware_http.end();
    return;
  }
  WiFiClient *firmwareStream = firmware_http.getStreamPtr();

  // --- Step 5: Flash the firmware block by block ---
  uint8_t buffer[CHUNK_SIZE];
  size_t bytes_remaining = firmware_size;
  size_t total_bytes_flashed = 0;
  uint32_t flash_address = 0x00000000;  // starting flash address (adjust if needed)

  while (bytes_remaining > 0) {
    size_t current_chunk_size = CHUNK_SIZE;
    if (bytes_remaining < CHUNK_SIZE)
      current_chunk_size = bytes_remaining;

    size_t bytes_read = 0;
    while (bytes_read < current_chunk_size) {
      if (firmwareStream->available()) {
        int c = firmwareStream->read();
        if (c < 0)
          break;
        buffer[bytes_read++] = static_cast<uint8_t>(c);
      } else {
        delay(10);
      }
    }
    if (bytes_read != current_chunk_size) {
      ESP_LOGE(TAG, "Incomplete chunk read: expected %d bytes, got %d", current_chunk_size, bytes_read);
      break;
    }

    if (!write_flash_block(flash_address, buffer, current_chunk_size)) {
      ESP_LOGE(TAG, "Failed to write flash block at address 0x%08X", flash_address);
      break;
    }

    flash_address += current_chunk_size;
    total_bytes_flashed += current_chunk_size;
    bytes_remaining -= current_chunk_size;
    ESP_LOGI(TAG, "Flashed %d / %d bytes", total_bytes_flashed, firmware_size);
  }
  firmware_http.end();

  // --- Step 6: Exit bootloader mode ---
  exit_bootloader();

  // --- Step 7: Revert UART baud rate to original (115200 or stored value) ---
  ESP_LOGI(TAG, "Reverting UART baud rate back to %u", original_baud_rate);
  this->set_baud_rate(original_baud_rate);
  delay(50);

  ESP_LOGI(TAG, "Firmware flashing completed");
}

//////////////////////
// ESPHome Methods  //
//////////////////////

void CC2652FlasherComponent::setup() {
  ESP_LOGI(TAG, "Setting up CC2652 Flasher Component");
  // For demonstration purposes, flash_firmware() is called immediately.
  // In production, consider registering a service to trigger flashing.
  flash_firmware();
}

void CC2652FlasherComponent::loop() {
  // No recurring tasks in this example.
}

}  // namespace cc2652_flasher
}  // namespace esphome
