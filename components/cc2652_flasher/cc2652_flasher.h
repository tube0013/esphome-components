#pragma once
#include "esphome.h"
#include "esphome/components/switch/switch.h"  // Include the switch component

namespace esphome {
namespace cc2652_flasher {

/** 
 * CC2652FlasherComponent
 *
 * This component flashes a TI CC2652 radio using a bootloader protocol inspired
 * by cc2538-bsl.py. It downloads a manifest JSON that contains the firmware metadata,
 * then downloads the firmware and flashes it block by block (248-byte chunks).
 *
 * It toggles two external outputs (provided as switch::Switch objects) to control:
 *   - The bootloader mode (BSL) signal.
 *   - The reset signal.
 *
 * It temporarily changes the UART baud rate to perform flashing and then reverts it.
 *
 * IMPORTANT: Verify the command codes, ACK values, and packet structure against your
 * CC2652 bootloader documentation.
 */
class CC2652FlasherComponent : public Component, public UARTDevice {
 public:
  /// Set the manifest URL from YAML.
  void set_manifest_url(const std::string &manifest_url) { manifest_url_ = manifest_url; }

  /// Set the flashing baud rate (e.g., 500000).
  void set_flashing_baud_rate(uint32_t baud) { flashing_baud_rate_ = baud; }

  /// Set the BSL (bootloader) output (provided as a switch::Switch).
  void set_bsl_output(switch::Switch *bsl_output) { bsl_output_ = bsl_output; }

  /// Set the reset output (provided as a switch::Switch).
  void set_reset_output(switch::Switch *reset_output) { reset_output_ = reset_output; }

  /// Trigger the flash process. (In production, you might register a service.)
  void flash_firmware();

  // Standard ESPHome component interface.
  void setup() override;
  void loop() override;

 protected:
  std::string manifest_url_{};

  /// Flash block size.
  static const size_t CHUNK_SIZE = 248;

  /// External outputs for bootloader mode and reset control.
  switch::Switch *bsl_output_{nullptr};
  switch::Switch *reset_output_{nullptr};

  // Bootloader protocol command definitions.
  static constexpr uint8_t CMD_SYNC  = 0x55;
  static constexpr uint8_t CMD_ERASE = 0xAA;
  static constexpr uint8_t CMD_WRITE = 0xA0;
  static constexpr uint8_t CMD_EXIT  = 0xA5;
  static constexpr uint8_t ACK       = 0xCC;  // Expected ACK byte

  /// The baud rate to use during flashing.
  uint32_t flashing_baud_rate_{500000};

  /// Helper: Read one byte from UART with a timeout (in ms). Returns -1 if timed out.
  int read_byte(uint32_t timeout_ms);

  /// Toggle the outputs to force the CC2652 into bootloader mode.
  bool enter_bootloader_mode();

  /// Send a sync command and wait for an ACK.
  bool sync_bootloader();

  /// Send an erase flash command and wait for an ACK.
  bool erase_flash();

  /// Send a write command for a flash block (with address, length, data, and CRC16).
  bool write_flash_block(uint32_t address, const uint8_t* data, size_t length);

  /// Send an exit bootloader command and wait for an ACK.
  bool exit_bootloader();

  /// Compute a CRC16 (polynomial 0x1021, initial value 0xFFFF) for the data.
  uint16_t compute_crc16(const uint8_t *data, size_t length);
};

}  // namespace cc2652_flasher
}  // namespace esphome
