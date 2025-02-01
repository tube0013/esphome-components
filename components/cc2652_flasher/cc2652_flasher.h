#pragma once
#include "esphome.h"
#include "esphome/components/gpio/gpio.h"

namespace esphome {
namespace cc2652_flasher {

/** 
 * CC2652FlasherComponent
 * 
 * This component downloads a manifest JSON that specifies the firmware
 * URL/size/version, then downloads the firmware and flashes it to a CC2652
 * radio in 248‑byte chunks using a protocol inspired by cc2538‑bsl.py.
 *
 * It toggles two extra GPIO outputs:
 *   - bsl_output: forces the radio into bootloader mode.
 *   - reset_output: pulses the radio’s reset pin.
 *
 * The flashing sequence is as follows:
 *   1. Enter bootloader mode (toggle BSL and reset pins).
 *   2. Change the UART baud rate to the flashing baud rate (e.g. 500000).
 *   3. Send a sync command (0x55) until an ACK (0xCC) is received.
 *   4. Send an erase‑flash command (0xAA) and wait for an ACK.
 *   5. For each firmware block:
 *        • Send a write command (0xA0) including address, length, data, and CRC16.
 *        • Wait for an ACK.
 *   6. Send an exit command (0xA5) and wait for an ACK.
 *   7. Revert the UART baud rate back to 115200 (or the original baud rate).
 *
 * IMPORTANT: Verify the command codes, ACK values, and packet structure against
 * your CC2652 bootloader documentation!
 */
class CC2652FlasherComponent : public Component, public UARTDevice {
 public:
  // Set the manifest URL from the YAML configuration.
  void set_manifest_url(const std::string &manifest_url) { manifest_url_ = manifest_url; }

  // Set the flashing baud rate (e.g., 500000).
  void set_flashing_baud_rate(uint32_t baud) { flashing_baud_rate_ = baud; }

  // Set the GPIO outputs.
  void set_bsl_output(gpio::GPIOBinaryOutput *bsl_output) { bsl_output_ = bsl_output; }
  void set_reset_output(gpio::GPIOBinaryOutput *reset_output) { reset_output_ = reset_output; }

  /// Trigger the flash process. (For production, you might want to register a service.)
  void flash_firmware();

  // Standard ESPHome component interface.
  void setup() override;
  void loop() override;

 protected:
  std::string manifest_url_{};

  /// The size of a firmware block that is flashed at once.
  static const size_t CHUNK_SIZE = 248;

  /// GPIO outputs for controlling the CC2652 bootloader and reset lines.
  gpio::GPIOBinaryOutput *bsl_output_{nullptr};
  gpio::GPIOBinaryOutput *reset_output_{nullptr};

  // --- Bootloader protocol details ---
  static constexpr uint8_t CMD_SYNC  = 0x55;
  static constexpr uint8_t CMD_ERASE = 0xAA;
  static constexpr uint8_t CMD_WRITE = 0xA0;
  static constexpr uint8_t CMD_EXIT  = 0xA5;
  static constexpr uint8_t ACK       = 0xCC;  // Expected acknowledgement byte

  // The baud rate to use during flashing.
  uint32_t flashing_baud_rate_{500000};

  /// Helper: Read one byte from UART with a timeout (in ms). Returns -1 if timeout.
  int read_byte(uint32_t timeout_ms);

  /// Toggle GPIOs to force the CC2652 into bootloader mode.
  bool enter_bootloader_mode();

  /// Send a sync command and wait for an ACK.
  bool sync_bootloader();

  /// Send an erase flash command and wait for ACK.
  bool erase_flash();

  /// Send a write command for one block (includes address, length, data, and CRC16).
  bool write_flash_block(uint32_t address, const uint8_t* data, size_t length);

  /// Send an exit bootloader command and wait for ACK.
  bool exit_bootloader();

  /// Compute a CRC16 (polynomial 0x1021, initial value 0xFFFF) over the given data.
  uint16_t compute_crc16(const uint8_t *data, size_t length);
};

}  // namespace cc2652_flasher
}  // namespace esphome
