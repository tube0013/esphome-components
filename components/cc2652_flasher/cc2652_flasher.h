#pragma once
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"      // Provides uart::UARTDevice and UARTComponent
#include "esphome/components/switch/switch.h"  // Provides esphome::switch_::Switch

namespace esphome {
namespace cc2652_flasher {

/**
 * CC2652FlasherComponent
 *
 * This component flashes a TI CC2652 radio using a bootloader protocol inspired
 * by cc2538-bsl.py. It downloads a manifest that specifies the firmware to flash,
 * temporarily changes the UART baud rate during flashing, and then reverts it back.
 *
 * Two external outputs (provided as switch_::Switch objects) are used to control the
 * bootloader (BSL) and reset lines.
 *
 * IMPORTANT: Verify command codes, ACK values, and packet structure against your
 * CC2652 bootloader documentation.
 */
class CC2652FlasherComponent : public Component, public uart::UARTDevice {
 public:
  // Setters called from the Python configuration.
  void set_manifest_url(const std::string &manifest_url) { manifest_url_ = manifest_url; }
  void set_flashing_baud_rate(uint32_t baud) { flashing_baud_rate_ = baud; }
  void set_bsl_output(switch_::Switch *bsl_output) { bsl_output_ = bsl_output; }
  void set_reset_output(switch_::Switch *reset_output) { reset_output_ = reset_output; }

  /// New setter for storing the pointer to the UART component.
  void set_parent_uart(uart::UARTComponent *uart) { parent_uart_ = uart; }

  // Inline baud rate accessor functions using the stored UART component pointer.
  uint32_t get_baud_rate() {
    return parent_uart_ != nullptr ? parent_uart_->get_baud_rate() : 115200;
  }
  void set_baud_rate(uint32_t baud) {
    if (parent_uart_ != nullptr) {
      parent_uart_->set_baud_rate(baud);
    }
  }

  /// Trigger the flash process. (In production you might register a service instead.)
  void flash_firmware();

  // Standard ESPHome component interface.
  void setup() override;
  void loop() override;

 protected:
  std::string manifest_url_{};
  static const size_t CHUNK_SIZE = 248;
  // External outputs for bootloader control (BSL and reset) provided as switches.
  switch_::Switch *bsl_output_{nullptr};
  switch_::Switch *reset_output_{nullptr};
  // Pointer to the UART component (set in the Python binding).
  uart::UARTComponent *parent_uart_{nullptr};

  // Bootloader protocol command definitions.
  static constexpr uint8_t CMD_SYNC  = 0x55;
  static constexpr uint8_t CMD_ERASE = 0xAA;
  static constexpr uint8_t CMD_WRITE = 0xA0;
  static constexpr uint8_t CMD_EXIT  = 0xA5;
  static constexpr uint8_t ACK       = 0xCC;

  // The baud rate to use during flashing.
  uint32_t flashing_baud_rate_{500000};

  // Helper functions.
  int read_byte(uint32_t timeout_ms);
  bool enter_bootloader_mode();
  bool sync_bootloader();
  bool erase_flash();
  bool write_flash_block(uint32_t address, const uint8_t* data, size_t length);
  bool exit_bootloader();
  uint16_t compute_crc16(const uint8_t *data, size_t length);
};

}  // namespace cc2652_flasher
}  // namespace esphome
