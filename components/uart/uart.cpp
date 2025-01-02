#include "uart.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include <cinttypes>

namespace esphome {
namespace uart {

static const char *const TAG = "uart";

void UARTDevice::check_uart_settings(uint32_t baud_rate, uint8_t stop_bits, UARTParityOptions parity,
                                     uint8_t data_bits, UARTHwFlowControl hw_flow_control) {
  if (this->parent_->get_baud_rate() != baud_rate) {
    ESP_LOGE(TAG, "  Invalid baud_rate: Integration requested baud_rate %" PRIu32 " but you have %" PRIu32 "!",
             baud_rate, this->parent_->get_baud_rate());
  }
  if (this->parent_->get_stop_bits() != stop_bits) {
    ESP_LOGE(TAG, "  Invalid stop bits: Integration requested stop_bits %u but you have %u!", stop_bits,
             this->parent_->get_stop_bits());
  }
  if (this->parent_->get_data_bits() != data_bits) {
    ESP_LOGE(TAG, "  Invalid number of data bits: Integration requested %u data bits but you have %u!", data_bits,
             this->parent_->get_data_bits());
  }
  if (this->parent_->get_parity() != parity) {
    ESP_LOGE(TAG, "  Invalid parity: Integration requested parity %s but you have %s!",
             LOG_STR_ARG(parity_to_str(parity)), LOG_STR_ARG(parity_to_str(this->parent_->get_parity())));
  }
  if (this->parent_->get_hw_flow_control() != hw_flow_control) {
    ESP_LOGE(TAG, "  Invalid hw_flow_control: Integration requested hw_flow_control %s but you have %s!",
             LOG_STR_ARG(hw_flow_control_to_str(hw_flow_control)), LOG_STR_ARG(hw_flow_control_to_str(this->parent_->get_hw_flow_control())));
  }
}

const LogString *parity_to_str(UARTParityOptions parity) {
  switch (parity) {
    case UART_CONFIG_PARITY_NONE:
      return LOG_STR("NONE");
    case UART_CONFIG_PARITY_EVEN:
      return LOG_STR("EVEN");
    case UART_CONFIG_PARITY_ODD:
      return LOG_STR("ODD");
    default:
      return LOG_STR("UNKNOWN");
  }
}

const LogString *hw_flow_control_to_str(UARTHwFlowControl hw_flow_control) {
  switch (hw_flow_control) {
    case UART_CONFIG_HW_FLOW_CONTROL_DISABLE:
      return LOG_STR("DISABLE");
    case UART_CONFIG_HW_FLOW_CONTROL_CTS_RTS:
      return LOG_STR("CTS_RTS");
    default:
      return LOG_STR("UNKNOWN");
  }
}

}  // namespace uart
}  // namespace esphome
