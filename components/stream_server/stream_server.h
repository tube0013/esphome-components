#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/defines.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <driver/uart.h>

#include <atomic>
#include <cstddef>
#include <cstdint>

/// Bridges one hardware UART to one TCP client.
///
/// The byte pump runs in a dedicated FreeRTOS task (task_loop_()) so it is not
/// paced by ESPHome's main loop. That task owns the listening socket, the client
/// socket, both staging buffers and — while not paused — the UART driver. The
/// main loop only publishes sensors from atomics and drives pause()/resume().
///
/// Backpressure is end to end and needs no intermediate ring buffer:
///  * UART -> TCP: the UART is read only when uart_to_tcp_ is empty. A slow or
///    stalled TCP peer leaves bytes in the IDF driver's RX buffer, which (with
///    hardware flow control) deasserts RTS towards the radio.
///  * TCP -> UART: the socket is read only when tcp_to_uart_ is empty. A busy
///    UART (FIFO full / CTS held) leaves bytes in lwIP's receive buffer, which
///    shrinks the advertised TCP window and throttles the host.
class StreamServerComponent : public esphome::Component {
 public:
  StreamServerComponent() = default;
  explicit StreamServerComponent(esphome::uart::UARTComponent *stream) : stream_{stream} {}

  void set_uart_parent(esphome::uart::UARTComponent *parent) { this->stream_ = parent; }
  /// Deprecated no-op: the ring buffer is gone; staging buffers are fixed at STAGING_SIZE.
  void set_buffer_size(size_t /*size*/) {}
  void set_port(uint16_t port) { this->port_ = port; }
  void set_keep_alive(int idle_s, int interval_s, int count) {
    this->keep_alive_idle_s_ = idle_s;
    this->keep_alive_interval_s_ = interval_s;
    this->keep_alive_count_ = count;
  }
  void set_trace(bool v) { this->trace_ = v; }

#ifdef USE_BINARY_SENSOR
  void set_connected_sensor(esphome::binary_sensor::BinarySensor *connected) { this->connected_sensor_ = connected; }
#endif
#ifdef USE_SENSOR
  void set_disconnects_sensor(esphome::sensor::Sensor *s) { this->disconnects_sensor_ = s; }
#endif

  /// Stop the pump and hand the UART to someone else (flasher / probe).
  /// Blocks until the pump task has acknowledged, so when this returns the task
  /// is guaranteed not to touch the UART until resume(). Safe to call from the
  /// main loop or any other task, and before setup() (the task then starts paused).
  void pause();
  /// Give the UART back to the pump. Discards whatever the previous owner left
  /// in the RX path so it is not forwarded to the next client.
  void resume();
  bool is_paused() const { return this->pause_requested_.load(); }
  bool is_connected() const { return this->connected_.load(); }

  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

  static constexpr size_t STAGING_SIZE = 512;

 protected:
  // ---- pump task (everything below runs only on the task) ----
  static void task_trampoline_(void *arg);
  void task_loop_();
  void enter_paused_();
  void accept_client_();
  void reject_pending_();
  void configure_client_socket_(int fd);
  void close_client_(bool count_disconnect, const char *reason);
  void discard_uart_rx_();
  bool fill_uart_to_tcp_();
  void flush_uart_to_tcp_();
  void fill_tcp_to_uart_();
  void flush_tcp_to_uart_();
  void log_trace_(const char *dir, const uint8_t *data, size_t len);

  struct Staging {
    uint8_t buf[STAGING_SIZE];
    size_t len{0};     ///< valid bytes in buf
    size_t offset{0};  ///< bytes of buf already delivered
    size_t pending() const { return this->len - this->offset; }
    bool empty() const { return this->offset >= this->len; }
    void clear() { this->len = this->offset = 0; }
  };

  esphome::uart::UARTComponent *stream_{nullptr};
  uart_port_t uart_num_{UART_NUM_MAX};
  uint16_t port_{6638};

  int listen_fd_{-1};
  int client_fd_{-1};
  char client_id_[56]{};

  Staging uart_to_tcp_;
  Staging tcp_to_uart_;

  /// send() hit ENOMEM/ENOBUFS: retry on the timeout, not on (spurious) writability.
  bool tx_nomem_{false};
  uint32_t tx_stall_since_{0};
  bool tx_stall_warned_{false};

  // Keepalive defaults catch a half-open client (host crashed / cable pulled)
  // in ~25s. That matters because the single client slot refuses newcomers.
  int keep_alive_idle_s_{10};
  int keep_alive_interval_s_{5};
  int keep_alive_count_{3};
  bool trace_{false};

  // ---- cross-thread state ----
  TaskHandle_t task_{nullptr};
  SemaphoreHandle_t pause_ack_sem_{nullptr};
  std::atomic<bool> pause_requested_{false};
  std::atomic<uint32_t> pause_seq_{0};      ///< bumped by every pause() that changes state
  std::atomic<uint32_t> pause_ack_seq_{0};  ///< last pause_seq_ the task has acknowledged
  std::atomic<bool> connected_{false};
  std::atomic<uint32_t> disconnects_{0};

  // ---- main-loop-only mirrors of what was last published ----
  bool published_connected_{false};
  uint32_t published_disconnects_{0};

#ifdef USE_BINARY_SENSOR
  esphome::binary_sensor::BinarySensor *connected_sensor_{nullptr};
#endif
#ifdef USE_SENSOR
  esphome::sensor::Sensor *disconnects_sensor_{nullptr};
#endif
};

template<typename... Ts> class PauseAction : public esphome::Action<Ts...> {
 public:
  explicit PauseAction(StreamServerComponent *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->pause(); }

 protected:
  StreamServerComponent *parent_;
};

template<typename... Ts> class ResumeAction : public esphome::Action<Ts...> {
 public:
  explicit ResumeAction(StreamServerComponent *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->resume(); }

 protected:
  StreamServerComponent *parent_;
};
