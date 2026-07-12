#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#include <memory>
#include <optional>
#include <string>

class StreamServerComponent : public esphome::Component {
public:
    StreamServerComponent() = default;
    explicit StreamServerComponent(esphome::uart::UARTComponent *stream) : stream_{stream} {}
    void set_uart_parent(esphome::uart::UARTComponent *parent) { this->stream_ = parent; }
    void set_buffer_size(size_t size) { this->buf_size_ = size; }
    void pause();
    void resume();
    void set_keep_alive(int idle_s, int interval_s, int count) {
        this->keep_alive_idle_s_ = idle_s;
        this->keep_alive_interval_s_ = interval_s;
        this->keep_alive_count_ = count;
    }
    void set_trace(bool v) { this->trace_ = v; }

#ifdef USE_BINARY_SENSOR
    void set_connected_sensor(esphome::binary_sensor::BinarySensor *connected) { this->connected_sensor_ = connected; }
#endif

    void setup() override;
    void loop() override;
    void dump_config() override;
    void on_shutdown() override;

    float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

    void set_port(uint16_t port) { this->port_ = port; }

protected:
    struct Client;

    void publish_sensor();

    void accept();
    void cleanup();
    void close_client_(Client &client);
    void read();
    void flush();
    void write();

    size_t buf_index(size_t pos) { return pos & (this->buf_size_ - 1); }
    /// Return the number of consecutive elements that are ahead of @p pos in memory.
    size_t buf_ahead(size_t pos) { return (pos | (this->buf_size_ - 1)) - pos + 1; }

    struct Client {
        Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier, size_t position);

        std::unique_ptr<esphome::socket::Socket> socket{nullptr};
        std::string identifier{};
        bool disconnected{false};
        size_t position{0};
    };

    esphome::uart::UARTComponent *stream_{nullptr};
    uint16_t port_;
    size_t buf_size_;

#ifdef USE_BINARY_SENSOR
    esphome::binary_sensor::BinarySensor *connected_sensor_;
#endif

    std::unique_ptr<uint8_t[]> buf_{};
    size_t buf_head_{0};
    size_t buf_tail_{0};

    std::unique_ptr<esphome::socket::Socket> socket_{};
    std::optional<Client> client_{};

    bool paused_{false};
    bool trace_{false};
    bool setup_complete_{false};

    // TCP -> UART staging for the non-blocking IDF FIFO write path; sized to the
    // hardware TX FIFO. Unused (but harmless) on other platforms.
    uint8_t tx_pending_[128];
    size_t tx_pending_len_{0};
    size_t tx_pending_pos_{0};
    uint32_t tx_stall_since_{0};
    int uart_num_{-1};
    // Runs the main loop continuously while a TX backlog exists, so sustained
    // host->radio bursts (e.g. OTA at 921600) drain at line rate instead of one
    // FIFO fill per 16ms loop pass.
    esphome::HighFrequencyLoopRequester high_freq_;

    int keep_alive_idle_s_{0};
    int keep_alive_interval_s_{0};
    int keep_alive_count_{0};
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
