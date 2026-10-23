#include "stream_server.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/uart/uart_component_esp_idf.h"

#include <lwip/inet.h>
#include <lwip/sockets.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>

static const char *const TAG = "stream_server";

using namespace esphome;

namespace {

constexpr uint32_t TASK_STACK_SIZE = 4096;
// Above the ESPHome loop task (1), well below lwIP's tcpip thread (18) and Wi-Fi.
constexpr UBaseType_t TASK_PRIORITY = 5;

// select() timeouts. With a client connected, UART RX has no fd to wait on, so
// the timeout bounds radio->host latency (one FreeRTOS tick at 1 kHz). Without
// a client it only paces accept() and the stale-RX discard.
constexpr uint32_t ACTIVE_SELECT_MS = 1;
constexpr uint32_t IDLE_SELECT_MS = 10;
// Consecutive zero-timeout passes allowed while the UART keeps producing, before
// forcing one tick of sleep so a sustained stream can never monopolise a core.
constexpr int MAX_BUSY_PASSES = 8;

constexpr uint32_t PAUSE_ACK_TIMEOUT_MS = 1000;
constexpr uint32_t PAUSED_POLL_MS = 100;
// On handover, let bytes already in the TX FIFO leave the wire before the
// flasher/probe takes over (bounded: a radio holding CTS would never finish).
constexpr uint32_t HANDOVER_TX_DRAIN_MS = 20;
// After a client leaves, keep pushing bytes it already sent us for this long.
constexpr uint32_t ORPHAN_TX_GRACE_MS = 100;
constexpr uint32_t TX_STALL_WARN_MS = 5000;
// Max newcomers refused per pass, so a connect storm cannot starve the pump.
constexpr int MAX_REJECTS_PER_PASS = 4;

bool is_would_block(int err) { return err == EAGAIN || err == EWOULDBLOCK || err == EINTR; }
// lwIP runs out of pbufs/segments transiently during bursts of small TCP_NODELAY
// writes; the pool frees as ACKs arrive, so this is backpressure, not a failure.
bool is_out_of_memory(int err) { return err == ENOMEM || err == ENOBUFS; }
// Peer is simply gone; not worth more than an info line.
bool is_peer_gone(int err) { return err == ECONNRESET || err == ENOTCONN || err == EPIPE || err == ECONNABORTED; }

void format_peer(const struct sockaddr_storage &ss, char *out, size_t out_len) {
  char ip[48] = "?";
  unsigned port = 0;
  if (ss.ss_family == AF_INET) {
    const auto *a = reinterpret_cast<const struct sockaddr_in *>(&ss);
    inet_ntop(AF_INET, &a->sin_addr, ip, sizeof(ip));
    port = ntohs(a->sin_port);
  }
#if LWIP_IPV6
  else if (ss.ss_family == AF_INET6) {
    const auto *a = reinterpret_cast<const struct sockaddr_in6 *>(&ss);
    inet_ntop(AF_INET6, &a->sin6_addr, ip, sizeof(ip));
    port = ntohs(a->sin6_port);
  }
#endif
  snprintf(out, out_len, "%s:%u", ip, port);
}

bool set_nonblocking(int fd) {
  int flags = lwip_fcntl(fd, F_GETFL, 0);
  if (flags < 0)
    return false;
  return lwip_fcntl(fd, F_SETFL, flags | O_NONBLOCK) >= 0;
}

}  // namespace

// ---------------------------------------------------------------------------
// Main-loop side
// ---------------------------------------------------------------------------

void StreamServerComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up stream server...");

  if (this->stream_ == nullptr) {
    ESP_LOGE(TAG, "No UART configured");
    this->mark_failed();
    return;
  }
  // Every ESP32 build (IDF and Arduino) uses IDFUARTComponent since 2026.3; the
  // config schema restricts this component to ESP32. RTTI is off, so static_cast.
  this->uart_num_ = static_cast<uart_port_t>(
      static_cast<uart::IDFUARTComponent *>(this->stream_)->get_hw_serial_number());

  struct sockaddr_storage bind_addr {};
  socklen_t bind_len =
      socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), this->port_);
  if (bind_len == 0) {
    ESP_LOGE(TAG, "Failed to build bind address");
    this->mark_failed();
    return;
  }

  this->listen_fd_ = lwip_socket(bind_addr.ss_family, SOCK_STREAM, IPPROTO_TCP);
  if (this->listen_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to create listening socket: errno %d", errno);
    this->mark_failed();
    return;
  }
  int one = 1;
  lwip_setsockopt(this->listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  // lwIP does not copy TCP_NODELAY to accepted sockets; it is set again per client.
  lwip_setsockopt(this->listen_fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  if (!set_nonblocking(this->listen_fd_) ||
      lwip_bind(this->listen_fd_, reinterpret_cast<struct sockaddr *>(&bind_addr), bind_len) != 0 ||
      lwip_listen(this->listen_fd_, 4) != 0) {
    ESP_LOGE(TAG, "Failed to listen on port %u: errno %d", this->port_, errno);
    lwip_close(this->listen_fd_);
    this->listen_fd_ = -1;
    this->mark_failed();
    return;
  }

  this->pause_ack_sem_ = xSemaphoreCreateBinary();
  if (this->pause_ack_sem_ == nullptr ||
      xTaskCreate(StreamServerComponent::task_trampoline_, "stream_server", TASK_STACK_SIZE, this, TASK_PRIORITY,
                  &this->task_) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start pump task");
    this->task_ = nullptr;
    lwip_close(this->listen_fd_);
    this->listen_fd_ = -1;
    this->mark_failed();
    return;
  }
  // If pause() ran before setup (an on_boot automation at our priority), the
  // task observes pause_requested_ on its first pass and starts out paused.

#ifdef USE_BINARY_SENSOR
  if (this->connected_sensor_ != nullptr)
    this->connected_sensor_->publish_state(false);
#endif
#ifdef USE_SENSOR
  if (this->disconnects_sensor_ != nullptr)
    this->disconnects_sensor_->publish_state(0);
#endif
}

void StreamServerComponent::loop() {
  // The pump task never publishes: entity publishing is main-loop only. It
  // updates atomics and this mirrors them out on change.
  const bool connected = this->connected_.load(std::memory_order_relaxed);
  if (connected != this->published_connected_) {
    this->published_connected_ = connected;
#ifdef USE_BINARY_SENSOR
    if (this->connected_sensor_ != nullptr)
      this->connected_sensor_->publish_state(connected);
#endif
  }
  const uint32_t disconnects = this->disconnects_.load(std::memory_order_relaxed);
  if (disconnects != this->published_disconnects_) {
    this->published_disconnects_ = disconnects;
#ifdef USE_SENSOR
    if (this->disconnects_sensor_ != nullptr)
      this->disconnects_sensor_->publish_state(disconnects);
#endif
  }
}

void StreamServerComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Stream Server:");
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 7, 0)
  char use_address_buf[network::USE_ADDRESS_BUFFER_SIZE];
  ESP_LOGCONFIG(TAG, "  Address: %s:%u", network::get_use_address_to(use_address_buf), this->port_);
#else
  ESP_LOGCONFIG(TAG, "  Address: %s:%u", network::get_use_address(), this->port_);
#endif
  ESP_LOGCONFIG(TAG, "  UART port: %d", (int) this->uart_num_);
  ESP_LOGCONFIG(TAG, "  Staging buffers: 2 x %u bytes", (unsigned) STAGING_SIZE);
  if (this->keep_alive_idle_s_ > 0) {
    ESP_LOGCONFIG(TAG, "  TCP keepalive: idle %ds, interval %ds, count %d", this->keep_alive_idle_s_,
                  this->keep_alive_interval_s_, this->keep_alive_count_);
  } else {
    ESP_LOGCONFIG(TAG, "  TCP keepalive: disabled");
  }
  ESP_LOGCONFIG(TAG, "  Paused: %s", YESNO(this->pause_requested_.load()));
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Connected", this->connected_sensor_);
#endif
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Disconnects", this->disconnects_sensor_);
#endif
}

void StreamServerComponent::on_shutdown() {
  // Park the task (closes the client with a FIN), then release the listener.
  this->pause();
  if (this->task_ == nullptr ||
      (int32_t) (this->pause_ack_seq_.load() - this->pause_seq_.load()) >= 0) {
    if (this->listen_fd_ >= 0) {
      lwip_close(this->listen_fd_);
      this->listen_fd_ = -1;
    }
  }
}

void StreamServerComponent::pause() {
  if (!this->pause_requested_.exchange(true)) {
    this->pause_seq_.fetch_add(1);
    ESP_LOGI(TAG, "Pausing stream server");
  }
  // Before setup there is no task, so nothing can be touching the UART. And the
  // task must never wait on its own acknowledgement.
  if (this->task_ == nullptr || xTaskGetCurrentTaskHandle() == this->task_)
    return;

  const uint32_t seq = this->pause_seq_.load();
  const uint32_t start = millis();
  // Wrap-safe: done once the task has acknowledged this pause or a later one.
  while ((int32_t) (this->pause_ack_seq_.load() - seq) < 0) {
    if (millis() - start > PAUSE_ACK_TIMEOUT_MS) {
      ESP_LOGE(TAG, "Pump task did not acknowledge pause within %ums", (unsigned) PAUSE_ACK_TIMEOUT_MS);
      return;
    }
    xSemaphoreTake(this->pause_ack_sem_, pdMS_TO_TICKS(10));
  }
}

void StreamServerComponent::resume() {
  if (!this->pause_requested_.load())
    return;
  ESP_LOGI(TAG, "Resuming stream server");
  // The task is parked, so the main loop owns the UART here. Discard what the
  // flasher/probe left behind through the component API, which also clears
  // IDFUARTComponent's peek cache (the task reads the driver directly and would
  // otherwise never see a peeked byte). Only after setup: before it, the driver
  // may not be installed yet.
  if (this->task_ != nullptr && this->stream_ != nullptr) {
    uint8_t scratch[64];
    size_t discarded = 0;
    while (discarded < 4096) {
      const size_t avail = static_cast<size_t>(this->stream_->available());
      if (avail == 0 || avail > 0x10000)
        break;
      const size_t n = std::min(avail, sizeof(scratch));
      if (!this->stream_->read_array(scratch, n))
        break;
      discarded += n;
    }
    if (discarded > 0)
      ESP_LOGD(TAG, "Discarded %u stale UART bytes on resume", (unsigned) discarded);
  }
  this->pause_requested_.store(false);
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

// ---------------------------------------------------------------------------
// Pump task
// ---------------------------------------------------------------------------

void StreamServerComponent::task_trampoline_(void *arg) {
  static_cast<StreamServerComponent *>(arg)->task_loop_();
  vTaskDelete(nullptr);
}

void StreamServerComponent::task_loop_() {
  int busy_passes = 0;
  for (;;) {
    // ---- pause handshake ----
    if (this->pause_requested_.load()) {
      const uint32_t seq = this->pause_seq_.load();
      if (this->pause_ack_seq_.load() != seq) {
        this->enter_paused_();
        this->pause_ack_seq_.store(seq);
        xSemaphoreGive(this->pause_ack_sem_);
      }
      // resume() notifies; the timeout just re-checks the flags periodically.
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PAUSED_POLL_MS));
      continue;
    }
    if (this->listen_fd_ < 0) {
      vTaskDelay(pdMS_TO_TICKS(PAUSED_POLL_MS));
      continue;
    }

    // ---- no client: keep the RX path empty, wait for a connection ----
    if (this->client_fd_ < 0) {
      busy_passes = 0;
      this->discard_uart_rx_();
      if (!this->tcp_to_uart_.empty()) {
        // Bytes the previous client sent before leaving still belong on the wire.
        this->flush_tcp_to_uart_();
        if (!this->tcp_to_uart_.empty() && this->tx_stall_since_ != 0 &&
            millis() - this->tx_stall_since_ > ORPHAN_TX_GRACE_MS) {
          ESP_LOGW(TAG, "Dropping %u bytes from departed client: UART TX stalled",
                   (unsigned) this->tcp_to_uart_.pending());
          this->tcp_to_uart_.clear();
          this->tx_stall_since_ = 0;
          this->tx_stall_warned_ = false;
        }
      }
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(this->listen_fd_, &rfds);
      const uint32_t wait_ms = this->tcp_to_uart_.empty() ? IDLE_SELECT_MS : ACTIVE_SELECT_MS;
      struct timeval tv {0, (long) (wait_ms * 1000)};
      const int n = lwip_select(this->listen_fd_ + 1, &rfds, nullptr, nullptr, &tv);
      if (n > 0 && FD_ISSET(this->listen_fd_, &rfds) && !this->pause_requested_.load()) {
        this->accept_client_();
      } else if (n < 0 && errno != EINTR) {
        ESP_LOGW(TAG, "select() failed: errno %d", errno);
        vTaskDelay(pdMS_TO_TICKS(IDLE_SELECT_MS));
      }
      continue;
    }

    // ---- client connected: move what can move, then wait for something to change ----
    this->flush_tcp_to_uart_();                     // staged host bytes -> TX FIFO
    const bool more_rx = this->fill_uart_to_tcp_();  // radio bytes -> staging (only if empty)
    this->flush_uart_to_tcp_();                     // staging -> socket
    if (this->client_fd_ < 0)
      continue;  // a fatal socket error closed the client

    fd_set rfds, wfds;
    FD_ZERO(&rfds);
    FD_ZERO(&wfds);
    FD_SET(this->listen_fd_, &rfds);  // so newcomers are refused promptly
    // Read the socket only when there is room: unread bytes stay in lwIP and
    // close the TCP window, throttling the host while the UART is busy.
    if (this->tcp_to_uart_.empty())
      FD_SET(this->client_fd_, &rfds);
    // Wait for writability only on real send-buffer backpressure. After
    // ENOMEM/ENOBUFS the socket still reports writable, so retry on the timeout.
    if (!this->uart_to_tcp_.empty() && !this->tx_nomem_)
      FD_SET(this->client_fd_, &wfds);

    const bool immediate = more_rx && this->uart_to_tcp_.empty() && busy_passes < MAX_BUSY_PASSES;
    busy_passes = immediate ? busy_passes + 1 : 0;
    struct timeval tv {0, immediate ? 0L : (long) (ACTIVE_SELECT_MS * 1000)};
    const int maxfd = std::max(this->listen_fd_, this->client_fd_);
    const int n = lwip_select(maxfd + 1, &rfds, &wfds, nullptr, &tv);
    if (n < 0) {
      if (errno != EINTR) {
        ESP_LOGW(TAG, "select() failed: errno %d", errno);
        vTaskDelay(1);
      }
      continue;
    }
    if (n == 0)
      continue;
    if (FD_ISSET(this->listen_fd_, &rfds))
      this->reject_pending_();
    if (this->client_fd_ >= 0 && this->tcp_to_uart_.empty() && FD_ISSET(this->client_fd_, &rfds))
      this->fill_tcp_to_uart_();
    // Writability is acted on by flush_uart_to_tcp_() at the top of the next pass.
  }
}

void StreamServerComponent::enter_paused_() {
  if (this->client_fd_ >= 0)
    this->close_client_(false, "stream server paused");
  this->uart_to_tcp_.clear();
  this->tcp_to_uart_.clear();
  this->tx_stall_since_ = 0;
  this->tx_stall_warned_ = false;
  uart_wait_tx_done(this->uart_num_, pdMS_TO_TICKS(HANDOVER_TX_DRAIN_MS));
}

void StreamServerComponent::configure_client_socket_(int fd) {
  if (!set_nonblocking(fd))
    ESP_LOGW(TAG, "Failed to make client socket non-blocking: errno %d", errno);

  int val = 1;
  // Disable Nagle: with Nagle + the host's delayed ACK (up to 40ms on Linux),
  // small ASH/ZNP frames stall per frame and break coordinator timing.
  if (lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val)) != 0)
    ESP_LOGW(TAG, "setsockopt TCP_NODELAY failed: errno %d", errno);

  if (this->keep_alive_idle_s_ <= 0)
    return;
  if (lwip_setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val)) != 0) {
    ESP_LOGW(TAG, "setsockopt SO_KEEPALIVE failed: errno %d", errno);
    return;
  }
  val = this->keep_alive_idle_s_;
  if (lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &val, sizeof(val)) != 0)
    ESP_LOGW(TAG, "setsockopt TCP_KEEPIDLE failed: errno %d", errno);
  val = this->keep_alive_interval_s_;
  if (lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &val, sizeof(val)) != 0)
    ESP_LOGW(TAG, "setsockopt TCP_KEEPINTVL failed: errno %d", errno);
  val = this->keep_alive_count_;
  if (lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &val, sizeof(val)) != 0)
    ESP_LOGW(TAG, "setsockopt TCP_KEEPCNT failed: errno %d", errno);
}

void StreamServerComponent::accept_client_() {
  struct sockaddr_storage addr {};
  socklen_t addr_len = sizeof(addr);
  const int fd = lwip_accept(this->listen_fd_, reinterpret_cast<struct sockaddr *>(&addr), &addr_len);
  if (fd < 0) {
    const int err = errno;
    if (!is_would_block(err)) {
      ESP_LOGW(TAG, "accept() failed: errno %d", err);
      vTaskDelay(pdMS_TO_TICKS(IDLE_SELECT_MS));  // e.g. socket pool exhausted: don't spin
    }
    return;
  }
  this->configure_client_socket_(fd);
  format_peer(addr, this->client_id_, sizeof(this->client_id_));

  // Frames the radio sent before this connect are not for this client.
  uart_flush_input(this->uart_num_);
  this->uart_to_tcp_.clear();
  this->tcp_to_uart_.clear();
  this->tx_nomem_ = false;
  this->tx_stall_since_ = 0;
  this->tx_stall_warned_ = false;

  this->client_fd_ = fd;
  this->connected_.store(true);
  ESP_LOGI(TAG, "Client %s connected", this->client_id_);
}

void StreamServerComponent::reject_pending_() {
  // The UART can serve one consumer. Refuse newcomers rather than dropping the
  // established session, so a stray probe (nc, discovery scan) cannot kill an
  // active ZHA/Z2M connection.
  for (int i = 0; i < MAX_REJECTS_PER_PASS; i++) {
    struct sockaddr_storage addr {};
    socklen_t addr_len = sizeof(addr);
    const int fd = lwip_accept(this->listen_fd_, reinterpret_cast<struct sockaddr *>(&addr), &addr_len);
    if (fd < 0) {
      if (!is_would_block(errno)) {
        ESP_LOGW(TAG, "accept() failed: errno %d", errno);
        vTaskDelay(pdMS_TO_TICKS(IDLE_SELECT_MS));
      }
      return;
    }
    char peer[sizeof(this->client_id_)];
    format_peer(addr, peer, sizeof(peer));
    ESP_LOGW(TAG, "Refusing connection from %s: client %s is already connected", peer, this->client_id_);
    // RST instead of FIN: a normal close parks the PCB in TIME_WAIT for ~2 min,
    // letting a probe/retry storm exhaust the lwIP socket pool.
    struct linger lin = {1, 0};
    lwip_setsockopt(fd, SOL_SOCKET, SO_LINGER, &lin, sizeof(lin));
    lwip_close(fd);
  }
}

void StreamServerComponent::close_client_(bool count_disconnect, const char *reason) {
  if (this->client_fd_ < 0)
    return;
  lwip_shutdown(this->client_fd_, SHUT_RDWR);
  lwip_close(this->client_fd_);
  this->client_fd_ = -1;
  // Radio bytes not yet delivered have nowhere to go. Host bytes already staged
  // for the UART are kept and drained (bounded) by the no-client branch.
  this->uart_to_tcp_.clear();
  this->tx_nomem_ = false;
  this->connected_.store(false);
  if (count_disconnect)
    this->disconnects_.fetch_add(1);
  ESP_LOGI(TAG, "Client %s disconnected (%s)", this->client_id_, reason);
}

void StreamServerComponent::discard_uart_rx_() {
  size_t avail = 0;
  if (uart_get_buffered_data_len(this->uart_num_, &avail) != ESP_OK || avail == 0)
    return;
  uart_flush_input(this->uart_num_);
  if (this->trace_)
    ESP_LOGD(TAG, "No client: discarded %u UART bytes", (unsigned) avail);
}

bool StreamServerComponent::fill_uart_to_tcp_() {
  size_t avail = 0;
  if (uart_get_buffered_data_len(this->uart_num_, &avail) != ESP_OK)
    return false;
  // Backpressure: never read more until the previous chunk is fully on the socket.
  if (!this->uart_to_tcp_.empty() || avail == 0)
    return avail > 0;
  const int n = uart_read_bytes(this->uart_num_, this->uart_to_tcp_.buf, std::min(avail, STAGING_SIZE), 0);
  if (n <= 0)
    return false;
  this->uart_to_tcp_.len = static_cast<size_t>(n);
  this->uart_to_tcp_.offset = 0;
  this->log_trace_("UART -> TCP", this->uart_to_tcp_.buf, this->uart_to_tcp_.len);
  return avail > static_cast<size_t>(n);
}

void StreamServerComponent::flush_uart_to_tcp_() {
  if (this->client_fd_ < 0 || this->uart_to_tcp_.empty())
    return;
  Staging &s = this->uart_to_tcp_;
  const ssize_t n = lwip_send(this->client_fd_, s.buf + s.offset, s.pending(), MSG_DONTWAIT);
  if (n > 0) {
    s.offset += static_cast<size_t>(n);
    if (s.empty())
      s.clear();
    this->tx_nomem_ = false;
    return;
  }
  if (n == 0) {
    this->close_client_(true, "send returned 0");
    return;
  }
  const int err = errno;
  if (is_would_block(err)) {
    this->tx_nomem_ = false;  // TCP send buffer full: wait for writability
    return;
  }
  if (is_out_of_memory(err)) {
    this->tx_nomem_ = true;  // pbuf pool exhausted: retry on the next tick
    return;
  }
  if (!is_peer_gone(err))
    ESP_LOGW(TAG, "send() to %s failed: errno %d (%s)", this->client_id_, err, strerror(err));
  this->close_client_(true, strerror(err));
}

void StreamServerComponent::fill_tcp_to_uart_() {
  if (this->client_fd_ < 0 || !this->tcp_to_uart_.empty())
    return;
  Staging &s = this->tcp_to_uart_;
  const ssize_t n = lwip_recv(this->client_fd_, s.buf, STAGING_SIZE, MSG_DONTWAIT);
  if (n > 0) {
    s.len = static_cast<size_t>(n);
    s.offset = 0;
    this->log_trace_("TCP -> UART", s.buf, s.len);
    this->flush_tcp_to_uart_();
    return;
  }
  if (n == 0) {
    this->close_client_(true, "closed by peer");
    return;
  }
  const int err = errno;
  if (is_would_block(err) || is_out_of_memory(err))
    return;
  if (!is_peer_gone(err))
    ESP_LOGW(TAG, "recv() from %s failed: errno %d (%s)", this->client_id_, err, strerror(err));
  this->close_client_(true, strerror(err));
}

void StreamServerComponent::flush_tcp_to_uart_() {
  Staging &s = this->tcp_to_uart_;
  if (s.empty())
    return;
  // uart_tx_chars() only fills the hardware TX FIFO and returns how much it took.
  // ESPHome installs the driver without a TX ring buffer, so write_array() /
  // uart_write_bytes() would block until every byte is in the FIFO — forever if
  // the radio holds CTS. Whatever does not fit stays staged and the socket is
  // not read again until it drains.
  const int sent = uart_tx_chars(this->uart_num_, reinterpret_cast<const char *>(s.buf + s.offset), s.pending());
  if (sent > 0) {
    s.offset += static_cast<size_t>(sent);
    if (s.empty())
      s.clear();
    if (this->tx_stall_warned_)
      ESP_LOGI(TAG, "UART TX resumed");
    this->tx_stall_since_ = 0;
    this->tx_stall_warned_ = false;
    return;
  }
  const uint32_t now = millis();
  if (this->tx_stall_since_ == 0) {
    this->tx_stall_since_ = now != 0 ? now : 1;
  } else if (!this->tx_stall_warned_ && now - this->tx_stall_since_ > TX_STALL_WARN_MS) {
    // Not an error by itself: the host is held back by the TCP window, nothing is lost.
    ESP_LOGW(TAG, "UART TX stalled for %us (radio holding CTS?); backpressuring client",
             (unsigned) (TX_STALL_WARN_MS / 1000));
    this->tx_stall_warned_ = true;
  }
}

void StreamServerComponent::log_trace_(const char *dir, const uint8_t *data, size_t len) {
  if (!this->trace_ || len == 0)
    return;
  char dump[3 * 32 + 1];
  const size_t dlen = std::min<size_t>(len, 32);
  for (size_t i = 0; i < dlen; i++)
    snprintf(&dump[i * 3], 4, "%02X ", data[i]);
  dump[dlen * 3] = 0;
  ESP_LOGD(TAG, "%s: %u bytes (%s)%s", dir, (unsigned) len, dump, len > dlen ? "..." : "");
}
