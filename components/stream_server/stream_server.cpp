#include "stream_server.h"

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/version.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

#include <algorithm>
#include <cerrno>
#include <cstring>

static const char *TAG = "stream_server";

namespace {
bool is_retryable_socket_error(int err) {
    return err == EWOULDBLOCK || err == EAGAIN || err == EINTR;
}
}  // namespace

using namespace esphome;

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    // The make_unique() wrapper doesn't like arrays, so initialize the unique_ptr directly.
    this->buf_ = std::unique_ptr<uint8_t[]>{new uint8_t[this->buf_size_]};

    struct sockaddr_storage bind_addr;
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2023, 4, 0)
    socklen_t bind_addrlen = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), this->port_);
#else
    socklen_t bind_addrlen = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), htons(this->port_));
#endif

    this->socket_ = socket::socket_ip(SOCK_STREAM, PF_INET);
    this->socket_->setblocking(false);
    this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), bind_addrlen);
    this->socket_->listen(8);

    this->publish_sensor();
}

void StreamServerComponent::loop() {
    App.feed_wdt();
    if (this->paused_) {
        // While paused, do not interact with UART or accept new clients.
        this->cleanup();
        return;
    }
    this->accept();
    this->read();
    App.feed_wdt();
    this->flush();
    App.feed_wdt();
    this->write();
    App.feed_wdt();
    this->cleanup();
}

void StreamServerComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stream Server:");
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 7, 0)
    char use_address_buf[esphome::network::USE_ADDRESS_BUFFER_SIZE];
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_use_address_to(use_address_buf), this->port_);
#elif ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_use_address(), this->port_);
#else
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_use_address().c_str(), this->port_);
#endif
#ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "Connected:", this->connected_sensor_);
#endif
}

void StreamServerComponent::on_shutdown() {
    if (this->client_.has_value())
        this->close_client_(*this->client_);
    this->client_.reset();
}

void StreamServerComponent::close_client_(Client &client) {
    if (client.socket) {
        client.socket->shutdown(SHUT_RDWR);
        client.socket->close();
        client.socket.reset();
    }
    client.disconnected = true;
}

void StreamServerComponent::publish_sensor() {
#ifdef USE_BINARY_SENSOR
    if (this->connected_sensor_)
        this->connected_sensor_->publish_state(this->client_.has_value());
#endif
}

void StreamServerComponent::accept() {
    struct sockaddr_storage client_addr;
    socklen_t client_addrlen = sizeof(client_addr);
    std::unique_ptr<socket::Socket> socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
        return;

    {
        int fd = socket->get_fd();
        int val = 1;
        // Disable Nagle algorithm to prevent the Nagle+delayed-ACK deadlock on Ethernet-connected
        // Linux hosts (HA server). Without TCP_NODELAY, Linux delayed ACK (up to 40ms) stalls small
        // Zigbee/Z-Wave frames: the ESP waits for ACK (Nagle) while Linux waits to piggyback ACK,
        // causing per-frame stalls that break coordinator timing. Critical at all baud rates;
        // more severe at 460800/921600 baud with EFR32 NCP/RCP firmware.
        if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val)) != 0)
            ESP_LOGW(TAG, "setsockopt TCP_NODELAY failed: %s", strerror(errno));

        if (this->keep_alive_idle_s_ > 0) {
            if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val)) != 0) {
                ESP_LOGW(TAG, "setsockopt SO_KEEPALIVE failed: %s", strerror(errno));
            } else {
                val = this->keep_alive_idle_s_;
                if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &val, sizeof(val)) != 0)
                    ESP_LOGW(TAG, "setsockopt TCP_KEEPIDLE failed: %s", strerror(errno));
                val = this->keep_alive_interval_s_;
                if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &val, sizeof(val)) != 0)
                    ESP_LOGW(TAG, "setsockopt TCP_KEEPINTVL failed: %s", strerror(errno));
                val = this->keep_alive_count_;
                if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &val, sizeof(val)) != 0)
                    ESP_LOGW(TAG, "setsockopt TCP_KEEPCNT failed: %s", strerror(errno));
            }
        }
    }

    socket->setblocking(false);

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 1, 0)
    std::string identifier = std::string{esphome::socket::SOCKADDR_STR_LEN, 0};
    auto identifier_span = std::span<char, esphome::socket::SOCKADDR_STR_LEN>(identifier.data(), identifier.size());
    identifier.resize(socket->getpeername_to(identifier_span));
#else
    std::string identifier = socket->getpeername();
#endif

    if (this->client_.has_value()) {
        ESP_LOGW(TAG, "Closing existing client %s for new connection from %s",
                 this->client_->identifier.c_str(), identifier.c_str());
        this->close_client_(*this->client_);
        this->client_.reset();
        this->buf_tail_ = this->buf_head_;
    }

    this->client_.emplace(std::move(socket), identifier, this->buf_head_);
    ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());
    this->publish_sensor();
}

void StreamServerComponent::cleanup() {
    if (this->client_.has_value() && this->client_->disconnected) {
        this->client_.reset();
        this->buf_tail_ = this->buf_head_;
        this->publish_sensor();
    }
}

void StreamServerComponent::read() {
    size_t len = 0;
    int available;
    while ((available = this->stream_->available()) > 0) {
        App.feed_wdt();
        size_t free = this->buf_size_ - (this->buf_head_ - this->buf_tail_);
        if (free == 0) {
            // Only overwrite if nothing has been added yet, otherwise give flush() a chance to empty the buffer first.
            if (len > 0)
                return;

            ESP_LOGE(TAG, "Incoming bytes available, but outgoing buffer is full: stream will be corrupted!");
            free = std::min<size_t>(available, this->buf_size_);
            this->buf_tail_ += free;
            if (this->client_.has_value() && this->client_->position < this->buf_tail_) {
                ESP_LOGW(TAG, "Dropped %u pending bytes for client %s",
                         this->buf_tail_ - this->client_->position, this->client_->identifier.c_str());
                this->client_->position = this->buf_tail_;
            }
        }

        // Fill all available contiguous space in the ring buffer.
        len = std::min<size_t>(available, std::min<size_t>(this->buf_ahead(this->buf_head_), free));
        size_t start = this->buf_index(this->buf_head_);
        this->stream_->read_array(&this->buf_[start], len);
        if (this->trace_ && len > 0) {
            char dump[3 * 32 + 1];
            size_t dlen = len > 32 ? 32 : len;
            for (size_t i = 0; i < dlen; i++) snprintf(&dump[i * 3], 4, "%02X ", this->buf_[start + i]);
            dump[dlen * 3] = 0;
            ESP_LOGD(TAG, "UART -> ring: %u bytes (sample: %s)%s", (unsigned) len, dump, len > dlen ? "…" : "");
        }
        this->buf_head_ += len;
    }
}

void StreamServerComponent::flush() {
    if (!this->client_.has_value() || this->client_->disconnected ||
            this->client_->position == this->buf_head_) {
        this->buf_tail_ = this->buf_head_;
        return;
    }

    Client &client = *this->client_;
    App.feed_wdt();

    // Split the write into two parts: from the current position to the end of the ring buffer, and from the start
    // of the ring buffer until the head. The second part might be zero if no wraparound is necessary.
    struct iovec iov[2];
    iov[0].iov_base = &this->buf_[this->buf_index(client.position)];
    iov[0].iov_len = std::min(this->buf_head_ - client.position, this->buf_ahead(client.position));
    iov[1].iov_base = &this->buf_[0];
    iov[1].iov_len = this->buf_head_ - (client.position + iov[0].iov_len);
    ssize_t written = client.socket->writev(iov, 2);
    if (written > 0) {
        client.position += written;
        if (this->trace_)
            ESP_LOGD(TAG, "ring -> client %s: %d bytes", client.identifier.c_str(), (int) written);
    } else if (written == 0) {
        ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
        this->close_client_(client);
    } else {
        int err = errno;
        if (!is_retryable_socket_error(err)) {
            ESP_LOGE(TAG, "Failed to write to client %s with error %d, closing socket", client.identifier.c_str(), err);
            this->close_client_(client);
        }
    }

    // If the client disconnected this flush, free the ring buffer; otherwise hold it at the client's position.
    this->buf_tail_ = client.disconnected ? this->buf_head_ : client.position;
}

void StreamServerComponent::write() {
    if (!this->client_.has_value() || this->client_->disconnected)
        return;

    Client &client = *this->client_;
    App.feed_wdt();

    uint8_t buf[128];
    ssize_t nread;
    while ((nread = client.socket->read(buf, sizeof(buf))) > 0) {
        App.feed_wdt();
        if (this->trace_) {
            char dump[3 * 32 + 1];
            size_t dlen = nread > 32 ? 32 : (size_t) nread;
            for (size_t i = 0; i < dlen; i++) snprintf(&dump[i * 3], 4, "%02X ", buf[i]);
            dump[dlen * 3] = 0;
            ESP_LOGD(TAG, "TCP <- client %s: %d bytes (sample: %s)%s", client.identifier.c_str(), (int) nread, dump, nread > (ssize_t) dlen ? "…" : "");
        }
        this->stream_->write_array(buf, nread);
    }

    if (nread == 0) {
        ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
        this->close_client_(client);
    } else if (nread < 0) {
        int err = errno;
        if (!is_retryable_socket_error(err)) {
            ESP_LOGW(TAG, "Failed to read from client %s with error %d, closing socket", client.identifier.c_str(), err);
            this->close_client_(client);
        }
    }
}

StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier, size_t position)
    : socket(std::move(socket)), identifier{identifier}, position{position} {}

void StreamServerComponent::pause() {
    if (this->paused_)
        return;
    this->paused_ = true;
    ESP_LOGI(TAG, "Pausing stream server and disconnecting clients");
    if (this->client_.has_value())
        this->close_client_(*this->client_);
    this->cleanup();
}

void StreamServerComponent::resume() {
    if (!this->paused_)
        return;
    ESP_LOGI(TAG, "Resuming stream server");
    this->paused_ = false;
}
