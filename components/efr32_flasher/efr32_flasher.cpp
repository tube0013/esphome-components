// ====== EFR32 Flasher (external component) ======
#include "esphome/components/efr32_flasher/efr32_flasher.h"
#include "esphome/components/efr32_info/ash_util.h"
#include <vector>
#include <string>
#include <cstring>
#include <algorithm>
#include <strings.h>
#include <cctype>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/md5/md5.h"
#include <ArduinoJson.h>

#ifdef USE_ESP_IDF
  #include "esp_http_client.h"
  #include "esp_crt_bundle.h"
#else
  #error "EFR32 flasher requires ESP-IDF framework."
#endif

namespace esphome { namespace efr32_flasher {

static const char *const TAG = "efr32_flasher";

// Forward declarations for ASH helpers
void write_escaped_(esphome::uart::UARTComponent *uart, const uint8_t *data, size_t len);
bool recv_ash_frame_(esphome::uart::UARTComponent *uart, std::vector<uint8_t> &out, uint32_t timeout_ms);
// Minimal data/ack send helpers (file-static implementations below)
static void ash_send_data_(esphome::uart::UARTComponent *uart, const std::vector<uint8_t> &ezsp_payload, uint8_t frm_num, uint8_t ack_num, bool re_tx);
static void ash_send_ack_(esphome::uart::UARTComponent *uart, uint8_t ack_num, bool ncp_ready);
static void ash_randomize_(uint8_t *data, size_t len);
std::vector<uint8_t> build_ezsp_get_version_();

// XMODEM constants
static constexpr uint8_t SOH = 0x01; // 128B
static constexpr uint8_t STX = 0x02; // 1024B (not used initially)
static constexpr uint8_t EOT = 0x04;
static constexpr uint8_t ACK = 0x06;
static constexpr uint8_t NAK = 0x15;
static constexpr uint8_t CAN = 0x18;
static constexpr uint8_t CCHR = 'C';

// HTTP event handler to accumulate body during esp_http_client_perform()
static esp_err_t http_event_handler(esp_http_client_event_t *evt){
  if (evt->event_id == HTTP_EVENT_ON_DATA && evt->user_data && evt->data && evt->data_len > 0){
    auto *body = reinterpret_cast<std::string*>(evt->user_data);
    body->append(reinterpret_cast<const char*>(evt->data), evt->data_len);
  }
  return ESP_OK;
}

EFR32Flasher::~EFR32Flasher() {}

void EFR32Flasher::setup(){
  // Avoid any UART activity at boot to prevent consuming ASH startup frames.
  // If needed, a manual probe action can be invoked later.
}

void EFR32Flasher::delay_(uint32_t ms){
  uint32_t start = millis();
  while (millis() - start < ms) { App.feed_wdt(); esphome::delay(1); }
}

bool EFR32Flasher::read_byte_(uint8_t &b, uint32_t timeout_ms){
  uint32_t start = millis();
  while (millis() - start < timeout_ms){
    if (uart_ && uart_->available()){
      if (uart_->read_byte(&b)) return true;
    }
    delay_(1);
  }
  return false;
}

bool EFR32Flasher::await_char_(uint8_t expect, uint32_t timeout_ms){
  uint8_t b=0;
  if(!read_byte_(b, timeout_ms)) return false;
  if (b == expect) return true;
  uint32_t end = millis() + timeout_ms;
  while (millis() < end){ if (uart_ && uart_->available() && uart_->read_byte(&b)) { if (b==expect) return true; } delay_(1); }
  return false;
}

void EFR32Flasher::flush_uart_(){
  uint8_t b;
  for(int i=0;i<64;i++){
    bool any=false;
    while(uart_ && uart_->available()){ uart_->read_byte(&b); any=true; App.feed_wdt(); }
    if(!any) break; delay_(2);
  }
}

bool EFR32Flasher::http_open_(const std::string &url, esp_http_client_handle_t &client, int timeout_ms){
  std::string cur = url;
  for (int redirects = 0; redirects < 5; redirects++){
    esp_http_client_config_t cfg = {};
    cfg.url = cur.c_str();
    cfg.timeout_ms = timeout_ms;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.disable_auto_redirect = true; // handle redirects manually for reliability
    client = esp_http_client_init(&cfg);
    if(!client) return false;
    if(esp_http_client_open(client, 0) != ESP_OK){ esp_http_client_cleanup(client); return false; }

    (void)esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if(status == 200){
      return true;
    }
    if (status == 301 || status == 302 || status == 303 || status == 307 || status == 308){
      // Try to follow Location header
      char *loc = nullptr;
      esp_http_client_get_header(client, "Location", &loc);
      ESP_LOGW(TAG, "HTTP redirect %d to: %s", status, (loc && *loc) ? loc : "<none>");
      esp_http_client_close(client); esp_http_client_cleanup(client);
      if (loc && *loc){ cur = loc; continue; }
      return false;
    }
    char first[256]; int n = esp_http_client_read(client, first, sizeof(first));
    ESP_LOGE(TAG,"HTTP status %d, first bytes: %.*s", status, n>0?n:0, first);
    esp_http_client_close(client); esp_http_client_cleanup(client);
    return false;
  }
  ESP_LOGE(TAG, "Too many HTTP redirects while fetching: %s", cur.c_str());
  return false;
}

bool EFR32Flasher::fetch_manifest_(const std::string &url, std::string &fw_url_out){
  // Use perform API to follow redirects automatically and accumulate full body
  std::string body; body.reserve(1024);
  esp_http_client_config_t cfg = {};
  cfg.url = url.c_str();
  cfg.timeout_ms = 15000;
  cfg.crt_bundle_attach = esp_crt_bundle_attach;
  cfg.disable_auto_redirect = false; // allow IDF to follow redirects
  cfg.event_handler = http_event_handler;
  cfg.user_data = &body;

  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) return false;
  esp_err_t perr = esp_http_client_perform(client);
  int status = esp_http_client_get_status_code(client);
  if (perr != ESP_OK || status != 200){
    ESP_LOGE(TAG, "Manifest HTTP perform failed: %s status=%d", esp_err_to_name(perr), status);
    esp_http_client_cleanup(client);
    return false;
  }
  esp_http_client_cleanup(client);

  JsonDocument doc;
  auto err = deserializeJson(doc, body);
  if(err){ ESP_LOGE(TAG,"JSON parse error: %s", err.c_str()); return false; }

  fw_url_out.clear(); expected_md5_.clear(); expected_size_=0; manifest_version_.clear();

  // Try variants object: choose based on forced variant or autodetected key
  if (doc["variants"].is<JsonObjectConst>()){
    auto obj = doc["variants"].as<JsonObjectConst>();
    std::string key;
    if (!variant_key_override_.empty()) key = variant_key_override_;
    else if (variant_force_ == 1) key = "MGM24";
    else if (variant_force_ == 2) key = "BM24";

    if (!key.empty()){
      auto v = obj[key.c_str()];
      if (!v.isNull()){
        if (v["fw_url"].is<const char*>()) fw_url_out = v["fw_url"].as<const char*>();
        if (v["md5"].is<const char*>()) expected_md5_ = v["md5"].as<const char*>();
        if (v["size"].is<uint32_t>()) expected_size_ = v["size"].as<uint32_t>();
        ESP_LOGI(TAG, "Manifest variant '%s' selected", key.c_str());
      } else {
        ESP_LOGW(TAG, "Manifest does not provide variant '%s'", key.c_str());
      }
    }

    if (fw_url_out.empty() && !obj.isNull() && obj.size() > 0){
      ESP_LOGW(TAG, "Manifest has %u variants but none matched autodetect/override; defaulting to first entry",
               (unsigned) obj.size());
      for (auto kv : obj) {
        auto v = kv.value();
        if (v["fw_url"].is<const char*>()) fw_url_out = v["fw_url"].as<const char*>();
        if (v["md5"].is<const char*>()) expected_md5_ = v["md5"].as<const char*>();
        if (v["size"].is<uint32_t>()) expected_size_ = v["size"].as<uint32_t>();
        ESP_LOGI(TAG, "Using manifest variant '%s' as fallback", kv.key().c_str());
        break;
      }
    }
  }
  if (fw_url_out.empty()){
    if (doc["fw_url"].is<const char*>()) fw_url_out = doc["fw_url"].as<const char*>();
    else if (doc["firmware_url"].is<const char*>()) fw_url_out = doc["firmware_url"].as<const char*>();
    else if (doc["url"].is<const char*>()) fw_url_out = doc["url"].as<const char*>();
  }
  // Optional metadata
  if (expected_md5_.empty() && doc["md5"].is<const char*>()) expected_md5_ = doc["md5"].as<const char*>();
  if (!expected_size_ && doc["size"].is<uint32_t>()) expected_size_ = doc["size"].as<uint32_t>();
  if (doc["version"].is<const char*>()) manifest_version_ = doc["version"].as<const char*>();

  if (fw_url_out.empty()) { ESP_LOGE(TAG, "Manifest missing firmware URL."); return false; }
  if (!expected_md5_.empty()) ESP_LOGI(TAG, "Manifest MD5=%s", expected_md5_.c_str());
  if (expected_size_) ESP_LOGI(TAG, "Manifest size=%u", (unsigned)expected_size_);
  if (!manifest_version_.empty()) ESP_LOGI(TAG, "Manifest version=%s", manifest_version_.c_str());
  return true;
}

void EFR32Flasher::enter_bootloader_(){
  if (!bl_sw_ || !rst_sw_) return;
  ESP_LOGI(TAG, "Entering Gecko bootloader (BL+RST)…");
  // Mirror the proven sequence used in your YAML script:
  // 1) Assert boot pin
  // 2) Hold ~1s, then assert reset for ~1s
  // 3) Release reset, keep boot asserted for ~5s
  // 4) Release boot
  bl_sw_->turn_on();           // inverted: drives boot pin active
  delay_(1000);
  rst_sw_->turn_on();          // inverted: drives reset active (low)
  delay_(1000);
  rst_sw_->turn_off();         // release reset
  delay_(5000);
  bl_sw_->turn_off();          // release boot pin
  delay_(200);
  // Clear any spuriously buffered bytes before menu selection
  flush_uart_();
}

void EFR32Flasher::leave_bootloader_(){
  if (!rst_sw_) return;
  ESP_LOGD(TAG, "Exiting bootloader via reset…");
  rst_sw_->turn_on(); delay_(15); rst_sw_->turn_off();
}

bool EFR32Flasher::xmodem_send_(esp_http_client_handle_t client, uint32_t content_len){
  // XMODEM-CRC 128-byte blocks
  std::vector<uint8_t> net_buf(1024);
  std::vector<uint8_t> block(128, 0x1A);
  uint32_t sent = 0; uint8_t seq = 1; uint32_t last_pc=0;
  esphome::md5::MD5Digest md5; md5.init();

  ESP_LOGD(TAG, "Waiting for receiver 'C'…");
  uint32_t tstart = millis();
  bool got_c = false;
  uint32_t last_cr = 0;
  bool resent_menu_6s = false;
  bool resent_menu_12s = false;
  while (millis() - tstart < 15000){
    uint8_t b;
    if (read_byte_(b, 250) && b == CCHR) { got_c = true; break; }
    uint32_t elapsed = millis() - tstart;
    if (elapsed >= 3000 && (elapsed - last_cr) >= 2000) {
      ESP_LOGD(TAG, "No 'C' yet (~%us), sending CR to prompt bootloader", (unsigned)(elapsed/1000));
      uart_->write_byte('\r');
      last_cr = elapsed;
    }
    if (!resent_menu_6s && elapsed >= 6000) {
      ESP_LOGD(TAG, "No 'C' yet (~6s), re-sending '1'+CR to select upload");
      uart_->write_byte('1');
      uart_->write_byte('\r');
      resent_menu_6s = true;
    }
    if (!resent_menu_12s && elapsed >= 12000) {
      ESP_LOGD(TAG, "No 'C' yet (~12s), re-sending '1'+CR to select upload");
      uart_->write_byte('1');
      uart_->write_byte('\r');
      resent_menu_12s = true;
    }
  }
  if (!got_c){
    ESP_LOGE(TAG, "Receiver did not send 'C' to start XMODEM");
    return false;
  }

  while (sent < content_len){
    // Fill a 128-byte payload from network
    size_t need = 128; size_t off=0;
    while (off < need){
      int r = esp_http_client_read(client, (char*)net_buf.data(), std::min((size_t)net_buf.size(), need-off));
      if (r < 0) return false;
      if (r == 0) break; // EOF
    std::memcpy(block.data()+off, net_buf.data(), r);
      off += r; sent += r; App.feed_wdt();
    }
    if (off == 0) break; // no more data
    if (off < 128){ std::fill(block.begin()+off, block.end(), 0x1A); }

    // Build and send XMODEM frame
    uint8_t hdr[3] = { SOH, seq, (uint8_t)(0xFF - seq) };
    uint16_t crc = crc16_ccitt_(block.data(), 128);

    uart_->write_array(hdr, 3);
    uart_->write_array(block.data(), 128);
    uint8_t crc_be[2] = { (uint8_t)(crc>>8), (uint8_t)(crc & 0xFF) };
    uart_->write_array(crc_be, 2);

    // Await ACK/NAK
    uint8_t resp=0; uint32_t wait_ms=3000; uint32_t start=millis(); bool ok=false;
    while (millis() - start < wait_ms){ if (read_byte_(resp, 250)) { if(resp==ACK){ ok=true; break; } if(resp==NAK){ ok=false; break; } if(resp==CAN){ ESP_LOGE(TAG, "Receiver cancelled (CAN)"); return false; } } }
    if(!ok){ ESP_LOGW(TAG, "NAK or timeout on block %u, retrying…", (unsigned)seq); continue; }
    // Update MD5 only for the actual payload size read from network (exclude padding)
    if (off > 0) md5.add(block.data(), off);
    seq++;
    update_progress_(sent, content_len, last_pc);
  }

  // Send EOT
  uart_->write_byte(EOT);
  uint8_t resp=0; if(!read_byte_(resp, 2000) || resp!=ACK){ ESP_LOGE(TAG, "No ACK after EOT"); return false; }
  md5.calculate(); char md5hex[33]; md5.get_hex(md5hex); md5hex[32]=0;
  if (!expected_md5_.empty() && expected_md5_.size()==32 && strcasecmp(expected_md5_.c_str(), md5hex)!=0){
    ESP_LOGE(TAG, "MD5 mismatch: expected %s, got %s", expected_md5_.c_str(), md5hex);
    return false;
  }
  ESP_LOGI(TAG, "XMODEM transfer complete. bytes=%u md5=%s", (unsigned)sent, md5hex);
  return true;
}

void EFR32Flasher::update_progress_(uint32_t total, uint32_t expected, uint32_t &last_pc){
  if(!show_progress_) return;
  if(expected>0){ uint32_t pc=(uint64_t)total*100/expected; if(pc>=last_pc+progress_step_){ last_pc=pc-(pc%progress_step_); ESP_LOGI(TAG,"Progress: %u%% (%u/%u)",(unsigned)pc,(unsigned)total,(unsigned)expected);} }
  else { if(total/65536>last_pc){ last_pc=total/65536; ESP_LOGI(TAG,"Progress: %u bytes",(unsigned)total);} }
}

void EFR32Flasher::run_update_(){
  if(!uart_ || !bl_sw_ || !rst_sw_){ ESP_LOGE(TAG, "Not configured (uart/switches)"); return; }
  variant_key_override_.clear();
  if (variant_force_ == 0){
    variant_key_override_ = detect_variant_key_();
    if (variant_key_override_.empty()){
      ESP_LOGW(TAG, "Auto variant detection did not yield a match; using manifest fallback");
    }
  }
  set_busy_(true);
  std::string fw_url;
  if (!fetch_manifest_(manifest_url_, fw_url)) { ESP_LOGE(TAG, "Manifest fetch/parse failed"); set_busy_(false); return; }
  ESP_LOGI(TAG, "Firmware: %s", fw_url.c_str());

  esp_http_client_handle_t client;
  if(!http_open_(fw_url, client)) { ESP_LOGE(TAG, "Firmware HTTP open failed"); set_busy_(false); return; }
  int64_t len64 = esp_http_client_get_content_length(client);
  uint32_t content_len = len64 > 0 ? (uint32_t)len64 : 0;

  enter_bootloader_();
  // Allow BL banner/prompt to appear, then select '1' (upload gbl)
  delay_(1000);
  // Nudge BL prompt
  uart_->write_byte('\r');
  delay_(200);
  flush_uart_();
  ESP_LOGD(TAG, "Selecting 'upload gbl' (sending '1' + CR)");
  uart_->write_byte('1');
  uart_->write_byte('\r');
  delay_(200);

  bool ok = xmodem_send_(client, content_len);

  esp_http_client_close(client); esp_http_client_cleanup(client);

  leave_bootloader_();
  if (ok) {
    ESP_LOGI(TAG, "EFR32 update finished OK. Waiting for NCP start marker…");
    if (wait_for_ncp_start_(1500)) {
      ESP_LOGI(TAG, "NCP start marker {~ detected.");
    } else {
      ESP_LOGW(TAG, "No NCP start marker seen in 1.5s (firmware may still boot later).");
    }
  } else {
    ESP_LOGE(TAG, "EFR32 update failed");
  }
  set_busy_(false);
}

void EFR32Flasher::run_check_update_(){
  ESP_LOGI(TAG, "Checking EFR32 firmware manifest…");
  if (variant_force_ == 0 && variant_key_override_.empty()){
    variant_key_override_ = detect_variant_key_();
    if (variant_key_override_.empty()){
      ESP_LOGW(TAG, "Auto variant detection did not yield a match; using manifest fallback");
    }
  }
  std::string fw_url;
  if (!fetch_manifest_(manifest_url_, fw_url)) { ESP_LOGE(TAG, "Manifest fetch/parse failed"); return; }
  if (!manifest_version_.empty() && latest_text_) latest_text_->publish_state(manifest_version_.c_str());
}

#if 0  // legacy probing removed; handled by efr32_tools

// Legacy probe_info removed

// Non-blocking probe-lite: send ASH RST and try to capture a single RSTACK frame.
/* void EFR32Flasher::step_probe_lite_(){ // removed
  switch (pl_state_) {
    case PL_IDLE:
      if (!want_probe_lite_) return;
      want_probe_lite_ = false;
      ESP_LOGI(TAG, "probe_lite: begin (RST -> RSTACK, non-blocking)");
      set_busy_(true);
      pl_settle_until_ = millis() + 500;  // slightly longer settle for pause switch
      pl_state_ = PL_SETTLE;
      break;

    case PL_PAUSE_SS:
      // Unused (kept for possible future refinement)
      pl_state_ = PL_SETTLE;
      break;

    case PL_SETTLE:
      if (millis() < pl_settle_until_) break;
      // minimal UART cleanup; keep quick
      flush_uart_();
      pl_state_ = PL_SEND_RST;
      break;

    case PL_SEND_RST: {
      if (!uart_) { pl_state_ = PL_RESUME_SS; break; }
      const uint8_t rst_frame[] = { 0x1A, 0xC0, 0x38, 0xBC, 0x7E };
      uart_->write_array(rst_frame, sizeof(rst_frame));
      // Start capturing immediately; some RSTACKs may already have emitted the start flag.
      pl_in_frame_ = true; pl_esc_ = false; pl_buf_.clear();
      pl_deadline_ = millis() + 1500; // extend capture window; still in small loop slices
      pl_state_ = PL_WAIT_RSTACK;
      break;
    }

    case PL_WAIT_RSTACK: {
      if (!uart_) { pl_state_ = PL_RESUME_SS; break; }
      // Consume only what is available now; stop at FLAG-terminated frame
      for (int i = 0; i < 48; i++) {  // small chunk each loop
        uint8_t b;
        if (!(uart_->available() && uart_->read_byte(&b))) break;
        if (b == 0x7E) { // FLAG
          if (pl_in_frame_) {
            // End of frame; drop trailing FCS (2B) if present
            if (pl_buf_.size() >= 2) pl_buf_.resize(pl_buf_.size() - 2);
            // Try to extract version: search for 0xC1 <ver>
            if (ezsp_protocol_text_) {
              uint8_t verb = 0;
              bool found = false;
              for (size_t j = 0; j + 1 < pl_buf_.size(); j++) {
                if (pl_buf_[j] == 0xC1) { verb = pl_buf_[j+1]; found = true; break; }
              }
              if (found) {
                char ver[16]; snprintf(ver, sizeof(ver), "ASHv%u", (unsigned) verb);
                ezsp_protocol_text_->publish_state(ver);
                ESP_LOGI(TAG, "probe_lite: captured RSTACK, version=%u", (unsigned) verb);
              } else {
                ESP_LOGD(TAG, "probe_lite: ASH frame captured (len=%u), version tag not found", (unsigned) pl_buf_.size());
              }
            }
            pl_state_ = PL_RESUME_SS;
            break;
          } else {
            pl_in_frame_ = true; pl_esc_ = false; pl_buf_.clear();
          }
          continue;
        }
        if (!pl_in_frame_) continue;
        if (pl_esc_) { pl_buf_.push_back(b ^ 0x20); pl_esc_ = false; }
        else if (b == 0x7D) { pl_esc_ = true; }
        else { pl_buf_.push_back(b); }
      }
      if (millis() >= pl_deadline_) {
        ESP_LOGW(TAG, "probe_lite: no ASH frame within window");
        pl_state_ = PL_RESUME_SS;
      }
      break;
    }

    case PL_RESUME_SS:
      pl_state_ = PL_DONE;
      break;

    case PL_DONE:
      // one-shot; return to idle and release busy flag
      set_busy_(false);
      pl_state_ = PL_IDLE;
      break;
  }
} */

// Probe EZSP version: minimal ASH exchange to request EZSP version, publish response.
/* void EFR32Flasher::step_probe_ezsp_version_(){ // removed
  switch (pv_state_) {
    case PV_IDLE:
      if (!want_probe_ezsp_ver_) return;
      want_probe_ezsp_ver_ = false;
      ESP_LOGI(TAG, "probe_ezsp_version: begin (non-blocking)");
      set_busy_(true);
      // Give YAML on_press automation time to pause stream_server
      pv_deadline_ = millis() + 500; // base settle window
      pv_quiet_start_ = millis();
      pv_cap_until_ = millis() + 1200; // absolute cap
      pv_state_ = PV_BUSY_ON;
      break;
    case PV_BUSY_ON:
      // Track quiet time on UART; reset quiet start if bytes are flowing
      if (uart_ && uart_->available()) pv_quiet_start_ = millis();
      if (millis() < pv_deadline_) break; // initial settle not elapsed yet
      // Require ~250ms of quiet UART or hit the cap before proceeding
      if ((millis() - pv_quiet_start_) < 250 && millis() < pv_cap_until_) break;
      pv_state_ = PV_FLUSH;
      break;
    case PV_FLUSH:
      flush_uart_();
      { const uint8_t rst[] = {0x1A,0xC0,0x38,0xBC,0x7E}; uart_->write_array(rst, sizeof(rst)); }
      pv_deadline_ = millis() + 800;
      pv_state_ = PV_WAIT_RSTACK;
      break;
    case PV_WAIT_RSTACK: {
      std::vector<uint8_t> fr;
      if (recv_ash_frame_(uart_, fr, 80)) {
        // Extract ASH version and publish
        if (!fr.empty() && ezsp_protocol_text_) {
          uint8_t verb = 0; bool found=false;
          for (size_t j=0;j+1<fr.size();j++){ if (fr[j]==0xC1){ verb=fr[j+1]; found=true; break; } }
          if (found){ char ver[16]; snprintf(ver,sizeof(ver),"ASHv%u",(unsigned)verb); ezsp_protocol_text_->publish_state(ver); }
        }
        // Send a minimal Supervisory ACK control to complete link after RSTACK.
        { ash_send_ack_(uart_, /*ack_num=*/0, /*ncp_ready=*/false); delay_(20); }

        // Move on to sending EZSP getVersion (v7 first, then v8 fallback)
        pv_state_ = PV_SEND_GETVER;
      } else if (millis() >= pv_deadline_) {
        pv_state_ = PV_SEND_GETVER;
      }
      break; }
    case PV_SEND_GETVER: {
      // Send EZSP v8-style getValue(VERSION_INFO)
      {
        std::vector<uint8_t> ez = build_ezsp_get_version_();
        ash_send_data_(uart_, ez, /*frm_num=*/0, /*ack_num=*/0, /*re_tx=*/false);
      }
      pv_deadline_ = millis() + 800;
      pv_state_ = PV_WAIT_REPLY;
      break; }
    case PV_WAIT_REPLY: {
      // Try to read one DATA frame; skip ACK/CTRL-only frames (e.g., 0xA0)
      std::vector<uint8_t> fr;
      uint32_t until = millis() + 800;
      bool got_data = false;
      while (millis() < until){
        if (!recv_ash_frame_(uart_, fr, 60)) { delay_(10); continue; }
        if (fr.empty()) { continue; }
        // DATA frames have first byte <= 0x1F; control/ACK frames have >= 0x80
        if (fr[0] > 0x1F) { continue; }
        got_data = true;
        break;
      }
      if (got_data){
        // De-randomize EZSP payload before sampling (skip ASH control byte)
        std::vector<uint8_t> ez(fr.begin() + 1, fr.end());
        if (!ez.empty()) ash_randomize_(ez.data(), ez.size());
        if (xncp_version_text_ || installed_stack_text_) {
          char sample[64]; size_t dlen = ez.size() > 12 ? 12 : ez.size(); size_t pos=0;
          for (size_t i=0;i<dlen && pos+3<sizeof(sample);i++) pos += snprintf(sample+pos, sizeof(sample)-pos, "%02X", ez[i]);
          sample[(pos < sizeof(sample)) ? pos : (sizeof(sample)-1)]='\0';
          char buf[96]; snprintf(buf, sizeof(buf), "EZSPv7 ezlen=%u hex=%s%s", (unsigned) ez.size(), sample, ez.size()>dlen?"…":"");
          if (xncp_version_text_) xncp_version_text_->publish_state(buf);
          if (installed_stack_text_) installed_stack_text_->publish_state(buf);
        }
        pv_state_ = PV_DONE; break;
      }
      if (millis() >= pv_deadline_) {
        // Retry once: send again
        {
          std::vector<uint8_t> ez = build_ezsp_get_version_();
          ash_send_data_(uart_, ez, /*frm_num=*/0, /*ack_num=*/0, /*re_tx=*/true);
        }
        // wait briefly for any DATA reply
        uint32_t until2 = millis() + 700;
        while (millis() < until2){
          if (!recv_ash_frame_(uart_, fr, 60)) { delay_(10); continue; }
          if (fr.empty() || fr[0] > 0x1F) { continue; }
          if (xncp_version_text_ || installed_stack_text_) {
            std::vector<uint8_t> ez(fr.begin() + 1, fr.end());
            if (!ez.empty()) ash_randomize_(ez.data(), ez.size());
            char sample[64]; size_t dlen = ez.size() > 12 ? 12 : ez.size(); size_t pos=0;
            for (size_t i=0;i<dlen && pos+3<sizeof(sample);i++) pos += snprintf(sample+pos, sizeof(sample)-pos, "%02X", ez[i]);
            sample[(pos < sizeof(sample)) ? pos : (sizeof(sample)-1)]='\0';
            char buf[96]; snprintf(buf, sizeof(buf), "EZSPv8 ezlen=%u hex=%s%s", (unsigned) ez.size(), sample, ez.size()>dlen?"…":"");
            if (xncp_version_text_) xncp_version_text_->publish_state(buf);
            if (installed_stack_text_) installed_stack_text_->publish_state(buf);
          }
          break;
        }
        pv_state_ = PV_DONE;
      }
      break; }
    case PV_DONE:
      ESP_LOGD(TAG, "probe_ezsp_version: done");
      set_busy_(false);
      pv_state_ = PV_IDLE;
      break;
  }
} */
#endif  // legacy probing

void EFR32Flasher::loop(){
  if (want_check_){ want_check_ = false; run_check_update_(); return; }
  if (!want_update_) return; want_update_ = false; run_update_();
}

std::string EFR32Flasher::detect_variant_key_(){
  auto normalize = [](const std::string &input) {
    std::string norm;
    norm.reserve(input.size());
    for (char ch : input) {
      if (ch == ' ' || ch == '-' || ch == '_' || ch == '/' || ch == '\\' || ch == '.')
        continue;
      norm.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
    }
    return norm;
  };

  auto match_from_source = [&](const char *label, esphome::text_sensor::TextSensor *sensor) -> std::string {
    if (sensor == nullptr)
      return {};
    std::string value = sensor->get_state();
    if (value.empty())
      return {};
    std::string norm = normalize(value);
    if (norm.empty())
      return {};

    ESP_LOGD(TAG, "Variant probe %s='%s' (norm='%s')", label, value.c_str(), norm.c_str());

    if (norm.find("BMMG24") != std::string::npos || norm.find("BM24") != std::string::npos) {
      ESP_LOGI(TAG, "Variant detected as BM24 via %s='%s'", label, value.c_str());
      return "BM24";
    }
    if (norm.find("MGM24") != std::string::npos || norm.find("MG24") != std::string::npos) {
      ESP_LOGI(TAG, "Variant detected as MGM24 via %s='%s'", label, value.c_str());
      return "MGM24";
    }
    return {};
  };

  if (auto key = match_from_source("board_name", board_name_text_); !key.empty())
    return key;
  if (auto key = match_from_source("mfg_string", mfg_string_text_); !key.empty())
    return key;
  if (auto key = match_from_source("manufacturer", manuf_id_text_); !key.empty())
    return key;
  if (auto key = match_from_source("chip", chip_text_); !key.empty())
    return key;

  ESP_LOGW(TAG, "Unable to determine EFR32 variant from probed text sensors (board/mfg/manufacturer)");
  return {};
}

bool EFR32Flasher::wait_for_ncp_start_(uint32_t ms){
  uint32_t t0 = esphome::millis();
  uint8_t prev = 0;
  while (millis() - t0 < ms){
    uint8_t b;
    if (uart_ && uart_->available() && uart_->read_byte(&b)){
      if (prev == '{' && b == '~') return true;
      prev = b;
    } else {
      delay_(2);
    }
  }
  return false;
}

} } // namespace
// PPP FCS-16 (used by ASH). Polynomial 0x8408, init 0xFFFF, output one's complement, LSB first.
// CRC-HQX (poly 0x1021, init 0xFFFF), big-endian append, as used by bellows' ASH
static uint16_t ash_crc_hqx_be_(const uint8_t *data, size_t len) {
  return esphome::efr32::ash::crc_hqx_be(data, len);
}

void esphome::efr32_flasher::write_escaped_(esphome::uart::UARTComponent *uart, const uint8_t *data, size_t len) {
  esphome::efr32::ash::write_escaped(uart, data, len);
}

// Build and send a generic ASH frame (control+data already prepared)
namespace esphome { namespace efr32_flasher {
static void ash_send_frame_(esphome::uart::UARTComponent *uart, const uint8_t *frame, size_t len) {
  if (uart == nullptr)
    return;
  std::vector<uint8_t> tmp(frame, frame + len);
  uint16_t crc = ash_crc_hqx_be_(frame, len);
  tmp.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  tmp.push_back(static_cast<uint8_t>(crc & 0xFF));
  uart->write_byte(esphome::efr32::ash::FLAG);
  esphome::efr32_flasher::write_escaped_(uart, tmp.data(), tmp.size());
  uart->write_byte(esphome::efr32::ash::FLAG);
}

// Pseudo-random data sequence generator (same algorithm as bellows)
static void ash_randomize_(uint8_t *data, size_t len) { esphome::efr32::ash::randomize(data, len); }
// Send a DATA frame with minimal sequencing (frm_num=0, re_tx=0, ack_num=0 by default)
static void ash_send_data_(esphome::uart::UARTComponent *uart, const std::vector<uint8_t> &ezsp_payload, uint8_t frm_num, uint8_t ack_num, bool re_tx) {
  if (uart == nullptr)
    return;
  std::vector<uint8_t> buf;
  uint8_t ctrl = static_cast<uint8_t>(((frm_num & 0x07) << 4) | (re_tx ? 0x08 : 0x00) | (ack_num & 0x07));
  buf.push_back(ctrl);
  std::vector<uint8_t> rnd(ezsp_payload);
  if (!rnd.empty())
    ash_randomize_(rnd.data(), rnd.size());
  buf.insert(buf.end(), rnd.begin(), rnd.end());
  ash_send_frame_(uart, buf.data(), buf.size());
}
// Send an ACK frame (supervisory, not-extended)
static void ash_send_ack_(esphome::uart::UARTComponent *uart, uint8_t ack_num, bool ncp_ready) {
  uint8_t ctrl = static_cast<uint8_t>(0x80 | (ncp_ready ? 0x08 : 0x00) | (ack_num & 0x07));
  ash_send_frame_(uart, &ctrl, 1);
}
} } // namespace
