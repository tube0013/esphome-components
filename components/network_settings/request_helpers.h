#pragma once
#include "request_security.h"
#include "esphome/components/json/json_util.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/helpers.h"
#include <cstdio>
#include <string>

namespace esphome::network_settings::web {
inline std::string make_nonce() {
  char value[33];
  snprintf(value, sizeof(value), "%08x%08x%08x%08x", unsigned(random_uint32()), unsigned(random_uint32()),
           unsigned(random_uint32()), unsigned(random_uint32()));
  return value;
}
inline bool authorized(AsyncWebServerRequest *request, const char *header, const std::string &nonce) {
  const auto token = request->get_header(header);
  const auto origin = request->get_header("Origin");
  const auto host = request->get_header("Host");
  return authorized(nonce, token.has_value() ? std::optional<std::string_view>(*token) : std::nullopt,
                    origin.has_value() ? std::optional<std::string_view>(*origin) : std::nullopt,
                    host.has_value() ? std::optional<std::string_view>(*host) : std::nullopt);
}
inline void send_json(AsyncWebServerRequest *request, int code, const std::string &body) {
  auto *response = request->beginResponse(code, "application/json", body);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}
inline void send_error(AsyncWebServerRequest *request, int code, const char *message) {
  auto body = json::build_json([message](JsonObject root) { root["error"] = message; });
  send_json(request, code, std::string(body.c_str(), body.size()));
}
}  // namespace esphome::network_settings::web
