#pragma once
#include <optional>
#include <string_view>

namespace esphome::network_settings::web {
inline bool origin_matches(std::string_view origin, std::string_view host) {
  const size_t prefix = origin.compare(0, 7, "http://") == 0 ? 7 : origin.compare(0, 8, "https://") == 0 ? 8 : 0;
  return prefix != 0 && origin.size() == prefix + host.size() && origin.substr(prefix) == host;
}
inline bool authorized(std::string_view nonce, std::optional<std::string_view> token,
                       std::optional<std::string_view> origin, std::optional<std::string_view> host) {
  return token.has_value() && *token == nonce &&
         (!origin.has_value() || (host.has_value() && origin_matches(*origin, *host)));
}
}  // namespace esphome::network_settings::web
