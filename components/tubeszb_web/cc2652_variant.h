#pragma once
#include <algorithm>
#include <cctype>
#include <string>

namespace esphome::tubeszb_web {
// The saved select uses stable labels; accept either case at the HTTP boundary.
inline const char *cc2652_variant_option(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return std::tolower(c); });
  if (value == "auto")
    return "Auto";
  if (value == "p2" || value == "cc2652p2")
    return "P2";
  if (value == "p7" || value == "cc2652p7")
    return "P7";
  return nullptr;
}
}  // namespace esphome::tubeszb_web
