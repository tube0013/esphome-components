#pragma once
#include <cstdlib>
#include <string>

namespace esphome::tubeszb_web {
struct CC2652LogEvent {
  enum class Kind {
    KIND_NONE,
    KIND_CHECK_START,
    KIND_FLASH_START,
    KIND_PROBE_START,
    KIND_FLASH_OK,
    KIND_PROBE_OK,
    KIND_PROBE_FAILED,
    KIND_BOOTLOADER_START,
    KIND_BOOTLOADER_READY,
    KIND_PROGRESS
  };
  Kind kind{Kind::KIND_NONE};
  int progress{-1};
};

inline CC2652LogEvent decode_cc2652_log(const std::string &line) {
  using E = CC2652LogEvent::Kind;
  if (line.find("Starting firmware update") != std::string::npos)
    return {E::KIND_FLASH_START};
  if (line.find("Checking CC2652 firmware against manifest") != std::string::npos)
    return {E::KIND_CHECK_START};
  if (line.find("Detecting CC2652 variant via ROM bootloader") != std::string::npos)
    return {E::KIND_PROBE_START};
  if (line.find("Update finished OK.") != std::string::npos)
    return {E::KIND_FLASH_OK};
  if (line.find("Variant detection: CC2652P2") != std::string::npos ||
      line.find("Variant detection: CC2652P7") != std::string::npos)
    return {E::KIND_PROBE_OK};
  if (line.find("Variant detection failed") != std::string::npos ||
      line.find("Variant detection inconclusive") != std::string::npos)
    return {E::KIND_PROBE_FAILED};
  if (line.find("Manual bootloader requested.") != std::string::npos)
    return {E::KIND_BOOTLOADER_START};
  if (line.find("Manual bootloader ready.") != std::string::npos)
    return {E::KIND_BOOTLOADER_READY};
  const auto start = line.find("Progress: ");
  if (start != std::string::npos) {
    const char *digits = line.c_str() + start + 10;
    char *end = nullptr;
    const unsigned long value = std::strtoul(digits, &end, 10);
    if (end != digits && *end == '%' && value <= 100)
      return {E::KIND_PROGRESS, static_cast<int>(value)};
  }
  return {};
}
}  // namespace esphome::tubeszb_web
