#include "components/tubeszb_web/cc2652_log.h"
#include "components/tubeszb_web/cc2652_variant.h"
#include <cassert>

int main() {
  using esphome::tubeszb_web::cc2652_variant_option;
  for (const auto *value : {"auto", "Auto", "AUTO"}) assert(std::string(cc2652_variant_option(value)) == "Auto");
  for (const auto *value : {"p2", "P2", "cc2652p2", "CC2652P2"}) assert(std::string(cc2652_variant_option(value)) == "P2");
  for (const auto *value : {"p7", "P7", "cc2652p7", "CC2652P7"}) assert(std::string(cc2652_variant_option(value)) == "P7");
  for (const auto *value : {"", "P3", "P27", "unknown"}) assert(cc2652_variant_option(value) == nullptr);
  using esphome::tubeszb_web::decode_cc2652_log;
  using E = esphome::tubeszb_web::CC2652LogEvent::Kind;
  assert(decode_cc2652_log("Starting firmware update…").kind == E::KIND_FLASH_START);
  assert(decode_cc2652_log("Checking CC2652 firmware against manifest…").kind == E::KIND_CHECK_START);
  assert(decode_cc2652_log("Detecting CC2652 variant via ROM bootloader…").kind == E::KIND_PROBE_START);
  auto progress = decode_cc2652_log("Progress: 55% (275/500)");
  assert(progress.kind == E::KIND_PROGRESS && progress.progress == 55);
  assert(decode_cc2652_log("Progress: 100% (500/500)").kind == E::KIND_PROGRESS);
  assert(decode_cc2652_log("Progress: 65536 bytes").kind == E::KIND_NONE);
  assert(decode_cc2652_log("Progress: 101% (501/500)").kind == E::KIND_NONE);
  assert(decode_cc2652_log("Progress: %").kind == E::KIND_NONE);
  assert(decode_cc2652_log("BIN transfer complete. bytes=500 md5=abc").kind == E::KIND_NONE);
  assert(decode_cc2652_log("Update failed.").kind != E::KIND_FLASH_OK);
  assert(decode_cc2652_log("MD5 mismatch: expected abc").kind != E::KIND_FLASH_OK);
  assert(decode_cc2652_log("Update finished OK. Resetting target…").kind == E::KIND_FLASH_OK);
  assert(decode_cc2652_log("Transfer verified. Resetting target…").kind == E::KIND_NONE);
  assert(decode_cc2652_log("Update finished OK. Application UART baud ready.").kind == E::KIND_FLASH_OK);
  assert(decode_cc2652_log("Variant detection: CC2652P2").kind == E::KIND_PROBE_OK);
  assert(decode_cc2652_log("Variant detection: CC2652P7").kind == E::KIND_PROBE_OK);
  assert(decode_cc2652_log("Using configured variant: CC2652P2 (ROM probe skipped)").kind == E::KIND_PROBE_OK);
  assert(decode_cc2652_log("Using configured variant: CC2652P7 (ROM probe skipped)").kind == E::KIND_PROBE_OK);
  assert(decode_cc2652_log("Variant detection failed.").kind == E::KIND_PROBE_FAILED);
  assert(decode_cc2652_log("Variant detection inconclusive.").kind == E::KIND_PROBE_FAILED);
  assert(decode_cc2652_log("Manual bootloader requested.").kind == E::KIND_BOOTLOADER_START);
  assert(decode_cc2652_log("Manual bootloader ready.").kind == E::KIND_BOOTLOADER_READY);
}
