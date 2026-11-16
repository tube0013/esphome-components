#include "components/tubeszb_web/operation_start.h"
#include "components/network_settings/single_flight.h"
#include "components/zigbee_firmware_baud/flash_status.h"
#include <cassert>
#include <thread>
#include <vector>
using namespace esphome;
int main() {
  tubeszb_web::PendingStart start;
  assert(!start.pending() && !start.expired(40000));
  start.arm(100); assert(start.pending()); assert(!start.expired(30099)); assert(start.expired(30100));
  start.clear(); assert(!start.expired(99999)); // Actual OTA start hands ownership to the backend.
  start.arm(UINT32_MAX - 100); assert(!start.expired(29898)); assert(start.expired(29899));
  for (const char *url : {"", "http://", "https://", "file:///ota.bin", "https:///ota.bin"})
    assert(!tubeszb_web::valid_ota_url(url));
  assert(tubeszb_web::valid_ota_url("https://example.test/ota.bin"));
  assert(tubeszb_web::valid_ota_url("http://192.168.1.5/ota.bin"));
  tubeszb_web::RestartGuard restart;
  assert(!restart.allowed(true,false)); assert(!restart.allowed(false,true));
  assert(!restart.request(true,false)); assert(!restart.request(false,true));
  // Queued/no-start/check states have no active writer and permit recovery.
  assert(restart.request(false,false)); assert(!restart.allowed(false,false));
  assert(!restart.request(false,false)); assert(restart.ready(false,false));
  // A writer starting during the response grace period must postpone reboot.
  assert(!restart.ready(true,false)); assert(!restart.ready(false,true));
  assert(restart.ready(false,false));
  network_settings::SingleFlight flight;
  std::atomic<unsigned> winners{0}; std::vector<std::thread> workers;
  for (int i=0;i<8;++i) workers.emplace_back([&]() { if (flight.begin()) ++winners; });
  for (auto &worker : workers) worker.join();
  assert(winners == 1); flight.end(); assert(flight.begin()); flight.end(); // Also recovers failed task creation.
  zigbee_firmware_baud::FlashStatus status; std::vector<uint8_t> events;
  bool cleanup_finished = false;
  status.listener = [&](const auto &value) { events.push_back(value.result); if(value.result >= 2) assert(cleanup_finished); };
  {
    zigbee_firmware_baud::FlashAttempt attempt(status);
    status.advance(60); status.advance(100); assert(status.result == 1);
    cleanup_finished = true; attempt.succeed();
  }
  assert(events.front() == 1 && events.back() == 2 && status.progress == 100);
  cleanup_finished = false;
  { zigbee_firmware_baud::FlashAttempt failed(status); cleanup_finished = true; }
  assert(status.result == 3 && status.sequence == 2);
}
