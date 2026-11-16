#include "components/cc2652_flash_guard/cc2652_flash_session.h"
#include <cassert>
using esphome::zigbee_firmware_baud::FlashStatus;
using esphome::zigbee_firmware_baud::FlashAttempt;
using esphome::cc2652_flash_guard::FlashSession;
int main() {
  FlashSession session; FlashStatus status;
  status.start(); status.finish(true); // A previous flash must not complete a newly queued one.
  assert(session.begin(false, status.sequence)); assert(session.resume_after);
  assert(!session.begin(true, status.sequence)); assert(session.resume_after);
  session.observe(status); assert(!session.completed);
  {
    FlashAttempt attempt(status); session.observe(status); assert(!session.completed);
    status.advance(100); session.observe(status); assert(!session.completed);
    attempt.succeed();
  }
  session.observe(status); assert(session.completed && session.success && session.active);
  session.end(); assert(!session.active);
  assert(session.begin(false,status.sequence));
  { FlashAttempt failed(status); } // Any early return completes as failure, even without logs.
  session.observe(status); assert(session.completed && !session.success && session.resume_after);
  session.end();
  assert(session.begin(true,status.sequence)); assert(!session.resume_after);
  { FlashAttempt retry(status); retry.succeed(); }
  session.observe(status); assert(session.success); session.end(); assert(!session.resume_after);
}
