#include "esphome/components/cc2652_flasher/cc2652_flasher.h"

// NOTE: This file intentionally only contains out-of-class definitions and no
// class re-declarations. Keep the declarations in the header only.

namespace esphome {
namespace cc2652_flasher {

// Provide a key function so the vtable is emitted in this TU
CC2652Flasher::~CC2652Flasher() {}

// The remaining method bodies are identical to the ones in your current
// implementation and must be moved here as out-of-class definitions, e.g.:
//
// void CC2652Flasher::setup() { /* ... */ }
// void CC2652Flasher::loop()  { /* ... */ }
// float CC2652Flasher::get_setup_priority() const { return setup_priority::HARDWARE; }
//
// bool CC2652Flasher::http_open_(...) { /* ... */ }
// bool CC2652Flasher::fetch_manifest_(...) { /* ... */ }
// ... and so on for all methods declared in the header.
//
// For brevity, this template leaves the full bodies out. Copy the bodies from
// your existing implementation and prefix each with "CC2652Flasher::".

} // namespace cc2652_flasher
} // namespace esphome

