#include "components/efr32_info/ezsp_info.h"
#include <cassert>
#include <iostream>
using namespace esphome::efr32_info;
using Bytes = std::vector<uint8_t>;

Bytes reply(uint16_t id, Bytes payload) {
  Bytes result{6, 0x80, 1, uint8_t(id), uint8_t(id >> 8)};
  result.insert(result.end(), payload.begin(), payload.end());
  return result;
}
Bytes custom_reply(uint8_t version, uint16_t command, Bytes value) {
  Bytes payload(version >= 14 ? 4 : 1, 0);
  payload.insert(payload.end(), {uint8_t(value.size() + 3), uint8_t(command), uint8_t((command >> 8) | 0x80), 0});
  payload.insert(payload.end(), value.begin(), value.end());
  return reply(protocol::CUSTOM_FRAME, payload);
}
int main() {
  std::string text = "stale board";
  // Exact empty response from the user's capture: 0680010B0000.
  assert(!protocol::token({6,0x80,1,0x0b,0,0}, text) && text.empty());
  Bytes erased{16}; erased.insert(erased.end(), 16, 0xFF);
  assert(!protocol::token(reply(protocol::GET_MFG_TOKEN, erased), text) && text.empty());
  assert(!protocol::token(reply(protocol::GET_MFG_TOKEN, {3,0,0,0}), text));
  assert(!protocol::token(reply(protocol::GET_MFG_TOKEN, {2,' ',' '}), text));
  assert(!protocol::token(reply(protocol::GET_MFG_TOKEN, {5,'M','G'}), text));
  assert(!protocol::token(reply(protocol::GET_MFG_TOKEN, {2,'M','G','X'}), text));
  assert(!protocol::token(reply(protocol::GET_MFG_TOKEN, {2,'M',1}), text));
  assert(protocol::token(reply(protocol::GET_MFG_TOKEN, {9,'M','G','2','4','-','2','0','2','4'}), text));
  assert(text == "MG24-2024");
  assert(protocol::token(reply(protocol::GET_MFG_TOKEN, {9,'T','u','b','e','s','Z','B',0,0xff}), text));
  assert(text == "TubesZB");
  // A legal 13-byte token must not be confused with a 0x0D status.
  Bytes thirteen{13}; thirteen.insert(thirteen.end(), 13, 'A');
  assert(protocol::token(reply(protocol::GET_MFG_TOKEN, thirteen), text) && text.size()==13);
  for (uint8_t version : {13,19}) {
    assert(protocol::identity_feature(custom_reply(version,0,{4,0,0,0}),version));
    assert(!protocol::identity_feature(custom_reply(version,0,{0,0,0,0}),version));
    assert(!protocol::identity_feature(custom_reply(version,0,{4,0}),version));
    assert(!protocol::identity_feature(custom_reply(version,2,{4,0,0,0}),version));
    Bytes value;
    auto identity = custom_reply(version,2,{'M','G','M','2','4'});
    assert(protocol::custom(identity,version,2,value) && protocol::identity_text(value,text) && text=="MGM24");
    auto error=identity; error[5]=1;
    assert(!protocol::custom(error,version,2,value));
    error=identity; error[5+(version>=14?4:1)+3]=0x2d; // XNCP not found.
    assert(!protocol::custom(error,version,2,value));
    auto truncated=identity; truncated.pop_back();
    assert(!protocol::custom(truncated,version,2,value));
    Bytes payload(version>=14?4:1,0);
    payload.insert(payload.end(),{7,0x34,0x12,9,1,2,1,0xAA});
    protocol::Version parsed{};
    auto frame=reply(protocol::GET_VALUE,payload);
    assert(protocol::version_info(frame,version,parsed));
    assert(parsed.build==0x1234 && parsed.major==9 && parsed.minor==1 && parsed.patch==2 && parsed.special==1);
    frame.pop_back(); assert(!protocol::version_info(frame,version,parsed));
    frame=reply(2,payload); assert(!protocol::version_info(frame,version,parsed));
    // Gecko SDK replies can omit the trailing version type byte.
    payload[version>=14?4:1]=6; payload.pop_back();
    assert(protocol::version_info(reply(protocol::GET_VALUE,payload),version,parsed));
    assert(parsed.build==0x1234 && parsed.special==1);
  }
  Bytes high_status{0,1,0,0,7,0,0,9,1,2,1,0};
  protocol::Version v{};
  assert(!protocol::version_info(reply(protocol::GET_VALUE,high_status),19,v));
  std::cout << "EFR32 identity/version protocol regression tests passed\n";
}
