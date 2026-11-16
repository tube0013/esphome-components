#include "components/network_settings/request_security.h"
#include <cassert>
#include <new>
#include <cstdlib>
static size_t allocations = 0;
void *operator new(size_t n) { ++allocations; if (auto *p = std::malloc(n)) return p; throw std::bad_alloc(); }
void operator delete(void *p) noexcept { std::free(p); }
using namespace esphome::network_settings::web;
int main() {
  assert(authorized("token", "token", "http://192.168.1.5", "192.168.1.5"));
  assert(authorized("token", "token", "https://gw.local:443", "gw.local:443"));
  assert(authorized("token", "token", std::nullopt, std::nullopt)); // CLI still requires token.
  assert(!authorized("token", std::nullopt, std::nullopt, std::nullopt));
  assert(!authorized("token", "wrong", "http://gw.local", "gw.local"));
  assert(!authorized("token", "token", "http://gw.local", std::nullopt));
  assert(!authorized("token", "token", "http://evil.test", "gw.local"));
  assert(!authorized("token", "token", "http://gw.local.evil.test", "gw.local"));
  assert(!authorized("token", "token", "http://gw.local/", "gw.local"));
  assert(!authorized("token", "token", "null", "gw.local"));
  const size_t before = allocations;
  for (unsigned i = 0; i < 10000; ++i)
    assert(authorized("token", "token", "http://long-gateway-hostname.local:8080", "long-gateway-hostname.local:8080"));
  assert(allocations == before); // Origin matching must not concatenate temporary strings.
}
