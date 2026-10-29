"""Persistent offline policy and transport enforcement."""
from component_paths import COMPONENTS_ROOT
from pathlib import Path
import subprocess
ROOT = Path(__file__).resolve().parents[1]

def test_policy_restore_and_commit(tmp_path):
    exe = tmp_path / 'internet-policy'
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(COMPONENTS_ROOT.parent),
                    str(ROOT / 'tests/internet_policy_test.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


def test_native_manifest_transport_cannot_bypass_policy(tmp_path):
    """Direct ESPHome UpdateEntity checks also hit the actual guarded transport."""
    include = tmp_path/'include'
    http = include/'esphome/components/http_request/http_request.h'
    http.parent.mkdir(parents=True)
    http.write_text('''#pragma once
#include <memory>
#include <string>
#include <vector>
namespace esphome::http_request {
struct Header {};
struct HttpContainer {};
class HttpRequestComponent {
 public:
  virtual ~HttpRequestComponent() = default;
  void set_timeout(unsigned v) { timeout_ = v; }
  unsigned get_timeout() const { return timeout_; }
  std::shared_ptr<HttpContainer> start(const char *u, const char *m, const std::string &b,
    const std::vector<Header> &h, const std::vector<std::string> &c) { return perform(u,m,b,h,c); }
 protected:
  virtual std::shared_ptr<HttpContainer> perform(const char*, const char*, const std::string&,
    const std::vector<Header>&, const std::vector<std::string>&) = 0;
  unsigned timeout_{4500};
};
}
''')
    log = include/'esphome/core/log.h'
    log.parent.mkdir(parents=True)
    log.write_text('#pragma once\n#define ESP_LOGI(...) ((void)0)\n')
    source = tmp_path/'transport.cpp'
    source.write_text('''#include "components/network_settings/guarded_http.h"
#include <cassert>
using namespace esphome;
struct Parent : http_request::HttpRequestComponent {
 int requests = 0;
 std::shared_ptr<http_request::HttpContainer> perform(const char *url, const char *method,
    const std::string&, const std::vector<http_request::Header>&, const std::vector<std::string>&) override {
  ++requests;
  assert(std::string(url) == "https://example.test/manifest.json");
  assert(std::string(method) == "GET");
  return std::make_shared<http_request::HttpContainer>();
 }
};
int main() {
 network_settings::InternetPolicy policy;
 Parent parent; network_settings::GuardedHttp transport; transport.bind(&policy, &parent);
 auto check = [&]() { return transport.start("https://example.test/manifest.json", "GET", "", {}, {}); };
 assert(!check()); assert(parent.requests == 0); // Not restored yet.
 policy.restore(true); assert(!check()); assert(parent.requests == 0); // Saved offline boot.
 policy.set(false, [](bool) { return true; }); assert(check()); assert(parent.requests == 1);
 policy.set(true, [](bool) { return true; }); assert(!check()); assert(parent.requests == 1);
}
''')
    exe = tmp_path/'transport'
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(include),
                    '-I', str(COMPONENTS_ROOT.parent), str(source), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)



def test_restore_order_and_update_parent():
    source = (COMPONENTS_ROOT / 'network_settings/network_settings.cpp').read_text().replace('this->', '')
    assert 'nvs_get_u8(handle, "offline_v1"' in source
    assert 'nvs_commit(handle)' in source
    assert 'update_->set_request_parent(&guarded_http_)' in source
    assert 'setup_priority::ETHERNET + 1.0f' in (COMPONENTS_ROOT / 'network_settings/network_settings.h').read_text()





def test_guarded_http_public_api_contract() -> None:
    """An ESPHome bump must recheck the real headers, not just our host doubles."""
    import esphome
    import re
    source=(Path(esphome.__file__).parent/'components/http_request/http_request.h').read_text()
    perform=re.search(r'virtual\s+std::shared_ptr<HttpContainer>\s+perform\((.*?)\)\s*=\s*0',source,re.S)
    assert perform, "Review GuardedHttp: ESPHome's transport override changed"
    assert 'const char *url' in perform[1] and 'const char *method' in perform[1]
    assert 'const std::string &body' in perform[1]
    assert 'const std::vector<Header> &' in perform[1]
    assert 'const std::vector<std::string> &' in perform[1]
    update=(Path(esphome.__file__).parent/'components/http_request/update/http_request_update.h').read_text()
    assert 'void set_request_parent(HttpRequestComponent *' in update
    assert 'const char *get_source_url() const' in update
