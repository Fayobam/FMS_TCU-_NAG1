// ============================================================================
// FILE: test/stubs/Preferences.h   (NATIVE HOST BUILD ONLY)
// In-memory stand-in for the ESP32 NVS Preferences library. Each instance owns
// its own key/blob map, mirroring the per-namespace isolation the real library
// gives EngineProfile / AdaptiveMemory / DtcManager.
//
// Blank on construction, so a fresh test run always exercises the
// "blank flash -> seed defaults" path — which is exactly the state a newly
// flashed ECU boots into.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <map>
#include <vector>
#include <string>
#include <algorithm>

class Preferences {
  private:
    std::map<std::string, std::vector<uint8_t>> _store;

  public:
    bool begin(const char* /*name*/, bool /*readOnly*/ = false) { return true; }
    void end() {}
    bool clear() { _store.clear(); return true; }

    size_t getBytesLength(const char* key) {
        auto it = _store.find(key);
        return (it == _store.end()) ? 0 : it->second.size();
    }

    size_t getBytes(const char* key, void* buf, size_t maxLen) {
        auto it = _store.find(key);
        if (it == _store.end()) return 0;
        size_t n = std::min(maxLen, it->second.size());
        std::memcpy(buf, it->second.data(), n);
        return n;
    }

    size_t putBytes(const char* key, const void* buf, size_t len) {
        const uint8_t* p = static_cast<const uint8_t*>(buf);
        _store[key].assign(p, p + len);
        return len;
    }
};
