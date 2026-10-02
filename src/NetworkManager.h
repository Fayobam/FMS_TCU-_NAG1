#pragma once
#include <WiFi.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>

namespace tcu {
class NetworkManager {
public:
    enum State { Idle, Scanning, Connecting, Connected, ApFallback, Reconnecting };
    enum Mode { Automatic, StationOnly, AccessPointOnly };
    void begin();
    void update();
    void describe(JsonDocument& doc) const;
    // Called only by the service task, never an HTTP/Wi-Fi callback.
    const char* configure(JsonDocument& doc);
    bool apActive() const { return apUp; }
private:
    struct Known { char ssid[33]; char password[65]; };
    struct Settings { uint32_t magic; uint8_t mode; uint8_t count; Known known[5]; } settings{};
    Preferences prefs;
    bool storageOk = false, apUp = false, mdnsUp = false;
    State state = Idle;
    uint32_t stateSince = 0, lastScan = 0, unavailableSince = 0, lastApTry = 0, lastMdnsTry = 0;
    uint8_t attempted = 0;
    bool restartPending = false;
    uint32_t restartAt = 0;
    void startAp();
    void scan();
    bool connectNext();
    void restart();
};
}
