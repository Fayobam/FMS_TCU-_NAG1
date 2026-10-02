#include "NetworkManager.h"
#include <cstring>
#if !defined(UNIT_TEST) && __has_include("WifiSecrets.h")
#include "WifiSecrets.h"
#endif

namespace tcu {
static constexpr uint32_t SETTINGS_MAGIC = 0x4E455431;
static constexpr uint32_t CONNECT_MS = 12000, SCAN_MS = 60000, FALLBACK_MS = 20000, AP_RETIRE_MS = 10000;
static const char* HOSTNAME = "tcu";

void NetworkManager::begin() {
    storageOk = prefs.begin("tcu_network", false);
    bool loaded = storageOk && prefs.getBytesLength("settings") == sizeof(settings)
        && prefs.getBytes("settings", &settings, sizeof(settings)) == sizeof(settings)
        && settings.magic == SETTINGS_MAGIC && settings.count <= 5 && settings.mode <= AccessPointOnly;
    if (!loaded) {
        settings = {};
        settings.magic = SETTINGS_MAGIC;
#if defined(WIFI_STA_SSID) && defined(WIFI_STA_PASS)
        if (strlen(WIFI_STA_SSID) > 0 && strlen(WIFI_STA_SSID) <= 32 && strlen(WIFI_STA_PASS) <= 64) {
            settings.count = 1;
            strlcpy(settings.known[0].ssid, WIFI_STA_SSID, sizeof(settings.known[0].ssid));
            strlcpy(settings.known[0].password, WIFI_STA_PASS, sizeof(settings.known[0].password));
        }
#endif
    }
    for (auto& k : settings.known) { k.ssid[32] = 0; k.password[64] = 0; }
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false); // this state machine owns retries
    WiFi.setHostname(HOSTNAME);
    restart();
}
void NetworkManager::restart() {
    if (mdnsUp) { MDNS.end(); mdnsUp = false; }
    WiFi.scanDelete();
    WiFi.disconnect(false, false);
    // Keep a working fallback AP during settings changes and station recovery.
    if (settings.mode == StationOnly && apUp) { WiFi.softAPdisconnect(false); apUp = false; }
    WiFi.mode(settings.mode == AccessPointOnly ? WIFI_AP : (apUp ? WIFI_AP_STA : WIFI_STA));
    WiFi.setSleep(false);
    state = Idle;
    unavailableSince = millis();
    lastScan = millis() - SCAN_MS;
    lastApTry = millis() - 10000;
    if (settings.mode == AccessPointOnly || (!settings.count && settings.mode == Automatic)) startAp();
}
void NetworkManager::startAp() {
    lastApTry = millis();
    WiFi.mode(settings.mode == AccessPointOnly ? WIFI_AP : WIFI_AP_STA);
    IPAddress ip(192,168,4,1);
    apUp = WiFi.softAPConfig(ip, ip, IPAddress(255,255,255,0))
        && WiFi.softAP("7226-TCU", nullptr, 6, false, 4);
    if (apUp) {
        Serial.print("Fallback access point \"7226-TCU\" up: http://");
        Serial.println(WiFi.softAPIP().toString().c_str());
    }
    if (apUp && state != Connecting && state != Scanning) state = ApFallback;
}
void NetworkManager::scan() {
    lastScan = stateSince = millis();
    attempted = 0;
    int result = WiFi.scanNetworks(true); // asynchronous; never wait for results
    state = (result == WIFI_SCAN_FAILED) ? Reconnecting : Scanning;
}
bool NetworkManager::connectNext() {
    int count = WiFi.scanComplete();
    for (uint8_t k = 0; k < settings.count; ++k) {
        if (attempted & (1u << k)) continue;
        int best = -1;
        for (int i = 0; i < count; ++i)
            if (WiFi.SSID(i) == settings.known[k].ssid && (best < 0 || WiFi.RSSI(i) > WiFi.RSSI(best))) best = i;
        attempted |= 1u << k;
        if (best < 0) continue;
        WiFi.begin(settings.known[k].ssid, settings.known[k].password);
        state = Connecting;
        stateSince = millis();
        return true;
    }
    WiFi.scanDelete();
    state = apUp ? ApFallback : Reconnecting;
    return false;
}
void NetworkManager::update() {
    uint32_t now = millis();
    if (restartPending && now - restartAt >= 1000) { restartPending = false; restart(); }
    bool connected = settings.mode != AccessPointOnly && WiFi.status() == WL_CONNECTED;
    if (connected && state != Connected) {
        state = Connected;
        stateSince = now;
        // The address is the one thing an operator needs and cannot guess, and
        // tcu.local only resolves where mDNS does. Printed on the transition only,
        // so it can never become a repeating log on the control console.
        Serial.print("Dashboard: http://");
        Serial.print(WiFi.localIP().toString().c_str());
        Serial.print("  (or http://");
        Serial.print(HOSTNAME);
        Serial.println(".local)");
        WiFi.scanDelete();
        if (mdnsUp) { MDNS.end(); mdnsUp = false; }
        lastMdnsTry = now - 10000;
    }
    if (!connected && state == Connected) {
        state = Reconnecting;
        unavailableSince = now;
        lastScan = now - SCAN_MS;
        if (mdnsUp) { MDNS.end(); mdnsUp = false; }
    }
    if ((connected || apUp) && !mdnsUp && now - lastMdnsTry >= 10000) {
        lastMdnsTry = now;
        mdnsUp = MDNS.begin(HOSTNAME);
        if (mdnsUp) MDNS.addService("http", "tcp", 80);
    }
    // Retire the fallback AP once the station link has settled. ESP32 AP+STA is ONE
    // radio on ONE channel: associating to a router on another channel drags the
    // softAP off its configured channel onto the router's, and the station link then
    // time-shares airtime with AP beacons and management traffic. The result is
    // jittery throughput rather than a clean failure — which is what a stalling
    // dashboard looks like. Nothing else ever cleared apUp in Automatic mode, so a
    // single slow-router boot (the 20 s FALLBACK_MS path) left the AP up for the
    // rest of the session.
    //
    // Only drop it when nobody is associated, so a browser working over the AP is
    // never kicked off mid-session. If the station link later drops, the same
    // FALLBACK_MS path brings the AP straight back.
    if (connected && apUp && settings.mode == Automatic && now - stateSince >= AP_RETIRE_MS
        && WiFi.softAPgetStationNum() == 0) {
        WiFi.softAPdisconnect(true);          // also drops the AP interface
        apUp = false;
        WiFi.setSleep(false);                 // mode changes can restore power save
        if (mdnsUp) { MDNS.end(); mdnsUp = false; }   // re-announce on station only
        lastMdnsTry = now - 10000;
    }
    if (connected) return;
    if (!apUp && settings.mode != StationOnly && now - lastApTry >= 10000
        && (settings.mode == AccessPointOnly || !settings.count || now - unavailableSince >= FALLBACK_MS)) startAp();
    if (settings.mode == AccessPointOnly || !settings.count) return;
    if (state == Scanning) {
        int n = WiFi.scanComplete();
        if (n >= 0) connectNext();
        else if (n == WIFI_SCAN_FAILED || now - stateSince > 15000) {
            WiFi.scanDelete(); state = Reconnecting;
        }
    } else if (state == Connecting) {
        if (now - stateSince >= CONNECT_MS) { WiFi.disconnect(false, false); connectNext(); }
    } else if (now - lastScan >= SCAN_MS) scan();
}
void NetworkManager::describe(JsonDocument& d) const {
    static const char* states[] = {"IDLE","SCANNING","CONNECTING","CONNECTED","AP_FALLBACK","RECONNECTING"};
    d["type"] = "network";
    d["state"] = states[state]; d["mode"] = settings.mode;
    bool sta = WiFi.status() == WL_CONNECTED;
    d["sta"] = sta; d["ap"] = apUp; d["ssid"] = sta ? WiFi.SSID() : String("");
    d["ip"] = sta ? WiFi.localIP().toString() : String("");
    d["apIp"] = apUp ? WiFi.softAPIP().toString() : String("");
    d["apSsid"] = "7226-TCU"; d["rssi"] = sta ? WiFi.RSSI() : 0;
    d["hostname"] = HOSTNAME; d["mdns"] = "tcu.local"; d["mdnsActive"] = mdnsUp;
    d["storageOk"] = storageOk;
    JsonArray a = d["known"].to<JsonArray>();
    for (uint8_t i = 0; i < settings.count; ++i) {
        JsonObject k = a.add<JsonObject>();
        k["ssid"] = settings.known[i].ssid;
        k["secured"] = settings.known[i].password[0] != 0;
    }
}
const char* NetworkManager::configure(JsonDocument& doc) {
    if (!storageOk) return "Network storage unavailable";
    Settings next = settings;
    const char* op = doc["op"] | "";
    if (!strcmp(op, "mode")) {
        if (!doc["mode"].is<int>() || doc["mode"].as<int>() < 0 || doc["mode"].as<int>() > 2) return "Invalid mode";
        next.mode = doc["mode"].as<int>();
        if (next.mode == StationOnly && !next.count) return "Remember a network before selecting station only";
    } else if (!strcmp(op, "add")) {
        if (!doc["ssid"].is<const char*>() || !doc["password"].is<const char*>()) return "SSID and password required";
        const char* ssid = doc["ssid"], *pass = doc["password"];
        size_t sn = strlen(ssid), pn = strlen(pass);
        if (!sn || sn > 32 || pn > 63 || (pn > 0 && pn < 8)) return "SSID: 1-32 bytes; password: empty or 8-63 bytes";
        uint8_t i = 0;
        while (i < next.count && strcmp(next.known[i].ssid, ssid)) ++i;
        if (i == 5) return "Maximum five remembered networks";
        if (i == next.count) ++next.count;
        strlcpy(next.known[i].ssid, ssid, sizeof(next.known[i].ssid));
        strlcpy(next.known[i].password, pass, sizeof(next.known[i].password));
    } else if (!strcmp(op, "remove") || !strcmp(op, "up")) {
        if (!doc["index"].is<int>()) return "Index required";
        int i = doc["index"];
        if (i < 0 || i >= next.count) return "Invalid network index";
        if (!strcmp(op, "remove")) {
            if (next.count == 1 && next.mode == StationOnly) return "Select automatic mode before removing the last network";
            for (int j = i; j + 1 < next.count; ++j) next.known[j] = next.known[j+1];
            next.known[--next.count] = {};
        } else if (i > 0) { Known tmp = next.known[i-1]; next.known[i-1] = next.known[i]; next.known[i] = tmp; }
    } else return "Unknown network operation";
    if (prefs.putBytes("settings", &next, sizeof(next)) != sizeof(next)) return "Network storage write failed";
    settings = next;
    restartPending = true; restartAt = millis(); // allow the save acknowledgement to leave first
    return nullptr;
}
} // namespace tcu
