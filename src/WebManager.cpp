// Core 0 web services. Transport callbacks enqueue bounded envelopes; validated
// control commands cross ControlBridge. Static assets are self-contained PROGMEM.
// Telemetry snapshots and shared WS buffers bound serialization and client cost.
#include "WebManager.h"
#include "EngineProfile.h"
#include "TuneOverlay.h"
#include "DtcManager.h"
#include "tcu_index_html.h"

#include <cstring>
#include "CommandValidation.h"
#include "ControlBridge.h"
#include "TelemetryConfig.h"
#include "TelemetryJson.h"
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

static const uint32_t TELEM_MS = TCU_TELEMETRY_INTERVAL_MS;

// Bounded transport queue. Callbacks only copy bytes, never apply commands.
static portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
struct Envelope { uint32_t client; uint16_t length; char text[3072]; };
static Envelope commands[4];
static uint8_t commandHead = 0, commandTail = 0, commandCount = 0;

static bool storedCalibrationMatches(const char* name, const void* data, size_t length) {
    // Existing calibration APIs return void from save(). Read back the bounded
    // blob so a failed NVS write is not presented as a successful save.
    uint8_t stored[sizeof(TuneData) > sizeof(EngineProfileData) ? sizeof(TuneData) : sizeof(EngineProfileData)];
    if (length > sizeof(stored)) return false;
    Preferences check;
    if (!check.begin(name,true)) return false;
    bool ok=check.getBytesLength("data")==length && check.getBytes("data",stored,length)==length
        && memcmp(stored,data,length)==0;
    check.end();
    return ok;
}

WebManager::WebManager() : server(80), ws("/ws") {
    _adaptives = nullptr;
    _dns_ok = false;
    _last_telem_ms = 0;
    _last_cleanup_ms = 0;
    _pend = 0;
}

void WebManager::setAdaptiveMemory(AdaptiveMemory* adaptives) { _adaptives = adaptives; }

void WebManager::sendDashboard(AsyncWebServerRequest *request) {
    AsyncWebServerResponse *res = request->beginResponse(
        200, "text/html", TCU_INDEX_HTML_GZ, TCU_INDEX_HTML_GZ_LEN);
    res->addHeader("Content-Encoding", "gzip");
    res->addHeader("Cache-Control", "no-store");
    res->addHeader("X-Content-Type-Options", "nosniff");
    request->send(res);
}

// Phone OS connectivity checks must stay tiny. Serving the 22 KB gzip
// dashboard on every generate_204 / hotspot-detect during join floods the
// AP and iOS/Android then drop the association.
static void sendCaptive204(AsyncWebServerRequest *req) { req->send(204); }
static void sendCaptiveOk(AsyncWebServerRequest *req) {
    req->send(200, "text/html",
              "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
}
static void sendCaptiveText(AsyncWebServerRequest *req, const char *body) {
    req->send(200, "text/plain", body);
}

void WebManager::wsSend(const char* buf, size_t len, bool bestEffort, bool telemetryPacket) {
    AsyncWebSocketSharedBuffer shared;
    for (auto& c : ws.getClients()) {
        if (c.status() != WS_CONNECTED) continue;
        if (bestEffort && c.queueLen() >= 2) {
            if (telemetryPacket) _telemetryQueueSkips.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        if (c.canSend()) {
            if (!shared) shared = std::make_shared<std::vector<uint8_t>>(buf,buf+len);
            const bool queued = c.text(shared);
            if (telemetryPacket && !queued) _telemetryQueueSkips.fetch_add(1, std::memory_order_relaxed);
        } else {
            if (telemetryPacket) _telemetryQueueSkips.fetch_add(1, std::memory_order_relaxed);
            if (!bestEffort) c.close(1013,"Client too slow; reconnect");
        }
    }
}

void WebManager::fillTelemetry(JsonDocument& doc) {
    fillTelemetryJson(doc, controlBridge.read(), ws.count(), ESP.getFreeHeap(), _fs_ok);
    doc["txQueueSkips"] = _telemetryQueueSkips.load(std::memory_order_relaxed);
    doc["txOversize"] = _telemetryOversize.load(std::memory_order_relaxed);
    doc["serviceGapMaxMs"] = _serviceGapMaxMs.load(std::memory_order_relaxed);
}

void WebManager::pumpTelemetry() {
    if (ws.count() == 0) return;
    JsonDocument doc;
    fillTelemetry(doc);
    char buf[TCU_TELEMETRY_BUFFER_BYTES];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    if (n >= sizeof(buf) - 1) {
        _telemetryOversize.fetch_add(1, std::memory_order_relaxed);
        return; // Never emit truncated JSON; /api/status still exposes the counter.
    }
    wsSend(buf, n, true, true);
}

void WebManager::pumpTrace() {
    if (!shiftTrace.ready || ws.count() == 0 || ESP.getFreeHeap() < 110000) return;
    bool space = false;
    for (auto& c : ws.getClients()) {
        if (c.status() == WS_CONNECTED && c.queueLen() < 2 && c.canSend()) { space = true; break; }
    }
    if (!space) return;

    // Serialize the existing integer wire format directly. A JSON DOM for 4,800
    // values costs much more heap than this bounded buffer plus shared WS payload.
    uint16_t n = min((uint16_t)shiftTrace.count, (uint16_t)TRACE_MAX);
    String packed;
    if (!packed.reserve(40000)) return;
    char number[160];
    snprintf(number,sizeof(number),"{\"type\":\"shift_trace\",\"cls\":%u,\"pd\":%u,\"from\":%u,\"to\":%u,\"n\":%u",
        shiftTrace.shift_class,shiftTrace.pd_type,shiftTrace.from_gear,shiftTrace.to_gear,n);
    packed += number;
    const char* keys[]={"t","ph","spc","mpc","ratio","eng","turb","out","clErr","fl","onClutch","offClutch"};
    for (uint8_t field=0;field<12;++field) {
        packed += ",\""; packed += keys[field]; packed += "\":[";
        for (uint16_t i=0;i<n;++i) {
            const auto& s=shiftTrace.s[i];
            const int values[]={s.t_ms,s.phase,s.spc,s.mpc,s.ratio_x1000,s.eng,s.turb,s.out,s.cl_err_x1000,s.flags,s.on_clutch,s.off_clutch};
            snprintf(number,sizeof(number),"%s%d",i?",":"",values[field]); packed += number;
        }
        packed += ']';
    }
    packed += '}';
    wsSend(packed.c_str(), packed.length(), true);
    shiftTrace.ready = false;
}

void WebManager::sendProfile() {
    EngineProfileData* p = engineProfile.raw();
    JsonDocument resp;
    resp["type"] = "profile_data";
    JsonArray tq = resp["torque"].to<JsonArray>();
    for (int i = 0; i < EP_RPM_BINS; i++)
        for (int j = 0; j < EP_MAP_BINS; j++) tq.add(p->torque[i][j]);
    JsonArray rb = resp["rpm"].to<JsonArray>();
    for (int i = 0; i < EP_RPM_BINS; i++) rb.add(p->rpm_bp[i]);
    JsonArray mb = resp["map"].to<JsonArray>();
    for (int j = 0; j < EP_MAP_BINS; j++) mb.add(p->map_bp[j]);
    resp["tmax"] = p->t_max_ref; resp["overrev"] = p->overrev_rpm; resp["lug"] = p->lug_rpm;
    resp["engPpr"] = p->eng_ppr; resp["outPpr"] = p->out_ppr;
    resp["clEn"]   = p->cl_spc_enable; resp["clKp"] = p->cl_spc_kp;
    resp["kmhRpm"] = p->kmh_per_outrpm;
    resp["transVariant"] = p->trans_variant;
    resp["tcStall"] = p->tc_stall_mult_x100; resp["tcCoupSr"] = p->tc_coupling_sr_x100;
    resp["clPwr"] = p->cl_pressure_enable; resp["pFull"] = p->p_full_scale_mbar;
    resp["clSpeed"] = p->cl_speed_transitions;
    resp["coefStat"] = p->coef_stationary; resp["coefRel"] = p->coef_releasing;
    resp["coefCold"] = p->coef_apply_cold; resp["coefHot"] = p->coef_apply_hot;
    JsonArray af = resp["applyFric"].to<JsonArray>();
    JsonArray rf = resp["relFric"].to<JsonArray>();
    JsonArray as = resp["applySpring"].to<JsonArray>();
    JsonArray rs = resp["relSpring"].to<JsonArray>();
    for (int i = 0; i < 4; i++) {
        af.add(p->apply_friction[i]); rf.add(p->release_friction[i]);
        as.add(p->apply_spring_mbar[i]); rs.add(p->release_spring_mbar[i]);
    }
    resp["tpsC"] = p->tps_closed_v; resp["tpsW"] = p->tps_wot_v;
    resp["map0"] = p->map_kpa_at_0v; resp["mapV"] = p->map_kpa_per_volt;
    JsonArray fp = resp["fillp"].to<JsonArray>();
    JsonArray ft = resp["fillt"].to<JsonArray>();
    for (int i = 0; i < 4; i++) { fp.add(p->fill_p[i]); ft.add(p->fill_t[i]); }
    String out;
    serializeJson(resp, out);
    wsSend(out.c_str(), out.length());
}

void WebManager::sendParams() {
    JsonDocument resp;
    resp["type"] = "param_list";
    uint8_t n = TuneOverlay::paramCount();
    resp["n"] = n;
    JsonArray arr = resp["params"].to<JsonArray>();
    for (uint8_t i = 0; i < n; i++) {
        const ParamDesc& p = TuneOverlay::paramDesc(i);
        JsonObject o = arr.add<JsonObject>();
        o["type"] = "param";
        o["idx"]  = i;
        o["n"]    = n;
        o["id"]   = p.id;
        o["name"] = p.name;
        o["group"]= p.group;
        o["kind"] = (uint8_t)p.kind;
        o["min"]  = p.vmin;
        o["max"]  = p.vmax;
        o["rows"] = p.rows;
        o["cols"] = p.cols;
        o["unit"] = p.unit;
        o["help"] = p.help;
        JsonArray v = o["v"].to<JsonArray>();
        for (uint8_t r = 0; r < p.rows; r++)
            for (uint8_t c = 0; c < p.cols; c++) v.add(tuneOverlay.paramGet(i, r, c));
    }
    String out;
    serializeJson(resp, out);
    wsSend(out.c_str(), out.length());
}

void WebManager::sendCells() {
    JsonDocument responseDoc;
    responseDoc["type"] = "cell_data";
    JsonArray arr = responseDoc["data"].to<JsonArray>();
    if (_adaptives) {
        auto snapshot = controlBridge.read();
        AdaptCell* cells = snapshot.cells;
        int n = _adaptives->cellCount();
        responseDoc["classes"] = ADAPT_CLASSES;
        responseDoc["shifts"]  = ADAPT_SHIFTS;
        responseDoc["tbins"]   = ADAPT_TBINS;
        for (int i = 0; i < n; i++) {
            arr.add(cells[i].fill_t_cycles);
            arr.add(cells[i].fill_p_trim);
            arr.add(cells[i].apply_trim);
        }
    }
    String out;
    serializeJson(responseDoc, out);
    wsSend(out.c_str(), out.length());
}

void WebManager::sendDtcs() {
    JsonDocument resp;
    resp["type"] = "dtc_data";
    JsonArray arr = resp["dtcs"].to<JsonArray>();
    DtcSnapshot dtc = dtcManager.snapshot();   // coherent: Core 1 writes while we serialize
    for (uint8_t i = 0; i < DTC_COUNT; i++) {
        JsonObject o = arr.add<JsonObject>();
        o["code"]   = i;
        o["name"]   = dtcName(i);
        o["count"]  = dtc.count[i];
        o["active"] = dtc.active[i];
        o["lastMs"] = dtc.last_ms[i];
    }
    String out;
    serializeJson(resp, out);
    wsSend(out.c_str(), out.length());
}

void WebManager::pumpPending() {
    if (_pend & PEND_DTCS)    { sendDtcs();    _pend &= ~PEND_DTCS; }
    else if (_pend & PEND_CELLS)   { sendCells();   _pend &= ~PEND_CELLS; }
    else if (_pend & PEND_PROFILE) { sendProfile(); _pend &= ~PEND_PROFILE; }
    else if (_pend & PEND_PARAMS)  { sendParams();  _pend &= ~PEND_PARAMS; }
}

void WebManager::reply(uint32_t client, uint32_t requestId, bool ok, const char* message) {
    auto* c = ws.client(client);
    if (!c || !c->canSend()) return;
    JsonDocument d;
    d["type"] = "command_result"; d["requestId"] = requestId;
    d["ok"] = ok; d["message"] = message;
    char buf[256]; size_t n = serializeJson(d, buf, sizeof(buf));
    c->text(buf,n);
}

void WebManager::drainCmd() {
    if (_inflight) {
        bool ok; ControlAction action;
        if (!controlBridge.complete(ok,action)) return;
        // Persistence stays off the control core. No subsequent edit is dispatched
        // until the completed transaction has been saved and acknowledged.
        if (ok) {
            bool saved=true;
            if (action == ControlAction::AtfSelector) {
                Preferences prefs;
                if (!prefs.begin("tcu_selector", false)) saved=false;
                else {
                    // Save the acknowledged request, not a possibly older snapshot.
                    const bool requested=_atfSelectorRequested;
                    saved=prefs.putBool("atf_only",requested)==1
                        && prefs.getBool("atf_only",!requested)==requested;
                    prefs.end();
                }
            }
            if (action == ControlAction::Profile) {
                engineProfile.save();
                saved=storedCalibrationMatches("engine_prof",engineProfile.raw(),sizeof(EngineProfileData));
            }
            if (action == ControlAction::Tune && _persistTune) {
                tuneOverlay.save();
                saved=storedCalibrationMatches("tcu_tune",tuneOverlay.raw(),sizeof(TuneData));
            }
            if (action == ControlAction::Cells && _adaptives) _adaptives->markAllDirtyAndFlush();
            if (!saved) {
                reply(_replyClient,_requestId,false,"Applied in RAM, but NVS verification failed. Reload before retrying.");
                _inflight=false;return;
            }
        }
        reply(_replyClient,_requestId,ok,ok ? "Applied" : "Rejected by controller: stop shafts, engine off, P/N; check bench state");
        _inflight = false;
        return;
    }
    if (millis() - _lastCommandMs < 20) return; // bounded command service rate
    Envelope local;
    bool have = false;
    portENTER_CRITICAL(&cmdMux);
    if (commandCount) {
        local = commands[commandTail];
        commandTail = (commandTail + 1) % 4; --commandCount; have = true;
    }
    portEXIT_CRITICAL(&cmdMux);
    if (!have) return;
    _lastCommandMs = millis();
    // Commands from a closed session must never execute later.
    if (!ws.client(local.client)) return;
    JsonDocument doc;
    if (deserializeJson(doc,local.text,local.length) || !doc["cmd"].is<const char*>()) {
        reply(local.client,0,false,"Malformed command"); return;
    }
    if (!doc["requestId"].isNull() && !doc["requestId"].is<uint32_t>()) {
        reply(local.client,0,false,"requestId must be an unsigned integer"); return;
    }
    _replyClient=local.client; _requestId=doc["requestId"] | 0u;
    handleCommand(doc);
}

void WebManager::handleCommand(JsonDocument& doc) {
    const char* cmd=doc["cmd"] | "";
    if (!strcmp(cmd,"get_profile")) { _pend |= PEND_PROFILE; }
    else if (!strcmp(cmd,"get_cells")) { _pend |= PEND_CELLS; }
    else if (!strcmp(cmd,"get_dtcs")) { _pend |= PEND_DTCS; }
    else if (!strcmp(cmd,"param.list")) { _pend |= PEND_PARAMS; }
    else if (!strcmp(cmd,"network.get")) {
        JsonDocument d; network.describe(d); String out; serializeJson(d,out); wsSend(out.c_str(),out.length());
    } else if (!strcmp(cmd,"network.set")) {
        auto snap=controlBridge.read();
        if(snap.data.engine_rpm >= 100 || snap.data.output_rpm >= 50 ||
            (snap.data.prnd_state != 'P' && snap.data.prnd_state != 'N')) {
            reply(_replyClient,_requestId,false,"Network settings require engine off, stopped, P/N"); return;
        }
        const char* error=network.configure(doc);
        reply(_replyClient,_requestId,error==nullptr,error?error:"Network settings saved; reconnect if needed"); return;
    } else if (!strcmp(cmd,"param.persist")) {
        auto snap=controlBridge.read();
        if(snap.data.engine_rpm >= 100 || snap.data.output_rpm >= 50 ||
            (snap.data.prnd_state != 'P' && snap.data.prnd_state != 'N')) {
            reply(_replyClient,_requestId,false,"Saving tuning requires engine off, stopped, P/N"); return;
        }
        tuneOverlay.save();
        if (!storedCalibrationMatches("tcu_tune",tuneOverlay.raw(),sizeof(TuneData))) {
            reply(_replyClient,_requestId,false,"Tuning NVS verification failed");return;
        }
    } else {
        ControlCommand command{};
        const char* error=validateCommand(doc,command);
        if(error) { reply(_replyClient,_requestId,false,error); return; }
        _persistTune=!strcmp(cmd,"param.reset");
        if (!strcmp(cmd,"selector.atf")) _atfSelectorRequested=doc["on"].as<bool>();
        if(!controlBridge.submit(command)) { reply(_replyClient,_requestId,false,"Controller command busy"); return; }
        _inflight=true; return;
    }
    reply(_replyClient,_requestId,true,"OK");
}

void WebManager::begin() {
    network.begin();
    _fs_ok = SPIFFS.begin(false); // Never format storage as a side effect of boot.

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) { WebManager::sendDashboard(req); });
    server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *req) { WebManager::sendDashboard(req); });
    server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest *req) {
        JsonDocument doc; fillTelemetry(doc);
        String json; serializeJson(doc,json);
        auto* response=req->beginResponse(200,"application/json",json);
        response->addHeader("Cache-Control","no-store");
        response->addHeader("X-Content-Type-Options","nosniff");
        req->send(response);
    });

    if (SPIFFS.exists("/three.min.js")) {
        server.serveStatic("/three.min.js", SPIFFS, "/three.min.js")
            .setCacheControl("max-age=31536000, immutable");
    } else {
        server.on("/three.min.js", HTTP_GET, [](AsyncWebServerRequest *req) {
            req->send(404, "text/plain", "three.min.js not on SPIFFS");
        });
    }

    server.on("/generate_204", HTTP_ANY, sendCaptive204);
    server.on("/gen_204", HTTP_ANY, sendCaptive204);
    server.on("/hotspot-detect.html", HTTP_ANY, sendCaptiveOk);
    server.on("/library/test/success.html", HTTP_ANY, sendCaptiveOk);
    server.on("/canonical.html", HTTP_ANY, sendCaptiveOk);
    server.on("/connecttest.txt", HTTP_ANY, [](AsyncWebServerRequest *r){ sendCaptiveText(r, "Microsoft Connect Test"); });
    server.on("/ncsi.txt", HTTP_ANY, [](AsyncWebServerRequest *r){ sendCaptiveText(r, "Microsoft NCSI"); });
    server.on("/success.txt", HTTP_ANY, [](AsyncWebServerRequest *r){ sendCaptiveText(r, "success"); });
    server.on("/kindle-wifi/wifiredirect.html", HTTP_ANY, sendCaptiveOk);
    server.on("/fwlink/", HTTP_ANY, sendCaptive204);
    server.on("/redirect", HTTP_ANY, sendCaptive204);

    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                  void *arg, uint8_t *data, size_t len) {
        (void)server;
        if (type == WS_EVT_CONNECT) {
            if (server->count() > 4) { client->close(1013,"Four client limit"); return; }
            client->setCloseClientOnQueueFull(false);
            return;
        }
        if (type != WS_EVT_DATA) return;
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
            client->text("{\"type\":\"error\",\"message\":\"Send one complete text frame\"}"); return;
        }
        if (len == 0 || len >= sizeof(commands[0].text)) {
            client->text("{\"type\":\"error\",\"message\":\"Command too large\"}"); return;
        }
        bool queued = false;
        portENTER_CRITICAL(&cmdMux);
        if (commandCount < 4) {
            auto& c = commands[commandHead]; c.client = client->id(); c.length = len;
            memcpy(c.text,data,len); c.text[len]=0;
            commandHead=(commandHead+1)%4; ++commandCount; queued=true;
        }
        portEXIT_CRITICAL(&cmdMux);
        if (!queued) client->text("{\"type\":\"error\",\"message\":\"Command queue busy\"}");
    });
    server.addHandler(&ws);

    server.onNotFound([](AsyncWebServerRequest *req) {
        if (req->method() == HTTP_OPTIONS) {
            req->send(204);
            return;
        }
        // Do not dump the gzip dashboard onto every DNS-hijacked URL — that
        // is what made the AP vanish mid-join. One 204, phone stays associated.
        req->send(404,"text/plain","Not found");
    });
    server.begin();

}

void WebManager::update() {
    const uint32_t serviceNow = millis();
    if (_lastServiceMs) {
        const uint32_t gap = serviceNow - _lastServiceMs;
        if (gap > _serviceGapMaxMs.load(std::memory_order_relaxed))
            _serviceGapMaxMs.store(gap, std::memory_order_relaxed);
    }
    _lastServiceMs = serviceNow;
    // Send due telemetry before configuration serialization, NVS or bulk traces.
    if (serviceNow - _last_telem_ms >= TELEM_MS) {
        _last_telem_ms = serviceNow;
        pumpTelemetry();
    }
    network.update();
    if (network.apActive() && !_dns_ok) {
        dns.setTTL(30);
        _dns_ok=dns.start(53,"*",IPAddress(192,168,4,1));
    } else if (!network.apActive() && _dns_ok) { dns.stop(); _dns_ok=false; }

    if (_dns_ok) dns.processNextRequest();
    drainCmd();
    pumpPending();

    uint32_t now = millis();
    pumpTrace();

    if (now - _last_cleanup_ms >= 1000) {
        _last_cleanup_ms = now;
        ws.cleanupClients();
    }
    if (_adaptives) _adaptives->processFlush();
}
