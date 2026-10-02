#include <cassert>
#include <iostream>
#include "CommandValidation.h"
#include "DtcManager.h"
#include "TelemetryJson.h"
#include "TelemetryConfig.h"

TCU_Telemetry telemetry;
ShiftTrace shiftTrace;
EngineProfile engineProfile;
void check(const char* json, bool valid) {
    JsonDocument d;
    assert(!deserializeJson(d,json));
    ControlCommand c{};
    const char* error=validateCommand(d,c);
    if ((error==nullptr)!=valid) { std::cerr << json << ": " << (error?error:"accepted") << '\n'; std::abort(); }
}
int main() {
    // Serialize the production telemetry builder after its source storage changes.
    // Runtime const char arrays must not be mistaken for permanent string literals.
    ControlSnapshot source{};
    strcpy(source.data.limp_mode_reason, "Sensor \"test\"\n");
    strcpy(source.data.last_safety_event, "Ready");
    source.data.prnd_state = 'P'; source.sampledMs = 123;
    JsonDocument packet;
    fillTelemetryJson(packet, source, 1, 170000, true);
    memset(source.data.limp_mode_reason, 'X', sizeof(source.data.limp_mode_reason));
    memset(source.data.last_safety_event, 'Y', sizeof(source.data.last_safety_event));
    source.data.prnd_state = 'R';
    std::string wire; serializeJson(packet, wire);
    JsonDocument decoded; assert(!deserializeJson(decoded, wire));
    assert(decoded["limpReason"].as<std::string>() == "Sensor \"test\"\n");
    assert(decoded["safety"].as<std::string>() == "Ready");
    assert(decoded["prnd"].as<std::string>() == "P");
    assert(decoded["sampledMs"].as<int>() == 123);
    packet.clear();
    fillTelemetryJson(packet, ControlSnapshot{}, 0, 170000, true);
    assert(packet["limpReason"].as<std::string>().empty());
    assert(packet["safety"].as<std::string>().empty());
    assert(packet["batteryV"].isNull() && packet["solenoidCurrentA"].isNull());
    assert(packet["atfMeasuredC"].isNull());
    assert(packet["atfSource"].as<std::string>() == "default");
    source = {};
    source.data.atf_sampled = source.data.atf_has_measurement = source.data.atf_signal_valid = true;
    source.data.atf_temp_c = 82; source.data.atf_v = 1.2f;
    source.data.n2_signal_recent = true;
    fillTelemetryJson(packet,source,1,170000,true);
    assert(packet["atfMeasuredC"].as<float>() == 82);
    assert(packet["n2Recent"].as<bool>() && !packet["n3Recent"].as<bool>());
    source.data.atf_signal_valid = false; source.data.atf_v = 3.2f;
    fillTelemetryJson(packet,source,1,170000,true);
    assert(packet["atfMeasuredC"].isNull());
    assert(packet["atfSource"].as<std::string>() == "held");
    assert(packet["atfCircuit"].as<std::string>() == "pn_or_open");
    assert(packet["atfTemp"].as<float>() == 82); // control fallback is preserved
    assert(measureJson(packet) < TCU_TELEMETRY_BUFFER_BYTES-1); // production telemetry buffer
    source.data.atf_only_selector=source.data.atf_forward_confirmed=true;
    memset(source.data.limp_mode_reason,'X',sizeof(source.data.limp_mode_reason)-1);
    memset(source.data.last_safety_event,'Y',sizeof(source.data.last_safety_event)-1);
    source.data.engine_rpm=6500;source.data.turbine_rpm=6000;source.data.output_rpm=3000;
    fillTelemetryJson(packet,source,4,170000,true);
    assert(measureJson(packet)<TCU_TELEMETRY_BUFFER_BYTES-1);
    source.sampledMs=100;g_now_ms=250;
    fillTelemetryJson(packet,source,4,170000,true);
    assert(packet["snapshotAgeMs"].as<uint32_t>()==150);
    source.sampledMs=UINT32_MAX-49;g_now_ms=50;
    fillTelemetryJson(packet,source,4,170000,true);
    assert(packet["snapshotAgeMs"].as<uint32_t>()==100);
    // JSON escaping and full-sized diagnostics must fit the production buffer.
    memset(source.data.limp_mode_reason,'"',sizeof(source.data.limp_mode_reason)-1);
    memset(source.data.last_safety_event,'\n',sizeof(source.data.last_safety_event)-1);
    fillTelemetryJson(packet,source,4,170000,true);
    packet["txQueueSkips"]=UINT32_MAX;packet["txOversize"]=UINT32_MAX;packet["serviceGapMaxMs"]=UINT32_MAX;
    char serialized[TCU_TELEMETRY_BUFFER_BYTES];
    const auto bytes=serializeJson(packet,serialized,sizeof(serialized));
    assert(bytes==measureJson(packet) && bytes<sizeof(serialized)-1);
    JsonDocument parsed;assert(!deserializeJson(parsed,serialized,bytes));
    engineProfile.begin(); tuneOverlay.begin(); AdaptiveMemory adapt; adapt.begin();
    check(R"({"cmd":"param.set","idx":256,"row":0,"col":0,"val":80})",false);
    check(R"({"cmd":"param.set","idx":0,"row":5,"col":0,"val":80})",false);
    check(R"({"cmd":"param.set","idx":0,"row":0,"col":0,"val":65590})",false);
    check(R"({"cmd":"param.set","idx":0,"row":0,"col":0,"val":"80"})",false);
    check(R"({"cmd":"param.set","idx":0,"row":0,"col":0,"val":80})",true);
    check(R"({"cmd":"param.set","idx":8,"row":0,"col":0,"val":3})",false);
    check(R"({"cmd":"set_profile","tpsC":2.9,"tpsW":0.5})",false);
    check(R"({"cmd":"set_profile","tpsC":0.4,"tpsW":2.9})",true);
    check(R"({"cmd":"set_profile","torque":[1,2]})",false);
    check(R"({"cmd":"set_profile","overrev":10000})",false);
    check(R"({"cmd":"set_profile","overrev":null})",false);
    check(R"({"cmd":"set_profile","unknown":4})",false);
    check(R"({"cmd":"set_cells","data":[]})",false);
    check(R"({"cmd":"test_mode","on":"true"})",false);
    check(R"({"cmd":"test_mode","on":true})",true);
    check(R"({"cmd":"test_io","id":"mpc","on":true,"v":101})",false);
    check(R"({"cmd":"test_io","id":"mpc","on":false})",true);
    check(R"({"cmd":"test_prnd","v":"DRIVE"})",false);
    check(R"({"cmd":"test_paddle","dir":0})",false);
    check(R"({"cmd":"unknown"})",false);
    JsonDocument d; deserializeJson(d,R"({"cmd":"set_profile","overrev":6500})");
    ControlCommand c{}; assert(validateCommand(d,c)==nullptr);
    uint16_t old=engineProfile.overrevRpm();
    telemetry.prnd_state='D'; telemetry.engine_rpm=2500; telemetry.output_rpm=400;
    assert(controlBridge.submit(c)); assert(!controlBridge.submit(c)); controlBridge.consume(adapt);
    bool ok; ControlAction a; assert(controlBridge.complete(ok,a));assert(!ok);assert(engineProfile.overrevRpm()==old);
    telemetry={};telemetry.prnd_state='P';
    assert(controlBridge.submit(c));controlBridge.consume(adapt);assert(controlBridge.complete(ok,a));assert(ok);assert(engineProfile.overrevRpm()==6500);
    // Snapshot remains stable until the next control publication.
    g_now_ms=100;telemetry.engine_rpm=1234;controlBridge.publish(adapt);telemetry.engine_rpm=4321;
    assert(controlBridge.read().data.engine_rpm==1234);
    c.action=ControlAction::TestMode;c.value=1;
    assert(controlBridge.submit(c));controlBridge.consume(adapt);assert(controlBridge.complete(ok,a));assert(!ok);
    telemetry.engine_rpm=0;assert(controlBridge.submit(c));controlBridge.consume(adapt);assert(controlBridge.complete(ok,a));assert(ok);
    // Release requests and payload values are applied together on the control core.
    telemetry.test_mode=true;c.action=ControlAction::Output;c.value=4;c.extra=-1;
    assert(controlBridge.submit(c));controlBridge.consume(adapt);assert(controlBridge.complete(ok,a));assert(ok);
    assert(telemetry.test_sol_req==4 && telemetry.test_sol_req_v==-1);
    check(R"({"cmd":"selector.atf","on":true})",true);
    check(R"({"cmd":"selector.atf","on":1})",false);
    check(R"({"cmd":"selector.atf"})",false);
    telemetry={}; telemetry.prnd_state='?';
    c.action=ControlAction::AtfSelector;c.value=1;
    auto apply=[&]() { assert(controlBridge.submit(c)); controlBridge.consume(adapt);
        assert(controlBridge.complete(ok,a)); return ok; };
    assert(!apply()); // unsampled ATF cannot enable mode
    telemetry.atf_sampled=true;telemetry.atf_sample_ms=millis();telemetry.atf_range_evidence=1;
    telemetry.engine_rpm=1000; assert(!apply());
    telemetry.engine_rpm=0;telemetry.output_rpm=100; assert(!apply());
    telemetry.output_rpm=0;telemetry.test_mode=true; assert(!apply());
    telemetry.test_mode=false;telemetry.shift_phase=2; assert(!apply());
    telemetry.shift_phase=0; assert(apply()); assert(telemetry.atf_only_selector);
    c.action=ControlAction::TestMode;c.value=1; assert(!apply());
    c.action=ControlAction::AtfSelector;c.value=0;
    // Leaving the experimental mode must NOT depend on ATF health. Disabling hands
    // authority back to the TRRS and the normal resync path, so stale evidence must
    // not be able to strand the controller in the inferred-authority mode.
    g_now_ms+=21; assert(apply()); assert(!telemetry.atf_only_selector);
    // Enabling still demands fresh OPEN evidence, in both order of operations.
    telemetry.atf_sample_ms=millis();telemetry.atf_range_evidence=1;
    c.value=1; assert(apply()); assert(telemetry.atf_only_selector);
    // The lockout case: a sensor that fails reading "engaged" cannot re-enable the
    // mode, and critically cannot keep it on either — it is persisted to NVS, so a
    // reboot would not clear it.
    telemetry.atf_range_evidence=2;
    c.value=1; assert(!apply());
    c.value=0; assert(apply()); assert(!telemetry.atf_only_selector);
    // Movement still blocks BOTH directions: no mode switch while the shafts turn.
    telemetry.atf_range_evidence=1;telemetry.atf_sample_ms=millis();
    c.value=1; assert(apply()); assert(telemetry.atf_only_selector);
    telemetry.output_rpm=100; c.value=0; assert(!apply()); assert(telemetry.atf_only_selector);
    telemetry.output_rpm=0; assert(apply()); assert(!telemetry.atf_only_selector);
    source={};source.data.atf_only_selector=true;
    fillTelemetryJson(packet,source,1,170000,true);
    assert(packet["gear"].isNull() && packet["targetRatio"].isNull());
    std::cout << "Command validation and control handoff: PASS\n";
}
