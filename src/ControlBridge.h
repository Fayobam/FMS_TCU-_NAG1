#pragma once
#include <atomic>
#include "EngineProfile.h"
#include "TuneOverlay.h"
#include "AdaptiveMemory.h"

// Single producer (service task), single consumer (control task). No JSON,
// networking, allocation, persistence or waiting in consume().
enum class ControlAction : uint8_t { Profile, Tune, Cells, TestMode, Selector,
    Paddle, Output, Nudge, LimpReset, ClearDtcs, AtfSelector };
struct ControlCommand {
    ControlAction action;
    EngineProfileData profile;
    TuneData tune;
    AdaptCell cells[ADAPT_CLASSES * ADAPT_SHIFTS * ADAPT_TBINS];
    int value = 0;
    int extra = 0;
};
struct ControlSnapshot {
    TCU_Telemetry data;
    AdaptCell cells[ADAPT_CLASSES * ADAPT_SHIFTS * ADAPT_TBINS];
    uint32_t sampledMs = 0;
    float expectedRatio = 0;
    float targetRatio = 0;
};
class ControlBridge {
    std::atomic<uint8_t> state{0}; // idle, pending, completed
    ControlCommand command{};
    bool accepted = false;
    portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;
    ControlSnapshot snapshot{};
    uint32_t lastSample = 0;
public:
    bool submit(const ControlCommand& cmd);
    bool complete(bool& ok, ControlAction& action);
    void consume(AdaptiveMemory& adaptives);
    void publish(AdaptiveMemory& adaptives);
    ControlSnapshot read();
};
extern ControlBridge controlBridge;
