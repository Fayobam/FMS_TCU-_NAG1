#pragma once
#include <stdint.h>

// Observation only. An open ATF circuit is not proof of Park/Neutral, and a
// speed magnitude is not a direction sensor. This class never authorizes outputs.
// How stale a sample may be and still count as CURRENT evidence. Was 20 ms, which
// had no physical justification and coupled gear authority to control-loop jitter:
// an NVS commit on core 0 disables the flash cache and can stall core 1 for tens of
// ms, which revoked authority and de-energized the valve body at road speed. The
// window only has to be short enough to notice the lever physically moving out of a
// forward range — a gear cannot change otherwise without the TCU commanding it — and
// 100 ms is ample for that while being far longer than any plausible stall.
constexpr uint32_t ATF_EVIDENCE_FRESH_MS = 100;

// Separate meaning: how long one reading must hold before it is believed at all.
constexpr uint32_t ATF_QUALIFY_MS = 100;

enum class AtfRangeEvidence : uint8_t { Unknown, OpenCircuit, EngagedCircuit };
class AtfRangeObserver {
    AtfRangeEvidence candidate = AtfRangeEvidence::Unknown;
    AtfRangeEvidence stable = AtfRangeEvidence::Unknown;
    uint32_t candidateSince = 0;
    uint32_t lastSampleAt = 0;
    bool hasSample = false;
public:
    // Call for each fresh ADC acquisition; sampled=false invalidates evidence.
    AtfRangeEvidence update(float volts, bool sampled, uint32_t now);
    void reset() { *this = AtfRangeObserver{}; }
};
