#pragma once
#include <stdint.h>

// Observation only. An open ATF circuit is not proof of Park/Neutral, and a
// speed magnitude is not a direction sensor. This class never authorizes outputs.
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
