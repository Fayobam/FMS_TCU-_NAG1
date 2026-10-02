#include "AtfRangeObserver.h"
#include <cmath>

AtfRangeEvidence AtfRangeObserver::update(float volts, bool sampled, uint32_t now) {
    // ATF normally samples every 3 ms. A gap cannot establish continuous dwell.
    if (!hasSample || now - lastSampleAt > 20) {
        candidate = stable = AtfRangeEvidence::Unknown;
        candidateSince = now;
    }
    lastSampleAt = now;
    hasSample = sampled;
    // Guard bands avoid chattering at the existing 0.1/3.0 V temperature limits.
    auto next = AtfRangeEvidence::Unknown;
    if (sampled && std::isfinite(volts)) {
        if (volts >= 3.05f && volts <= 3.3f) next = AtfRangeEvidence::OpenCircuit;
        else if (volts >= 0.15f && volts <= 2.95f) next = AtfRangeEvidence::EngagedCircuit;
    }
    if (next == AtfRangeEvidence::Unknown) {
        candidate = stable = next;
        candidateSince = now;
        return stable;
    }
    if (next != candidate) {
        candidate = next;
        candidateSince = now;
        stable = AtfRangeEvidence::Unknown;
    }
    if (now - candidateSince >= 100) stable = candidate;
    return stable;
}
