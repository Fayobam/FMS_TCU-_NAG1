#include "AtfRangeObserver.h"
#include <cassert>
#include <limits>
#include <iostream>

int main() {
    using E = AtfRangeEvidence;
    AtfRangeObserver o;
    assert(o.update(3.2f,false,0)==E::Unknown);
    assert(o.update(3.2f,true,1)==E::Unknown);
    for (uint32_t t=11; t<101; t+=10) assert(o.update(3.2f,true,t)==E::Unknown);
    assert(o.update(3.2f,true,101)==E::OpenCircuit);
    assert(o.update(1.2f,true,102)==E::Unknown);
    assert(o.update(3.2f,true,110)==E::Unknown); // contact bounce restarts dwell
    assert(o.update(1.2f,true,111)==E::Unknown);
    for (uint32_t t=121; t<211; t+=10) assert(o.update(1.2f,true,t)==E::Unknown);
    assert(o.update(1.2f,true,211)==E::EngagedCircuit);
    assert(o.update(0.0f,true,212)==E::Unknown); // short is not P/N
    assert(o.update(std::numeric_limits<float>::quiet_NaN(),true,213)==E::Unknown);
    assert(o.update(4.0f,true,214)==E::Unknown);
    assert(o.update(3.0f,true,215)==E::Unknown); // threshold guard band
    o.reset();
    assert(o.update(1.2f,true,UINT32_MAX-50)==E::Unknown);
    for (uint32_t dt=10; dt<100; dt+=10)
        assert(o.update(1.2f,true,uint32_t(UINT32_MAX-50+dt))==E::Unknown);
    assert(o.update(1.2f,true,49)==E::EngagedCircuit); // millis wrap
    o.reset();
    assert(o.update(1.2f,true,500)==E::Unknown); // reboot never restores evidence
    assert(o.update(1.2f,true,1000)==E::Unknown); // missing samples cannot qualify
    for (uint32_t t=1010; t<1100; t+=10) assert(o.update(1.2f,true,t)==E::Unknown);
    assert(o.update(1.2f,true,1100)==E::EngagedCircuit);
    assert(o.update(1.2f,true,1121)==E::Unknown); // gap revokes established evidence
    assert(o.update(1.2f,false,1122)==E::Unknown);
    std::cout << "ATF range evidence: PASS (no actuation authority)\n";
}
