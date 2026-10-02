"""Run firmware contract tests with the installed host compiler (no hardware)."""
from pathlib import Path
import shutil, subprocess, os
ROOT=Path(__file__).resolve().parents[1]
os.chdir(ROOT)
compiler=shutil.which('g++') or str(Path.home()/'scoop/apps/gcc/current/bin/g++.exe')
out=ROOT/'.pio/host-contract-tests';out.mkdir(parents=True,exist_ok=True)
includes=['-Itest/stubs','-Isrc','-I.pio/libdeps/esp32doit-devkit-v1/ArduinoJson/src']
common=[compiler,'-std=c++17','-DUNIT_TEST','-O2',*includes]
core=['src/InputManager.cpp','src/AtfRangeObserver.cpp','src/ShiftSelector.cpp','src/ShiftScheduler.cpp','src/ShiftPhases.cpp','src/ShiftCalibration.cpp','src/ShiftSafety.cpp','src/ShiftBench.cpp','src/SolenoidDriver.cpp','src/EngineProfile.cpp','src/AdaptiveMemory.cpp','src/DtcManager.cpp','src/TuneOverlay.cpp']
tests=[('atf-range',['src/AtfRangeObserver.cpp','test/test_atf_range/test_atf_range.cpp'],[]),
       ('control-regression',[*core,'test/test_gear_control/test_gear_control.cpp','.pio/libdeps/esp32doit-devkit-v1/Unity/src/unity.c'],['-I.pio/libdeps/esp32doit-devkit-v1/Unity/src']),
       ('web-contract',[*core,'src/ControlBridge.cpp','src/CommandValidation.cpp','src/TelemetryJson.cpp','test/test_web_contract/test_web_contract.cpp'],[]),
       ('network',['src/NetworkManager.cpp','test/test_network/test_network.cpp'],['-Itest/network_stubs','-include','cassert'])]
for name,sources,flags in tests:
    binary=out/(name+'.exe')
    subprocess.run([*common,*flags,*sources,'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
