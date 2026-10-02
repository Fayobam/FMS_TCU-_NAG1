# Simulated protocol fixtures for offline preview and browser tests.
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def dtc_names():
    """Read the code names out of the firmware rather than restating them here.

    The preview used to carry its own copy of this list, which silently fell one
    code behind when DTC_ATF_CIRCUIT was added. Parsing the real array means the
    preview cannot drift, and a parse failure is loud rather than a short list.
    """
    source = (ROOT / 'src/DtcManager.cpp').read_text(encoding='utf-8')
    block = re.search(r'DTC_NAMES\[DTC_COUNT\]\s*=\s*\{(.*?)\};', source, re.S)
    if not block:
        raise SystemExit('web_fixtures: DTC_NAMES not found in src/DtcManager.cpp')
    names = re.findall(r'"([^"]*)"', block.group(1))
    if not names:
        raise SystemExit('web_fixtures: DTC_NAMES parsed empty')
    return names


profile=dict(type='profile_data',torque=[70+i*4 for i in range(64)],rpm=list(range(0,8000,1000)),map=[20,50,80,110,140,170,205,240],
    tmax=450,overrev=6300,lug=1100,engPpr=60,outPpr=24,clEn=0,clKp=80,kmhRpm=.038,transVariant=0,tcStall=200,tcCoupSr=85,
    clPwr=0,clSpeed=0,pFull=16000,coefStat=100,coefRel=120,coefCold=185,coefHot=140,tpsC=.5,tpsW=2.9,map0=-10,mapV=86,
    fillp=[80,82,88,78],fillt=[140,150,180,130],applyFric=[3800,4200,5000,3800],relFric=[3800,4200,5000,3800],applySpring=[600,700,900,600],relSpring=[600,700,900,600])
params=[dict(idx=0,id='line.map',name='Cruise line pressure',group='Shift feel',kind=2,min=10,max=100,rows=5,cols=16,unit='%',help='Holding pressure by gear and load. 100 = de-energized = maximum pressure.',v=[70+i//16*2+min(i%16*2,20) for i in range(80)]),
    dict(idx=8,id='auto.up',name='Upshift speeds',group='Auto schedule',kind=2,min=3,max=250,rows=4,cols=11,unit='km/h',help='Road speed by TPS. Upshift must exceed downshift.',v=[10+r*25+c*3 for r in range(4) for c in range(11)])]
network=dict(type='network',state='CONNECTED',mode=0,sta=True,ap=True,ssid='Workshop',ip='192.168.1.50',apIp='192.168.4.1',apSsid='7226-TCU',rssi=-54,hostname='tcu',mdns='tcu.local',mdnsActive=True,storageOk=True,known=[dict(ssid='Workshop',secured=True),dict(ssid='<img src=x onerror=alert(1)>',secured=False)])
telemetry=dict(type='telemetry',sampledMs=123400,prnd='D',gear=3,tgt=4,mode=1,modeName='STANDARD AUTO',engRpm=3240,turbRpm=3020,outRpm=2012,tps=38.4,map=114,mpc=78,spc=64,shiftTime=420,ratio=1.486,kmh=76.5,flare=False,bind=False,tccPwm=24,tccTarget=50,tccActual=220,limp=False,limpReason='',safety='No active safety intervention',atfTemp=84.2,htMode=False,phase=4,revAbuse=False,testMode=False,tpsV=1.42,mapV=1.44,atfV=1.1,din=12,tout=0,n2=3020,n3=3020,tEstNm=192,loadPct=42,shiftClass=0,pdType=0,onClutch=180,offClutch=410,tInput=210,tqCut=False,dtcN=0,spdHwOk=True,inTrust=True,tpsOk=True,mapOk=True,clients=1,expectedRatio=1.486,heap=160000,fsOk=False,assets='embedded',atfSignalOk=True)

telemetry.update(n2Recent=True,n3Recent=True,outRecent=True,engRecent=True,atfSource="live",atfCircuit="temperature",atfMeasuredC=84.2,atfLastValidMs=123400,batteryV=None,solenoidCurrentA=None,batterySupported=False,currentSupported=False)

# Control-loop health and throttle rate. loopMaxUs is the number to read after a
# drive: the 1 kHz budget is 1000 us, and a healthy loop stays near it.
telemetry.update(intervalMs=50,tpsRocPctMs=0.0,loopMaxUs=1180,loopOverrunSoft=3,loopOverrunHard=0)
