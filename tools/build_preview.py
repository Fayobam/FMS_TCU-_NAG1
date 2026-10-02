"""Generate a file:// compatible HTML preview with clearly simulated data."""
from pathlib import Path
import json
from package_web import bundle
from web_fixtures import profile, params, network, telemetry
from preview_maps import extend
import copy

ROOT=Path(__file__).resolve().parents[1]

def build():
    fixtures=dict(profile=profile,params=extend(copy.deepcopy(params)),network=network,telemetry=telemetry,
        cells=[v for i in range(64) for v in (i%3-1,(i%5-2)*2,i%7-3)],
        dtcs=[dict(code=i,name=name,count=0,active=False,lastMs=0) for i,name in enumerate([
            'SPEED N2/N3 MISMATCH','SPEED HW INIT FAIL','TPS SIGNAL RAILED','MAP SIGNAL RAILED','LIMP: FATAL SLIP',
            'REVERSE AT SPEED','OVERREV UPSHIFT','LOOP OVERRUN','SHIFT UNVERIFIED','TEST MODE USED'])])
    # Friendly simulated networks; no stored or real credentials enter this file.
    fixtures['network']={**network,'known':[{'ssid':'Workshop','secured':True},{'ssid':'Phone hotspot','secured':True}]}
    html=bundle(ROOT).decode('utf-8')
    mock='window.__PREVIEW_DATA='+json.dumps(fixtures).replace('</','<\\/')+';\n'+(ROOT/'tools/preview.js').read_text(encoding='utf-8')
    html=html.replace('<script>','<script>\n'+mock+'\n</script><script>',1)
    banner='<div class="notice warning" id="preview-banner"><strong>OFFLINE PREVIEW</strong> · Simulated telemetry. All edits stay in this browser and reset when you reload. No controller connection is made.</div>'
    html=html.replace('<div id="offline"',banner+'<div id="offline"',1)
    html=html.replace('<title>722.6 · Control Studio</title>','<title>OFFLINE PREVIEW · 722.6 Control Studio</title>')
    out=ROOT/'preview.html';out.write_text(html,encoding='utf-8');print('Created',out)
    return out

if __name__=='__main__':build()
