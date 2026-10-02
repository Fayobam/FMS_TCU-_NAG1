"""Headless Edge tests against a simulated TCU protocol, never real hardware.

Install playwright into .pio/browser-tools, then: python tools/test_browser.py
Screenshots and test artifacts go to .pio/browser-results.
"""
from pathlib import Path
import sys, json, threading, functools
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'.pio/browser-tools'))
from playwright.sync_api import sync_playwright
from package_web import bundle

ART=ROOT/'.pio/browser-results';ART.mkdir(parents=True,exist_ok=True)
page_bytes=bundle(ROOT)
class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path.split('?')[0] in ('/','/index.html'):
            self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.end_headers();self.wfile.write(page_bytes)
        else: super().do_GET()
    def log_message(self,*args): pass
server=ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Handler,directory=str(ROOT/'data')))
threading.Thread(target=server.serve_forever,daemon=True).start()
url='http://127.0.0.1:%d/' % server.server_port

from web_fixtures import profile, params, network, telemetry
commands=[];sockets=[];errors=[]
def websocket(ws):
    sockets.append(ws)
    def receive(message):
        d=json.loads(message);commands.append(d);cmd=d['cmd']
        ws.send(json.dumps(dict(type='command_result',requestId=d['requestId'],ok=True,message='OK')))
        payload=None
        if cmd=='get_profile':payload=profile
        elif cmd=='param.list':payload=dict(type='param_list',n=len(params),params=params)
        elif cmd=='get_cells':payload=dict(type='cell_data',classes=4,shifts=4,tbins=4,data=[0,2,-1]*64)
        elif cmd=='get_dtcs':payload=dict(type='dtc_data',dtcs=[dict(code=0,name='SPEED N2/N3 MISMATCH',count=0,active=False,lastMs=0)])
        elif cmd=='network.get':payload=network
        if payload:ws.send(json.dumps(payload))
    ws.on_message(receive)

with sync_playwright() as p:
    browser=p.chromium.launch(executable_path='C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',headless=True)
    context=browser.new_context(viewport=dict(width=1920,height=1080))
    context.route_web_socket('**/ws',websocket)
    page=context.new_page();page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(url);page.wait_for_function('window.TCU && TCU.connected');page.wait_for_function('document.querySelectorAll("#torque-grid td").length === 64')
    page.wait_for_timeout(300)
    assert all(x['cmd'] in ('get_profile','param.list','get_cells','get_dtcs','network.get') for x in commands),'Page load wrote configuration'
    assert page.locator('#map-grid td').count()==80
    def sample():
        telemetry['sampledMs']+=100;sockets[-1].send(json.dumps(telemetry));page.wait_for_timeout(120)
    for _ in range(15):sample()
    # An open socket repeating a frozen snapshot is still stale.
    for _ in range(5):
        sockets[-1].send(json.dumps(telemetry));page.wait_for_timeout(450)
    assert not page.evaluate('TCU.isFresh()')
    assert page.evaluate('TCU.delivery().state')=='snapshot_stale'
    sample()
    assert page.evaluate('TCU.isFresh()')
    assert page.locator('#offline').is_hidden()
    # A newly received but already-old controller snapshot cannot clear the warning.
    telemetry['snapshotAgeMs']=3000;sample()
    assert not page.evaluate('TCU.isFresh()')
    telemetry['snapshotAgeMs']=0;sample()
    assert page.evaluate('TCU.isFresh()')
    sockets[-1].send('{broken');page.wait_for_timeout(50)
    assert page.evaluate('TCU.delivery().invalidPackets')==1
    sample()
    # Live ADC voltage remains read-only and separate from the MAP slope field.
    assert page.locator('[data-sensor-reading="tpsV"]').inner_text()=='1.420 V'
    assert page.locator('[data-sensor-reading="mapV"]').inner_text()=='1.440 V'
    assert page.locator('#profile-fields input[data-key="mapV"]').input_value()=='86'
    # Held controller temperature must not masquerade as a live measurement.
    telemetry.update(atfSignalOk=False,atfMeasuredC=None,atfSource='held',atfCircuit='pn_or_open')
    sample()
    assert page.locator('[data-field="atfTemp"]').inner_text()=='—'
    assert 'held' in page.locator('#atf-reading-status').inner_text()
    assert 'P/N contact open or wiring open' in page.locator('#health-grid').inner_text()
    assert 'Sensing not configured' in page.locator('#health-grid').inner_text()
    telemetry.update(atfSignalOk=True,atfMeasuredC=84.2,atfSource='live',atfCircuit='temperature')
    sample()
    # Browser layouts use mock data and the exact embedded bundle, without a filesystem.
    for name,width,height in [('desktop',1920,1080),('laptop',1366,768),('tablet',820,1180),('phone',390,844)]:
        page.set_viewport_size(dict(width=width,height=height));sample();page.screenshot(path=str(ART/(name+'.png')),full_page=True)
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth+1'),name+' overflow'
    page.set_viewport_size(dict(width=1366,height=768))
    for name in ['analysis','adaptation','profile','diagnostics','network','bench','maps']:
        page.locator('nav a[href="#'+name+'"]').click();sample();assert page.locator('#'+name).is_visible()
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth+1'),name+' overflow'
    # Editing is local; dirty values survive broadcasts from another client.
    before=len(commands);cell=page.locator('#map-grid td').first;cell.dblclick();cell.locator('input').fill('82');cell.locator('input').press('Enter')
    assert page.locator('#map-dirty').inner_text()=='Unsaved changes';assert len(commands)==before
    sockets[-1].send(json.dumps(dict(type='param_list',params=params)));page.wait_for_timeout(100);assert cell.inner_text()=='82'
    cell.dblclick();cell.locator('input').fill('200');cell.locator('input').press('Enter');assert cell.inner_text()=='82'
    # Apply only after explicit confirmation; acknowledgement drives UI state.
    page.locator('#map-apply').click();assert len(commands)==before
    page.locator('#confirm-dialog button[value="confirm"]').click();page.wait_for_function('document.querySelector("#map-dirty").textContent.includes("not saved")')
    assert any(d['cmd']=='param.set' and d['val']==82 and d['idx']==0 and d['row']==0 and d['col']==0 for d in commands)
    page.screenshot(path=str(ART/'maps.png'),full_page=True)
    page.locator('nav a[href="#network"]').click();page.wait_for_timeout(200)
    assert page.locator('#known-networks img').count()==0,'SSID interpreted as markup'
    assert 'password1' not in page.content()
    page.locator('nav a[href="#bench"]').click();assert page.locator('#bench-controls').is_hidden()
    page.locator('#bench-enable').click();assert page.locator('#confirm-dialog').is_visible();page.locator('#confirm-dialog button[value="cancel"]').click()
    assert not any(d['cmd']=='test_mode' for d in commands)
    page.locator('nav a[href="#profile"]').click();sample()
    before=len(commands)
    page.locator('#selector-enable').click()
    assert len(commands)==before
    page.locator('#confirm-dialog button[value="cancel"]').click()
    assert len(commands)==before
    page.locator('#selector-enable').click()
    page.locator('#confirm-dialog button[value="confirm"]').click()
    page.wait_for_timeout(150)
    assert any(d['cmd']=='selector.atf' and d['on'] is True for d in commands)
    telemetry.update(atfOnly=True,forwardConfirmed=False,gear=None,tgt=None,expectedRatio=None,targetRatio=None)
    sample()
    assert 'GEAR UNKNOWN' in page.locator('#selector-status').inner_text()
    assert page.locator('#selector-enable').is_disabled()
    telemetry.update(atfOnly=False,forwardConfirmed=False,gear=3,tgt=3)
    sample()
    # Socket drop, stale display, reconnect, repeated reload: no mutating replay.
    writes=[d for d in commands if d['cmd'] not in ('get_profile','param.list','get_cells','get_dtcs','network.get')]
    old=len(sockets);sockets[-1].close();page.wait_for_function('!TCU.connected');page.wait_for_function('TCU.connected',timeout=20000);assert len(sockets)>old
    sample()
    # Discard navigation prompt in this test only; controller state is not touched.
    page.on('dialog',lambda d:d.accept())
    for _ in range(3):page.reload();page.wait_for_function('TCU.connected');page.wait_for_timeout(200)
    assert len(writes)==len([d for d in commands if d['cmd'] not in ('get_profile','param.list','get_cells','get_dtcs','network.get')])
    page.wait_for_timeout(2300);assert page.locator('#offline').is_visible()
    # Independent page sessions can coexist and reload without mutation replay.
    extra_pages=[]
    for _ in range(3):
        other=context.new_page();other.on('pageerror',lambda e:errors.append(str(e)));other.goto(url)
        other.wait_for_function('TCU.connected');extra_pages.append(other)
    for other in extra_pages:other.close()
    # The deliverable preview must work directly from disk without HTTP or Wi-Fi.
    from build_preview import build
    preview_path=build()
    preview_context=browser.new_context(viewport=dict(width=1366,height=768),offline=True)
    preview_page=preview_context.new_page();preview_page.on('pageerror',lambda e:errors.append(str(e)))
    external=[]
    preview_page.on('request',lambda r:external.append(r.url) if r.url.startswith(('http:','https:','ws:','wss:')) else None)
    preview_page.goto(preview_path.as_uri());preview_page.wait_for_function('TCU.isFresh()')
    assert preview_page.locator('#preview-banner').is_visible()
    assert preview_page.locator('#map-select option').count()==17
    preview_page.wait_for_timeout(1500)
    preview_page.screenshot(path=str(ART/'offline-preview.png'),full_page=True)
    assert not external,external
    # Each tab must remain usable with the network disabled.
    for name in ['analysis','maps','adaptation','profile','diagnostics','network','bench']:
        preview_page.locator('nav a[href="#'+name+'"]').click()
        preview_page.locator('#'+name).wait_for(state='visible')
        assert preview_page.locator('#'+name).is_visible()
    preview_page.locator('nav a[href="#maps"]').click();preview_page.locator('#maps').wait_for(state='visible')
    first=preview_page.locator('#map-grid td').first
    preview_page.evaluate('''() => {
      const transfer = new DataTransfer(); transfer.setData('text/plain','80\\t81\\n82\\t83');
      document.querySelector('#map-grid td').dispatchEvent(new ClipboardEvent('paste',{clipboardData:transfer,bubbles:true,cancelable:true}));
    }''')
    assert first.inner_text()=='80'
    assert preview_page.locator('#map-grid td').nth(17).inner_text()=='83'
    first.focus();first.press('ArrowRight');assert preview_page.locator('#map-grid td').nth(1).evaluate('(e)=>e===document.activeElement')
    assert not errors,errors
    browser.close()
server.shutdown()
print('Browser tests: layout at four sizes, all pages, staged edit, validation, confirmation, ACK, safe SSID, reconnect, no write replay, reloads and stale telemetry: PASS')
print('Screenshots:',ART)
