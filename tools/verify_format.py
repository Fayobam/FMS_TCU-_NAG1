"""Prove that formatting data/ did not change what the browser renders.

Formatting HTML is only safe if no whitespace text node is introduced between
inline elements, which would render as a gap that was not there. Rather than
reason about that, render both versions and compare pixels.

Every .page section is un-hidden first, so a single full-page screenshot covers
all of the markup instead of only the default tab.

Run:  python tools/verify_format.py <dir-with-original-data>
"""
import sys
import threading
import functools
import shutil
import tempfile
from pathlib import Path
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.pio/browser-tools'))
sys.path.insert(0, str(ROOT / 'tools'))
from playwright.sync_api import sync_playwright
from package_web import bundle

VIEWPORTS = [(1440, 1000), (820, 1100), (390, 1400)]
SHOW_ALL = """
  document.querySelectorAll('.page').forEach(p => p.hidden = false);
  document.querySelectorAll('[hidden]').forEach(p => p.hidden = false);
"""


def shoot(page_bytes, data_dir, label):
    handler = functools.partial(Handler, directory=str(data_dir), page=page_bytes)
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = 'http://127.0.0.1:%d/' % server.server_port
    shots = {}
    with sync_playwright() as pw:
        # Same browser the suite in test_browser.py uses; no bundled Chromium here.
        browser = pw.chromium.launch(
            executable_path='C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',
            headless=True)
        for w, h in VIEWPORTS:
            page = browser.new_page(viewport={'width': w, 'height': h})
            page.goto(url, wait_until='load')
            page.evaluate(SHOW_ALL)
            # Charts redraw on a timer; settle well past it so the comparison
            # measures layout, not a race against the first paint.
            page.wait_for_timeout(1800)
            shots[(w, h)] = page.screenshot(full_page=True)
            page.close()
        browser.close()
    server.shutdown()
    print('  captured %s at %d viewports' % (label, len(shots)))
    return shots


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, page=b'', **kw):
        self._page = page
        super().__init__(*a, **kw)

    def do_GET(self):
        if self.path.split('?')[0] in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(self._page)
        else:
            super().do_GET()

    def log_message(self, *a):
        pass


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    original = Path(sys.argv[1])
    current = ROOT / 'data'

    after = shoot(bundle(ROOT), current, 'formatted')

    # Swap the original data/ in, bundle it, then put the formatted tree back.
    stash = Path(tempfile.mkdtemp()) / 'data'
    shutil.copytree(current, stash)
    try:
        for name in ('index.html', 'css/app.css'):
            shutil.copy(original / name, current / name)
        for js in (original / 'js').glob('*.js'):
            shutil.copy(js, current / 'js' / js.name)
        before = shoot(bundle(ROOT), current, 'original')
    finally:
        shutil.rmtree(current)
        shutil.copytree(stash, current)

    bad = [vp for vp in VIEWPORTS if before[vp] != after[vp]]
    for vp in VIEWPORTS:
        same = before[vp] == after[vp]
        print('  %-12s %s' % ('%dx%d' % vp, 'IDENTICAL' if same else 'DIFFERS'))
        if not same:
            out = ROOT / '.pio/browser-results'
            out.mkdir(parents=True, exist_ok=True)
            (out / ('before-%dx%d.png' % vp)).write_bytes(before[vp])
            (out / ('after-%dx%d.png' % vp)).write_bytes(after[vp])
    if bad:
        print('rendering CHANGED -- see .pio/browser-results/{before,after}-*.png')
        return 1
    print('rendering is pixel-identical: formatting is render-neutral')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
