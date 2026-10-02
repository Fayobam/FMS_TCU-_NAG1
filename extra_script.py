"""PlatformIO pre-build: embed the entire interface; no uploadfs dependency."""
Import('env')
from pathlib import Path
import sys
project = Path(env['PROJECT_DIR'])
sys.path.insert(0, str(project / 'tools'))
from package_web import generate
print('TCU: bundled UI %d -> %d gzip bytes' % generate(project))
