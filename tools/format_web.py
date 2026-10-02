"""Format the dashboard source for review.

Minified source saves ~650 gzip bytes (0.03% of the 2 MB app partition) and costs
the entire reviewability of data/: an 11 KB CSS line cannot be diffed, reviewed or
merged, and a one-word change shows up as that whole line changing. The pre-build
hook gzips the bundle either way, so formatting is effectively free.

CSS and JS go through js-beautify. HTML uses the conservative pass below, because a
general HTML formatter will happily turn `</span><span>` into two lines, which adds
a whitespace text node and renders a gap that was not there.

Install the formatters once:
    python -m pip install --target .pio/web-tools jsbeautifier cssbeautifier

Run:
    python tools/format_web.py            rewrite data/ in place
    python tools/format_web.py --check    non-zero exit if reformatting is due
"""
import sys
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.pio/web-tools'))
import jsbeautifier
import cssbeautifier

JS = jsbeautifier.default_options()
JS.indent_size = 2
JS.end_with_newline = True
JS.preserve_newlines = True
JS.max_preserve_newlines = 2
JS.brace_style = 'collapse'
JS.wrap_line_length = 100

CSS = cssbeautifier.default_options()
CSS.indent_size = 2
CSS.end_with_newline = True
CSS.newline_between_rules = True

# Elements whose contents are emitted byte-for-byte. Breaking inside or between
# these can introduce rendering whitespace, so they are never split.
INLINE = {
    'span', 'b', 'i', 'em', 'strong', 'small', 'label', 'a', 'option', 'td', 'th',
    'code', 'summary', 'title', 'button', 'select', 'h1', 'h2', 'h3', 'p', 'legend',
    'script', 'style', 'textarea', 'pre',
}
# Tags that never nest, so they must not open an indent level.
VOID = {
    'meta', 'link', 'br', 'hr', 'img', 'input', 'source', 'col', 'area', 'base', 'wbr',
}

# One tag, or one run of text. Attribute values may contain '>', so quoted runs are
# matched explicitly rather than scanning to the next '>'.
TOKEN = re.compile(r'<[^>"\']*(?:"[^"]*"|\'[^\']*\'[^>"\']*)*[^>]*>|[^<]+')
TAG_NAME = re.compile(r'</?\s*([a-zA-Z0-9-]+)')


def format_html(text):
    out, indent, skip = [], 0, 0
    for tok in TOKEN.findall(text):
        is_tag = tok.startswith('<')
        if not is_tag and not tok.strip():
            continue                       # inter-tag whitespace: the indent replaces it
        name, closing, selfclose = '', False, False
        if is_tag:
            match = TAG_NAME.match(tok)
            name = match.group(1).lower() if match else ''
            closing = tok.startswith('</')
            # A nameless tag is <!doctype ...> or a comment: never an indent level.
            selfclose = tok.rstrip().endswith('/>') or name in VOID or not name
        if skip:                           # inside an inline element: verbatim
            out[-1] += tok
            if is_tag and not selfclose and name in INLINE:
                skip += -1 if closing else 1
            continue
        if not is_tag:
            out.append('  ' * indent + tok.strip())
            continue
        if closing:
            indent = max(0, indent - 1)
        out.append('  ' * indent + tok)
        if not closing and not selfclose:
            if name in INLINE:
                skip = 1
            else:
                indent += 1
    return '\n'.join(out) + '\n'


def targets():
    yield 'data/index.html', format_html
    yield 'data/css/app.css', lambda t: cssbeautifier.beautify(t, CSS)
    for path in sorted((ROOT / 'data/js').glob('*.js')):
        yield path.relative_to(ROOT).as_posix(), lambda t: jsbeautifier.beautify(t, JS)


def main():
    check = '--check' in sys.argv
    stale = []
    for rel, fmt in targets():
        path = ROOT / rel
        before = path.read_text(encoding='utf-8')
        after = fmt(before)
        if before == after:
            print('  unchanged  %s' % rel)
            continue
        stale.append(rel)
        if check:
            continue
        path.write_text(after, encoding='utf-8')
        print('  formatted  %s: longest line %d -> %d' % (
            rel,
            max(len(l) for l in before.splitlines()),
            max(len(l) for l in after.splitlines())))
    if check and stale:
        print('needs formatting: ' + ', '.join(stale))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
