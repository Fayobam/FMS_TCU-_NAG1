#!/usr/bin/env python
"""
Static syntax gate for data/index.html.

The dashboard is one large inline script with no build step, so a single parse-time
error takes the WHOLE page down: no telemetry, no tabs, nothing. That failure is
silent from the firmware's side - the file serves fine, it just never runs.

This caught a real one: a second top-level `const PHASE_NAMES` added by a new tab,
which is a SyntaxError ("Identifier has already been declared") and killed every tab
at once. Bracket counting does NOT catch that, so both checks are here.

Run:  python tools/check_dashboard.py data/index.html
Exit: 0 clean, 1 problem.
"""
import re, sys, collections

BS = chr(92)

def strip_and_check(code):
    """Walk the code string-, comment- and template-aware; return (errors, unclosed)."""
    i, n, line = 0, len(code), 1
    stack, errs, out = [], [], []
    while i < n:
        ch = code[i]
        if ch == '\n':
            line += 1; i += 1; out.append('\n'); continue
        if ch == '/' and i + 1 < n:
            if code[i+1] == '/':
                j = code.find('\n', i); i = n if j < 0 else j; continue
            if code[i+1] == '*':
                j = code.find('*/', i + 2)
                if j < 0:
                    errs.append((line, 'unterminated block comment')); break
                line += code[i:j].count('\n'); i = j + 2; continue
        if ch in '"\'`':
            q, j = ch, i + 1
            while j < n:
                if code[j] == BS: j += 2; continue
                if code[j] == q: break
                if code[j] == '\n':
                    if q != '`':
                        errs.append((line, 'unterminated %s string' % q)); break
                    line += 1
                j += 1
            i = j + 1; continue
        if ch in '([{':
            stack.append((ch, line)); i += 1; out.append(ch); continue
        if ch in ')]}':
            if not stack:
                errs.append((line, 'unmatched closing %s' % ch))
            else:
                o, ol = stack.pop()
                if '([{'.index(o) != ')]}'.index(ch):
                    errs.append((line, '%s closes %s opened on line %d' % (ch, o, ol)))
            i += 1; out.append(ch); continue
        out.append(ch); i += 1
    return errs, stack, ''.join(out)

def top_level_redeclarations(code):
    """
    Find const/let declared twice at the SAME brace depth 0 of this script.
    Depth is tracked on the comment/string-stripped text so braces inside strings
    do not confuse it.
    """
    depth, line, i, n = 0, 1, 0, len(code)
    seen = collections.defaultdict(list)
    decl_re = re.compile(r'(const|let)\s+([A-Za-z_$][\w$]*)')
    # re-walk with the same stripper, but record depth at each declaration
    _, _, stripped = strip_and_check(code)
    for m in decl_re.finditer(stripped):
        d = stripped[:m.start()].count('{') - stripped[:m.start()].count('}')
        if d == 0:
            ln = stripped[:m.start()].count('\n') + 1
            seen[m.group(2)].append((m.group(1), ln))
    return {k: v for k, v in seen.items() if len(v) > 1}

def main(path):
    src = open(path, encoding='utf-8', errors='replace').read()
    blocks = re.findall(r'<script[^>]*>(.*?)</script>', src, re.S)
    bad = False
    print('%s: %d inline script block(s)' % (path, len(blocks)))
    for bi, code in enumerate(blocks):
        if len(code) > 200000:
            print('  block %d: %d bytes (vendor library, skipped)' % (bi, len(code))); continue
        errs, unclosed, _ = strip_and_check(code)
        dups = top_level_redeclarations(code)
        ok = not errs and not unclosed and not dups
        print('  block %d: %d bytes -> %s' % (bi, len(code), 'OK' if ok else 'PROBLEM'))
        for l, e in errs[:10]:
            print('      line %d: %s' % (l, e)); bad = True
        for o, ol in unclosed[:10]:
            print("      unclosed '%s' opened on line %d" % (o, ol)); bad = True
        for name, where in sorted(dups.items()):
            print('      top-level redeclaration of %r: %s  <-- SyntaxError, kills the page'
                  % (name, ', '.join('%s line %d' % (k, l) for k, l in where))); bad = True
    return 1 if bad else 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else 'data/index.html'))
