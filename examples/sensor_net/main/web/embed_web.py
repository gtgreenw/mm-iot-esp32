#!/usr/bin/env python3
"""
Embed web assets (HTML template + CSS + JS) into a single C source file
for the Sensor Gateway dashboard. Run from main/ with:
  python3 web/embed_web.py
"""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MAIN_DIR = os.path.dirname(SCRIPT_DIR)
WEB_DIR = SCRIPT_DIR
SRC_DIR = os.path.join(MAIN_DIR, "src")


def c_escape(s):
    """Escape a string for use inside a C string literal."""
    out = []
    for c in s:
        if c == "\\":
            out.append("\\\\")
        elif c == '"':
            out.append('\\"')
        elif c == "\n":
            out.append("\\n")
        elif c == "\r":
            out.append("\\r")
        elif c == "\t":
            out.append("\\t")
        elif ord(c) < 32 or ord(c) > 126:
            o = ord(c)
            # Use \uNNNN for BMP (avoids \xNN followed by hex digit = out-of-range in C)
            if 0x100 <= o <= 0xFFFF:
                out.append("\\u%04x" % o)
            else:
                out.append("\\x%02x" % (o & 0xFF))
        else:
            out.append(c)
    return "".join(out)


def embed():
    """Read web/* and write src/dashboard_embedded.c."""
    html_path = os.path.join(WEB_DIR, "dashboard.html")
    css_path = os.path.join(WEB_DIR, "dashboard.css")
    js_path = os.path.join(WEB_DIR, "dashboard.js")
    out_path = os.path.join(SRC_DIR, "dashboard_embedded.c")
    for p in (html_path, css_path, js_path):
        if not os.path.isfile(p):
            print("Missing: %s" % p, file=sys.stderr)
            sys.exit(1)
    with open(html_path, "r", encoding="utf-8") as f:
        html = f.read()
    with open(css_path, "r", encoding="utf-8") as f:
        css = f.read()
    with open(js_path, "r", encoding="utf-8") as f:
        js = f.read()
    html = html.replace("{{INLINE_CSS}}", css).replace("{{INLINE_JS}}", js)
    # Output as C: one long string, split into chunks to avoid compiler limits.
    # Never end a chunk with backslash (would escape the next chunk's opening quote).
    # In C, \x consumes all following hex digits; avoid \xNN followed by 0-9a-fA-F in the same literal.
    escaped = c_escape(html)
    chunk_size = 2000
    hex_digits = set("0123456789abcdefABCDEF")
    chunks = []
    i = 0
    while i < len(escaped):
        end = min(i + chunk_size, len(escaped))
        # Force a chunk break after any \xNN when the next character is a hex digit
        last_hex_escape_end = -1
        j = i
        while j <= end - 5 and j < len(escaped):
            if (
                escaped[j : j + 2] == "\\x"
                and escaped[j + 2] in hex_digits
                and escaped[j + 3] in hex_digits
                and escaped[j + 4] in hex_digits
            ):
                last_hex_escape_end = j + 4
            j += 1
        if last_hex_escape_end >= 0:
            end = last_hex_escape_end
        if end < len(escaped) and escaped[end - 1] == "\\":
            end -= 1
        chunks.append(escaped[i:end])
        i = end
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(
            "/* Auto-generated from main/web/ by embed_web.py. Do not edit. */\n"
            '#include "sensor_gateway_http.h"\n\n'
            "static const char s_html[] =\n"
        )
        for c in chunks:
            f.write('  "%s"\n' % c)
        f.write(";\n\n")
        f.write("const char *sensor_gateway_get_dashboard_html(void) { return s_html; }\n\n")
        f.write("size_t sensor_gateway_get_dashboard_html_len(void)\n")
        f.write("{\n")
        f.write("    return sizeof(s_html) - 1;\n")
        f.write("}\n")
    print("Wrote %s" % out_path)


if __name__ == "__main__":
    embed()
