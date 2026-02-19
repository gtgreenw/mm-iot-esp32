# Dashboard web assets

The gateway dashboard is built from editable HTML, CSS, and JS in this directory and embedded into the firmware at build time.

## Files

- **dashboard.html** – Page structure and body. Placeholders `{{INLINE_CSS}}` and `{{INLINE_JS}}` are replaced with the contents of `dashboard.css` and `dashboard.js`.
- **dashboard.css** – All styles (themes, layout, panels).
- **dashboard.js** – All dashboard logic (fetch, tabs, sensors, env graph, BLE, etc.).
- **embed_web.py** – Script run by CMake to produce `src/dashboard_embedded.c` from the three assets above.

## Build flow

1. CMake runs `python3 web/embed_web.py` when any of `dashboard.html`, `dashboard.css`, or `dashboard.js` (or the script) changes.
2. The script writes `src/dashboard_embedded.c`, which defines `sensor_gateway_get_dashboard_html()` and `sensor_gateway_get_dashboard_html_len()`.

## Editing

Edit `dashboard.html`, `dashboard.css`, and `dashboard.js` as normal. Rebuild the project; the embed step runs automatically. No need to touch C.

