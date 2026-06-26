# Vendored uPlot (offline)

- uPlot v1.6.32 (https://github.com/leeoniya/uPlot), MIT license
- Source: npm `uplot@1.6.32` tarball (registry.npmjs.org)
- tarball sha1: c800a63b432bad692d6d746f44f0882aa73a49ae (verified)

## Files
- uPlot.iife.min.js  — IIFE build, exposes global `uPlot` (51 KB)
- uPlot.min.css      — stylesheet (1.9 KB)

## Place into the repo at:
    rb_gui/rb_servo_gui/static/scope/uPlot.iife.min.js
    rb_gui/rb_servo_gui/static/scope/uPlot.min.css

Referenced by static/scope/index.html via relative paths:
    <link rel="stylesheet" href="./uPlot.min.css">
    <script src="./uPlot.iife.min.js"></script>   // global `uPlot`

No CDN / external requests — closed-network safe.
