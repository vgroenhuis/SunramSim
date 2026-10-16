#!/usr/bin/env python3
"""Extract the closed contours of a CorelDRAW (.cdr) drawing as JSON, in millimetres.

Requires `cdr2xhtml` from libcdr (Debian/Ubuntu: apt install libcdr-tools).

    python3 tools/cdr_contours.py drawing.cdr > contours.json

Output: {"contours": [{"stroke": "#rrggbb", "points": [[x, y], ...],
                        "circle": [cx, cy, r] | null, "bbox": [x0, y0, x1, y1]}, ...]}
Coordinates are in mm with +y up (the drawing's y axis is flipped). Bezier segments are
flattened to polylines; contours that fit a circle within 0.5 % are reported as circles.
"""
import json
import re
import subprocess
import sys

PT = 25.4 / 72  # cdr2xhtml writes SVG user units in points


def bezier(p0, p1, p2, p3, n):
    out = []
    for k in range(1, n + 1):
        s = k / n
        a, b, c, d = (1 - s) ** 3, 3 * (1 - s) ** 2 * s, 3 * (1 - s) * s ** 2, s ** 3
        out.append((a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0], a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1]))
    return out


def parse_path(d, segs_per_curve=8):
    toks = re.findall(r'[MmCcLlZz]|-?\d*\.?\d+(?:[eE]-?\d+)?', d)
    contours, pts, cur, cmd, i = [], [], None, None, 0
    def flush():
        nonlocal pts
        if len(pts) > 2:
            contours.append(pts)
        pts = []
    while i < len(toks):
        t = toks[i]
        if t.isalpha():
            cmd = t.upper()
            i += 1
            if cmd in 'MZ':
                flush()
            continue
        if cmd in ('M', 'L'):
            cur = (float(toks[i]), float(toks[i + 1])); i += 2
            pts.append(cur)
            cmd = 'L'
        elif cmd == 'C':
            c = [float(v) for v in toks[i:i + 6]]; i += 6
            pts.extend(bezier(cur, c[0:2], c[2:4], c[4:6], segs_per_curve))
            cur = (c[4], c[5])
        else:
            i += 1
    flush()
    return contours


def fit_circle(pts):
    n = len(pts)
    cx = (min(p[0] for p in pts) + max(p[0] for p in pts)) / 2
    cy = (min(p[1] for p in pts) + max(p[1] for p in pts)) / 2
    rs = [((p[0] - cx) ** 2 + (p[1] - cy) ** 2) ** 0.5 for p in pts]
    r = sum(rs) / n
    if r > 0 and max(abs(x - r) for x in rs) / r < 0.005:
        return [round(cx, 4), round(cy, 4), round(r, 4)]
    return None


def main(path):
    svg = subprocess.run(['cdr2xhtml', path], check=True, capture_output=True, text=True).stdout
    out = []
    for d, attrs in re.findall(r'<svg:path\s+d="([^"]*)"([^>]*)>', svg):
        m = re.search(r'stroke:\s*(#[0-9a-fA-F]{6})', attrs)
        for c in parse_path(d):
            pts = [(x * PT, -y * PT) for x, y in c]
            # Drop the duplicated closing point.
            if abs(pts[0][0] - pts[-1][0]) < 1e-3 and abs(pts[0][1] - pts[-1][1]) < 1e-3:
                pts = pts[:-1]
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            out.append({
                'stroke': m.group(1).lower() if m else None,
                'points': [[round(x, 4), round(y, 4)] for x, y in pts],
                'circle': fit_circle(pts),
                'bbox': [round(min(xs), 4), round(min(ys), 4), round(max(xs), 4), round(max(ys), 4)],
            })
    json.dump({'source': path, 'units': 'mm', 'contours': out}, sys.stdout)


if __name__ == '__main__':
    main(sys.argv[1])
