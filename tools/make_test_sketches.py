"""Generate curved test sketches (outline + dimension text) for Round 1.

    python tools/make_test_sketches.py

Writes PNGs into test_sketches/. Each drawing is to scale; the expected
values are in the file name and in EXPECTED below.
"""
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

OUT = Path(__file__).resolve().parent.parent / "test_sketches"
OUT.mkdir(exist_ok=True)


def arc_pts(cx, cy, r, a0, a1, n=60):
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def shapes():
    s = {}
    s["circle_d80"] = (arc_pts(0, 0, 40, 0, 360), 80, 80)
    # rounded rectangle 120 x 80, corner radius 20
    w, h, r = 120, 80, 20
    p = []
    p += arc_pts(w - r, r, r, -90, 0, 15); p += arc_pts(w - r, h - r, r, 0, 90, 15)
    p += arc_pts(r, h - r, r, 90, 180, 15); p += arc_pts(r, r, r, 180, 270, 15)
    s["roundrect_120x80_r20"] = (p, w, h)
    # stadium 150 x 60
    p = arc_pts(120, 30, 30, -90, 90, 30) + arc_pts(30, 30, 30, 90, 270, 30)
    s["stadium_150x60"] = (p, 150, 60)
    # D-shape: rectangle 80 x 100 + semicircle r50 on the right
    p = [(0, 0)] + arc_pts(80, 50, 50, -90, 90, 40) + [(0, 100)]
    s["dshape_130x100"] = (p, 130, 100)
    # ellipse 140 x 80
    p = [(70 * math.cos(t / 80 * 2 * math.pi), 40 * math.sin(t / 80 * 2 * math.pi)) for t in range(81)]
    s["ellipse_140x80"] = (p, 140, 80)
    # kidney: two arcs joined by a tangent line and a concave arc
    p = arc_pts(40, 60, 40, 90, 225, 30) + [(40 + 40 * math.cos(math.radians(225)) + 70, 60 + 40 * math.sin(math.radians(225)) - 50)]
    p += arc_pts(130, 50, 45, -100, 110, 40) + arc_pts(85, 115, 30, -20, -160, 20)[1:-1]
    s["kidney"] = (p, None, None)
    return s


def weird_shapes():
    """Harder outlines for stress tests (all in metres, roughly mine-scale)."""
    s = {}
    # 5-point star
    p = []
    for k in range(10):
        r = 110 if k % 2 == 0 else 50
        a = math.pi / 2 + k * math.pi / 5
        p.append((r * math.cos(a), r * math.sin(a)))
    s["weird_star"] = p
    # plus / cross
    a, b = 70, 210
    s["weird_cross"] = [(a, 0), (b - a, 0), (b - a, a), (b, a), (b, b - a), (b - a, b - a), (b - a, b), (a, b), (a, b - a), (0, b - a), (0, a), (a, a)]
    # long thin strip
    s["weird_strip"] = [(0, 0), (320, 0), (320, 45), (0, 45)]
    # crescent: big circle minus offset circle
    # circles x²+y²=110² and (x-60)²+y²=80² meet at x=77.5, y=±78.06
    outer = arc_pts(0, 0, 110, 45.2, 314.8, 60)
    inner = arc_pts(60, 0, 80, 282.6, 77.4, 50)[1:-1]
    s["weird_crescent"] = outer + inner
    # dumbbell: two lobes joined by a narrow neck
    p = arc_pts(-110, 0, 70, 20, 340, 50) + [(-40, -25), (40, -25)] + arc_pts(110, 0, 70, 200, 520, 50)[::1] + [(40, 25), (-40, 25)]
    s["weird_dumbbell"] = p
    # comb: base with 4 teeth
    p = [(0, 0), (260, 0), (260, 160)]
    for i in range(4):
        x1 = 260 - i * 65
        p += [(x1, 160), (x1 - 40, 160), (x1 - 40, 60), (x1 - 65, 60)]
    p[-1] = (0, 60)
    p += [(0, 0)][:0]
    s["weird_comb"] = p
    # triangle
    s["weird_triangle"] = [(0, 0), (240, 0), (60, 200)]
    # T-shape
    s["weird_T"] = [(80, 0), (160, 0), (160, 140), (240, 140), (240, 200), (0, 200), (0, 140), (80, 140)]
    # notched blob
    p = []
    for k in range(36):
        a = k / 36 * 2 * math.pi
        r = 120 + 25 * math.sin(3 * a) + (-45 if 8 <= k <= 10 else 0)
        p.append((r * math.cos(a), 0.8 * r * math.sin(a)))
    s["weird_notched"] = p
    return s


def draw(name, pts, W, H):
    xs, ys = zip(*pts)
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    fig, ax = plt.subplots(figsize=(7, 5.5), dpi=110)
    ax.plot(list(xs) + [xs[0]], list(ys) + [ys[0]], color="black", lw=2)
    pad = 0.12 * max(x1 - x0, y1 - y0)
    # overall width (below) and height (right) with dimension lines
    wv = W if W else round(x1 - x0, 2)
    hv = H if H else round(y1 - y0, 2)
    ax.annotate("", (x0, y0 - pad * 0.6), (x1, y0 - pad * 0.6), arrowprops=dict(arrowstyle="<->", lw=1))
    ax.text((x0 + x1) / 2, y0 - pad * 1.0, f"{wv:g}", ha="center", va="center", fontsize=15, fontweight="bold")
    ax.annotate("", (x1 + pad * 0.6, y0), (x1 + pad * 0.6, y1), arrowprops=dict(arrowstyle="<->", lw=1))
    ax.text(x1 + pad * 1.0, (y0 + y1) / 2, f"{hv:g}", ha="center", va="center", rotation=90, fontsize=15, fontweight="bold")
    ax.set_xlim(x0 - pad * 1.6, x1 + pad * 1.8); ax.set_ylim(y0 - pad * 1.6, y1 + pad * 0.8)
    ax.set_aspect("equal"); ax.axis("off")
    fig.savefig(OUT / f"{name}.png", facecolor="white"); plt.close(fig)
    return wv, hv


if __name__ == "__main__":
    for name, (pts, W, H) in shapes().items():
        wv, hv = draw(name, pts, W, H)
        print(f"{name}.png  width {wv}  height {hv}")
    for name, pts in weird_shapes().items():
        wv, hv = draw(name, pts, None, None)
        print(f"{name}.png  width {wv}  height {hv}")
