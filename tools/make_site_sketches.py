"""Five practical, mine-scale dump sites (50–200 acres) drawn as dimensioned sketches.

    python tools/make_site_sketches.py

Writes test_sketches/site_*.png. Each outline is scaled to its target area; the overall
width and height are written on the drawing (metres), like a survey sketch.
"""
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent.parent / "test_sketches"
ACRE = 4046.856


def arc(cx, cy, r, a0, a1, n=24):
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)), cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def area(p):
    return abs(sum(p[i][0] * p[(i + 1) % len(p)][1] - p[(i + 1) % len(p)][0] * p[i][1] for i in range(len(p)))) / 2


def sites():
    s = {}
    # 1. Waste dump with a haul-road notch and cut corners (150 acres)
    s["site_waste_dump_150ac"] = (150, [(0, 60), (60, 0), (520, 0), (520, 180), (640, 180), (640, 0), (900, 0), (900, 560), (820, 700), (90, 700), (0, 610)])
    # 2. Valley fill: long, tapered, curved sides (50 acres)
    top = [(x, 160 + 90 * math.sin(x / 900 * math.pi) + 40 * x / 900) for x in range(0, 901, 60)]
    bot = [(x, -60 - 70 * math.sin(x / 900 * math.pi * 1.2) - 50 * x / 900) for x in range(900, -1, -60)]
    s["site_valley_fill_50ac"] = (50, bot[::-1] + top[::-1][::-1][::-1] if False else [(x, y) for x, y in bot[::-1]] + [(x, y) for x, y in top[::-1]])
    # 3. L-shaped paddock wrapped around an old dump, rounded inner corner (200 acres)
    p = [(0, 0), (1100, 0), (1100, 420)] + arc(560, 520, 100, -90, -180, 12)[1:] + [(460, 1000), (0, 1000)]
    s["site_L_paddock_200ac"] = (200, p)
    # 4. Pit-edge crescent with an irregular crest (80 acres)
    outer = arc(0, 0, 520, 200, 340, 40)
    crest = [(520 * math.cos(math.radians(a)) * 0.62 + 18 * math.sin(a / 6), 520 * math.sin(math.radians(a)) * 0.62 + 120 + 22 * math.cos(a / 4)) for a in range(340, 199, -7)]
    s["site_pit_crescent_80ac"] = (80, outer + crest)
    # 5. Two lobes joined by an access neck (120 acres)
    p = arc(-330, 0, 300, 25, 335, 40) + [(-60, -70), (60, -70)] + arc(330, 0, 280, 205, 515, 40) + [(60, 70), (-60, 70)]
    s["site_two_lobes_120ac"] = (120, p)
    return s


def draw(name, pts, acres):
    k = math.sqrt(acres * ACRE / area(pts))
    pts = [(x * k, y * k) for x, y in pts]
    xs, ys = zip(*pts)
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    W, H = round(x1 - x0, 1), round(y1 - y0, 1)
    fig, ax = plt.subplots(figsize=(8, 6), dpi=110)
    ax.plot(list(xs) + [xs[0]], list(ys) + [ys[0]], color="black", lw=2)
    pad = 0.12 * max(W, H)
    ax.annotate("", (x0, y0 - pad * 0.6), (x1, y0 - pad * 0.6), arrowprops=dict(arrowstyle="<->", lw=1))
    ax.text((x0 + x1) / 2, y0 - pad * 1.0, f"{W:g}", ha="center", va="center", fontsize=15, fontweight="bold")
    ax.annotate("", (x1 + pad * 0.6, y0), (x1 + pad * 0.6, y1), arrowprops=dict(arrowstyle="<->", lw=1))
    ax.text(x1 + pad * 1.0, (y0 + y1) / 2, f"{H:g}", ha="center", va="center", rotation=90, fontsize=15, fontweight="bold")
    ax.set_xlim(x0 - pad * 1.6, x1 + pad * 1.8); ax.set_ylim(y0 - pad * 1.6, y1 + pad * 0.8)
    ax.set_aspect("equal"); ax.axis("off")
    fig.savefig(OUT / f"{name}.png", facecolor="white"); plt.close(fig)
    return W, H, area(pts)


if __name__ == "__main__":
    for name, (acres, pts) in sites().items():
        W, H, A = draw(name, pts, acres)
        print(f"{name}.png  width {W} m  height {H} m  area {A:.0f} m² ({A / ACRE:.0f} acres)")
