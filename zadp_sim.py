"""
ZADP (Zoned Adaptive Dump Plan) - lightweight 2D simulation.

Runs on any laptop: pure 2D geometry (shapely) + matplotlib, no physics engine.
Compares the baseline fixed-spot-point method against ZADP and animates the
multi-truck dispatcher (zone locking, wait gate, corridor lock, isolation check).

    python zadp_sim.py            # mine scale (metres), shows plots + animation
    python zadp_sim.py --rover    # scaled to your rover test arena
    python zadp_sim.py --gif      # also saves zadp_run.gif
"""
import argparse
import heapq
import math
import random

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from shapely.geometry import Point, Polygon, box, LineString
from shapely.ops import unary_union

# ---------------------------------------------------------------- parameters
MINE = dict(
    polygon=[(0, 0), (120, 0), (120, 50), (60, 50), (60, 100), (0, 100)],  # L-shape
    entry=(30, -15),          # wait gate, outside the polygon
    margin_side=5.0, margin_rear=8.0, margin_front=3.0,
    corridor_width=12.0,      # Rule 4: permanent no-dump strip at the entry
    fleet=[("Cat 793", 4.0, 0.6), ("Small truck", 2.5, 0.4)],  # name, pile radius, share
    baseline_spacing=7.38 + 2 * 4.0,   # fixed-spot method: 7.38 m gap between piles
    truck_half_width=3.5, speed=8.0, dump_time=4.0, n_trucks=4,
)
# Rover test arena: same algorithm, everything scaled (1 unit = 1 cm here).
SCALE = 1 / 40
ROVER = {k: (v * SCALE if isinstance(v, float) else v) for k, v in MINE.items()}
ROVER.update(
    polygon=[(x * SCALE * 100, y * SCALE * 100) for x, y in MINE["polygon"]],
    entry=(30 * SCALE * 100, -15 * SCALE * 100),
    fleet=[("Rover big bucket", 10.0, 0.6), ("Rover small bucket", 6.25, 0.4)],
    margin_side=12.5, margin_rear=20.0, margin_front=7.5, corridor_width=30.0,
    baseline_spacing=(7.38 + 2 * 4.0) * 2.5, truck_half_width=9.0,
    speed=20.0, dump_time=4.0, n_trucks=2,
)


# ---------------------------------------------------- Step 1: Minkowski erosion
def erode(poly: Polygon, P):
    """Shrink by the side margin, then trim extra at rear (top) and front (bottom)."""
    safe = poly.buffer(-P["margin_side"], join_style=2)
    minx, miny, maxx, maxy = poly.bounds
    trim = box(minx - 1, miny + P["margin_front"], maxx + 1, maxy - P["margin_rear"])
    return safe.intersection(trim)


# ---------------------------------------- Step 2: Boustrophedon decomposition
def boustrophedon(safe: Polygon, angle_deg=0.0):
    """Slice with vertical lines at every vertex x -> simple strips (zones)."""
    xs = sorted({round(x, 6) for x, _ in safe.exterior.coords})
    _, miny, _, maxy = safe.bounds
    zones = []
    for a, b in zip(xs, xs[1:]):
        piece = safe.intersection(box(a, miny - 1, b, maxy + 1))
        geoms = getattr(piece, "geoms", [piece])
        zones += [g for g in geoms if g.area > 1e-6]
    # merge thin neighbouring strips that have identical vertical extent
    merged = []
    for z in zones:
        if merged and _same_span(merged[-1], z):
            merged[-1] = unary_union([merged[-1], z]).buffer(1e-6).buffer(-1e-6)
        else:
            merged.append(z)
    return merged


def _same_span(a, b):
    return abs(a.bounds[1] - b.bounds[1]) < 1e-3 and abs(a.bounds[3] - b.bounds[3]) < 1e-3


# --------------------------------------------- Step 3: hexagonal spot points
def hex_points(zone: Polygon, r):
    """Hex lattice of pile centres; keep a point only if its whole circle fits."""
    dx, dy = 2 * r, math.sqrt(3) * r
    minx, miny, maxx, maxy = zone.bounds
    pts, row, y = [], 0, miny + r
    while y <= maxy:
        x = minx + r + (r if row % 2 else 0)
        while x <= maxx:
            if zone.contains(Point(x, y).buffer(r * 0.999)):
                pts.append((x, y))
            x += dx
        y += dy
        row += 1
    return pts


def baseline_points(safe, spacing):
    minx, miny, maxx, maxy = safe.bounds
    pts, y = [], miny + spacing / 2
    while y < maxy:
        x = minx + spacing / 2
        while x < maxx:
            if safe.contains(Point(x, y)):
                pts.append((x, y))
            x += spacing
        y += spacing
    return pts


def coverage(poly, piles):
    if not piles:
        return 0.0
    disc = unary_union([Point(x, y).buffer(r) for x, y, r in piles])
    return disc.intersection(poly).area / poly.area


# ------------------------------------ Step 4 + 5: dispatcher with safety rules
class Truck:
    def __init__(self, tid, name, r):
        self.id, self.name, self.r = tid, name, r
        self.state, self.pos, self.path, self.target, self.zone, self.timer = (
            "gate", None, [], None, None, 0.0)


class DumpPlanner:
    def __init__(self, poly, safe, zones, corridor, P):
        self.poly, self.safe, self.zones, self.corridor, self.P = poly, safe, zones, corridor, P
        self.locked = set()
        self.piles = []
        self.skipped = 0
        # back-to-front: deepest zone (highest y = rear) first
        self.order = sorted(range(len(zones)), key=lambda i: -zones[i].centroid.y)
        self.cand = {}
        for zi in self.order:
            for name, r, _ in P["fleet"]:
                self.cand[(zi, r)] = [(-y, x, y) for x, y in hex_points(zones[zi], r)
                                      if not corridor.intersects(Point(x, y).buffer(r))]
                heapq.heapify(self.cand[(zi, r)])

    def _free_space(self, extra=None):
        piles = self.piles + ([extra] if extra else [])
        blocked = unary_union([Point(x, y).buffer(r) for x, y, r in piles]) if piles else Polygon()
        # space a truck centre can occupy = free floor shrunk by half the truck width
        return self.poly.difference(blocked).buffer(-self.P["truck_half_width"])

    def _reachable(self, extra=None):
        free = self._free_space(extra)
        parts = [p for p in getattr(free, "geoms", [free]) if p.intersects(self.corridor)]
        return unary_union(parts).buffer(1e-3) if parts else Polygon()

    def _approach(self, x, y, r):
        # truck reverses up to the pile from the entry side
        return Point(x, y - r - self.P["truck_half_width"])

    def _isolation_ok(self, pile, zi):
        """The new pile must itself be reachable, and must not cut off any other
        zone's next candidate that is reachable right now."""
        before, after = self._reachable(), self._reachable(pile)
        if not before.contains(self._approach(*pile)):
            return False
        for (z, r), heap in self.cand.items():
            if z == zi:
                continue
            for _, x, y in heap[:5]:
                a = self._approach(x, y, r)
                if before.contains(a):
                    if not after.contains(a):
                        return False
                    break
        return True

    def request(self, truck):
        """'ready - payload X' -> returns (zone, point) or None (wait at gate)."""
        for zi in self.order:
            if zi in self.locked:                       # Rule 2: zone locking
                continue
            heap = self.cand[(zi, truck.r)]
            while heap:
                _, x, y = heap[0]
                pile = (x, y, truck.r)
                if any(math.hypot(x - px, y - py) < truck.r + pr - 1e-6 for px, py, pr in self.piles):
                    heapq.heappop(heap); continue       # overlaps another fleet's pile
                if not self._isolation_ok(pile, zi):
                    heapq.heappop(heap); self.skipped += 1; continue
                heapq.heappop(heap)
                self.locked.add(zi)
                return zi, (x, y)
        return None

    def complete(self, truck):
        self.piles.append((*truck.target, truck.r))
        # drop now-overlapping candidates of other sizes
        self.locked.discard(truck.zone)


def run(P, make_gif=False):
    poly = Polygon(P["polygon"])
    safe = erode(poly, P)
    zones = boustrophedon(safe)
    ex, ey = P["entry"]
    corridor = box(ex - P["corridor_width"] / 2, poly.bounds[1],
                   ex + P["corridor_width"] / 2, poly.bounds[1] + 0.35 * (poly.bounds[3] - poly.bounds[1]))
    corridor = corridor.intersection(poly)

    # ---- static comparison
    base = [(x, y, P["fleet"][0][1]) for x, y in baseline_points(safe, P["baseline_spacing"])]
    big_r = P["fleet"][0][1]
    hexall = [(x, y, big_r) for z in zones for x, y in hex_points(z, big_r)]
    print(f"Polygon area           : {poly.area:9.1f}")
    print(f"Safe zone (eroded) area: {safe.area:9.1f}")
    print(f"Zones                  : {len(zones)}")
    print(f"Baseline fixed points  : {len(base):4d}  coverage {coverage(poly, base)*100:5.1f}%")
    print(f"ZADP hex points        : {len(hexall):4d}  coverage {coverage(poly, hexall)*100:5.1f}%"
          f"  (of safe zone {coverage(safe, hexall)*100:5.1f}%)")

    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    for ax, piles, title in [(axs[0], base, "Baseline fixed spot points"),
                             (axs[1], hexall, "ZADP hex grid (single truck size)")]:
        _draw_base(ax, poly, safe, zones, corridor)
        for x, y, r in piles:
            ax.add_patch(Circle((x, y), r, color="tab:brown", alpha=0.6))
        ax.set_title(f"{title}\ncoverage {coverage(poly, piles)*100:.1f}%")
    plt.tight_layout()

    # ---- dynamic dispatcher simulation
    random.seed(1)
    planner = DumpPlanner(poly, safe, zones, corridor, P)
    names = [f for f in P["fleet"]]
    trucks = []
    for i in range(P["n_trucks"]):
        n, r, _ = random.choices(names, weights=[f[2] for f in names])[0]
        trucks.append(Truck(i, n, r))
    lane_in, lane_out = ex - P["corridor_width"] / 4, ex + P["corridor_width"] / 4  # Rule 1
    corridor_top = corridor.bounds[3]
    dt, t, frames, idle = 0.5, 0.0, [], 0

    def path_to(target):
        return [(lane_in, ey), (lane_in, corridor_top), target]

    def path_out(src):
        return [src, (lane_out, corridor_top), (lane_out, ey)]

    while idle < 40 and t < 20000:
        moved = False
        for tr in trucks:
            if tr.state == "gate":                      # Rule 3: wait gate
                a = planner.request(tr)
                if a:
                    tr.zone, tr.target = a
                    tr.pos, tr.path, tr.state = (lane_in, ey), path_to(tr.target), "in"
                    moved = True
            elif tr.state in ("in", "out"):
                tr.pos, tr.path = _step(tr.pos, tr.path, P["speed"] * dt)
                moved = True
                if not tr.path:
                    if tr.state == "in":
                        tr.state, tr.timer = "dump", P["dump_time"]
                    else:
                        planner.locked.discard(tr.zone)  # unlock after clearing
                        n, r, _ = random.choices(names, weights=[f[2] for f in names])[0]
                        tr.name, tr.r, tr.state, tr.pos = n, r, "gate", None
            elif tr.state == "dump":
                tr.timer -= dt
                moved = True
                if tr.timer <= 0:
                    planner.piles.append((*tr.target, tr.r))
                    tr.state, tr.path = "out", path_out(tr.target)
        idle = 0 if moved else idle + 1
        frames.append(([(tr.pos, tr.r, tr.state) for tr in trucks], len(planner.piles), t))
        t += dt

    cov = coverage(poly, planner.piles)
    print(f"Dispatcher run         : {len(planner.piles)} dumps, coverage {cov*100:5.1f}%, "
          f"sim time {t:.0f}s, points skipped by isolation check {planner.skipped}")

    fig2, ax = plt.subplots(figsize=(7, 7))
    _draw_base(ax, poly, safe, zones, corridor)
    ax.plot([lane_in, lane_in], [ey, corridor_top], "g--", lw=1)
    ax.plot([lane_out, lane_out], [ey, corridor_top], "r--", lw=1)
    pile_patches, truck_patches = [], []
    title = ax.set_title("")
    step = max(1, len(frames) // 600)

    def update(k):
        ts, npiles, tt = frames[k * step]
        while len(pile_patches) < npiles:
            x, y, r = planner.piles[len(pile_patches)]
            pile_patches.append(ax.add_patch(Circle((x, y), r, color="tab:brown", alpha=0.6)))
        for p in truck_patches:
            p.remove()
        truck_patches.clear()
        for pos, r, st in ts:
            if pos:
                c = {"in": "green", "dump": "orange", "out": "red"}[st]
                truck_patches.append(ax.add_patch(Circle(pos, P["truck_half_width"], color=c)))
        title.set_text(f"ZADP dispatcher  t={tt:.0f}s  piles={npiles}")
        return pile_patches + truck_patches + [title]

    anim = FuncAnimation(fig2, update, frames=len(frames) // step, interval=30, blit=False, repeat=False)
    if make_gif:
        anim.save("zadp_run.gif", fps=30)
        print("saved zadp_run.gif")
    plt.show()


def _step(pos, path, d):
    while path and d > 0:
        tx, ty = path[0]
        dist = math.hypot(tx - pos[0], ty - pos[1])
        if dist <= d:
            pos, d, path = (tx, ty), d - dist, path[1:]
        else:
            f = d / dist
            pos, d = (pos[0] + (tx - pos[0]) * f, pos[1] + (ty - pos[1]) * f), 0
    return pos, path


def _draw_base(ax, poly, safe, zones, corridor):
    ax.fill(*poly.exterior.xy, color="#eeeeee", ec="k")
    ax.plot(*safe.exterior.xy, "b--", lw=1, label="eroded safe zone")
    for i, z in enumerate(zones):
        ax.plot(*z.exterior.xy, color="purple", lw=0.6)
        c = z.centroid
        ax.text(c.x, c.y, f"Z{i}", color="purple", ha="center")
    ax.fill(*corridor.exterior.xy, color="yellow", alpha=0.4, label="no-dump corridor")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=7)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--rover", action="store_true", help="use rover test-arena scale (cm)")
    ap.add_argument("--gif", action="store_true", help="save animation as GIF")
    a = ap.parse_args()
    run(ROVER if a.rover else MINE, a.gif)
