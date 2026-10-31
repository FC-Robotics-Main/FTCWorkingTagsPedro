"""Snap-to-arc button, V05 (Option D only, all metric): checks the exact math the robot code uses.

Robot directions, same as the Java: FRONT = intake side, BACK = shooter side, LEFT = the robot's own left when
facing its front, UP. The camera is on the back and looks backward and up at the cell.

1. Builds the four cells' AprilTag rows from the manual (TU06 Figures 9-15 to 9-17): tag centers 1267.7 mm up,
   offsets -165.1 / -69.85 / +69.85 / +165.1 mm along the row, and the ID order. Seen from a robot shooting at the
   cell, the IDs always increase from the camera picture's left to its right.
2. Simulates what a Limelight reports (each tag's position in camera space: x right, y down, z out of the lens,
   in mm) for a camera that is mounted slightly differently from what the code assumes.
3. Runs the robot code's math on those numbers: tag positions -> robot directions -> row middle -> aim point ->
   distance and turn.
4. Compares the result before and after one teach press on the red audience cell's primary, at points around the
   arc of all four cells.

Field frame: mm, x from the red side wall, y from the audience wall. Writes snapmath.txt and snapmath.json.
"""
import json, math, os

HERE = os.path.dirname(os.path.abspath(__file__))
IN = 25.4                                    # the manual's drawings are in inches; converted once, here

# ---- cell geometry (same for every cell, in the cell's own frame), mm ----
OFFSETS = (-6.5 * IN, -2.75 * IN, 2.75 * IN, 6.5 * IN)   # tag middles along the row, lowest ID first
TAG_Z = 49.91 * IN                           # tag middle height above the tiles: 1267.7 mm
ROW_Y_RED_AUD = 58.96 * IN                   # tag row middle, red audience cell up: 1497.6 mm from the audience wall
LIP_Y_RED_AUD = 52.73 * IN                   # bottom lip of the opening: 1339.3 mm
LIP_FROM_ROW = ROW_Y_RED_AUD - LIP_Y_RED_AUD # 158.2 mm: lip is this far from the row middle, toward the wall
AIM_DEPTH = 75.0                             # Option D: aim point this far behind the lip (the tunable setting)
FIELD = 144 * IN                             # 3657.6 mm
PRIMARY = (59.0 * IN, 9.0 * IN, -90.0)       # robot center on the primary; heading = direction the FRONT faces

# right = field direction in which the IDs increase; wall = direction from the cell toward its shooting side.
CELLS = {
    "red audience (34-37)": {"first": 34, "row": (59.0 * IN, ROW_Y_RED_AUD), "right": (1, 0), "wall": (0, -1)},
    "red rear (30-33)": {"first": 30, "row": (59.0 * IN, FIELD - ROW_Y_RED_AUD), "right": (-1, 0), "wall": (0, 1)},
    "blue audience (38-41)": {"first": 38, "row": (85.0 * IN, ROW_Y_RED_AUD), "right": (1, 0), "wall": (0, -1)},
    "blue rear (42-45)": {"first": 42, "row": (85.0 * IN, FIELD - ROW_Y_RED_AUD), "right": (-1, 0), "wall": (0, 1)},
}


def tags_of(cell):
    c = CELLS[cell]
    return {c["first"] + k: (c["row"][0] + s * c["right"][0], c["row"][1] + s * c["right"][1], TAG_Z)
            for k, s in enumerate(OFFSETS)}


def aim_point(cell, depth=AIM_DEPTH):
    c = CELLS[cell]
    k = LIP_FROM_ROW - depth
    return (c["row"][0] + k * c["wall"][0], c["row"][1] + k * c["wall"][1])


def arc_pose(cell, beta, r):
    """Robot center r mm from the cell's aim point, beta degrees around (+ = toward the side where the IDs
    increase), with its BACK pointed at the aim point. Heading = direction the front faces."""
    c = CELLS[cell]; a = aim_point(cell)
    wx, wy = c["wall"]; rx, ry = c["right"]
    b = math.radians(beta)
    dx, dy = wx * math.cos(b) + rx * math.sin(b), wy * math.cos(b) + ry * math.sin(b)
    x, y = a[0] + r * dx, a[1] + r * dy
    return (x, y, math.degrees(math.atan2(y - a[1], x - a[0])))


# ---- the camera ----
ASSUMED = {"back": 180.0, "left": 0.0, "up": 255.0, "pitch": 44.0, "yaw": 0.0, "roll": 0.0, "scale": 1.0}


def rot(v, axis, ang):
    """Rotate vector v about unit axis by ang degrees (Rodrigues)."""
    a = math.radians(ang); c, s = math.cos(a), math.sin(a)
    kx, ky, kz = axis; x, y, z = v
    d = kx * x + ky * y + kz * z
    cx, cy, cz = ky * z - kz * y, kz * x - kx * z, kx * y - ky * x
    return (x * c + cx * s + kx * d * (1 - c), y * c + cy * s + ky * d * (1 - c), z * c + cz * s + kz * d * (1 - c))


def cam_axes_robot(m):
    """Camera x (right in the picture), y (down), z (out of the lens) in robot directions (front, left, up).
    The camera looks out the back, tilted up, so the picture's right is the robot's left."""
    t = math.radians(m["pitch"])
    Z = (-math.cos(t), 0.0, math.sin(t)); Y = (-math.sin(t), 0.0, -math.cos(t)); X = (0.0, 1.0, 0.0)
    X, Y = rot(X, Z, m["roll"]), rot(Y, Z, m["roll"])
    X, Y, Z = (rot(v, (0, 0, 1), m["yaw"]) for v in (X, Y, Z))
    return X, Y, Z


def measure(pose, cell, true_mount, ids=None):
    """What the Limelight would report: each tag's position in camera space, mm."""
    x, y, h = pose
    F = (math.cos(math.radians(h)), math.sin(math.radians(h)), 0.0)
    L = (-F[1], F[0], 0.0); U = (0.0, 0.0, 1.0)
    def to_field(v): return tuple(v[0] * F[k] + v[1] * L[k] + v[2] * U[k] for k in range(3))
    cam = (x - true_mount["back"] * F[0] + true_mount["left"] * L[0],
           y - true_mount["back"] * F[1] + true_mount["left"] * L[1], true_mount["up"])
    X, Y, Z = (to_field(v) for v in cam_axes_robot(true_mount))
    out = {}
    for tid, p in tags_of(cell).items():
        if ids is not None and tid not in ids: continue
        d = (p[0] - cam[0], p[1] - cam[1], p[2] - cam[2])
        out[tid] = tuple(true_mount["scale"] * sum(d[k] * A[k] for k in range(3)) for A in (X, Y, Z))
    return out


# ---- the robot code's math (mirrors SnapAim.compute in section 5.4) ----
def snap_math(meas, m=ASSUMED, depth=AIM_DEPTH):
    t = math.radians(m["pitch"]); st, ct = math.sin(t), math.cos(t)
    pts = []
    for tid, (cx, cy, cz) in meas.items():
        f = -m["back"] - cy * st - cz * ct             # + = toward the robot's front
        l = m["left"] + cx                             # + = robot's left
        u = m["up"] - cy * ct + cz * st                # + = up
        pts.append((OFFSETS[(tid - 30) % 4], f, l, u))
    n = len(pts)
    sm = sum(p[0] for p in pts) / n
    fm = sum(p[1] for p in pts) / n; lm = sum(p[2] for p in pts) / n; um = sum(p[3] for p in pts) / n
    ss = sum((p[0] - sm) ** 2 for p in pts)
    uf = sum((p[0] - sm) * (p[1] - fm) for p in pts) / ss
    ul = sum((p[0] - sm) * (p[2] - lm) for p in pts) / ss
    rowlen = math.hypot(uf, ul)                        # should be about 1.00 (scale check)
    uf, ul = uf / rowlen, ul / rowlen                  # row direction, toward slot 4
    cf, cl = fm - sm * uf, lm - sm * ul                # row middle, seen from above
    wf, wl = ul, -uf                                   # straight across the row
    if wf * cf + wl * cl > 0: wf, wl = -wf, -wl        # make it point from the cell toward the robot
    k = LIP_FROM_ROW - depth
    af, al = cf + k * wf, cl + k * wl                  # the aim point, behind the robot
    return {"dist": math.hypot(af, al), "turn": math.degrees(math.atan2(-al, -af)), "tag_height": um,
            "row_scale": rowlen}


def teach(true_mount):
    r = snap_math(measure(PRIMARY, "red audience (34-37)", true_mount))
    return r["dist"], r["turn"]


def true_dist_primary():
    a = aim_point("red audience (34-37)")
    return math.hypot(a[0] - PRIMARY[0], a[1] - PRIMARY[1])


CASES = [
    ("camera mounted exactly as the code assumes", {}),
    ("turned 1 deg sideways on the robot (yaw)", {"yaw": 1.0}),
    ("tilted 45 deg instead of 44 (pitch)", {"pitch": 45.0}),
    ("rotated 1 deg around its lens (roll)", {"roll": 1.0}),
    ("10 mm to one side of where the code thinks", {"left": -10.0}),
    ("10 mm farther back than the code thinks", {"back": 190.0}),
    ("measures 2% long (tag size or lens setting)", {"scale": 1.02}),
    ("all of the above at once", {"yaw": 1.0, "pitch": 45.0, "roll": 1.0, "left": -10.0, "back": 190.0,
                                  "scale": 1.02}),
]
BETAS = (0, 15, 32, 50, -32)


def run():
    R = true_dist_primary()
    res = {"primary_true_distance": R, "lip_from_row": LIP_FROM_ROW, "cases": [], "cells": [], "subsets": []}
    lines = [f"primary: robot center to aim point ({AIM_DEPTH:.0f} mm behind the lip) = {R:.1f} mm",
             f"lip is {LIP_FROM_ROW:.1f} mm from the tag row middle, toward the wall", ""]
    lines.append("EXPECTED CAMERA READINGS on the primary, camera as assumed (right / down / out, mm):")
    for tid, (x, y, z) in sorted(measure(PRIMARY, "red audience (34-37)", ASSUMED).items()):
        lines.append(f"  tag {tid}: {x:+7.1f} {y:+7.1f} {z:7.1f}")
    lines.append("")
    lines.append("MOUNT ERRORS, red audience cell. Worst error over 0/15/32/50/-32 deg around:")
    lines.append(f"{'case':48s} {'no teach: dist':>15s} {'aim':>7s} {'after teach: dist':>18s} {'aim':>7s}"
                 f" {'tag height':>11s} {'row scale':>10s} {'taught dist':>12s}")
    for name, err in CASES:
        tm = dict(ASSUMED); tm.update(err)
        d0, b0 = teach(tm)
        wd = wb = wd2 = wb2 = 0.0
        for beta in BETAS:
            r = snap_math(measure(arc_pose("red audience (34-37)", beta, R), "red audience (34-37)", tm))
            ed, eb = r["dist"] - R, r["turn"]; ed2, eb2 = r["dist"] - d0, r["turn"] - b0
            wd = ed if abs(ed) > abs(wd) else wd; wb = eb if abs(eb) > abs(wb) else wb
            wd2 = ed2 if abs(ed2) > abs(wd2) else wd2; wb2 = eb2 if abs(eb2) > abs(wb2) else wb2
        rp = snap_math(measure(PRIMARY, "red audience (34-37)", tm))
        res["cases"].append({"case": name, "no_teach_dist": wd, "no_teach_aim": wb, "teach_dist": wd2,
                             "teach_aim": wb2, "tag_height": rp["tag_height"], "row_scale": rp["row_scale"],
                             "taught_dist": d0, "taught_turn": b0})
        lines.append(f"{name:48s} {wd:+12.1f} mm {wb:+6.2f}d {wd2:+15.1f} mm {wb2:+6.2f}d {rp['tag_height']:9.1f} "
                     f"{rp['row_scale']:9.3f} {d0:10.1f}")
    lines.append("")
    lines.append("ONE TEACH, ALL FOUR CELLS: teach on the red audience primary with every error above at once,")
    lines.append("then snap at each cell. Errors after teach (distance mm, aim deg) at 0/15/32/50/-32 deg around:")
    tm = dict(ASSUMED); tm.update(CASES[-1][1]); d0, b0 = teach(tm)
    for cell in CELLS:
        row = []
        for beta in BETAS:
            r = snap_math(measure(arc_pose(cell, beta, R), cell, tm))
            row.append((round(r["dist"] - d0, 2), round(r["turn"] - b0, 2)))
        res["cells"].append({"cell": cell, "errors": row})
        lines.append(f"  {cell:24s} " + "  ".join(f"{a:+.1f}/{b:+.2f}" for a, b in row))
    lines.append("")
    lines.append("WHICH TAGS ARE SEEN: camera exactly as assumed, red audience cell, 32 deg around. dist / turn:")
    for ids in ((34, 35, 36, 37), (34, 35, 36), (35, 36), (34, 35), (36, 37), (34, 37)):
        r = snap_math(measure(arc_pose("red audience (34-37)", 32, R), "red audience (34-37)", ASSUMED, ids))
        res["subsets"].append({"ids": ids, "dist": r["dist"], "turn": r["turn"]})
        lines.append(f"  tags {str(ids):20s} {r['dist']:.1f} mm  {r['turn']:+.2f} deg")
    lines.append("")
    lines.append("OPTION D: where the aim line crosses the bottom lip, mm to your side of the lip's middle")
    lines.append(f"  {'aim depth':>10s} " + " ".join(f"{b:>7d}d" for b in (15, 32, 45, 55)))
    res["lip"] = {}
    for depth in (50.0, 75.0, 115.0):
        row = [depth * math.tan(math.radians(b)) for b in (15, 32, 45, 55)]
        res["lip"][str(int(depth))] = row
        lines.append(f"  {depth:8.0f}mm " + " ".join(f"{v:8.0f}" for v in row))
    txt = "\n".join(lines) + "\n"
    open(os.path.join(HERE, "snapmath.txt"), "w", encoding="utf-8").write(txt)
    json.dump(res, open(os.path.join(HERE, "snapmath.json"), "w"), indent=1)
    print(txt)


if __name__ == "__main__":
    run()
