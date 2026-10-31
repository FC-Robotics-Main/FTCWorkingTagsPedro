"""Figures for the snap-to-arc report V05. Option D only, all metric, black and white for printing.

Rules for every figure: red alliance only (blue is the same picture turned around); key BELOW the drawing so the
drawing uses the full page width in landscape; plain white backgrounds, except the field top views, where one set
of tiles is light grey and the other white, and the robot is shaded grey. The robot's BACK (shooter side) is the
thick black edge; its FRONT is the intake side.

Fig 1  one press, anywhere on the arc (field top view)
Fig 2  where Option D aims at the bottom lip (close-up from above)
Fig 3  where to teach and where to test-shoot (field top view)
Fig 4  the "camera turned 1 degree" problem and how the teach press cancels it (top view, three panels)
Fig 5  the "camera tilted 1 degree" problem and how the teach press cancels it (side view, three panels)
Fig 6  how any tag, on any cell, finds the same aim point (tag row, cell top view, the four cells)

Numbers come from snapmath.py (mm). Uses the Svg helper of the first shooting-position report (diagrams.py),
which renders SVG to PNG with PyMuPDF.
"""
import math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OLD = os.path.join(ROOT, "2026-10-27-Shooting-Position-Target-Area-V01-source")
sys.path.insert(0, OLD); sys.path.insert(0, HERE)
cwd = os.getcwd(); os.chdir(OLD)
import diagrams as D                                     # noqa: E402
os.chdir(cwd)
import snapmath as M                                     # noqa: E402

D.OUT = os.path.join(ROOT, "2026-10-27-Snap-To-Arc-Aim-Button-V05-diagrams")
os.makedirs(D.OUT, exist_ok=True)
Svg = D.Svg
IN = M.IN
INK = "#000000"; DARK = "#333333"; MID = "#666666"; RULE = "#999999"
TILE = "#e4e4e4"; ROBOT = "#b8b8b8"
AIM = M.aim_point("red audience (34-37)")              # (1498.6, 1415.5) mm
R = M.true_dist_primary()                                # 1185.7 mm
LIP_Y = M.LIP_Y_RED_AUD                                  # 1339.3 mm
BODY = [(x * IN, y * IN) for x, y in ((48.6, 52.0), (69.4, 52.0), (69.4, 66.2), (59, 70.1), (48.6, 66.2))]
PRI = M.PRIMARY
HALF = 9.0 * IN                                          # robot is 457 mm square
W = 1600                                                 # every figure is this wide (rendered at 2x)
F_TITLE, F_SUB, F_TXT, F_KEY = 31, 20, 19, 19            # font sizes


def wrap(text, px, size, bold=False):
    """Split text into lines that fit px pixels wide (Arial, rough character width)."""
    cw = size * (0.57 if bold else 0.52)
    out, cur = [], ""
    for para in text.split("\n"):
        for w in para.split(" "):
            t = (cur + " " + w).strip()
            if len(t) * cw > px and cur:
                out.append(cur); cur = w
            else:
                cur = t
        out.append(cur); cur = ""
    return out


def title(s, t1, t2):
    s.text(W / 2, 44, t1, F_TITLE, INK, "middle", True)
    for i, ln in enumerate(wrap(t2, W - 80, F_SUB)):
        s.text(W / 2, 78 + i * 25, ln, F_SUB, DARK, "middle")


def key_cols(cols, size=F_KEY):
    cw = (W - 60) / len(cols)
    return [(h, wrap(t, cw - 30, size)) for h, t in cols]


def key_height(cols, size=F_KEY):
    return 50 + max(len(l) + 1 for _, l in key_cols(cols, size)) * size * 1.35


def key_below(s, y, cols, size=F_KEY):
    """Key text in columns under the drawing. cols = list of (heading, paragraph)."""
    cc = key_cols(cols, size); cw = (W - 60) / len(cc)
    s.line(30, y - 18, W - 30, y - 18, RULE, 1.5)
    for i, (head, lines) in enumerate(cc):
        x = 30 + i * cw + 10
        s.text(x, y + 10, head, size + 1, INK, "start", True)
        for j, t in enumerate(lines):
            s.text(x, y + 10 + (j + 1) * size * 1.35, t, size, INK)


def box(s, x, y, lines, size=F_TXT, anchor="start", bold_first=True):
    """White text box with a black border. lines[0] is bold if bold_first."""
    cw = size * 0.55; lh = size * 1.3
    wmax = max(len(t) * (cw * 1.05 if (i == 0 and bold_first) else cw) for i, t in enumerate(lines))
    x0 = x if anchor == "start" else (x - wmax / 2 if anchor == "middle" else x - wmax)
    s.rect(x0 - 10, y - size - 6, wmax + 20, lh * len(lines) + 12, fill="#ffffff", stroke=INK, sw=1.5, rx=4)
    for i, t in enumerate(lines):
        s.text(x, y + i * lh, t, size, INK, anchor, bold_first and i == 0)


def target(s, x, y, r=11):
    """The aim point: white disc, black ring, black cross."""
    s.circle(x, y, r, fill="#ffffff", stroke=INK, sw=3)
    s.line(x - r * 1.5, y, x + r * 1.5, y, INK, 2.2); s.line(x, y - r * 1.5, x, y + r * 1.5, INK, 2.2)


def pose(beta, r=R, dh=0.0):
    """Robot center on the left (beta > 0) or right (beta < 0) of the red audience cell, back toward the aim
    point. Heading = direction the FRONT (intake) faces, degrees."""
    b = math.radians(beta)
    x, y = AIM[0] - r * math.sin(b), AIM[1] - r * math.cos(b)
    return (x, y, math.degrees(math.atan2(y - AIM[1], x - AIM[0])) + dh)


def corners(p, half=HALF):
    """Corners in order: front-left, front-right, back-right, back-left."""
    x, y, h = p
    hx, hy = math.cos(math.radians(h)), math.sin(math.radians(h)); lx, ly = -hy, hx
    return [(x + f * half * hx + l * half * lx, y + f * half * hy + l * half * ly)
            for f, l in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def back_mid(p, half=HALF):
    x, y, h = p
    return (x - half * math.cos(math.radians(h)), y - half * math.sin(math.radians(h)))


def lip_cross(p):
    """Where the line out of the robot's back crosses the bottom lip."""
    x, y, h = p
    bx, by = -math.cos(math.radians(h)), -math.sin(math.radians(h))
    t = (LIP_Y - y) / by
    return (x + t * bx, LIP_Y)


class Plot:
    """Field mm -> pixels."""
    def __init__(self, X0, Y1, S, OX, OY):
        self.X0, self.Y1, self.S, self.OX, self.OY = X0, Y1, S, OX, OY

    def __call__(self, x, y):
        return self.OX + (x - self.X0) * self.S, self.OY + (self.Y1 - y) * self.S


def field_base(s, P, X0, X1, Y1):
    T = 24 * IN
    for i in range(6):
        for j in range(4):
            cx, cy = i * T, j * T
            xa, xb = max(cx, X0), min(cx + T, X1); ya, yb = cy, min(cy + T, Y1)
            if xa >= xb or ya >= yb: continue
            p0 = P(xa, yb)
            s.rect(p0[0], p0[1], (xb - xa) * P.S, (yb - ya) * P.S,
                   fill=TILE if (i + j) % 2 == 0 else "#ffffff", stroke=RULE, sw=1)
    a, b = P(-40, 0), P(X1, -40); s.rect(a[0], a[1], b[0] - a[0], b[1] - a[1], fill=INK)
    a, b = P(-40, Y1), P(0, -40); s.rect(a[0], a[1], b[0] - a[0], b[1] - a[1], fill=INK)
    q = P(AIM[0], -40); s.text(q[0], q[1] + 28, "AUDIENCE WALL (your drivers stand here)", 21, INK, "middle", True)
    q = P(-40, 900); s.text(q[0] - 14, q[1], "RED SIDE WALL", 21, INK, "middle", True, rot=-90)
    s.poly([P(*q) for q in BODY], fill="#ffffff", stroke=INK, sw=3.5)
    a, b = P(AIM[0] - 254, LIP_Y), P(AIM[0] + 254, LIP_Y); s.line(a[0], a[1], b[0], b[1], INK, 8)
    q = P(AIM[0], 1640); s.text(q[0], q[1], "UP CELL", 22, INK, "middle", True)
    q = P(AIM[0] + 290, LIP_Y); s.text(q[0], q[1] + 7, "bottom lip", 18, INK, "start", True)


def draw_arc(s, P, lo=-62, hi=62):
    pts = [P(AIM[0] - R * math.sin(math.radians(k)), AIM[1] - R * math.cos(math.radians(k))) for k in range(lo, hi + 1)]
    for i in range(0, len(pts) - 1, 2):
        s.line(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], INK, 4)


def robot(s, P, p, dashed=False):
    pts = [P(*c) for c in corners(p)]
    if dashed:
        for i in range(4):
            s.dline(pts[i][0], pts[i][1], pts[(i + 1) % 4][0], pts[(i + 1) % 4][1], DARK, 3, 10, 7)
        return
    s.poly(pts, fill=ROBOT, stroke=INK, sw=2.5)
    s.line(pts[2][0], pts[2][1], pts[3][0], pts[3][1], INK, 10)      # the back (shooters)


# ================================================================== Fig 1
def fig1():
    X0, X1, Y1 = -80.0, 121.0 * IN, 75.0 * IN
    S = (W - 60) / (X1 - X0)
    OY = 130
    P = Plot(X0, Y1, S, 30, OY)
    cols = [("What you see", "Long dashes = the arc. Every point on it is the same distance from the aim point "
                             "(ringed cross, the middle of the cell floor) as the primary is."),
            ("The robots", "Grey square = the robot after one press. Dashed outline = where the driver stopped. "
                           "Thick black edge = the robot's back, where the shooters are. The front (intake) faces "
                           "away from the cell."),
            ("The aim", "Short dashes = where the robot's back points. Black dot = where that line crosses the "
                        "bottom lip. Robot 457 x 457 mm. One tile = 610 mm.")]
    yk = int(OY + (Y1 + 80) * S + 60)
    s = Svg(W, int(yk + key_height(cols)))
    title(s, "One press fixes distance and aim, anywhere on the arc",
          "Seen from above, red audience cell. You choose where on the arc to stop; the button only turns in place "
          "and drives forward or in reverse.")
    field_base(s, P, X0, X1, Y1)
    draw_arc(s, P)
    q = P(AIM[0] - (R + 110) * math.sin(math.radians(-60)), AIM[1] - (R + 110) * math.cos(math.radians(-60)))
    s.text(q[0] + 4, q[1], "the arc", 21, INK, "start", True)
    robot(s, P, PRI)
    for yy, t, sz in ((PRI[1] + 160, "BACK (shooters)", 16), (PRI[1] + 55, "PRIMARY", 19),
                      (PRI[1] - 25, "teach here once", 16), (PRI[1] - 175, "FRONT (intake)", 16)):
        q = P(PRI[0], yy); s.text(q[0], q[1], t, sz, INK, "middle", True)
    beta = 40
    before = pose(beta, R + 120, -10.0); after = pose(beta)
    robot(s, P, after)
    robot(s, P, before, dashed=True)
    a, b = P(before[0], before[1]), P(after[0], after[1])
    s.line(a[0], a[1], b[0], b[1], INK, 4)
    s.arrowhead(b[0], b[1], math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])), INK, 20)
    right = pose(-38)
    robot(s, P, right)
    for p in (after, right):
        lc = lip_cross(p); a, b = P(*back_mid(p)), P(*lc)
        s.dline(a[0], a[1], b[0], b[1], INK, 3, 7, 7)
        s.circle(b[0], b[1], 7, fill=INK)
    c = P(*AIM); target(s, c[0], c[1])
    lc_l = lip_cross(after); lc_r = lip_cross(right)
    q = P(40, 1840)
    box(s, q[0], q[1], ["LEFT: 40 degrees around"] +
        wrap("Dashed outline: the driver stopped 120 mm too far from the cell and 10 degrees off. One press: the "
             "robot turns to aim, then reverses 120 mm toward the cell along its aim line. Its back now aims "
             f"{AIM[0] - lc_l[0]:.0f} mm left of the lip's middle.", 410, F_TXT))
    q = P(3020, 1840)
    box(s, q[0], q[1], ["RIGHT: 38 degrees around"] +
        wrap("The mirror image: the back aims "
             f"{lc_r[0] - AIM[0]:.0f} mm right of the lip's middle. The button never slides the robot along the arc; where you stop "
             "on the arc is the driver's choice.", 410, F_TXT), anchor="end")
    key_below(s, yk, cols)
    s.save("fig1-one-press-anywhere-on-the-arc")


# ================================================================== Fig 2
def fig2():
    X0, X1, Y0, Y1 = AIM[0] - 780, AIM[0] + 560, 1030.0, 60.0 * IN
    S = (W - 60) / (X1 - X0)
    OY = 130
    P = Plot(X0, Y1, S, 30, OY)
    cols = [("Option D", "Every aim line points at the aim point (ringed cross), the middle of the cell floor, "
                         "75 mm behind the lip. Where a line crosses the lip (black dot) moves toward your side "
                         "the farther around the arc you are."),
            ("The rule", "The sideways shift at the lip grows with how far around you are. At 45 degrees it "
                         "equals the aim depth: 75 mm. Change the aim depth setting and every shift grows or "
                         "shrinks in proportion."),
            ("Reading it", "Ruler marks along the lip are 10 mm apart, numbered every 50 mm from the lip's "
                           "middle. Robots on the right half of the arc are the mirror image.")]
    yk = int(OY + (Y1 - Y0) * S + 60)
    s = Svg(W, int(yk + key_height(cols)))
    title(s, "Where Option D aims, at the cell's bottom lip",
          "Close-up from above, red audience cell, robots on the left half of the arc. The robot's back points "
          "along each line.")
    s.rect(30, OY, (X1 - X0) * S, (Y1 - Y0) * S, fill="#ffffff", stroke=RULE, sw=1.5)
    s.poly([P(min(max(x, X0), X1), min(y, Y1)) for x, y in BODY], fill="#ffffff", stroke=INK, sw=3.5)
    a, b = P(AIM[0] - 254, LIP_Y), P(AIM[0] + 254, LIP_Y); s.line(a[0], a[1], b[0], b[1], INK, 8)
    q = P(AIM[0], Y1 - 30); s.text(q[0], q[1], "CELL OPENING (seen from above)", 23, INK, "middle", True)
    q = P(AIM[0] + 260, LIP_Y); s.text(q[0] + 10, q[1] + 7, "bottom lip", 21, INK, "start", True)
    ylab = Y0 + 30
    rows = []
    for beta in (55, 45, 32, 15):
        p = pose(beta)
        dx, dy = -math.cos(math.radians(p[2])), -math.sin(math.radians(p[2]))   # out of the back
        t0 = max((X0 - AIM[0]) / dx if dx > 1e-9 else -1e9, (Y0 - AIM[1]) / dy)
        st = (AIM[0] + t0 * dx, AIM[1] + t0 * dy)
        a, b = P(*st), P(*AIM); s.line(a[0], a[1], b[0], b[1], INK, 3.2)
        lc = lip_cross(p); q = P(*lc); s.circle(q[0], q[1], 8, fill=INK)
        xl = AIM[0] + (ylab - AIM[1]) / dy * dx
        q = P(xl, ylab)
        box(s, q[0] + 14, q[1], [f"{beta} deg"], 19)
        rows.append(f"{beta} deg around: crosses the lip {AIM[0] - lc[0]:.0f} mm left")
    for k in range(-20, 21):
        a = P(AIM[0] + 10 * k, LIP_Y)
        s.line(a[0], a[1] + 6, a[0], a[1] + (26 if k % 5 == 0 else 15), INK, 2)
        if k % 5 == 0:
            lab = "middle" if k == 0 else f"{abs(10 * k)}"
            s.rect(a[0] - len(lab) * 5.6 - 4, a[1] + 33, len(lab) * 11.2 + 8, 24, fill="#ffffff")
            s.text(a[0], a[1] + 51, lab, 18, INK, "middle", True)
    c = P(*AIM); target(s, c[0], c[1], 13)
    s.text(c[0] + 28, c[1] - 6, "aim point: middle of the cell", 20, INK, "start", True)
    s.text(c[0] + 28, c[1] + 20, "floor, 75 mm behind the lip", 20, INK, "start", True)
    q = P(X0 + 25, 1300)
    box(s, q[0], q[1], ["Robot this far around the arc:"] + list(reversed(rows)))
    key_below(s, yk, cols)
    s.save("fig2-where-option-d-aims")


# ================================================================== Fig 3
def fig3():
    X0, X1, Y1 = -80.0, 121.0 * IN, 75.0 * IN
    S = (W - 60) / (X1 - X0)
    OY = 130
    P = Plot(X0, Y1, S, 30, OY)
    cols = [("Step 1: teach", "Robot exactly on the primary: square against the wall, on the cell's center line, "
                              "where you hit 40 of 40. One press saves two numbers."),
            ("Steps 2 and 3: test-shoot", "Numbered circles = where to shoot 20 balls, by step number. Step 2 "
                                          "picks the aim depth. Step 3 checks 15 and 50 degrees on both sides."),
            ("Reading it", "Long dashes = the arc. Ringed cross = the aim point. Degrees = how far around the arc "
                           "from straight in. One tile = 610 mm.")]
    yk = int(OY + (Y1 + 80) * S + 60)
    s = Svg(W, int(yk + key_height(cols)))
    title(s, "Where to teach and where to test-shoot",
          "Seen from above, red audience cell. Blue is the same layout at the rear wall.")
    field_base(s, P, X0, X1, Y1)
    draw_arc(s, P)
    robot(s, P, PRI)
    for yy, t, sz in ((PRI[1] + 160, "BACK (shooters)", 16), (PRI[1] + 55, "1. TEACH", 21),
                      (PRI[1] - 25, "one press here", 16), (PRI[1] - 175, "FRONT (intake)", 16)):
        q = P(PRI[0], yy); s.text(q[0], q[1], t, sz, INK, "middle", True)
    c = P(*AIM); target(s, c[0], c[1])
    for sg in (-1, 1):
        for beta, num in ((15, "3"), (32, "2"), (50, "3")):
            b_ = math.radians(sg * beta)
            q = P(AIM[0] - R * math.sin(b_), AIM[1] - R * math.cos(b_))
            s.circle(q[0], q[1], 23, fill="#ffffff", stroke=INK, sw=4)
            s.text(q[0], q[1] + 8, num, 23, INK, "middle", True)
            o = P(AIM[0] - (R + 165) * math.sin(b_), AIM[1] - (R + 165) * math.cos(b_))
            s.text(o[0], o[1] + 7, f"{beta} deg", 20, INK, "middle", True)
    q = P(40, 1840)
    box(s, q[0], q[1], ["THE ORDER",
                        "1. Teach on the primary (one press).",
                        "2. At about 32 deg, try aim depths of",
                        "    50, 75 and 115 mm; keep the best.",
                        "3. Check 15 and 50 deg on each side.",
                        "4. Fix only a spot that shoots worse."])
    key_below(s, yk, cols)
    s.save("fig3-where-to-teach-and-test")


# ================================================================== panels for Figs 4 and 5
def panel_frame(s, x, y, w, h, head, sub):
    s.rect(x, y, w, h, fill="#ffffff", stroke=MID, sw=2, rx=6)
    s.text(x + w / 2, y + 36, head, 23, INK, "middle", True)
    s.text(x + w / 2, y + 64, sub, 18, DARK, "middle")


def top_robot(s, cx, cy, ang, size):
    """Robot seen from above. ang = direction its BACK faces (degrees, 90 = up the page). Returns the back middle."""
    a = math.radians(ang); bx, by = math.cos(a), -math.sin(a); lx, ly = -by, bx
    h = size / 2
    pts = [(cx + f * h * bx + l * h * lx, cy + f * h * by + l * h * ly) for f, l in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    s.poly(pts, fill=ROBOT, stroke=INK, sw=2.5)
    s.line(pts[0][0], pts[0][1], pts[1][0], pts[1][1], INK, 10)
    return cx + h * bx, cy + h * by


def camera_box(s, cx, cy, ang):
    """Camera seen from above or the side, lens toward angle ang (degrees, 90 = up the page)."""
    a = math.radians(ang); fx, fy = math.cos(a), -math.sin(a); lx, ly = -fy, fx
    pts = [(cx + f * 11 * fx + l * 17 * lx, cy + f * 11 * fy + l * 17 * ly) for f, l in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    s.poly(pts, fill=INK, stroke=INK, sw=1.5)
    s.circle(cx + 11 * fx, cy + 11 * fy, 5.5, fill="#ffffff", stroke=INK, sw=1.5)


def ray(s, x, y, ang, L, sw=3.5, dashed=False, col=INK):
    a = math.radians(ang); x2, y2 = x + L * math.cos(a), y - L * math.sin(a)
    if dashed: s.dline(x, y, x2, y2, col, sw, 12, 8)
    else: s.line(x, y, x2, y2, col, sw)
    return x2, y2


def panel_text(s, x0, y, pw, text):
    for j, t in enumerate(wrap(text, pw - 50, 18)):
        s.text(x0 + 24, y + j * 24, t, 18, INK)


# ================================================================== Fig 4
def fig4():
    EX = 8.0                                             # angles drawn 8x larger than 1 degree so they show
    ang_err = -M.json.load(open(os.path.join(HERE, "snapmath.json")))["cases"][1]["no_teach_aim"]
    cols = [("Lines", "Solid line = where the robot's back really points. Dashed line = where the camera looks. "
                      "Ringed cross = the aim point. Thick edge = the robot's back (shooters); the black box on it is the "
                      "camera."),
            ("Angles", "The 1 degree error is drawn 8 times larger so you can see it. Real size: about "
                       f"{ang_err:.2f} degrees of aim, or about 16 mm sideways at the opening."),
            ("Why it cancels", "The error is between the camera and the robot. It doesn't depend on where the "
                               "robot is, so the number saved on the primary fits every spot.")]
    PH = 820; OY = 130
    yk = int(OY + PH + 60)
    s = Svg(W, int(yk + key_height(cols)))
    title(s, "The \"camera turned 1 degree\" problem, and how one teach press cancels it",
          "Seen from above, cell at the top of each panel. The camera is bolted on turned 1 degree to the left, as "
          "you look out the robot's back.")
    pw = (W - 60 - 40) / 3
    heads = [("A. Without teaching", "The code believes camera = robot"),
             ("B. The teach press, on the primary", "The back is known to point at the cell"),
             ("C. Any spot on the arc, after teaching", "The saved number is taken off every time")]
    for i, (h1, h2) in enumerate(heads):
        x0 = 30 + i * (pw + 20); y0 = OY
        panel_frame(s, x0, y0, pw, PH, h1, h2)
        cx = x0 + pw / 2; aimy = y0 + 150
        target(s, cx, aimy, 13)
        s.text(cx + (-24 if i == 0 else 24), aimy - 16, "aim point", 18, INK, "end" if i == 0 else "start", True)
        ry = y0 + PH - 370
        if i == 0:
            ang = 90 - EX                                # back ends up turned EX degrees right of the aim point
            ex, ey = top_robot(s, cx, ry, ang, 130)
            camera_box(s, ex, ey, ang + EX)
            gx, gy = ray(s, ex, ey, ang, ey - aimy + 30)
            ray(s, ex, ey, 90, ey - aimy - 20, 2.5, True)
            s.text(gx + 12, gy + 12, "back points here", 18, INK, "start", True)
            s.text(cx - 16, aimy + 80, "camera looks here", 18, DARK, "end", True)
            txt = ("The camera sees the aim point a little to the right of its own straight-ahead. The code thinks "
                   "the camera looks exactly where the robot's back points, so it turns the robot until the camera "
                   f"looks straight at the aim point. Result: the back points about {ang_err:.2f} degrees right. "
                   "Every shot drifts right.")
        elif i == 1:
            ex, ey = top_robot(s, cx, ry, 90, 130)
            camera_box(s, ex, ey, 90 + EX)
            ray(s, ex, ey, 90, ey - aimy - 20)
            ray(s, ex, ey, 90 + EX, ey - aimy + 10, 2.5, True)
            s.text(ex - 46, aimy + 40, "camera's", 18, DARK, "end", True)
            s.text(ex - 46, aimy + 63, "straight-ahead", 18, DARK, "end", True)
            box(s, ex + 26, ey - 150, ["camera reads:", "\"aim point is", f"{ang_err:.2f} deg right\""], 18, bold_first=False)
            txt = ("Here you KNOW the back points exactly at the aim point: the robot is square against the wall, "
                   "on the cell's center line, where you hit 40 of 40. The camera still says "
                   f"\"{ang_err:.2f} degrees right\". The press saves that number: "
                   f"\"{ang_err:.2f} degrees right means on target.\"")
        else:
            ang = 90 + 25
            ex, ey = top_robot(s, cx + 70, ry, ang, 130)
            camera_box(s, ex, ey, ang + EX)
            ray(s, ex, ey, math.degrees(math.atan2(ey - aimy, cx - ex)), math.hypot(cx - ex, ey - aimy) - 20)
            txt = ("At a side spot the camera reads, say, \"aim point is 3.85 degrees right\". The code takes away "
                   "the saved 0.85: 3.85 - 0.85 = turn 3 degrees. That is exactly the turn needed. Result: the back "
                   "points exactly at the aim point, on any spot, at any cell.")
        panel_text(s, x0, y0 + PH - 190, pw, txt)
    key_below(s, yk, cols)
    s.save("fig4-camera-turned-1-degree")


# ================================================================== Fig 5
def fig5():
    EX = 6.0
    res = M.json.load(open(os.path.join(HERE, "snapmath.json")))
    pitch = res["cases"][2]
    true_d = res["primary_true_distance"]; read_d = pitch["taught_dist"]; err = read_d - true_d
    above = M.TAG_Z - M.ASSUMED["up"]
    drop = res["cases"][0]["tag_height"] - pitch["tag_height"]
    cols = [("Lines", "Solid line = the real line from the camera to the tags. Dashed line and hollow circle = "
                      "where the code thinks the tags are. Black box = the camera, on the robot's back."),
            ("Angles", f"The 1 degree error is drawn 6 times larger. Real size: the code puts the tags about "
                       f"{err:.0f} mm too far away and {drop:.0f} mm too low."),
            ("Why it cancels", f"The tags are about {above / 1000:.1f} m above the camera from every spot on the "
                               f"arc, so a 1 degree tilt error shifts the reading by the same {err:.0f} mm "
                               "everywhere.")]
    PH = 760; OY = 130
    yk = int(OY + PH + 60)
    s = Svg(W, int(yk + key_height(cols)))
    title(s, "The \"camera tilted 1 degree\" problem, and how the same press cancels it",
          "Seen from the side, cell to the right. The camera is really tilted up 45 degrees, but the code was "
          "told 44.")
    pw = (W - 60 - 40) / 3
    heads = [("A. Without teaching", "The code places the tags too far away"),
             ("B. The teach press, on the primary", "The distance there is known to be right"),
             ("C. Any spot on the arc, after teaching", "The same error, so it cancels")]
    for i, (h1, h2) in enumerate(heads):
        x0 = 30 + i * (pw + 20); y0 = OY
        panel_frame(s, x0, y0, pw, PH, h1, h2)
        fy = y0 + PH - 270                               # floor line
        s.line(x0 + 20, fy, x0 + pw - 20, fy, DARK, 3)
        s.text(x0 + pw - 30, fy + 24, "tiles", 17, DARK, "end", True)
        rx = x0 + 40
        s.rect(rx, fy - 44, 170, 44, fill=ROBOT, stroke=INK, sw=2.5)
        s.line(rx + 170, fy - 44, rx + 170, fy, INK, 10)
        s.text(rx + 8, fy + 24, "FRONT", 16, INK, "start", True)
        s.text(rx + 172, fy + 24, "BACK", 16, INK, "end", True)
        camx, camy = rx + 170, fy - 58
        tagx, tagy = camx + 220, y0 + 140
        s.line(tagx - 80, tagy - 44, tagx + 50, tagy + 30, INK, 7)
        s.circle(tagx, tagy, 9, fill=INK)
        s.text(tagx + 16, tagy - 16, "tags", 19, INK, "start", True)
        a = math.atan2(camy - tagy, tagx - camx)
        s.line(camx, camy, tagx, tagy, INK, 3.5)
        camera_box(s, camx, camy, 44 + EX)
        if i in (0, 2):
            L = math.hypot(tagx - camx, camy - tagy)
            b = a - math.radians(EX)
            gx, gy = camx + L * math.cos(b), camy - L * math.sin(b)
            s.dline(camx, camy, gx, gy, DARK, 2.5, 10, 7)
            s.circle(gx, gy, 9, fill="#ffffff", stroke=INK, sw=2.5)
            s.text(x0 + 24, y0 + 120, "Hollow circle = where", 18, DARK, "start", True)
            s.text(x0 + 24, y0 + 143, "the code thinks the", 18, DARK, "start", True)
            s.text(x0 + 24, y0 + 166, "tags are", 18, DARK, "start", True)
        if i == 1:
            box(s, x0 + 30, fy + 70, [f"camera reads {read_d:.0f} mm;", f"the real distance is {true_d:.0f} mm"],
                18, bold_first=False)
        if i == 0:
            txt = (f"The code reads the tags as farther away than they are ({read_d:.0f} mm instead of "
                   f"{true_d:.0f} mm), so the snap reverses the robot about {err:.0f} mm too close to the cell. "
                   "Shots land a little long.")
        elif i == 1:
            txt = (f"On the primary you KNOW the distance is right. The press saves what the camera reads there, "
                   f"{read_d:.0f} mm, as \"the right distance\". The true number, {true_d:.0f} mm, never matters.")
        else:
            txt = (f"Around the arc the tags are the same height above the camera, so the code is off by the same "
                   f"{err:.0f} mm in the same direction. It snaps to a reading of {read_d:.0f} mm, which is truly "
                   f"{true_d:.0f} mm: just right.")
        panel_text(s, x0, fy + 140, pw, txt)
    key_below(s, yk, cols)
    s.save("fig5-camera-tilted-1-degree")


# ================================================================== Fig 6
def fig6():
    cols = [("Top left: one row of tags", "Each tag's place along the row is printed in the manual. From any "
                                          "one tag, slide its listed distance along the row to reach the row's "
                                          "middle."),
            ("Top right: row middle to aim point", f"The lip is {M.LIP_FROM_ROW:.0f} mm from the row's middle, "
                                                   f"toward you. The aim point is 75 mm behind the lip: "
                                                   f"{M.LIP_FROM_ROW - M.AIM_DEPTH:.0f} mm from the row's middle."),
            ("Bottom: the four cells", "Same sticker, same shape, different numbers. Seen from the robot, the "
                                       "lowest number is always on the LEFT of the camera picture.")]
    TOP = 620; BOT = 330; OY = 130
    yk = int(OY + TOP + 30 + BOT + 60)
    s = Svg(W, int(yk + key_height(cols)))
    title(s, "How any tag, on any cell, leads to the same aim point",
          "The code knows one sticker layout. The tag numbers only say which cell and which slot.")
    # ---- top left: the tag row as the camera sees it
    x0, y0, pw = 30, OY, 900
    panel_frame(s, x0, y0, pw, TOP, "One row of four tags, as the camera sees it",
                "Example: red audience cell, tags 34-37, looking up at the cell's underside")
    sc = 1.85                                            # px per mm
    cx = x0 + pw / 2; ty = y0 + 250
    tag = 82.55
    s.rect(cx - 218 * sc, ty - 66 * sc, 436 * sc, 127 * sc, fill="#ffffff", stroke=MID, sw=2, rx=8)
    for k, off in enumerate(M.OFFSETS):
        x = cx + off * sc
        s.rect(x - tag / 2 * sc, ty - tag / 2 * sc - 8, tag * sc, tag * sc, fill=INK)
        s.rect(x - 23 * sc, ty - 23 * sc - 8, 46 * sc, 46 * sc, fill="#ffffff")
        s.text(x, ty + 3, f"{34 + k}", 32, INK, "middle", True)
    s.line(cx, ty - 66 * sc, cx, ty + 61 * sc + 92, INK, 3)
    s.circle(cx, ty - 8, 10, fill="#ffffff", stroke=INK, sw=3)
    s.text(cx, ty - 66 * sc - 12, "middle of the row", 20, INK, "middle", True)
    for k, off in enumerate(M.OFFSETS):
        x = cx + off * sc; yy = ty + 61 * sc + 36 + (k % 2) * 40
        s.line(x, yy, cx + (12 if off < 0 else -12), yy, INK, 2.5)
        s.arrowhead(cx + (12 if off < 0 else -12), yy, 0 if off < 0 else 180, INK, 14)
        s.circle(x, yy, 6, fill=INK)
        s.text((x + cx) / 2, yy - 8, f"{abs(off):.0f} mm {'right' if off < 0 else 'left'}", 19, INK, "middle", True)
    panel_text(s, x0, y0 + TOP - 52, pw, "Slot 1 (lowest number) is 165 mm left of the middle, slot 2 is 70 mm "
                                         "left, slot 3 is 70 mm right, slot 4 is 165 mm right.")
    # ---- top right: top view of the cell, row middle -> lip -> aim point
    x1 = x0 + pw + 20; pw2 = W - 30 - x1
    panel_frame(s, x1, y0, pw2, TOP, "From the row's middle to the aim point", "Top view of the up cell")
    sc2 = 0.82
    ccx = x1 + pw2 / 2 + 50; rowy = y0 + 110 + 282 * sc2
    def Q(dx, dy): return ccx + dx * sc2, rowy + dy * sc2   # dy > 0 = toward your wall (down the page)
    s.poly([Q(-264, 178), Q(264, 178), Q(264, -183), Q(0, -282), Q(-264, -183)], fill="#ffffff", stroke=INK, sw=3)
    lip = M.LIP_FROM_ROW; k = lip - M.AIM_DEPTH
    a, b = Q(-254, lip), Q(254, lip); s.line(a[0], a[1], b[0], b[1], INK, 7)
    for off in M.OFFSETS:
        p = Q(off, 0); s.rect(p[0] - 41 * sc2, p[1] - 41 * sc2, 82 * sc2, 82 * sc2, fill="#ffffff", stroke=INK, sw=2.5)
    p = Q(0, 0); s.circle(p[0], p[1], 9, fill="#ffffff", stroke=INK, sw=3)
    q = Q(0, -55); s.text(q[0], q[1], "row middle (1268 mm up)", 17, INK, "middle", True)
    p2 = Q(0, k); target(s, p2[0], p2[1], 10)
    s.text(p2[0] + 20, p2[1] + 6, "aim point", 18, INK, "start", True)
    s.text(Q(-264, 178)[0], Q(-264, 178)[1] + 26, "bottom lip (the thick line)", 18, INK, "start", True)
    xa = Q(-300, 0)[0]
    for (u0, u1, t) in ((0, k, f"{k:.0f} mm"), (k, lip, f"{M.AIM_DEPTH:.0f} mm")):
        a, b = (xa, Q(0, u0)[1]), (xa, Q(0, u1)[1]); s.line(a[0], a[1], b[0], b[1], INK, 2.5)
        s.line(xa - 8, a[1], xa + 8, a[1], INK, 2.5); s.line(xa - 8, b[1], xa + 8, b[1], INK, 2.5)
        s.text(xa - 12, (a[1] + b[1]) / 2 + 6, t, 18, INK, "end", True)
    a = Q(0, 178); s.line(a[0], a[1] + 36, a[0], a[1] + 70, INK, 3); s.arrowhead(a[0], a[1] + 70, 90, INK, 14)
    s.text(a[0] + 14, a[1] + 56, "toward you (your wall)", 18, INK, "start", True)
    p3 = Q(0, -140); s.text(p3[0], p3[1], "four tags (on the underside)", 17, DARK, "middle", True)
    # ---- bottom: the four cells, as each robot's camera sees them
    yb = y0 + TOP + 30
    s.rect(30, yb, W - 60, BOT, fill="#ffffff", stroke=MID, sw=2, rx=6)
    s.text(W / 2, yb + 38, "The four cells, each as seen by the camera of a robot shooting at it", 23, INK,
           "middle", True)
    cells = [("Red audience cell", 34), ("Red rear cell", 30), ("Blue audience cell", 38), ("Blue rear cell", 42)]
    cw = (W - 60) / 4
    for i, (name, first) in enumerate(cells):
        mx = 30 + i * cw + cw / 2; my = yb + 160; sc3 = 0.72
        s.text(mx, yb + 88, name, 21, INK, "middle", True)
        s.rect(mx - 218 * sc3, my - 60 * sc3, 436 * sc3, 117 * sc3, fill="#ffffff", stroke=MID, sw=1.5, rx=5)
        for k, off in enumerate(M.OFFSETS):
            x = mx + off * sc3
            s.rect(x - tag / 2 * sc3, my - tag / 2 * sc3 - 4, tag * sc3, tag * sc3, fill=INK)
            s.text(x, my + 4, f"{first + k}", 22, "#ffffff", "middle", True)
        s.text(mx, my + 82, f"{first} = slot 1, on the left", 18, INK, "middle", True)
        s.text(mx, my + 106, f"{first + 3} = slot 4, on the right", 18, INK, "middle", True)
    key_below(s, yk, cols)
    s.save("fig6-any-tag-any-cell")


if __name__ == "__main__":
    fig1(); fig2(); fig3(); fig4(); fig5(); fig6()
    print("figures in", D.OUT)
