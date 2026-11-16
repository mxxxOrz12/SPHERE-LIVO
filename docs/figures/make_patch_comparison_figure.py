#!/usr/bin/env python3
"""Image-space reference patch vs. plane-anchored spherical patch (EN / ZH).

All image-plane shapes are computed: an equidistant fisheye model projects
the same scene into a forward camera A and a side camera B, so the curved
footprint and its affine approximation in camera B are real projections.
Writes sphere_patch_comparison_{en,zh}.{svg,pdf,png} (PDF/PNG via render_svg.js).
"""
import math
import os
import subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 1800, 800
COL = dict(ink="#111111", grp="#EFEFEF", plane="#F6C2B6", plane_s="#C9533F", cam="#F49A8C",
           new="#A4D89E", new_a="#3E9C47", ref="#F7B9A9", ref_s="#C9533F", lidar_a="#1C9AD6",
           img="#F7F7F7", ring="#9A9A9A", warp="#1C9AD6")
EN = dict(font="'Liberation Serif','Times New Roman',Times,serif",
          ta="(a) Image-space reference patch", tb="(b) Plane-anchored spherical patch (ours)",
          camA="Cam A (reference)", camB="Cam B", camA_b="Cam A", wall="LiDAR plane",
          true="True footprint", aff="Affine warp", ref="Reference pixels", warp="Warp",
          samp="Projected samples", world="World samples q_k",
          sa=["Stored: reference image, pixel, normal", "Per-view warp, error grows near FoV edge"],
          sb=["Stored: 8×8 world points + canonical intensity", "Shared by all cameras, no warp"])
ZH = dict(font="'Liberation Serif','Times New Roman','Noto Serif CJK SC',serif",
          ta="(a) 图像空间参考图块", tb="(b) 平面锚定的球面图块（本文）",
          camA="相机 A（参考）", camB="相机 B", camA_b="相机 A", wall="LiDAR 平面",
          true="真实投影区域", aff="仿射变形近似", ref="参考像素块", warp="变形",
          samp="投影采样点", world="世界系采样点 q_k",
          sa=["存储：参考图像、像素坐标、法向", "逐视角变形，视场边缘误差增大"],
          sb=["存储：8×8 世界点 + 规范灰度", "所有相机共享，无需变形"])

# ---------------- geometry ----------------
def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])  # world -> camera, yaw a

CAMS = {"A": (np.array([0.0, 0.0, 0.0]), rot_y(0.0)),
        "B": (np.array([0.45, 0.0, -0.1]), rot_y(math.radians(86)))}
C0 = np.array([1.25, 0.15, 4.0])                       # plane point (patch centre)
NRM = np.array([-0.55, 0.05, -1.0]); NRM /= np.linalg.norm(NRM)
RIMG = 128.0                                           # image-circle radius (θ = 90°)
F = RIMG / (math.pi / 2)


def project(cam, p):
    t, R = CAMS[cam]
    pc = R @ (p - t)
    th = math.atan2(math.hypot(pc[0], pc[1]), pc[2])
    ph = math.atan2(pc[1], pc[0])
    return np.array([F * th * math.cos(ph), -F * th * math.sin(ph)])


def unproject(cam, uv):
    t, R = CAMS[cam]
    r = math.hypot(uv[0], uv[1]); th = r / F; ph = math.atan2(-uv[1], uv[0])
    d = np.array([math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th)])
    return t, R.T @ d


def hit_plane(o, d):
    lam = NRM @ (C0 - o) / (NRM @ d)
    return o + lam * d


def ref_square(half=26, n=24):
    """Boundary of a square pixel patch in A, mapped through the plane into B."""
    ca = project("A", C0)
    edge = []
    for k in range(4 * n):
        s = (k % n) / n * 2 - 1
        side = k // n
        off = [(s, -1), (1, s), (-s, 1), (-1, -s)][side]
        edge.append(ca + half * np.array(off))
    true_b = [project("B", hit_plane(*unproject("A", e))) for e in edge]
    # affine (first-order) warp around the centre
    cb = project("B", C0); eps = 0.5; J = np.zeros((2, 2))
    for i in range(2):
        du = np.zeros(2); du[i] = eps
        J[:, i] = (project("B", hit_plane(*unproject("A", ca + du))) -
                   project("B", hit_plane(*unproject("A", ca - du)))) / (2 * eps)
    aff_b = [cb + J @ (e - ca) for e in [ca + half * np.array(o) for o in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]]
    sq_a = [ca + half * np.array(o) for o in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
    return sq_a, true_b, aff_b


def angular_grid(n=8, step=2 * 26 / F / 7):
    """8x8 equal-angle bearing grid around A's ray to C0, intersected with the plane."""
    o, R = CAMS["A"]
    b0 = (C0 - o) / np.linalg.norm(C0 - o)
    e1 = np.cross([0, 1, 0], b0); e1 /= np.linalg.norm(e1); e2 = np.cross(b0, e1)
    pts = []
    for r in range(n):
        row = []
        for c in range(n):
            du, dv = (c - (n - 1) / 2) * step, (r - (n - 1) / 2) * step
            d = b0 + du * e1 + dv * e2; d /= np.linalg.norm(d)
            row.append(hit_plane(o, d))
        pts.append(row)
    return pts

# ---------------- 3-D sketch view ----------------
EYE, LOOK = np.array([-0.6, 2.4, -3.2]), np.array([1.0, 0.0, 2.6])
_f = LOOK - EYE; _f /= np.linalg.norm(_f)
_r = np.cross([0, 1, 0], _f); _r /= np.linalg.norm(_r)
_u = np.cross(_f, _r)


def view(p, ox, oy, s=880):
    q = p - EYE
    z = q @ _f
    return ox + s * (q @ _r) / z, oy - s * (q @ _u) / z

# ---------------- svg helpers ----------------
def T(x, y, s, L, fs=22, anchor="middle", color=None, weight="bold"):
    s = s.replace("&", "&amp;").replace("<", "&lt;")
    if "_k" in s:
        s = s.replace("_k", '<tspan font-size="%.1f" dy="5" font-style="italic">k</tspan>' % (fs * .7))
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-family="{L["font"]}" font-size="{fs}" font-weight="{weight}" '
            f'fill="{color or COL["ink"]}" text-anchor="{anchor}">{s}</text>')


def poly(pts, fill, stroke, sw=2.0, dash=None, close=True, extra=""):
    d = "M" + " L".join(f"{x:.1f},{y:.1f}" for x, y in pts) + (" Z" if close else "")
    ds = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{ds} stroke-linejoin="round"{extra}/>'


def line(a, b, col, sw=2, dash=None, marker=None):
    ds = f' stroke-dasharray="{dash}"' if dash else ""
    mk = f' marker-end="url(#{marker})"' if marker else ""
    return f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{col}" stroke-width="{sw}"{ds}{mk}/>'


def defs():
    out = ["<defs>"]
    for name, col in (("mb", COL["lidar_a"]), ("mk", COL["ink"]), ("mw", COL["warp"])):
        out.append(f'<marker id="{name}" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="4.5" markerHeight="4.5" '
                   f'orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="{col}"/></marker>')
    for name, cx in (("clipA1", 0), ("clipB1", 1), ("clipA2", 2), ("clipB2", 3)):
        pass
    out.append("</defs>")
    return "".join(out)


def scene(x0, L, ours):
    o = []
    a1 = np.cross(NRM, [0, 1, 0]); a1 /= np.linalg.norm(a1); a2 = np.cross(NRM, a1)
    if a2[1] < 0:
        a2 = -a2
    corners = [C0 + 2.4 * a1 * sx + 1.7 * a2 * sy for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
    tips = {k: CAMS[k][0] + 0.9 * CAMS[k][1].T @ np.array([0, 0, 1.0]) for k in CAMS}
    pts = corners + [CAMS[k][0] for k in CAMS] + list(tips.values())
    raw = np.array([view(p, 0, 0, 1.0) for p in pts])
    lo, hi = raw.min(0), raw.max(0)
    bx0, by0, bx1, by1 = x0 + 40, 110, x0 + 440, 640
    sc = min((bx1 - bx0) / (hi[0] - lo[0]), (by1 - by0) / (hi[1] - lo[1]))
    offx = bx0 + ((bx1 - bx0) - sc * (hi[0] - lo[0])) / 2 - sc * lo[0]
    offy = by0 + ((by1 - by0) - sc * (hi[1] - lo[1])) / 2 - sc * lo[1]

    def V(p):
        u, v = view(p, 0, 0, 1.0)
        return offx + sc * u, offy + sc * v
    o.append(poly([V(c) for c in corners], COL["plane"], COL["plane_s"], 2.5))
    pc = [V(c) for c in corners]
    if ours:   # top-right corner, leaves room for the sample label
        wl = max(pc, key=lambda q: q[0] - q[1])
        o.append(T(wl[0], wl[1] - 14, L["wall"], L, fs=21, anchor="end", color=COL["plane_s"]))
    else:      # top-left corner
        wl = min(pc, key=lambda q: q[0] + q[1])
        o.append(T(wl[0], wl[1] - 14, L["wall"], L, fs=21, anchor="start", color=COL["plane_s"]))
    camp = {k: V(CAMS[k][0]) for k in CAMS}
    cv = V(C0)
    if ours:
        g = angular_grid()
        for k in ("A", "B"):
            for p in (g[0][0], g[0][-1], g[-1][0], g[-1][-1]):
                o.append(line(camp[k], V(p), COL["new_a"], 1.5, "7 5"))
        for r in range(8):
            o.append(poly([V(p) for p in g[r]], "none", COL["new_a"], 1.0, close=False))
            o.append(poly([V(g[c][r]) for c in range(8)], "none", COL["new_a"], 1.0, close=False))
        for row in g:
            for p in row:
                x, y = V(p)
                o.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3.4" fill="{COL["new_a"]}"/>')
        gp = [V(p) for row in g for p in row]
        gx = min(q[0] for q in gp); gy = min(q[1] for q in gp)
        o.append(T(gx - 60, gy - 46, L["world"], L, fs=21, anchor="start", color=COL["new_a"]))
    else:
        for k in ("A", "B"):
            o.append(line(camp[k], cv, COL["ink"], 1.6, "7 5"))
        o.append(f'<circle cx="{cv[0]:.1f}" cy="{cv[1]:.1f}" r="5.5" fill="{COL["ink"]}"/>')
    nt = V(C0 + 0.9 * (-NRM))
    o.append(line(cv, nt, COL["lidar_a"], 3, marker="mb"))
    o.append(T(nt[0] + 14, nt[1] + 4, "n", L, fs=24, anchor="start", color=COL["lidar_a"]))
    o.append(line(camp["A"], camp["B"], "#777777", 7))
    for k in ("A", "B"):
        o.append(line(camp[k], V(tips[k]), COL["ink"], 2.2, marker="mk"))
    for k in ("A", "B"):
        x, y = camp[k]
        o.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="15" fill="{COL["cam"]}" stroke="{COL["ink"]}" stroke-width="2.5"/>'
                 f'<circle cx="{x:.1f}" cy="{y:.1f}" r="6" fill="#3A3A3A"/>')
        o.append(T(x, y + 44, k, L, fs=24))
    return "".join(o)


def fisheye(cx, cy, label, L):
    return (f'<circle cx="{cx}" cy="{cy}" r="{RIMG}" fill="{COL["img"]}" stroke="{COL["ink"]}" stroke-width="2.5"/>'
            f'<circle cx="{cx}" cy="{cy}" r="{RIMG*2/3:.1f}" fill="none" stroke="{COL["ring"]}" stroke-width="1" stroke-dasharray="3 5"/>'
            f'<circle cx="{cx}" cy="{cy}" r="{RIMG/3:.1f}" fill="none" stroke="{COL["ring"]}" stroke-width="1" stroke-dasharray="3 5"/>'
            + T(cx, cy + RIMG + 30, label, L, fs=22))


def panel(x0, L, ours):
    o = [f'<rect x="{x0}" y="14" width="872" height="{H-28}" fill="{COL["grp"]}" stroke="{COL["ink"]}" stroke-width="2" stroke-dasharray="8 5"/>']
    o.append(T(x0 + 24, 52, L["tb" if ours else "ta"], L, fs=28, anchor="start"))
    o.append(scene(x0, L, ours))
    RI = RIMG
    ax, bx = x0 + 690, x0 + 690
    ya, yb = 205, 520
    o.append(fisheye(ax, ya, L["camA_b"] if ours else L["camA"], L))
    o.append(fisheye(bx, yb, L["camB"], L))
    S = 1.0  # image drawn at true scale of the model

    def at(c, p):
        return (c[0] + S * p[0], c[1] + S * p[1])
    if ours:
        g = angular_grid()
        for cxy, k in (((ax, ya), "A"), ((bx, yb), "B")):
            proj = [[at(cxy, project(k, p)) for p in row] for row in g]
            for r in range(8):
                o.append(poly(proj[r], "none", COL["new_a"], 1.0, close=False))
                o.append(poly([proj[c][r] for c in range(8)], "none", COL["new_a"], 1.0, close=False))
            for row in proj:
                for x, y in row:
                    o.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.6" fill="{COL["new_a"]}"/>')
    else:
        sq, tb, ab = ref_square()
        o.append(poly([at((ax, ya), p) for p in sq], COL["ref"], COL["ref_s"], 2.4))
        o.append(poly([at((bx, yb), p) for p in tb], COL["ref"], COL["ref_s"], 2.4))
        o.append(poly([at((bx, yb), p) for p in ab], "none", COL["warp"], 2.4, dash="7 5"))
        pa = at((ax, ya), project("A", C0)); pb = at((bx, yb), project("B", C0))
        o.append(f'<path d="M{pa[0]-34:.1f},{pa[1]:.1f} C{ax-RIMG-60:.1f},{ya+40:.1f} {ax-RIMG-60:.1f},{yb-70:.1f} {pb[0]-6:.1f},{pb[1]-46:.1f}" '
                 f'fill="none" stroke="{COL["warp"]}" stroke-width="2.6" marker-end="url(#mw)"/>')
        o.append(T(ax - RIMG - 54, (ya + yb) / 2 - 10, L["warp"], L, fs=22, anchor="end", color=COL["warp"]))
        o.append(T(pa[0], pa[1] + 54, L["ref"], L, fs=19, color=COL["ref_s"]))
        # legend for B
        lx, ly = bx + RIMG - 40, ya + RIMG + 2
        o.append(f'<rect x="{x0+40}" y="{H-150}" width="24" height="16" fill="{COL["ref"]}" stroke="{COL["ref_s"]}" stroke-width="2"/>')
        o.append(T(x0 + 72, H - 136, L["true"], L, fs=19, anchor="start", color=COL["ref_s"]))
        o.append(line((x0 + 40, H - 112), (x0 + 64, H - 112), COL["warp"], 2.4, "6 4"))
        o.append(T(x0 + 72, H - 106, L["aff"], L, fs=19, anchor="start", color=COL["warp"]))
    note = L["sb" if ours else "sa"]
    col = COL["new_a"] if ours else COL["ref_s"]
    o.append(T(x0 + 436, H - 64, note[0], L, fs=22, color=col))
    o.append(T(x0 + 436, H - 34, note[1], L, fs=22, color=col))
    return "".join(o)


def build(L):
    return "".join([f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">', defs(),
                    f'<rect width="{W}" height="{H}" fill="#FFFFFF"/>', panel(18, L, False), panel(910, L, True), "</svg>"])


if __name__ == "__main__":
    for L, name in ((EN, "sphere_patch_comparison_en"), (ZH, "sphere_patch_comparison_zh")):
        svg = os.path.join(HERE, name + ".svg")
        open(svg, "w", encoding="utf-8").write(build(L))
        subprocess.run(["node", os.path.join(HERE, "render_svg.js"), svg], check=True)
