#!/usr/bin/env python3
"""Generate the SPHERE-LIVO pipeline figure (EN / ZH) as SVG, PDF and PNG.

The figure follows the configuration in config/sphere_oxford.yaml
(spherical_patch, shared_spherical_map, information_selection,
spherical_posterior_information, spherical_incremental_information,
spherical_shared_brightness, spherical_geometry_support,
spherical_descent_guard, spherical_transactional_update).

Usage: python3 make_pipeline_figure.py   (needs Python Playwright + Chromium
for PDF/PNG; the SVG is always written).
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 1600, 1010

C = {  # muted, print- and colour-blind-friendly palette
    "ink": "#1F2328", "mute": "#5B6170", "line": "#3A3F4A",
    "lio_f": "#F1F2F4", "lio_s": "#5B6170",
    "sph_f": "#E7EFF8", "sph_s": "#2F6497",
    "sel_f": "#E4F1EA", "sel_s": "#2B7A57",
    "upd_f": "#FBEEE1", "upd_s": "#B0601B",
    "map_f": "#FFFFFF", "grey": "#D5D8DD",
}

EN = dict(
    lang="en",
    a="(a) System overview", b="(b) Canonical spherical patch",
    c="(c) Posterior-information selection", d="(d) Guarded joint update",
    imu="IMU", lidar="LiDAR", rig=["Fisheye rig", "(N cams)"],
    prop="IMU propagation", deskew="Scan deskew",
    esikf=["Point-to-plane", "ESIKF LIO update"],
    sync=["Rig grouping", "|Δt| ≤ 5 ms"],
    retr=["Patch retrieval", "temporal + LiDAR seeds"],
    repr_=["Spherical reprojection", "ZNCC pre-gate"],
    info=["Whitened information", r"Ω_{i} = L^{T} H_{i} L"],
    selc=["Greedy log-det", r"|S| ≤ B, ≤ K cams"],
    upd=["Joint update", "LIO prior + visual", "Armijo guard · rollback"],
    upd_note="Armijo guard · rollback",
    pose=["Pose and covariance", r"x^{+}, P^{+}"],
    bank="Shared 3D patch map",
    bank_l=["Plane-anchored centre, normal", r"World samples q_{k}, canon. Ī_{k}",
            r"Range variance σ^{2}_{λ,k}", "FIFO-bounded, refreshed"],
    prior=r"\rm{LIO prior }(x_{l}, P_{l})",
    whiten=r"L L^{T} = P_{l}/σ^{2}",
    points="3D points", insert=["Insert / refresh", "on LiDAR planes"],
    loop="Retrieve world-anchored patches: one reference shared by all cameras",
    pts="Points q_{k}", bear="Bearing grid", plane="LiDAR plane", grid8="8 × 8 angular grid",
    support="Plane support", canon="Canonical reference",
    pool="Candidate pool", sel_lbl="selected", rej_lbl="not selected",
    inactive="inactive (≤ K cams)",
    weak="LiDAR-weak axis", cons=["1 obs. per shared patch", "|S| ≤ B,  |cam(S)| ≤ K"],
    f1="Linearize on frozen support", f2="Eliminate (g_{c}, b_{c}) by Schur",
    f3="Damped Gauss–Newton step", f4="Armijo decrease?",
    yes="yes", no="no", acc="Accept, iterate", rb=["Backtrack ×6,", r"else x^{+}=x_{l}, P^{+}=P_{l}"],
    gw="Geometry weight",
)

ZH = dict(
    lang="zh",
    a="(a) 系统总览", b="(b) 平面锚定的规范球面图块",
    c="(c) 后验信息驱动的观测选择", d="(d) 带保护的联合更新",
    imu="IMU", lidar="LiDAR", rig=["鱼眼相机", "阵列 (N)"],
    prop="IMU 前向传播", deskew="点云去畸变",
    esikf=["点到平面", "ESIKF LIO 更新"],
    sync=["多相机同步分组", "|Δt| ≤ 5 ms"],
    retr=["图块检索", "时序 + LiDAR 种子"],
    repr_=["球面重投影", "ZNCC 预筛选"],
    info=["白化信息矩阵", r"Ω_{i} = L^{T} H_{i} L"],
    selc=["贪心 log-det 选择", r"|S| ≤ B，≤ K 个相机"],
    upd=["联合更新", "LIO 先验 + 视觉", "Armijo 保护 · 回滚"],
    upd_note="Armijo 保护 · 失败回滚",
    pose=["位姿与协方差", r"x^{+}, P^{+}"],
    bank="共享三维图块地图",
    bank_l=["平面锚定的中心与法向", r"世界系采样 q_{k}，规范灰度 Ī_{k}",
            r"距离方差 σ^{2}_{λ,k}", "FIFO 限容，参考帧刷新"],
    prior=r"\rm{LIO 先验 }(x_{l}, P_{l})",
    whiten=r"L L^{T} = P_{l}/σ^{2}",
    points="三维点", insert=["插入 / 刷新", "（基于 LiDAR 平面）"],
    loop="检索世界系锚定图块：所有相机共享同一参考",
    pts="采样点 q_{k}", bear="方位角网格", plane="LiDAR 平面", grid8="8 × 8 等角网格",
    support="平面支撑", canon="规范参考灰度",
    pool="候选池", sel_lbl="已选", rej_lbl="未选",
    inactive="未激活（≤ K 个相机）",
    weak="LiDAR 弱约束方向", cons=["每个共享图块至多 1 个观测", "|S| ≤ B，|cam(S)| ≤ K"],
    f1="在冻结支撑集上线性化", f2="Schur 消元 (g_{c}, b_{c})",
    f3="阻尼 Gauss–Newton 步", f4="满足 Armijo 下降？",
    yes="是", no="否", acc="接受并迭代", rb=["回溯 ≤ 6 次，", r"否则 x^{+}=x_{l}, P^{+}=P_{l}"],
    gw="几何权重",
)

SERIF = "'Liberation Serif','Times New Roman',Times,'Noto Serif CJK SC',serif"


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _parse(s, i=0, stop=None):
    """Tiny TeX-like parser: _{..} ^{..} \\rm{..} \\bf{..}; returns runs."""
    runs = []
    while i < len(s):
        ch = s[i]
        if stop and ch == stop:
            return runs, i + 1
        if ch in "_^":
            kind = "sub" if ch == "_" else "sup"
            if s[i + 1] == "{":
                inner, i = _parse(s, i + 2, "}")
            else:
                inner, i = [(s[i + 1], {})], i + 2
            for t, st in inner:
                st = dict(st)
                st["lvl"] = st.get("lvl", []) + [kind]
                runs.append((t, st))
            continue
        if s.startswith("\\{", i) or s.startswith("\\}", i):
            runs.append((s[i + 1], {}))
            i += 2
            continue
        if s.startswith(r"\rm{", i) or s.startswith(r"\bf{", i):
            key = "rm" if s[i + 1] == "r" else "bf"
            inner, i = _parse(s, i + 4, "}")
            for t, st in inner:
                st = dict(st)
                st[key] = True
                runs.append((t, st))
            continue
        runs.append((ch, {}))
        i += 1
    return runs, i


def m(x, y, s, fs=22, anchor="middle", color=None, weight="normal", math=True, italic=False):
    color = color or C["ink"]
    runs, _ = _parse(s) if math else ([(ch, {}) for ch in s], 0)
    out = [f'<text x="{x:.1f}" y="{y:.1f}" font-size="{fs}" text-anchor="{anchor}" '
           f'fill="{color}" font-weight="{weight}" font-family="{SERIF}">']
    cur_shift = 0.0
    n = len(runs)
    ital = [False] * n
    if math or italic:
        k = 0
        while k < n:
            t, st = runs[k]
            lvl = tuple(st.get("lvl", []))
            if re.match(r"[A-Za-z]", t) and not st.get("rm"):
                j = k
                while j < n and re.match(r"[A-Za-z]", runs[j][0]) and tuple(runs[j][1].get("lvl", [])) == lvl \
                        and not runs[j][1].get("rm"):
                    j += 1
                word = j - k > 1 and not lvl
                for q in range(k, j):
                    ital[q] = italic or not word
                k = j
                continue
            if re.match(r"[α-ωĪθ]", t) and not st.get("rm"):
                ital[k] = True
            k += 1
    merged = []
    for (t, st), it in zip(runs, ital):
        key = (tuple(st.get("lvl", [])), it, st.get("bf", False))
        if merged and merged[-1][1] == key:
            merged[-1][0] += t
        else:
            merged.append([t, key])

    def est(t, size):
        return sum(0.3 if c in "ilj,.·()|" else 0.5 for c in t) * size

    group_lvl, group_w = None, 0.0   # current script group (level, width)
    first = None                     # (level, width) of a finished script group
    stacking = False
    for t, (lvl, it, bf) in merged:
        size = fs
        shift = 0.0
        for k in lvl:
            shift += (0.28 if k == "sub" else -0.42) * size
            size *= 0.72
        dy = shift - cur_shift
        cur_shift = shift
        attrs = f'font-size="{size:.1f}"'
        if dy:
            attrs += f' dy="{dy:.1f}"'
        w = est(t, size)
        dx = 0.0
        if lvl != group_lvl:                       # a group ended
            if stacking:
                dx += max(0.0, first[1] - group_w)  # move past the wider script
                stacking, first = False, None
            elif group_lvl and len(group_lvl) == 1:
                first = (group_lvl, group_w)
            else:
                first = None
            if first and len(lvl) == 1 and lvl != first[0]:
                dx -= first[1]
                stacking = True
            elif len(lvl) != 1:
                first = None
            group_lvl, group_w = lvl, 0.0
        group_w += w
        if dx:
            attrs += f' dx="{dx:.1f}"'
        if it:
            attrs += ' font-style="italic"'
        if bf:
            attrs += ' font-weight="bold"'
        out.append(f'<tspan {attrs}>{esc(t)}</tspan>')
    out.append("</text>")
    return "".join(out)


def box(x, y, w, h, kind, lines, fs=22, num=None, bold_first=False, r=4):
    f, s = C[kind + "_f"], C[kind + "_s"]
    out = [f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{f}" stroke="{s}" stroke-width="1.6"/>']
    n = len(lines)
    lh = fs * 1.25
    y0 = y + h / 2 - (n - 1) * lh / 2 + fs * 0.34
    for k, line in enumerate(lines):
        txt = line
        if k == 0 and num:
            txt = num + " " + line
        out.append(m(x + w / 2, y0 + k * lh, txt, fs=fs, weight="bold" if (bold_first and k == 0) else "normal"))
    return "".join(out)


def arrow(pts, color=None, dashed=False, width=1.6, head=True):
    color = color or C["line"]
    d = "M" + " L".join(f"{px:.1f},{py:.1f}" for px, py in pts)
    mk = {C["line"]: "aL", C["sph_s"]: "aB", C["sel_s"]: "aG", C["upd_s"]: "aO", C["mute"]: "aM"}[color]
    dash = ' stroke-dasharray="7 5"' if dashed else ""
    end = f' marker-end="url(#{mk})"' if head else ""
    return f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{dash}{end}/>'


def defs():
    out = ["<defs>"]
    for name, col in [("aL", C["line"]), ("aB", C["sph_s"]), ("aG", C["sel_s"]), ("aO", C["upd_s"]), ("aM", C["mute"])]:
        out.append(f'<marker id="{name}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" '
                   f'orient="auto-start-reverse"><path d="M0,0.8 L9.5,5 L0,9.2 z" fill="{col}"/></marker>')
    out.append('<pattern id="hatch" width="6" height="6" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">'
               f'<line x1="0" y1="0" x2="0" y2="6" stroke="{C["grey"]}" stroke-width="2"/></pattern>')
    out.append("</defs>")
    return "".join(out)


def circled(k):
    return "①②③"[k - 1]


def panel_a(L):
    o = [m(20, 30, L["a"], fs=27, anchor="start", weight="bold", math=False)]
    # ---- row 1: LIO back-end ------------------------------------------
    o.append(box(20, 58, 110, 50, "lio", [L["imu"]]))
    o.append(box(20, 140, 110, 50, "lio", [L["lidar"]]))
    o.append(box(160, 58, 165, 50, "lio", [L["prop"]]))
    o.append(box(160, 140, 165, 50, "lio", [L["deskew"]]))
    o.append(box(360, 70, 190, 108, "lio", L["esikf"]))
    o.append(arrow([(130, 83), (158, 83)]))
    o.append(arrow([(130, 165), (158, 165)]))
    o.append(arrow([(242, 108), (242, 138)]))
    o.append(arrow([(325, 83), (342, 83), (342, 105), (358, 105)]))
    o.append(arrow([(325, 165), (342, 165), (342, 145), (358, 145)]))
    # ---- row 2: visual front-end --------------------------------------
    y2, h2 = 268, 86
    o.append(box(20, y2 + 4, 110, 78, "lio", L["rig"], fs=20))
    o.append(box(160, y2, 165, h2, "lio", L["sync"], fs=21))
    o.append(box(360, y2, 190, h2, "sph", L["retr"], fs=19.5, num=circled(1)))
    o.append(box(580, y2, 200, h2, "sph", L["repr_"], fs=21))
    o.append(box(810, y2, 200, h2, "sel", L["info"], fs=21))
    o.append(box(1040, y2, 200, h2, "sel", L["selc"], fs=21, num=circled(2)))
    o.append(arrow([(130, y2 + 43), (158, y2 + 43)], C["sph_s"]))
    o.append(arrow([(325, y2 + 43), (358, y2 + 43)], C["sph_s"]))
    o.append(arrow([(550, y2 + 43), (578, y2 + 43)], C["sph_s"]))
    o.append(arrow([(780, y2 + 43), (808, y2 + 43)], C["sel_s"]))
    o.append(arrow([(1010, y2 + 43), (1038, y2 + 43)], C["sel_s"]))
    # ESIKF -> retrieval (points)
    o.append(arrow([(455, 178), (455, y2 - 2)]))
    o.append(m(465, 226, L["points"], fs=20, anchor="start", color=C["mute"], math=False))
    # ---- joint update + pose ------------------------------------------
    o.append(box(1040, 64, 200, 120, "upd", L["upd"], fs=20.5, num=circled(3)))
    o.append(arrow([(1140, y2), (1140, 186)], C["sel_s"]))
    o.append(arrow([(550, 124), (1038, 124)]))
    o.append(m(795, 113, L["prior"], fs=21))
    # whitening branch to the information box
    o.append(f'<circle cx="910" cy="124" r="3.5" fill="{C["line"]}"/>')
    o.append(arrow([(910, 124), (910, y2 - 2)]))
    o.append(m(920, 226, L["whiten"], fs=20, anchor="start"))
    o.append(box(1390, 76, 190, 96, "lio", L["pose"]))
    o.append(arrow([(1240, 124), (1388, 124)]))
    # ---- shared map ---------------------------------------------------
    bx, by, bw, bh = 1330, 262, 250, 150
    o.append(f'<rect x="{bx}" y="{by}" width="{bw}" height="{bh}" rx="4" fill="#FFFFFF" stroke="{C["sph_s"]}" stroke-width="1.8"/>')
    o.append(m(bx + bw / 2, by + 28, L["bank"], fs=22, weight="bold", color=C["sph_s"], math=False))
    o.append(f'<line x1="{bx+16}" y1="{by+40}" x2="{bx+bw-16}" y2="{by+40}" stroke="{C["grey"]}" stroke-width="1.2"/>')
    for k, line in enumerate(L["bank_l"]):
        o.append(m(bx + bw / 2, by + 66 + k * 25, line, fs=18.5))
    o.append(arrow([(1485, 172), (1485, by - 2)], C["sph_s"]))
    o.append(m(1475, 205, L["insert"][0], fs=19, anchor="end", color=C["sph_s"], math=False))
    o.append(m(1475, 229, L["insert"][1], fs=19, anchor="end", color=C["sph_s"], math=False))
    # ---- retrieval loop -----------------------------------------------
    o.append(arrow([(bx + bw / 2, by + bh), (bx + bw / 2, 438), (455, 438), (455, y2 + h2 + 2)], C["sph_s"], dashed=True))
    o.append(m(900, 430, L["loop"], fs=20, color=C["sph_s"], math=False))
    return "".join(o)


def panel_b(L, x0=20, y0=468):
    o = [m(x0, y0 + 26, L["b"], fs=26, anchor="start", weight="bold", math=False)]
    cx, cy = x0 + 40, y0 + 200
    # unit sphere arc
    o.append(f'<path d="M{cx+70},{cy-118} A136,136 0 0 1 {cx+70},{cy+118}" fill="none" stroke="{C["sph_s"]}" stroke-width="1.4" stroke-dasharray="6 4"/>')
    o.append(m(cx + 40, cy + 132, "S^{2}", fs=21, color=C["sph_s"]))
    # plane (parallelogram)
    P = [(x0 + 300, y0 + 92), (x0 + 455, y0 + 62), (x0 + 488, y0 + 250), (x0 + 333, y0 + 280)]
    o.append('<polygon points="' + " ".join(f"{a},{b}" for a, b in P) + f'" fill="{C["upd_f"]}" stroke="{C["upd_s"]}" stroke-width="1.6"/>')

    def bil(u, v):
        a = [P[0][i] * (1 - u) * (1 - v) + P[1][i] * u * (1 - v) + P[2][i] * u * v + P[3][i] * (1 - u) * v for i in (0, 1)]
        return a
    us = [0.22, 0.5, 0.78]
    for u in us:  # grid lines
        p, q = bil(u, 0.15), bil(u, 0.85)
        o.append(f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="{C["upd_s"]}" stroke-width="1" opacity=".7"/>')
        p, q = bil(0.15, u), bil(0.85, u)
        o.append(f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="{C["upd_s"]}" stroke-width="1" opacity=".7"/>')
    # rays to corner samples and centre
    rays = [bil(0.22, 0.22), bil(0.78, 0.22), bil(0.22, 0.78), bil(0.78, 0.78)]
    for p in rays:
        o.append(f'<line x1="{cx}" y1="{cy}" x2="{p[0]:.1f}" y2="{p[1]:.1f}" stroke="{C["sph_s"]}" stroke-width="1" opacity=".55"/>')
    pc = bil(0.5, 0.5)
    o.append(f'<line x1="{cx}" y1="{cy}" x2="{pc[0]:.1f}" y2="{pc[1]:.1f}" stroke="{C["sph_s"]}" stroke-width="2.4"/>')
    for u in us:
        for v in us:
            p = bil(u, v)
            o.append(f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="4.2" fill="{C["upd_s"]}"/>')
    # bearing grid on the sphere (small quad where rays cross r=136)
    def onr(p, r=136):
        dx, dy = p[0] - cx, p[1] - cy
        n = (dx * dx + dy * dy) ** 0.5
        return cx + dx / n * r, cy + dy / n * r
    q = [onr(rays[0]), onr(rays[1]), onr(rays[3]), onr(rays[2])]
    q = [(a + 0, b) for a, b in q]
    o.append('<polygon points="' + " ".join(f"{a:.1f},{b:.1f}" for a, b in q) + f'" fill="{C["sph_f"]}" stroke="{C["sph_s"]}" stroke-width="1.5"/>')
    mid = onr(pc)
    o.append(f'<circle cx="{mid[0]:.1f}" cy="{mid[1]:.1f}" r="3" fill="{C["sph_s"]}"/>')
    o.append(f'<circle cx="{cx}" cy="{cy}" r="5.5" fill="{C["ink"]}"/>')
    o.append(m(cx - 6, cy + 32, "C^{w}_{r}", fs=22))
    o.append(m(cx + 150, y0 + 72, L["bear"], fs=19, color=C["sph_s"], math=False))
    o.append(arrow([(cx + 150, y0 + 80), (mid[0] - 2, mid[1] - 28)], C["sph_s"], width=1.2))
    o.append(m(cx + 208, cy + 30, "R^{w}_{r} b^{r}_{k}", fs=21, color=C["sph_s"]))
    # normal
    o.append(arrow([(pc[0], pc[1]), (pc[0] + 70, pc[1] - 20)], C["upd_s"], width=1.8))
    o.append(m(pc[0] + 92, pc[1] - 18, "n^{w}_{i}", fs=22, color=C["upd_s"]))
    o.append(m(x0 + 420, y0 + 56, r"\rm{" + L["plane"] + r"} Π_{i}", fs=20, color=C["upd_s"]))
    o.append(m(x0 + 470, y0 + 300, "q^{w}_{k}", fs=22, color=C["upd_s"]))
    o.append(m(x0 + 410, y0 + 322, L["grid8"], fs=18.5, color=C["mute"], math=False))
    # equations
    ys = y0 + 370
    eq = [
        r"b^{r}_{k} = \rm{normalize}(b^{r}_{0} + u_{k} Δθ e^{r}_{1} + v_{k} Δθ e^{r}_{2})",
        r"q^{w}_{k} = C^{w}_{r} + λ_{k} R^{w}_{r} b^{r}_{k},   λ_{k} = \frac",
    ]
    o.append(m(x0 + 252, ys, eq[0], fs=21))
    o.append(m(x0 + 252, ys + 36, r"q^{w}_{k} = C^{w}_{r} + λ_{k} R^{w}_{r} b^{r}_{k},   λ_{k} = n^{T}_{i}(c_{i} − C_{r}) / n^{T}_{i}R b_{k}", fs=21))
    o.append(m(x0 + 8, ys + 80, r"\rm{" + L["support"] + r":}", fs=20, anchor="start", color=C["mute"]))
    o.append(m(x0 + 300, ys + 80, r"‖(I − n n^{T})(q_{k} − c_{i})‖ ≤ 3r_{i},  σ^{2}_{λ,k} = J_{k} Σ_{Π} J^{T}_{k}", fs=20))
    o.append(m(x0 + 8, ys + 116, r"\rm{" + L["canon"] + r":}", fs=20, anchor="start", color=C["mute"]))
    o.append(m(x0 + 300, ys + 116, r"Ī_{k} = g_{r} I_{r}(π_{r}(q_{k})) + b_{r}", fs=20))
    return "".join(o)


def panel_c(L, x0=548, y0=468):
    o = [m(x0, y0 + 26, L["c"], fs=26, anchor="start", weight="bold", math=False)]
    # covariance ellipses
    ex, ey = x0 + 92, y0 + 160
    o.append(f'<ellipse cx="{ex}" cy="{ey}" rx="78" ry="26" transform="rotate(-28 {ex} {ey})" fill="none" stroke="{C["lio_s"]}" stroke-width="1.6" stroke-dasharray="6 4"/>')
    o.append(f'<ellipse cx="{ex}" cy="{ey}" rx="30" ry="20" transform="rotate(-28 {ex} {ey})" fill="{C["sel_f"]}" stroke="{C["sel_s"]}" stroke-width="1.8"/>')
    o.append(m(ex - 70, ey - 46, "P_{l}", fs=22, color=C["lio_s"]))
    o.append(m(ex + 4, ey + 7, "P^{+}", fs=20, color=C["sel_s"]))
    import math as _m
    ca, sa = _m.cos(_m.radians(-28)), _m.sin(_m.radians(-28))
    o.append(f'<line x1="{ex-100*ca:.1f}" y1="{ey-100*sa:.1f}" x2="{ex+100*ca:.1f}" y2="{ey+100*sa:.1f}" stroke="{C["mute"]}" stroke-width="1.1" stroke-dasharray="2 4"/>')
    o.append(m(ex + 6, ey + 74, L["weak"], fs=18.5, color=C["mute"], math=False))
    # candidate pool
    px, py = x0 + 205, y0 + 76
    o.append(m(px + 140, py - 8, L["pool"], fs=19, color=C["mute"], math=False))
    rows = [("I_{0}", 7, 12), ("I_{1}", 5, 12), ("I_{2}", 0, 12)]
    sz, gap = 17, 5
    for r, (lab, nsel, ntot) in enumerate(rows):
        yy = py + 8 + r * 40
        o.append(m(px - 10, yy + 15, lab, fs=21, anchor="end"))
        for k in range(ntot):
            xx = px + k * (sz + gap)
            if r == 2:
                fill, st = "url(#hatch)", C["grey"]
            else:
                fill, st = (C["sel_s"], C["sel_s"]) if k < nsel else ("#E9EBEE", "#E9EBEE")
            o.append(f'<rect x="{xx}" y="{yy}" width="{sz}" height="{sz + 4}" rx="2" fill="{fill}" stroke="{st}"/>')
    o.append(m(px + 140, py + 152, L["inactive"], fs=18, color=C["mute"], math=False))
    # legend
    lx, ly = px, py + 176
    o.append(f'<rect x="{lx}" y="{ly-13}" width="13" height="15" fill="{C["sel_s"]}"/>')
    o.append(m(lx + 20, ly, L["sel_lbl"], fs=18, anchor="start", math=False))
    o.append(f'<rect x="{lx+110}" y="{ly-13}" width="13" height="15" fill="#E9EBEE"/>')
    o.append(m(lx + 130, ly, L["rej_lbl"], fs=18, anchor="start", math=False))
    # equations
    xs, ys = x0 + 252, y0 + 300
    o.append(m(xs, ys, r"L L^{T} = P^{\rm{pose}}_{l} / σ^{2},    Ω_{S} = L^{T} H_{v}(S) L", fs=21))
    o.append(m(xs, ys + 38, r"Δ_{i} = \rm{log det}(I + Ω_{S∪\{i\}}) − \rm{log det}(I + Ω_{S})", fs=21))
    o.append(m(xs, ys + 76, r"i^{*} = \rm{arg max}_{i} Δ_{i} ,   S ← S ∪ \{i^{*}\}", fs=21))
    o.append(m(xs, ys + 114, L["cons"][1], fs=20, color=C["sel_s"]))
    o.append(m(xs, ys + 142, L["cons"][0], fs=19, color=C["sel_s"], math=False))
    o.append(m(xs, ys + 176, r"\rm{(}H_{v}\rm{: per-camera }(g_{c},b_{c})\rm{ Schur-eliminated)}" if L["lang"] == "en"
               else r"（H_{v} 中已 Schur 消去各相机的 (g_{c}, b_{c})）", fs=18, color=C["mute"]))
    return "".join(o)


def panel_d(L, x0=1076, y0=468):
    o = [m(x0, y0 + 26, L["d"], fs=26, anchor="start", weight="bold", math=False)]
    xs, ys = x0 + 252, y0 + 70
    o.append(m(xs, ys, r"x^{+} = \rm{arg min}_{x} ‖x ⊟ x_{l}‖^{2}_{P_{l}^{−1}} + ‖θ_{c} − θ^{0}_{c}‖^{2}_{Λ}", fs=21))
    o.append(m(xs, ys + 36, r"+ σ^{−2} Σ_{i,k} w_{ik} ρ_{H}(r_{ik})", fs=21))
    o.append(m(xs, ys + 72, r"r_{ik} = g_{c} I_{c}(π_{c}(q_{k})) + b_{c} − Ī_{k}", fs=21))
    o.append(m(x0 + 8, ys + 110, r"\rm{" + L["gw"] + r":}", fs=20, anchor="start", color=C["mute"]))
    o.append(m(xs + 60, ys + 110, r"w_{ik} = σ^{2} / (σ^{2} + (j_{ik}·r̂_{k})^{2} σ^{2}_{λ,k})", fs=20))
    # flow chart
    fy = y0 + 210
    bw, bh, fx = 300, 38, x0 + 30
    steps = [L["f1"], L["f2"], L["f3"]]
    for k, s in enumerate(steps):
        yy = fy + k * 56
        o.append(f'<rect x="{fx}" y="{yy}" width="{bw}" height="{bh}" rx="4" fill="{C["upd_f"]}" stroke="{C["upd_s"]}" stroke-width="1.4"/>')
        o.append(m(fx + bw / 2, yy + 26, s, fs=19))
        o.append(arrow([(fx + bw / 2, yy + bh), (fx + bw / 2, yy + 54)], C["upd_s"], width=1.4))
    dy = fy + 3 * 56 + 22
    dcx = fx + bw / 2
    o.append(f'<polygon points="{dcx-120},{dy} {dcx},{dy-24} {dcx+120},{dy} {dcx},{dy+24}" fill="#FFFFFF" stroke="{C["upd_s"]}" stroke-width="1.4"/>')
    o.append(m(dcx, dy + 7, L["f4"], fs=18.5, math=False))
    # yes branch -> accept (loop back)
    ax = fx + bw + 30
    o.append(arrow([(dcx + 120, dy), (ax + 4, dy)], C["upd_s"], width=1.4))
    o.append(m(dcx + 140, dy - 8, L["yes"], fs=17, color=C["upd_s"], math=False))
    o.append(f'<rect x="{ax+6}" y="{dy-19}" width="128" height="38" rx="4" fill="{C["upd_f"]}" stroke="{C["upd_s"]}" stroke-width="1.4"/>')
    o.append(m(ax + 70, dy + 6, L["acc"], fs=18, math=False))
    o.append(arrow([(ax + 70, dy - 19), (ax + 70, fy + 19), (fx + bw + 2, fy + 19)], C["upd_s"], width=1.2, dashed=True))
    # no branch -> rollback
    o.append(arrow([(dcx, dy + 24), (dcx, dy + 50)], C["upd_s"], width=1.4))
    o.append(m(dcx + 10, dy + 42, L["no"], fs=17, anchor="start", color=C["upd_s"], math=False))
    o.append(m(dcx, dy + 74, L["rb"][0] + " " + L["rb"][1], fs=19))
    o.append(m(xs, y0 + 532, r"P^{+} = (P_{l}^{−1} + H_{v}/σ^{2})^{−1}", fs=22, weight="normal"))
    return "".join(o)


def build(L):
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
         defs(), f'<rect width="{W}" height="{H}" fill="#FFFFFF"/>']
    o.append(panel_a(L))
    o.append(f'<line x1="20" y1="458" x2="{W-20}" y2="458" stroke="{C["grey"]}" stroke-width="1.2"/>')
    o.append(f'<line x1="532" y1="478" x2="532" y2="{H-12}" stroke="{C["grey"]}" stroke-width="1.2"/>')
    o.append(f'<line x1="1060" y1="478" x2="1060" y2="{H-12}" stroke="{C["grey"]}" stroke-width="1.2"/>')
    o.append(panel_b(L))
    o.append(panel_c(L))
    o.append(panel_d(L))
    o.append("</svg>")
    return "".join(o)


def render(svg_path, stem):
    import subprocess
    try:
        subprocess.run(["node", os.path.join(HERE, "render_svg.js"), svg_path], check=True)
    except Exception as exc:  # noqa: BLE001
        print("PDF/PNG rendering skipped:", exc)


if __name__ == "__main__":
    for L, name in [(EN, "sphere_livo_pipeline_en"), (ZH, "sphere_livo_pipeline_zh")]:
        stem = os.path.join(HERE, name)
        with open(stem + ".svg", "w", encoding="utf-8") as f:
            f.write(build(L))
        render(stem + ".svg", stem)
        print("wrote", stem + ".{svg,pdf,png}")
