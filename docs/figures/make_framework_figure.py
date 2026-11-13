#!/usr/bin/env python3
"""SPHERE-LIVO framework figure (EN / ZH), journal style.

Writes sphere_livo_framework_{en,zh}.svg and, via render_svg.js
(Node Playwright), the matching .pdf (vector) and .png.
"""
import math
import os
import random
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 1900, 640

COL = {
    "imu": "#F9D4B8", "lidar": "#62C3EA", "cam": "#F49A8C", "new": "#A4D89E",
    "grp": "#E8E8E8", "imu_a": "#EFA675", "lidar_a": "#1C9AD6", "cam_a": "#E8705F",
    "new_a": "#3E9C47", "ink": "#111111", "plane": "#F6C2B6", "plane_s": "#C9533F",
}
FONT_EN = "'Liberation Serif','Times New Roman',Times,serif"
FONT_ZH = "'Liberation Serif','Times New Roman','Noto Serif CJK SC',serif"

EN = dict(
    font=FONT_EN,
    imu=["IMU"], lidar=["LiDAR", "10 Hz"], cam=["Fisheye", "Cameras ×N"],
    esikf="ESIKF", vis="Spherical Visual Measurement",
    fwd=["Forward", "Propagation"], deskew=["Scan", "Deskew"], p2p=["Point-to-plane", "Residual"],
    lupd=["LiDAR", "Update"], jupd=["Guarded Joint", "Photometric Update"],
    sync=["Rig", "Synchronization"], reproj=["Spherical Patch", "Reprojection"],
    zncc=["ZNCC", "Gating"], sel=["Posterior-Information", "Patch Selection"],
    retr=["Patch Retrieval", "(Temporal + LiDAR)"],
    map_title="Shared Patch Map", local=["Local", "Mapping"],
    retrieve="Retrieve", insert="Update", odom="Odometry", cov="Covariance",
    images="Images",
    lg=["LiDAR Point", "LiDAR Plane", "Spherical Patch", "Camera Ray", "Proposed Module"],
)
ZH = dict(
    font=FONT_ZH,
    imu=["IMU"], lidar=["LiDAR", "10 Hz"], cam=["鱼眼相机", "×N"],
    esikf="ESIKF", vis="球面视觉测量",
    fwd=["前向传播"], deskew=["点云去畸变"], p2p=["点到平面", "残差计算"],
    lupd=["LiDAR 更新"], jupd=["带保护的联合", "光度更新"],
    sync=["多相机同步"], reproj=["球面图块", "重投影"],
    zncc=["ZNCC 筛选"], sel=["后验信息", "图块选择"],
    retr=["图块检索", "（时序 + LiDAR）"],
    map_title="共享图块地图", local=["局部建图"],
    retrieve="检索", insert="更新", odom="里程计", cov="协方差",
    images="图像",
    lg=["LiDAR 点", "LiDAR 平面", "球面图块", "相机射线", "本文模块"],
)


def text(x, y, lines, L, fs=24, anchor="middle", color=None, weight="bold", lh=1.12):
    color = color or COL["ink"]
    if isinstance(lines, str):
        lines = [lines]
    y0 = y - (len(lines) - 1) * fs * lh / 2 + fs * 0.34
    out = []
    for k, s in enumerate(lines):
        s = s.replace("&", "&amp;").replace("<", "&lt;")
        out.append(f'<text x="{x:.1f}" y="{y0 + k*fs*lh:.1f}" font-family="{L["font"]}" font-size="{fs}" '
                   f'font-weight="{weight}" fill="{color}" text-anchor="{anchor}">{s}</text>')
    return "".join(out)


def box(x, y, w, h, fill, lines, L, fs=24, dashed=False):
    d = ' stroke-dasharray="9 6"' if dashed else ""
    return (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="14" fill="{COL[fill]}" '
            f'stroke="{COL["ink"]}" stroke-width="3"{d}/>' + text(x + w / 2, y + h / 2, lines, L, fs))


def group(x, y, w, h):
    return (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{COL["grp"]}" stroke="{COL["ink"]}" '
            f'stroke-width="2" stroke-dasharray="8 5"/>')


def arrow(pts, col, dashed=False, w=3.2):
    d = "M" + " L".join(f"{a:.1f},{b:.1f}" for a, b in pts)
    ds = ' stroke-dasharray="8 6"' if dashed else ""
    return (f'<path d="{d}" fill="none" stroke="{COL[col]}" stroke-width="{w}" stroke-linejoin="round"{ds} '
            f'marker-end="url(#m_{col})"/>')


def defs():
    out = ["<defs>"]
    for c in ("imu_a", "lidar_a", "cam_a", "new_a", "ink"):
        out.append(f'<marker id="m_{c}" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="4.2" '
                   f'markerHeight="4.2" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="{COL[c]}"/></marker>')
    out.append("</defs>")
    return "".join(out)


def map_panel(L, x0=1310, y0=12, w=578, h=616):
    o = [group(x0, y0, w, h)]
    o.append(f'<rect x="{x0+22}" y="{y0+20}" width="300" height="58" fill="#B5B5B5" stroke="{COL["ink"]}" stroke-width="3"/>')
    o.append(text(x0 + 172, y0 + 49, L["map_title"], L, fs=30))
    o.append(text(x0 + 470, y0 + 50, L["local"], L, fs=24))
    # LiDAR plane
    P = [(x0 + 70, y0 + 330), (x0 + 380, y0 + 240), (x0 + 545, y0 + 380), (x0 + 235, y0 + 470)]
    o.append('<polygon points="' + " ".join(f"{a},{b}" for a, b in P) +
             f'" fill="{COL["plane"]}" stroke="{COL["plane_s"]}" stroke-width="2.5"/>')

    def bil(u, v):
        return tuple(P[0][i]*(1-u)*(1-v) + P[1][i]*u*(1-v) + P[2][i]*u*v + P[3][i]*(1-u)*v for i in (0, 1))
    rnd = random.Random(3)
    for _ in range(26):
        u, v = rnd.uniform(.06, .94), rnd.uniform(.08, .92)
        px, py = bil(u, v)
        o.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="4.6" fill="{COL["ink"]}"/>')
    patches = [(.18, .30), (.50, .52), (.74, .26)]
    centers = []
    for (u, v) in patches:
        s = .16
        q = [bil(u, v), bil(u + s, v), bil(u + s, v + s), bil(u, v + s)]
        o.append('<polygon points="' + " ".join(f"{a:.1f},{b:.1f}" for a, b in q) +
                 f'" fill="{COL["new"]}" stroke="{COL["new_a"]}" stroke-width="2.2"/>')
        for t in (1/3, 2/3):
            a, b = bil(u + s*t, v), bil(u + s*t, v + s)
            o.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{COL["new_a"]}" stroke-width="1.2"/>')
            a, b = bil(u, v + s*t), bil(u + s, v + s*t)
            o.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{COL["new_a"]}" stroke-width="1.2"/>')
        c = bil(u + s/2, v + s/2)
        centers.append(c)
        o.append(f'<path d="M{c[0]:.1f},{c[1]:.1f} L{c[0]+6:.1f},{c[1]-46:.1f}" stroke="{COL["lidar_a"]}" '
                 f'stroke-width="3" marker-end="url(#m_lidar_a)"/>')
    # fisheye rig: three cameras around a common body
    rx, ry = x0 + 150, y0 + 165
    o.append(f'<circle cx="{rx}" cy="{ry}" r="30" fill="#D9D9D9" stroke="{COL["ink"]}" stroke-width="2.5"/>')
    cams = []
    for ang in (-25, 95, 215):
        a = math.radians(ang)
        cx, cy = rx + 48*math.cos(a), ry + 48*math.sin(a)
        cams.append((cx, cy))
        o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="16" fill="{COL["cam"]}" stroke="{COL["ink"]}" stroke-width="2.5"/>')
        o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="7" fill="#3A3A3A"/>')
    # rays: two cameras observe the same patch, a third observes another
    for (cx, cy), tgt in ((cams[0], centers[1]), (cams[1], centers[1]), (cams[0], centers[2]), (cams[1], centers[0])):
        o.append(f'<line x1="{cx:.1f}" y1="{cy:.1f}" x2="{tgt[0]:.1f}" y2="{tgt[1]:.1f}" stroke="{COL["new_a"]}" '
                 f'stroke-width="2.4" stroke-dasharray="9 6"/>')
    # legend
    lx, ly = x0 + 28, y0 + 512
    items = [
        f'<circle cx="{lx+14}" cy="{ly}" r="6" fill="{COL["ink"]}"/>',
        f'<polygon points="{lx},{ly+44} {lx+22},{ly+36} {lx+30},{ly+50} {lx+8},{ly+58}" fill="{COL["plane"]}" stroke="{COL["plane_s"]}" stroke-width="2"/>',
        f'<rect x="{lx+278}" y="{ly-11}" width="22" height="22" fill="{COL["new"]}" stroke="{COL["new_a"]}" stroke-width="2"/>',
        f'<line x1="{lx+274}" y1="{ly+47}" x2="{lx+306}" y2="{ly+47}" stroke="{COL["new_a"]}" stroke-width="2.6" stroke-dasharray="7 5"/>',
    ]
    o += items
    o.append(text(lx + 42, ly, L["lg"][0], L, fs=22, anchor="start"))
    o.append(text(lx + 42, ly + 47, L["lg"][1], L, fs=22, anchor="start"))
    o.append(text(lx + 318, ly, L["lg"][2], L, fs=22, anchor="start"))
    o.append(text(lx + 318, ly + 47, L["lg"][3], L, fs=22, anchor="start"))
    o.append(f'<rect x="{lx+2}" y="{ly+80}" width="34" height="22" rx="5" fill="{COL["new"]}" stroke="{COL["ink"]}" stroke-width="2.4"/>')
    o.append(text(lx + 48, ly + 91, L["lg"][4], L, fs=22, anchor="start"))
    return "".join(o)


def build(L):
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">', defs(),
         f'<rect width="{W}" height="{H}" fill="#FFFFFF"/>',
         f'<rect x="3" y="3" width="{W-6}" height="{H-6}" rx="10" fill="none" stroke="#9A9A9A" stroke-width="3"/>']
    # groups
    o.append(group(205, 12, 1050, 246))
    o.append(text(1240, 42, L["esikf"], L, fs=28, anchor="end"))
    o.append(group(205, 322, 1050, 306))
    o.append(text(1240, 606, L["vis"], L, fs=28, anchor="end"))
    # sensors
    o.append(box(18, 35, 160, 72, "imu", L["imu"], L, fs=30))
    o.append(box(18, 140, 160, 84, "lidar", L["lidar"], L, fs=28))
    o.append(box(18, 360, 160, 96, "cam", L["cam"], L, fs=26))
    # ESIKF row
    o.append(box(235, 35, 200, 72, "imu", L["fwd"], L))
    o.append(box(235, 145, 200, 76, "lidar", L["deskew"], L))
    o.append(box(490, 145, 215, 76, "lidar", L["p2p"], L))
    o.append(box(760, 145, 180, 76, "lidar", L["lupd"], L))
    o.append(box(990, 132, 245, 102, "new", L["jupd"], L))
    o.append(arrow([(178, 71), (231, 71)], "imu_a"))
    o.append(arrow([(178, 182), (231, 182)], "lidar_a"))
    o.append(arrow([(335, 107), (335, 141)], "imu_a"))
    o.append(arrow([(435, 71), (850, 71), (850, 141)], "imu_a"))
    o.append(arrow([(435, 183), (486, 183)], "lidar_a"))
    o.append(arrow([(705, 183), (756, 183)], "lidar_a"))
    o.append(arrow([(940, 183), (986, 183)], "lidar_a"))
    # map retrieve for point-to-plane
    o.append(arrow([(1310, 108), (598, 108), (598, 141)], "lidar_a"))
    o.append(text(1190, 96, L["retrieve"], L, fs=21, anchor="end"))
    # visual row (y=350..)
    yv, hv = 368, 86
    o.append(box(235, yv, 200, hv, "cam", L["sync"], L))
    o.append(box(490, yv, 215, hv, "new", L["reproj"], L))
    o.append(box(760, yv, 180, hv, "cam", L["zncc"], L))
    o.append(box(990, yv, 245, hv, "new", L["sel"], L, fs=22.5))
    o.append(arrow([(178, yv + 43), (231, yv + 43)], "cam_a"))
    o.append(arrow([(435, yv + 43), (486, yv + 43)], "cam_a"))
    o.append(arrow([(705, yv + 43), (756, yv + 43)], "cam_a"))
    o.append(arrow([(940, yv + 43), (986, yv + 43)], "cam_a"))
    o.append(arrow([(1112, yv), (1112, 238)], "new_a"))
    # LIO covariance -> selection
    o.append(arrow([(905, 221), (905, 290), (1030, 290), (1030, yv - 4)], "lidar_a", dashed=True, w=2.8))
    o.append(text(893, 290, L["cov"], L, fs=21, anchor="end"))
    # retrieval from the shared map
    o.append(box(490, 520, 215, 84, "new", L["retr"], L, fs=22))
    o.append(arrow([(1310, 562), (709, 562)], "new_a"))
    o.append(text(1190, 550, L["retrieve"], L, fs=21, anchor="end"))
    o.append(arrow([(597, 520), (597, yv + hv + 4)], "new_a"))
    # outputs of the joint update
    o.append(arrow([(1235, 165), (1306, 165)], "new_a"))
    o.append(text(1272, 145, L["insert"], L, fs=21))
    o.append(arrow([(1195, 234), (1195, 300)], "ink", w=3))
    o.append(text(1207, 296, L["odom"], L, fs=21, anchor="start"))
    o.append(map_panel(L))
    o.append("</svg>")
    return "".join(o)


if __name__ == "__main__":
    for L, name in [(EN, "sphere_livo_framework_en"), (ZH, "sphere_livo_framework_zh")]:
        svg = os.path.join(HERE, name + ".svg")
        open(svg, "w", encoding="utf-8").write(build(L))
        subprocess.run(["node", os.path.join(HERE, "render_svg.js"), svg], check=True)
