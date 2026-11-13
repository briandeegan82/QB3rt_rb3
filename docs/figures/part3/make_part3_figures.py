#!/usr/bin/env python3
"""Generate figures for the Part 3 guide (YOLOE detection -> map-frame point).

Outputs (in this directory):
  part3_pipeline.png / .svg  -- dataflow diagram
  part3_geometry.png / .svg  -- top-down camera/lidar geometry schematic

Run:  python3 make_part3_figures.py
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Arc, Polygon

# Okabe-Ito palette
BLUE = "#0072B2"
ORANGE = "#E69F00"
GREEN = "#009E73"
VERM = "#D55E00"
GREY = "#666666"
LGREY = "#BBBBBB"

plt.rcParams.update({
    "font.size": 9.5,
    "font.family": "sans-serif",
    "mathtext.default": "regular",
})

HERE = __file__.rsplit("/", 1)[0]


def box(ax, x, y, w, h, text, fc="white", ec=GREY, tc="black", fs=9.0, mono=True):
    """Draw a rounded box centred at (x, y) with text."""
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                              boxstyle="round,pad=0.02,rounding_size=0.06",
                              fc=fc, ec=ec, lw=1.2, zorder=3))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs,
            family="monospace" if mono else "sans-serif", color=tc,
            linespacing=1.25, zorder=4)


def arrow(ax, p0, p1, color=GREY, lw=1.4, style="-", shrink=0.0):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=11,
                                 lw=lw, color=color, linestyle=style,
                                 shrinkA=2, shrinkB=2, zorder=2))


def elbow_arrow(ax, pts, color=GREY, lw=1.4):
    """Draw a multi-segment orthogonal path ending in an arrowhead."""
    ax.plot([p[0] for p in pts[:-1]], [p[1] for p in pts[:-1]],
            color=color, lw=lw, solid_capstyle="round",
            solid_joinstyle="round", zorder=2)
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle="-|>",
                                 mutation_scale=11, lw=lw, color=color,
                                 shrinkA=0, shrinkB=2, zorder=2))


def fig_pipeline():
    fig, ax = plt.subplots(figsize=(8.6, 4.0))
    ax.set_xlim(0.0, 13.2)
    ax.set_ylim(0.2, 5.95)
    ax.axis("off")

    # --- camera branch (top row) ---
    box(ax, 1.15, 4.85, 1.95, 0.85, "IMX577\n1920x1080", ec=BLUE, fc="#EAF3FA")
    box(ax, 3.60, 4.85, 1.70, 0.85, "resize\n640x640", ec=BLUE, fc="#EAF3FA")
    box(ax, 6.15, 4.85, 2.25, 0.85, "YOLOE\n(NPU, ~14 Hz)", ec=BLUE, fc="#EAF3FA")
    box(ax, 6.15, 3.55, 2.75, 0.72, "/yoloe/detections", ec=BLUE, fc="white")
    arrow(ax, (2.15, 4.85), (2.73, 4.85), BLUE)
    arrow(ax, (4.47, 4.85), (5.00, 4.85), BLUE)
    arrow(ax, (6.15, 4.40), (6.15, 3.93), BLUE)

    # --- lidar branch (middle) ---
    box(ax, 1.15, 2.25, 1.75, 0.80, "/scan", ec=ORANGE, fc="#FDF4E3")
    box(ax, 4.50, 2.25, 3.55, 0.85, "nearest scan,\nshifted to image time", ec=ORANGE, fc="#FDF4E3")
    arrow(ax, (2.05, 2.25), (2.70, 2.25), ORANGE)

    # --- tf branch (bottom) ---
    box(ax, 1.15, 0.85, 1.75, 0.80, "/tf", ec=GREEN, fc="#E9F5F1")
    box(ax, 4.50, 0.85, 3.55, 0.85, "camera, laser\n& map transforms", ec=GREEN, fc="#E9F5F1")
    arrow(ax, (2.05, 0.85), (2.70, 0.85), GREEN)

    # --- localiser ---
    box(ax, 8.85, 2.25, 1.85, 1.00, "localiser", ec=VERM, fc="#FBECE7", fs=10.5)
    elbow_arrow(ax, [(7.55, 3.55), (8.85, 3.55), (8.85, 2.77)], BLUE)
    arrow(ax, (6.30, 2.25), (7.90, 2.25), ORANGE)
    elbow_arrow(ax, [(6.30, 0.85), (8.85, 0.85), (8.85, 1.73)], GREEN)

    # --- output ---
    box(ax, 11.75, 2.25, 2.45, 0.85, "/mission/\nobject_location", ec=VERM, fc="white", fs=8.8)
    arrow(ax, (9.80, 2.25), (10.50, 2.25), VERM)

    ax.text(0.2, 5.62, "YOLOE detection  +  lidar range  +  TF   ->   map-frame point",
            fontsize=10.5, color="black", ha="left", va="center")

    fig.tight_layout()
    fig.savefig(f"{HERE}/part3_pipeline.png", dpi=320, bbox_inches="tight")
    fig.savefig(f"{HERE}/part3_pipeline.svg", bbox_inches="tight")
    plt.close(fig)


def fig_geometry():
    fig, ax = plt.subplots(figsize=(7.2, 5.0))
    ax.set_aspect("equal")
    ax.set_xlim(-0.95, 2.15)
    ax.set_ylim(-0.88, 1.12)
    ax.axis("off")

    # --- robot footprint (top-down, x forward = +x axis to the right) ---
    body = Polygon([(-0.15, -0.14), (0.18, -0.14), (0.22, -0.10), (0.22, 0.10),
                    (0.18, 0.14), (-0.15, 0.14)],
                   closed=True, fc="#F2F2F2", ec=GREY, lw=1.2, zorder=2)
    ax.add_patch(body)
    ax.text(-0.20, -0.28, "robot\n(base_link)", fontsize=8.5, color=GREY,
            ha="center", va="top")

    # base_link origin
    ax.plot(0, 0, "o", ms=5, color="black", zorder=5)
    ax.text(0.05, 0.17, "base_link", fontsize=8.5, ha="left", va="bottom")

    # body axes (x fwd, y left)
    arrow(ax, (0, 0), (0.32, 0), "black", lw=1.1)
    arrow(ax, (0, 0), (0, 0.28), "black", lw=1.1)
    ax.text(0.34, 0.02, "x", fontsize=9, va="bottom")
    ax.text(0.0, 0.31, "y", fontsize=9, ha="center")

    # --- camera: ~0.07 m forward, slightly offset for clarity ---
    cam = np.array([0.07, 0.0])
    ax.add_patch(Circle(cam, 0.035, fc=BLUE, ec="none", zorder=5))
    ax.annotate("camera\n(~7 cm fwd)", xy=(cam[0], cam[1] - 0.035),
                xytext=(0.24, -0.30), fontsize=8.5, color=BLUE,
                ha="center", va="top",
                arrowprops=dict(arrowstyle="-", color=BLUE, lw=0.8))

    # --- lidar on centreline ---
    lid = np.array([0.0, 0.0])
    ax.add_patch(Circle(lid, 0.05, fc="none", ec=ORANGE, lw=1.6, zorder=5))
    ax.add_patch(Circle(lid, 0.012, fc=ORANGE, ec="none", zorder=5))
    ax.annotate("lidar\n(centreline)", xy=(-0.035, 0.035),
                xytext=(-0.38, 0.26), fontsize=8.5, color=ORANGE,
                ha="center", va="bottom",
                arrowprops=dict(arrowstyle="-", color=ORANGE, lw=0.8))

    # --- cone ~1.5 m ahead, slightly to the right (y<0 in robot frame) ---
    cone = np.array([1.55, -0.22])
    r_cone = 0.10
    ax.add_patch(Circle(cone, r_cone, fc="#FBECE7", ec=VERM, lw=1.5, zorder=4))
    ax.text(cone[0] + 0.14, cone[1] + 0.12, "traffic cone\n(target)",
            fontsize=8.5, color=VERM, ha="left", va="bottom")

    # camera bearing ray to cone centre
    arrow(ax, cam, cone, BLUE, lw=1.5)
    ax.text(0.88, -0.20, "camera bearing\n(from pixel ray)", fontsize=8.5,
            color=BLUE, ha="center", va="top")

    # camera forward axis (dashed) for bearing angle
    ax.plot([cam[0], 1.35], [cam[1], cam[1]], ls="--", lw=0.9, color=BLUE, alpha=0.6)
    arc_r = 0.58
    ang = np.degrees(np.arctan2(cone[1] - cam[1], cone[0] - cam[0]))
    ax.add_patch(Arc(cam, 2 * arc_r, 2 * arc_r, theta1=ang, theta2=0,
                     color=BLUE, lw=1.1))
    ax.text(cam[0] + arc_r + 0.04, cam[1] - 0.035, r"$\theta_{cam}$",
            fontsize=9.5, color=BLUE, ha="left", va="center")

    # lidar ray to near face of cone + scan points
    near = cone + r_cone * (lid - cone) / np.linalg.norm(lid - cone)
    arrow(ax, lid, near, ORANGE, lw=1.5)
    ax.text(0.72, 0.06, "lidar range\nalong bearing", fontsize=8.5,
            color=ORANGE, ha="center", va="bottom")

    # scattered scan points on near face
    face_dir = np.arctan2(lid[1] - cone[1], lid[0] - cone[0])
    for a in np.linspace(face_dir - 0.9, face_dir + 0.9, 7):
        p = cone + r_cone * np.array([np.cos(a), np.sin(a)])
        ax.plot(*p, "o", ms=3.5, color=ORANGE, zorder=6)

    # parallax: angle between camera ray and lidar ray at cone
    v_cam = cam - cone
    v_lid = lid - cone
    a_cam = np.degrees(np.arctan2(v_cam[1], v_cam[0]))
    a_lid = np.degrees(np.arctan2(v_lid[1], v_lid[0]))
    ax.add_patch(Arc(cone, 0.5, 0.5, theta1=a_cam, theta2=a_lid,
                     color=VERM, lw=1.2))
    mid = np.radians((a_cam + a_lid) / 2)
    ax.annotate("parallax:\ncamera & lidar rays\nstart ~7 cm apart",
                xy=cone + 0.26 * np.array([np.cos(mid), np.sin(mid)]),
                xytext=(1.50, -0.64), fontsize=8.5, color=VERM,
                ha="center", va="center",
                arrowprops=dict(arrowstyle="-", color=VERM, lw=0.9))

    # --- map frame axes (top-left, schematic) ---
    mo = np.array([-0.75, 0.72])
    arrow(ax, mo, mo + np.array([0.3, 0]), GREEN, lw=1.3)
    arrow(ax, mo, mo + np.array([0, 0.25]), GREEN, lw=1.3)
    ax.text(mo[0] + 0.32, mo[1], "x_map", fontsize=8.5, color=GREEN, va="center")
    ax.text(mo[0], mo[1] + 0.28, "y_map", fontsize=8.5, color=GREEN, ha="center")
    ax.text(mo[0] - 0.02, mo[1] - 0.08, "map", fontsize=9, color=GREEN,
            ha="right", va="top")
    ax.annotate("", xy=(0.55, 0.68), xytext=(1.05, 0.32),
                arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=1.1,
                                linestyle=":"))
    ax.text(0.95, 0.58, "TF puts the point\nin the map frame", fontsize=8.5,
            color=GREEN, ha="left", va="center")

    # --- lidar_yaw = pi note ---
    ax.add_patch(Arc(lid, 0.42, 0.42, theta1=180, theta2=290,
                     color=ORANGE, lw=1.0, linestyle=":"))
    arrow(ax, lid + 0.21 * np.array([np.cos(np.radians(290)),
                                     np.sin(np.radians(290))]),
          lid + 0.26 * np.array([np.cos(np.radians(300)),
                                 np.sin(np.radians(300))]),
          ORANGE, lw=1.0, style=":")
    ax.text(-0.48, -0.64,
            r"lidar_yaw = $\pi$: scan angle 0 points" "\n"
            r"backwards ($-x$); TF already includes this",
            fontsize=8.5, color=ORANGE, ha="center", va="center")
    # backwards zero-angle indicator
    ax.plot([lid[0], lid[0] - 0.42], [lid[1], lid[1]], ls=":", lw=1.2, color=ORANGE)
    ax.plot(lid[0] - 0.42, lid[1], ">", ms=5, color=ORANGE)
    ax.text(lid[0] - 0.44, lid[1] + 0.05, "scan 0", fontsize=8, color=ORANGE,
            ha="right", va="bottom")

    fig.tight_layout()
    fig.savefig(f"{HERE}/part3_geometry.png", dpi=320, bbox_inches="tight")
    fig.savefig(f"{HERE}/part3_geometry.svg", bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    fig_pipeline()
    fig_geometry()
    print("wrote part3_pipeline.{png,svg} and part3_geometry.{png,svg}")
