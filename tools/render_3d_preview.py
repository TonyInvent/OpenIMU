#!/usr/bin/env python3
"""Render the 3D preview image (2x2 grid) for the READMEs.

Input:  build/stl/*.stl produced by tools/export_3d_stl.py (freecadcmd)
Output: 3D/preview.png

    freecadcmd tools/export_3d_stl.py
    python tools/render_3d_preview.py
"""

import argparse
import math
import os
import struct

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

REC = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])


def load_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=REC)
    return rec["v"].reshape(-1, 3, 3).astype(np.float64)


def draw(ax, path, title, azim=-55.0, elev=24.0, base=(0.60, 0.67, 0.78)):
    tris = load_stl(path)
    az, el = math.radians(azim), math.radians(elev)
    Rz = np.array([[math.cos(az), -math.sin(az), 0.0],
                   [math.sin(az), math.cos(az), 0.0],
                   [0.0, 0.0, 1.0]])
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, math.cos(el), -math.sin(el)],
                   [0.0, math.sin(el), math.cos(el)]])
    R = Rx @ Rz

    v = tris @ R.T
    p2 = v[:, :, :2]
    depth = v[:, :, 2].mean(axis=1)

    n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ln[ln < 1e-12] = 1.0
    n = n / ln[:, None]
    light = np.array([0.35, 0.55, 0.75])
    light /= np.linalg.norm(light)
    shade = np.clip(np.abs(n @ light), 0.0, 1.0)

    colors = np.empty((len(tris), 4))
    for i in range(3):
        colors[:, i] = np.clip(base[i] * (0.28 + 0.85 * shade), 0.0, 1.0)
    colors[:, 3] = 1.0

    order = np.argsort(depth)
    ax.add_collection(PolyCollection(p2[order], facecolors=colors[order],
                                     edgecolors="none", antialiased=False))
    allp = p2.reshape(-1, 2)
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    pad = 0.06 * max(hi - lo)
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, fontsize=10)
    print("  ", title, len(tris), "triangles", flush=True)


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--stl-dir", default=os.path.join(root, "build", "stl"))
    p.add_argument("-o", "--out", default=os.path.join(root, "3D", "preview.png"))
    args = p.parse_args()

    items = [
        ("01-IMU.stl", "01-IMU (assembly)"),
        ("04-IMU-box-A.stl", "04-IMU-box-A"),
        ("04-IMU-box-B.stl", "04-IMU-box-B"),
        ("open_imu_3D_1022B.stl", "open_imu PCB (1022B)"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 9.0), dpi=120)
    for ax, (fn, title) in zip(axes.ravel(), items):
        draw(ax, os.path.join(args.stl_dir, fn), title)
    fig.tight_layout()
    fig.savefig(args.out, bbox_inches="tight", pad_inches=0.05, facecolor="white")
    plt.close(fig)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
