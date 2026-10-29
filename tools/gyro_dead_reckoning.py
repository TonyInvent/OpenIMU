#!/usr/bin/env python3
"""Pure-gyro dead-reckoning demo for the ICM-45686 (CAN FD, ID 0x01).

Same quaternion integration as the BeijingDynamics SixAxis filter, but with the
accelerometer correction REMOVED:

  1. 10 s static calibration: estimate the gyro bias, and take roll/pitch from
     gravity (yaw = 0).
  2. Attitude is then integrated purely from the gyro (no accelerometer).
  3. The attitude rotates the accelerometer into the world frame, gravity is
     subtracted, and the result is integrated ONCE to get velocity.

This shows how far open-loop dead reckoning gets with the ICM-45686 alone.

Usage:
    python tools/gyro_dead_reckoning.py -i pcan -c PCAN_USBBUS1
    python tools/gyro_dead_reckoning.py -i socketcan -c can0
    python tools/gyro_dead_reckoning.py --simulate
"""

import argparse
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ahrs_demo import (  # noqa: E402
    GRAVITY, CanReader, open_bus,
    quat_from_accel, quat_integrate, quat_to_euler, quat_to_rot,
)

DEG = 180.0 / math.pi


# ---------------------------------------------------------------------------
# Pure-gyro attitude + velocity integrator
# ---------------------------------------------------------------------------
class GyroOnly:
    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)

    def initialize(self, accel):
        """Roll/pitch from gravity, yaw = 0."""
        self.q = quat_from_accel(accel)

    def update(self, gyro, dt):
        self.q = quat_integrate(self.q, gyro - self.bias, dt)
        return self.q


# ---------------------------------------------------------------------------
# Synthetic source (for testing without hardware)
# ---------------------------------------------------------------------------
class SimSource(threading.Thread):
    """Static during calibration, then a tumble + a small linear acceleration."""

    def __init__(self, static_seconds=10.0, maxlen=20000):
        super().__init__(daemon=True)
        self.samples = deque(maxlen=maxlen)
        self.running = True
        self.error = None
        self.seq = 0
        self.static_until = time.perf_counter() + static_seconds

    def run(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        omega = np.array([0.15, 0.08, 0.22])          # rad/s tumble
        a_lin = np.array([0.40, 0.20, 0.0])           # m/s^2, world frame
        last = time.perf_counter()
        while self.running:
            now = time.perf_counter()
            dt = now - last
            last = now
            if dt <= 0.0 or dt > 0.05:
                dt = 1.0 / 800.0
            if now < self.static_until:
                w = np.zeros(3)
                a_w = np.zeros(3)
            else:
                w = omega
                a_w = a_lin * math.cos(2.0 * math.pi * 0.2 * (now - self.static_until))
            q = quat_integrate(q, w, dt)
            # f = a - g  (specific force), body frame
            f_body = quat_to_rot(q).T @ (a_w + np.array([0.0, 0.0, GRAVITY]))
            self.seq += 1
            self.samples.append((now, self.seq, f_body, w.copy(), 25.0))
            time.sleep(1.0 / 800.0)

    def stop(self):
        self.running = False


# ---------------------------------------------------------------------------
# Live plotting
# ---------------------------------------------------------------------------
class LivePlot:
    def __init__(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        self.plt = plt
        plt.ion()
        self.fig = plt.figure("open_imu gyro dead reckoning", figsize=(14, 8))
        gs = self.fig.add_gridspec(2, 3)
        self.ax3d = self.fig.add_subplot(gs[0, 0], projection="3d")
        self.ax_acc = self.fig.add_subplot(gs[0, 1])
        self.ax_vel = self.fig.add_subplot(gs[0, 2])
        self.ax_xy = self.fig.add_subplot(gs[1, 0])
        self.ax_pos = self.fig.add_subplot(gs[1, 1])
        self.ax_txt = self.fig.add_subplot(gs[1, 2])

        self.ax3d.set_xlim(-1, 1); self.ax3d.set_ylim(-1, 1); self.ax3d.set_zlim(-1, 1)
        self.ax3d.set_xlabel("X"); self.ax3d.set_ylabel("Y"); self.ax3d.set_zlabel("Z")
        self.ax3d.set_title("attitude (gyro only)")
        for axis, vec in enumerate(np.eye(3)):
            self.ax3d.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], color="0.7", lw=1, ls=":")
        self.body = []
        for color in ("#d62728", "#2ca02c", "#1f77b4"):
            ln, = self.ax3d.plot([], [], [], color=color, lw=2.5)
            self.body.append(ln)

        self.ax_acc.set_title("motion acceleration (world, g removed)")
        self.ax_acc.set_ylabel("m/s^2")
        self.ax_acc.grid(True, alpha=0.3)
        self.acc_lines = []
        for color, label in zip(("#1f77b4", "#ff7f0e", "#2ca02c", "0.3"),
                                ("ax", "ay", "az", "|a|")):
            ln, = self.ax_acc.plot([], [], color=color, lw=1.2, label=label)
            self.acc_lines.append(ln)
        self.ax_acc.legend(loc="upper right", fontsize=7)

        self.ax_vel.set_title("velocity = integral(motion acceleration)")
        self.ax_vel.set_xlabel("time [s]")
        self.ax_vel.set_ylabel("m/s")
        self.ax_vel.grid(True, alpha=0.3)
        self.vel_lines = []
        for color, label in zip(("#1f77b4", "#ff7f0e", "#2ca02c", "0.3"),
                                ("vx", "vy", "vz", "|v|")):
            ln, = self.ax_vel.plot([], [], color=color, lw=1.2, label=label)
            self.vel_lines.append(ln)
        self.ax_vel.legend(loc="upper left", fontsize=7)

        self.ax_xy.set_title("position X-Y [m] (drift path)")
        self.ax_xy.set_xlabel("X [m]")
        self.ax_xy.set_ylabel("Y [m]")
        self.ax_xy.grid(True, alpha=0.3)
        self.traj_line, = self.ax_xy.plot([], [], color="#1f77b4", lw=1.2)
        self.traj_start, = self.ax_xy.plot([], [], marker="o", color="#2ca02c", ms=5)

        self.ax_pos.set_title("position = integral(velocity)")
        self.ax_pos.set_xlabel("time [s]")
        self.ax_pos.set_ylabel("m")
        self.ax_pos.grid(True, alpha=0.3)
        self.pos_lines = []
        for color, label in zip(("#1f77b4", "#ff7f0e", "#2ca02c", "0.3"),
                                ("x", "y", "z", "|p|")):
            ln, = self.ax_pos.plot([], [], color=color, lw=1.2, label=label)
            self.pos_lines.append(ln)
        self.ax_pos.legend(loc="upper left", fontsize=7)

        self.ax_txt.axis("off")
        self.txt = self.ax_txt.text(0.02, 0.98, "", va="top", family="monospace", fontsize=9)

    def update(self, hist_t, hist_acc, hist_vel, hist_pos, q, info):
        t = np.asarray(hist_t)
        if len(t) == 0:
            return
        t = t - t[0]

        rot = quat_to_rot(q)
        for i, ln in enumerate(self.body):
            ln.set_data_3d([0, rot[0, i]], [0, rot[1, i]], [0, rot[2, i]])

        for i, key in enumerate(("ax", "ay", "az", "mag")):
            self.acc_lines[i].set_data(t, np.asarray(hist_acc[key]))
        for i, key in enumerate(("vx", "vy", "vz", "mag")):
            self.vel_lines[i].set_data(t, np.asarray(hist_vel[key]))
        for i, key in enumerate(("x", "y", "z", "mag")):
            self.pos_lines[i].set_data(t, np.asarray(hist_pos[key]))

        px = np.asarray(hist_pos["x"])
        py = np.asarray(hist_pos["y"])
        self.traj_line.set_data(px, py)
        if len(px):
            self.traj_start.set_data([px[0]], [py[0]])

        for ax in (self.ax_acc, self.ax_vel, self.ax_pos, self.ax_xy):
            ax.relim()
            ax.autoscale_view()

        self.txt.set_text(info)
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def alive(self):
        return self.plt.fignum_exists(self.fig.number)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="Pure-gyro attitude + velocity dead reckoning",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("-i", "--interface", default="auto", choices=("auto", "socketcan", "pcan"))
    p.add_argument("-c", "--channel", default=None)
    p.add_argument("--f-clock", type=int, default=80_000_000, help="PCAN FD clock [Hz]")
    p.add_argument("--calib-seconds", type=float, default=10.0,
                   help="static window for the gyro bias")
    p.add_argument("--simulate", action="store_true", help="synthetic source")
    p.add_argument("--duration", type=float, default=0.0, help="stop after N s (0 = forever)")
    p.add_argument("--history", type=float, default=60.0, help="plot history [s]")
    p.add_argument("--no-plot", action="store_true", help="console output only")
    return p.parse_args()


def main():
    args = parse_args()

    bus = None
    if args.simulate:
        reader = SimSource(static_seconds=args.calib_seconds)
        print(f"simulating: static for {args.calib_seconds:.0f} s, then moving")
    else:
        try:
            bus = open_bus(args)
        except Exception as exc:
            sys.exit(f"failed to open CAN bus: {exc}")
        reader = CanReader(bus)
    reader.start()
    if not args.simulate:
        print("waiting for CAN FD frames (ID 0x01) ...")

    # ---- static calibration ------------------------------------------------
    t_cal = time.perf_counter()
    calib = []
    print(f"keep the board still for {args.calib_seconds:.0f} s (gyro bias calibration) ...")
    while time.perf_counter() - t_cal < args.calib_seconds and reader.running:
        if reader.samples:
            calib.append(reader.samples.popleft())
        else:
            time.sleep(0.002)
    if reader.error:
        sys.exit(f"CAN read error: {reader.error}")
    if len(calib) < 100:
        sys.exit("no/too few CAN frames - check the bus and the interface setup")

    accels = np.asarray([c[2] for c in calib])
    gyros = np.asarray([c[3] for c in calib])
    bias = gyros.mean(axis=0)
    accel0 = accels.mean(axis=0)
    gyro_std = gyros.std(axis=0)
    pitch_deg = math.degrees(math.atan2(-accel0[0], math.hypot(accel0[1], accel0[2])))
    roll_deg = math.degrees(math.atan2(accel0[1], accel0[2]))
    print(f"calibration: {len(calib)} frames, |a|={np.linalg.norm(accel0):.3f} m/s^2, "
          f"roll={roll_deg:+.2f} pitch={pitch_deg:+.2f} deg")
    print(f"gyro bias = {np.degrees(bias)} deg/s, noise = {np.degrees(gyro_std)} deg/s")
    if abs(np.linalg.norm(accel0) - GRAVITY) > 0.1 * GRAVITY or np.linalg.norm(bias) > 0.1:
        print("WARNING: the board was not still during calibration")

    att = GyroOnly()
    att.bias = bias
    att.initialize(accel0)

    # ---- state -------------------------------------------------------------
    vel = np.zeros(3)
    pos = np.zeros(3)
    hist_t = deque(maxlen=max(100, int(args.history * 1000)))
    hist_acc = {k: deque(maxlen=hist_t.maxlen) for k in ("ax", "ay", "az", "mag")}
    hist_vel = {k: deque(maxlen=hist_t.maxlen) for k in ("vx", "vy", "vz", "mag")}
    hist_pos = {k: deque(maxlen=hist_t.maxlen) for k in ("x", "y", "z", "mag")}

    plot = None if args.no_plot else LivePlot()
    last_t = None
    last_redraw = 0.0
    last_print = 0.0
    t_start = time.perf_counter()
    t_motion = None
    running = True
    last_seq = 0

    try:
        while running:
            processed = 0
            while reader.samples and processed < 400:
                t, seq, accel, gyro, temp = reader.samples.popleft()
                processed += 1
                last_seq = seq
                if last_t is not None:
                    dt = t - last_t
                    if 0.0 < dt < 0.1:
                        if t_motion is None:
                            t_motion = t
                        att.update(gyro, dt)
                        rot = quat_to_rot(att.q)
                        a_w = rot @ accel + np.array([0.0, 0.0, -GRAVITY])
                        vel[:] = vel + a_w * dt
                        pos[:] = pos + vel * dt
                        hist_t.append(t)
                        hist_acc["ax"].append(a_w[0])
                        hist_acc["ay"].append(a_w[1])
                        hist_acc["az"].append(a_w[2])
                        hist_acc["mag"].append(float(np.linalg.norm(a_w)))
                        hist_vel["vx"].append(vel[0])
                        hist_vel["vy"].append(vel[1])
                        hist_vel["vz"].append(vel[2])
                        hist_vel["mag"].append(float(np.linalg.norm(vel)))
                        hist_pos["x"].append(pos[0])
                        hist_pos["y"].append(pos[1])
                        hist_pos["z"].append(pos[2])
                        hist_pos["mag"].append(float(np.linalg.norm(pos)))
                last_t = t

            now = time.perf_counter()
            elapsed = 0.0 if t_motion is None else now - t_motion
            r, p, y = np.degrees(quat_to_euler(att.q))
            v_mag = float(np.linalg.norm(vel))
            p_mag = float(np.linalg.norm(pos))
            drift = v_mag / elapsed if elapsed > 0.5 else 0.0
            info = (
                f"calibration            \n"
                f"  |a|      {np.linalg.norm(accel0):8.3f} m/s^2\n"
                f"  roll     {roll_deg:+8.2f} deg\n"
                f"  pitch    {pitch_deg:+8.2f} deg\n"
                f"  bias     ({np.degrees(bias[0]):+.4f}, {np.degrees(bias[1]):+.4f}, "
                f"{np.degrees(bias[2]):+.4f}) deg/s\n"
                f"  noise    ({np.degrees(gyro_std[0]):.4f}, {np.degrees(gyro_std[1]):.4f}, "
                f"{np.degrees(gyro_std[2]):.4f}) deg/s\n"
                f"\n"
                f"running                \n"
                f"  t        {elapsed:8.2f} s\n"
                f"  r/p/y    {r:+7.2f} {p:+7.2f} {y:+7.2f} deg\n"
                f"  |a|      {hist_acc['mag'][-1] if hist_acc['mag'] else 0:8.4f} m/s^2\n"
                f"  v        ({vel[0]:+.4f}, {vel[1]:+.4f}, {vel[2]:+.4f}) m/s\n"
                f"  |v|      {v_mag:8.4f} m/s\n"
                f"  |v|/t    {drift:8.4f} m/s^2\n"
                f"  p        ({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}) m\n"
                f"  |p|      {p_mag:8.4f} m\n"
            )

            if plot is not None:
                if now - last_redraw > 0.05:
                    plot.update(hist_t, hist_acc, hist_vel, hist_pos, att.q, info)
                    last_redraw = now
                    if not plot.alive():
                        running = False
                if now - last_print > 0.5:
                    print(f"t {elapsed:7.2f} s | v ({vel[0]:+.4f}, {vel[1]:+.4f}, "
                          f"{vel[2]:+.4f}) m/s | |v| {v_mag:.4f} m/s | "
                          f"|p| {p_mag:.4f} m")
                    last_print = now
            elif now - last_print > 0.5:
                print(f"t {elapsed:7.2f} s | r/p/y {r:+7.2f}/{p:+7.2f}/{y:+7.2f} | "
                      f"|a| {hist_acc['mag'][-1] if hist_acc['mag'] else 0:7.4f} m/s^2 | "
                      f"v ({vel[0]:+.4f}, {vel[1]:+.4f}, {vel[2]:+.4f}) | |v| {v_mag:.4f} m/s | "
                      f"|p| {p_mag:.4f} m")
                last_print = now
            else:
                time.sleep(0.002)

            if args.duration > 0.0 and (now - t_start) > args.duration:
                running = False
            if reader.error is not None:
                print(f"CAN read error: {reader.error}")
                running = False
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        reader.join(timeout=1.0)
        if bus is not None:
            bus.shutdown()
        print("bye")


if __name__ == "__main__":
    main()
