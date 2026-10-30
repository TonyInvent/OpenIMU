#!/usr/bin/env python3
"""
open_imu AHRS demo: ICM-45686 (CAN FD, ID 0x01) -> Mahony / Madgwick / SixAxis.

Reads the 16-byte CAN FD stream produced by the STM32G431 firmware and runs
three attitude estimators on the same samples, so they can be compared live:

  * Mahony    - nonlinear complementary filter (proportional + integral)
  * Madgwick  - gradient-descent AHRS (beta gain, no integral)
  * SixAxis   - BeijingDynamics open_sprite_runtime (native/src/imu_six_axis.hpp):
                nonlinear complementary filter with a startup static gyro bias
                and an accelerometer trust window (no magnetometer, no online
                bias learning)

Live view: 3-D body triads, roll/pitch comparison, and world-frame linear
("motion") acceleration.

CAN backends (python-can):
  Linux   : SocketCAN  (-i socketcan -c can0).  The interface must already be
            configured for CAN FD, see README section 7:
              sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
  Windows : PCAN-Basic (-i pcan -c PCAN_USBBUS1) for a PCAN-USB FD adapter.

Requires: python-can, numpy, matplotlib.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from collections import deque

import numpy as np

try:
    import can
except ImportError:  # pragma: no cover
    sys.exit("python-can is required:  pip install python-can")

# ---------------------------------------------------------------------------
# Frame / sensor constants (must match the firmware)
# ---------------------------------------------------------------------------
CAN_ID = 0x01
GRAVITY = 9.80665            # m/s^2
ACCEL_LSB_PER_G = 8192.0     # +/-4 g
GYRO_LSB_PER_DPS = 16.4      # +/-2000 dps
DEG = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

FILTER_NAMES = ("mahony", "madgwick", "sixaxis")
FILTER_COLORS = {"mahony": "#d62728", "madgwick": "#2ca02c", "sixaxis": "#1f77b4"}


# ---------------------------------------------------------------------------
# Frame decoding
# ---------------------------------------------------------------------------
def _s16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


def decode_frame(data: bytes):
    """16-byte big-endian payload -> (seq, accel[m/s^2], gyro[rad/s], temp[C])."""
    seq = (data[0] << 8) | data[1]
    ax = _s16(data[2], data[3]) / ACCEL_LSB_PER_G * GRAVITY
    ay = _s16(data[4], data[5]) / ACCEL_LSB_PER_G * GRAVITY
    az = _s16(data[6], data[7]) / ACCEL_LSB_PER_G * GRAVITY
    gx = _s16(data[8], data[9]) / GYRO_LSB_PER_DPS * DEG
    gy = _s16(data[10], data[11]) / GYRO_LSB_PER_DPS * DEG
    gz = _s16(data[12], data[13]) / GYRO_LSB_PER_DPS * DEG
    temp = _s16(data[14], data[15]) / 132.48 + 25.0
    return seq, np.array([ax, ay, az]), np.array([gx, gy, gz]), temp


# ---------------------------------------------------------------------------
# Quaternion helpers (q = [w, x, y, z], body -> world)
# ---------------------------------------------------------------------------
def quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler(q: np.ndarray):
    """Roll, pitch, yaw in radians (ZYX / aerospace convention)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def quat_from_accel(a: np.ndarray) -> np.ndarray:
    """Level the attitude from the measured up direction (yaw = 0)."""
    roll = math.atan2(a[1], a[2])
    pitch = math.atan2(-a[0], math.hypot(a[1], a[2]))
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return np.array([cr * cp, sr * cp, cr * sp, -sr * sp])


def quat_integrate(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """First-order quaternion integration over one step."""
    speed = float(np.linalg.norm(omega))
    half = 0.5 * speed * dt
    scale = math.sin(half) / speed if speed > 1e-12 else 0.5 * dt
    wq = math.cos(half)
    x, y, z = scale * omega[0], scale * omega[1], scale * omega[2]
    nq = np.array([
        q[0] * wq - q[1] * x - q[2] * y - q[3] * z,
        q[0] * x + q[1] * wq + q[2] * z - q[3] * y,
        q[0] * y - q[1] * z + q[2] * wq + q[3] * x,
        q[0] * z + q[1] * y - q[2] * x + q[3] * wq,
    ])
    n = float(np.linalg.norm(nq))
    return nq / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    """Angle between two orientations, in degrees."""
    d = min(1.0, abs(float(np.dot(q1, q2))))
    return 2.0 * math.acos(d) * RAD2DEG


# ---------------------------------------------------------------------------
# Attitude estimators
# ---------------------------------------------------------------------------
class Mahony:
    name = "mahony"

    def __init__(self, kp=1.0, ki=0.05):
        self.kp = kp
        self.ki = ki
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)
        self.integral = np.zeros(3)

    def reset(self, accel, bias):
        self.q = quat_from_accel(accel)
        self.bias = np.array(bias, dtype=float)
        self.integral = np.zeros(3)

    def update(self, accel, gyro, dt):
        omega = gyro - self.bias
        mag = float(np.linalg.norm(accel))
        if mag > 1e-6:
            measured = accel / mag
            up = quat_to_rot(self.q).T @ np.array([0.0, 0.0, 1.0])
            err = np.cross(measured, up)
            self.integral += err * dt
            omega = omega + self.kp * err + self.ki * self.integral
        self.q = quat_integrate(self.q, omega, dt)
        return self.q


class Madgwick:
    name = "madgwick"

    def __init__(self, beta=0.1):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)

    def reset(self, accel, bias):
        self.q = quat_from_accel(accel)
        self.bias = np.array(bias, dtype=float)

    def update(self, accel, gyro, dt):
        w, x, y, z = self.q
        gx, gy, gz = gyro - self.bias
        ax, ay, az = accel
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        if mag > 1e-9:
            ax, ay, az = ax / mag, ay / mag, az / mag
            # Objective function f(q) = predicted up - measured up
            f0 = 2 * (x * z - w * y) - ax
            f1 = 2 * (w * x + y * z) - ay
            f2 = 2 * (0.5 - x * x - y * y) - az
            # J^T f
            g0 = -2 * y * f0 + 2 * x * f1
            g1 = 2 * z * f0 + 2 * w * f1 - 4 * x * f2
            g2 = -2 * w * f0 + 2 * z * f1 - 4 * y * f2
            g3 = 2 * x * f0 + 2 * y * f1
            gn = math.sqrt(g0 * g0 + g1 * g1 + g2 * g2 + g3 * g3)
            if gn > 1e-9:
                g0, g1, g2, g3 = g0 / gn, g1 / gn, g2 / gn, g3 / gn
                qd0 = 0.5 * (-x * gx - y * gy - z * gz) - self.beta * g0
                qd1 = 0.5 * (w * gx + y * gz - z * gy) - self.beta * g1
                qd2 = 0.5 * (w * gy - x * gz + z * gx) - self.beta * g2
                qd3 = 0.5 * (w * gz + x * gy - y * gx) - self.beta * g3
                nq = np.array([w + qd0 * dt, x + qd1 * dt, y + qd2 * dt, z + qd3 * dt])
                n = float(np.linalg.norm(nq))
                if n > 1e-12:
                    self.q = nq / n
        return self.q


class SixAxis:
    """Port of open_sprite_runtime native/src/imu_six_axis.hpp (SixAxis)."""

    name = "sixaxis"

    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)
        self.trust = 0.0

    def reset(self, accel, bias):
        roll = math.atan2(accel[1], accel[2])
        pitch = math.atan2(-accel[0], math.hypot(accel[1], accel[2]))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        self.q = np.array([cr * cp, sr * cp, cr * sp, -sr * sp])
        self.bias = np.array(bias, dtype=float)
        self.trust = 0.0

    def _up(self) -> np.ndarray:
        w, x, y, z = self.q
        return np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])

    def update(self, accel, gyro, dt):
        omega = gyro - self.bias
        mag = float(np.linalg.norm(accel))
        # Trust the accelerometer only when |a| is close to 1 g.
        self.trust = min(1.0, max(0.0, (0.25 - abs(mag / GRAVITY - 1.0)) / 0.20))
        if self.trust > 0.0 and mag > 1e-9:
            measured = accel / mag
            err = np.cross(measured, self._up())
            omega = omega + self.trust * err
        self.q = quat_integrate(self.q, omega, dt)
        return self.q


# ---------------------------------------------------------------------------
# CAN reading
# ---------------------------------------------------------------------------
class CanReader(threading.Thread):
    """Background reader: pushes (t, accel, gyro, temp) into a deque."""

    def __init__(self, bus, maxlen=20000, log_path=None):
        super().__init__(daemon=True)
        self.bus = bus
        self.samples = deque(maxlen=maxlen)
        self.running = True
        self.error = None
        self.log_path = log_path
        self._log = None

    def run(self):
        try:
            if self.log_path:
                self._log = open(self.log_path, "w", encoding="utf-8")
                self._log.write("t,seq,ax,ay,az,gx,gy,gz,temp\n")
            while self.running:
                msg = self.bus.recv(timeout=0.2)
                if msg is None or msg.arbitration_id != CAN_ID:
                    continue
                data = bytes(msg.data)
                if len(data) < 16:
                    continue
                t = time.perf_counter()
                seq, accel, gyro, temp = decode_frame(data)
                self.samples.append((t, seq, accel, gyro, temp))
                if self._log is not None:
                    self._log.write(
                        f"{t:.6f},{seq},{accel[0]:.4f},{accel[1]:.4f},{accel[2]:.4f},"
                        f"{gyro[0]:.5f},{gyro[1]:.5f},{gyro[2]:.5f},{temp:.2f}\n"
                    )
        except Exception as exc:  # keep the message for the main thread
            self.error = exc
        finally:
            if self._log is not None:
                self._log.close()

    def stop(self):
        self.running = False


class SimReader(threading.Thread):
    """Synthetic 800 Hz source: static during calibration, then a slow tumble."""

    def __init__(self, static_seconds=2.0, maxlen=20000):
        super().__init__(daemon=True)
        self.samples = deque(maxlen=maxlen)
        self.running = True
        self.error = None
        self.seq = 0
        self.static_until = time.perf_counter() + static_seconds

    def run(self):
        dt = 1.0 / 800.0
        q = np.array([1.0, 0.0, 0.0, 0.0])
        tumble = np.array([0.25, 0.12, 0.30])   # rad/s
        while self.running:
            t0 = time.perf_counter()
            omega = np.zeros(3) if t0 < self.static_until else tumble
            q = quat_integrate(q, omega, dt)
            accel = quat_to_rot(q).T @ np.array([0.0, 0.0, GRAVITY])
            self.seq += 1
            self.samples.append((t0, self.seq, accel, omega.copy(), 25.0))
            time.sleep(dt)

    def stop(self):
        self.running = False


def open_bus(args):
    """Open a CAN FD bus on SocketCAN (Linux) or PCAN-Basic (Windows)."""
    interface = args.interface
    channel = args.channel
    if interface == "auto":
        if sys.platform.startswith("win"):
            interface, channel = "pcan", (channel or "PCAN_USBBUS1")
        else:
            interface, channel = "socketcan", (channel or "can0")

    if interface == "pcan":
        return can.Bus(
            interface="pcan",
            channel=channel,
            fd=True,
            f_clock=args.f_clock,
            nom_brp=1, nom_tseg1=63, nom_tseg2=16, nom_sjw=16,   # 1 Mbit/s, SP 80%
            data_brp=1, data_tseg1=11, data_tseg2=4, data_sjw=4,  # 5 Mbit/s, SP 75%
        )

    # SocketCAN: bitrates are configured outside python-can (`ip link ... fd on`).
    return can.Bus(interface="socketcan", channel=channel, fd=True)


# ---------------------------------------------------------------------------
# Live plotting
# ---------------------------------------------------------------------------
class LivePlot:
    def __init__(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        self.plt = plt
        plt.ion()
        self.fig = plt.figure("open_imu AHRS demo", figsize=(12, 9))
        gs = self.fig.add_gridspec(3, 2, height_ratios=[1.25, 1.0, 1.0])
        self.ax3d = self.fig.add_subplot(gs[0, 0], projection="3d")
        self.ax_acc = self.fig.add_subplot(gs[0, 1])
        self.ax_roll = self.fig.add_subplot(gs[1, 0])
        self.ax_pitch = self.fig.add_subplot(gs[1, 1])
        self.ax_yaw = self.fig.add_subplot(gs[2, :])

        # 3-D attitude: world triad (grey) + body Z/X per filter.
        self.ax3d.set_xlim(-1, 1)
        self.ax3d.set_ylim(-1, 1)
        self.ax3d.set_zlim(-1, 1)
        self.ax3d.set_xlabel("X")
        self.ax3d.set_ylabel("Y")
        self.ax3d.set_zlabel("Z")
        self.ax3d.set_title("attitude")
        for axis, vec in enumerate(np.eye(3)):
            self.ax3d.plot([0, vec[0]], [0, vec[1]], [0, vec[2]],
                           color="0.7", lw=1, ls=":")
        self.body_lines = {}
        for name in FILTER_NAMES:
            color = FILTER_COLORS[name]
            lz, = self.ax3d.plot([], [], [], color=color, lw=2.5, label=name)
            lx, = self.ax3d.plot([], [], [], color=color, lw=1.0, alpha=0.5)
            self.body_lines[name] = (lz, lx)
        self.ax3d.legend(loc="upper right", fontsize=7)

        # Linear acceleration.
        self.ax_acc.set_title("linear acceleration (world)")
        self.ax_acc.set_ylabel("m/s^2")
        self.ax_acc.grid(True, alpha=0.3)
        self.acc_lines = []
        for color, label in zip(("#1f77b4", "#ff7f0e", "#2ca02c", "0.3"),
                                ("ax", "ay", "az", "|a|")):
            ln, = self.ax_acc.plot([], [], color=color, lw=1.2, label=label)
            self.acc_lines.append(ln)
        self.ax_acc.legend(loc="upper right", fontsize=7)

        # Roll / pitch comparison.
        for ax, title in ((self.ax_roll, "roll [deg]"), (self.ax_pitch, "pitch [deg]")):
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
        self.roll_lines = {n: self.ax_roll.plot([], [], color=FILTER_COLORS[n], lw=1.3,
                                                 label=n)[0] for n in FILTER_NAMES}
        self.pitch_lines = {n: self.ax_pitch.plot([], [], color=FILTER_COLORS[n], lw=1.3,
                                                  label=n)[0] for n in FILTER_NAMES}
        self.ax_roll.legend(loc="upper right", fontsize=7)

        # Yaw is unobservable from the accelerometer, so on a static board its
        # slope is exactly the residual gyro bias - this is the drift to watch.
        self.ax_yaw.set_title("yaw [deg] (unwrapped; slope = gyro drift)")
        self.ax_yaw.set_xlabel("time [s]")
        self.ax_yaw.set_ylabel("deg")
        self.ax_yaw.grid(True, alpha=0.3)
        self.yaw_lines = {n: self.ax_yaw.plot([], [], color=FILTER_COLORS[n], lw=1.4,
                                              label=n)[0] for n in FILTER_NAMES}
        self.ax_yaw.legend(loc="upper left", fontsize=7)

    def update(self, hist_t, hist_euler, hist_acc, filters, seq, rate, temp):
        t = np.asarray(hist_t)
        if len(t) == 0:
            return
        t = t - t[0]

        for name, f in filters.items():
            rot = quat_to_rot(f.q)
            lz, lx = self.body_lines[name]
            lz.set_data_3d([0, rot[0, 2]], [0, rot[1, 2]], [0, rot[2, 2]])
            lx.set_data_3d([0, rot[0, 0]], [0, rot[1, 0]], [0, rot[2, 0]])

        for i, key in enumerate(("ax", "ay", "az", "mag")):
            self.acc_lines[i].set_data(t, np.asarray(hist_acc[key]))

        yaw6 = 0.0
        yaw_unwrapped6 = None
        for name in FILTER_NAMES:
            e = np.degrees(np.asarray(hist_euler[name]))
            self.roll_lines[name].set_data(t, e[:, 0])
            self.pitch_lines[name].set_data(t, e[:, 1])
            yaw_unwrapped = np.degrees(np.unwrap(np.radians(e[:, 2])))
            self.yaw_lines[name].set_data(t, yaw_unwrapped)
            if name == "sixaxis":
                yaw6 = float(yaw_unwrapped[-1])
                yaw_unwrapped6 = yaw_unwrapped

        for ax in (self.ax_acc, self.ax_roll, self.ax_pitch, self.ax_yaw):
            ax.relim()
            ax.autoscale_view()

        # Static-board yaw drift rate over the last ~2 s of history.
        yaw_rate = 0.0
        if yaw_unwrapped6 is not None and len(t) > 20:
            n2 = min(len(t), 1600)
            span = t[-1] - t[-n2]
            if span > 0.5:
                yaw_rate = (yaw_unwrapped6[-1] - yaw_unwrapped6[-n2]) / span

        ang_mh = quat_angle_deg(filters["mahony"].q, filters["sixaxis"].q)
        ang_mg = quat_angle_deg(filters["madgwick"].q, filters["sixaxis"].q)
        bias_z = math.degrees(filters["sixaxis"].bias[2])
        self.fig.suptitle(
            f"seq {seq} | {rate:.0f} Hz | yaw {yaw6:+.2f} deg ({yaw_rate:+.3f} deg/s)"
            f" | gyro bias Z {bias_z:+.4f} deg/s | trust {filters['sixaxis'].trust:.2f}"
            f" | dq(SixAxis vs Mahony/Madgwick) {ang_mh:.2f}/{ang_mg:.2f} deg",
            fontsize=9,
        )
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def alive(self) -> bool:
        return self.plt.fignum_exists(self.fig.number)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="ICM-45686 CAN FD demo: Mahony / Madgwick / SixAxis comparison",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("-i", "--interface", default="auto",
                   choices=("auto", "socketcan", "pcan"),
                   help="python-can backend (auto: socketcan on Linux, pcan on Windows)")
    p.add_argument("-c", "--channel", default=None,
                   help="CAN channel/interface (auto: can0 / PCAN_USBBUS1)")
    p.add_argument("--f-clock", type=int, default=80_000_000,
                   help="PCAN FD clock in Hz (PCAN-USB FD uses 80 MHz)")
    p.add_argument("--calib-seconds", type=float, default=2.0,
                   help="static window used to estimate the gyro bias")
    p.add_argument("--mahony-kp", type=float, default=1.0)
    p.add_argument("--mahony-ki", type=float, default=0.05)
    p.add_argument("--madgwick-beta", type=float, default=0.1)
    p.add_argument("--history", type=float, default=8.0,
                   help="seconds of history kept for the plots")
    p.add_argument("--log", default=None, help="write decoded samples to a CSV file")
    p.add_argument("--simulate", action="store_true",
                   help="use a synthetic 800 Hz source (no CAN hardware needed)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="stop after N seconds (0 = run until the window is closed)")
    p.add_argument("--no-plot", action="store_true",
                   help="print a status line instead of opening a window")
    return p.parse_args()


def main():
    args = parse_args()

    bus = None
    if args.simulate:
        reader = SimReader(static_seconds=args.calib_seconds)
    else:
        try:
            bus = open_bus(args)
        except Exception as exc:
            sys.exit(f"failed to open CAN bus: {exc}")
        reader = CanReader(bus, log_path=args.log)
    reader.start()
    print("waiting for CAN FD frames (ID 0x01) ..." if not args.simulate
          else "simulating a tumbling board at 800 Hz ...")

    # ---- startup static calibration -------------------------------------
    t_cal = time.perf_counter()
    calib = []
    while time.perf_counter() - t_cal < args.calib_seconds and reader.running:
        if reader.samples:
            calib.append(reader.samples.popleft())
        else:
            time.sleep(0.002)
    if reader.error:
        sys.exit(f"CAN read error: {reader.error}")
    if len(calib) < 50:
        sys.exit("no/too few CAN frames received - check the bus and the interface setup")

    accels = np.asarray([c[2] for c in calib])
    gyros = np.asarray([c[3] for c in calib])
    bias = gyros.mean(axis=0)
    accel0 = accels.mean(axis=0)
    print(f"calibrated with {len(calib)} frames: "
          f"|a|={np.linalg.norm(accel0):.2f} m/s^2, "
          f"gyro bias={np.degrees(bias)} deg/s")
    if abs(np.linalg.norm(accel0) - GRAVITY) > 0.1 * GRAVITY or np.linalg.norm(bias) > 0.1:
        print("WARNING: the board was not still during calibration; the bias may be off")

    filters = {
        "mahony": Mahony(args.mahony_kp, args.mahony_ki),
        "madgwick": Madgwick(args.madgwick_beta),
        "sixaxis": SixAxis(),
    }
    for f in filters.values():
        f.reset(accel0, bias)

    # ---- rolling history -------------------------------------------------
    maxlen = max(100, int(args.history * 1000))
    hist_t = deque(maxlen=maxlen)
    hist_euler = {n: deque(maxlen=maxlen) for n in FILTER_NAMES}
    hist_acc = {k: deque(maxlen=maxlen) for k in ("ax", "ay", "az", "mag")}

    plot = None if args.no_plot else LivePlot()
    last_t = None
    last_redraw = 0.0
    last_print = 0.0
    rate_t0 = time.perf_counter()
    rate_n0 = 0
    rate = 0.0
    last_seq = 0
    last_temp = 0.0
    running = True
    t_start = time.perf_counter()

    try:
        while running:
            processed = 0
            while reader.samples and processed < 400:
                t, seq, accel, gyro, temp = reader.samples.popleft()
                processed += 1
                last_seq, last_temp = seq, temp
                if last_t is not None:
                    dt = t - last_t
                    if 0.0 < dt < 0.1:
                        for f in filters.values():
                            f.update(accel, gyro, dt)
                        hist_t.append(t)
                        for name, f in filters.items():
                            hist_euler[name].append(quat_to_euler(f.q))
                        # Motion acceleration in the world frame:
                        # a_world = R * (specific force) + gravity
                        rot = quat_to_rot(filters["sixaxis"].q)
                        a_world = rot @ accel + np.array([0.0, 0.0, -GRAVITY])
                        hist_acc["ax"].append(a_world[0])
                        hist_acc["ay"].append(a_world[1])
                        hist_acc["az"].append(a_world[2])
                        hist_acc["mag"].append(float(np.linalg.norm(a_world)))
                last_t = t

            now = time.perf_counter()
            if now - rate_t0 >= 1.0:
                rate = (last_seq - rate_n0) / (now - rate_t0)
                rate_n0 = last_seq
                rate_t0 = now

            if plot is not None:
                if now - last_redraw > 0.05:
                    plot.update(hist_t, hist_euler, hist_acc, filters,
                                last_seq, rate, last_temp)
                    last_redraw = now
                    if not plot.alive():
                        running = False
                # Console yaw readout (easy to watch while aligning the board).
                if now - last_print > 0.25:
                    r, p, y = np.degrees(quat_to_euler(filters["sixaxis"].q))
                    print(f"yaw {y:+8.2f} deg   (roll {r:+7.2f}, pitch {p:+7.2f})")
                    last_print = now
            elif now - last_print > 0.2:
                e = {n: np.degrees(np.asarray(hist_euler[n][-1])) if hist_euler[n] else (0, 0, 0)
                     for n in FILTER_NAMES}
                print(f"seq {last_seq} {rate:5.0f} Hz | "
                      f"r/p/y {e['sixaxis'][0]:7.2f}/{e['sixaxis'][1]:7.2f}/{e['sixaxis'][2]:+8.2f} | "
                      f"Mahony {e['mahony'][0]:7.2f}/{e['mahony'][1]:7.2f}/{e['mahony'][2]:+8.2f} | "
                      f"Madgwick {e['madgwick'][0]:7.2f}/{e['madgwick'][1]:7.2f}/{e['madgwick'][2]:+8.2f} | "
                      f"|a| {hist_acc['mag'][-1] if hist_acc['mag'] else 0:5.2f} m/s^2")
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
