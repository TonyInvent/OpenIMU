#!/usr/bin/env python3
"""
open_imu AHRS comparison (ahrs library): ICM-45686 (CAN FD, ID 0x01)
-> Mahony / Madgwick / EKF / SixAxis.

Reads the 16-byte CAN FD stream produced by the STM32G431 firmware and runs
four attitude estimators on the same samples, so they can be compared live:

  * Mahony    - ahrs.filters.Mahony    (nonlinear complementary filter, PI)
  * Madgwick  - ahrs.filters.Madgwick  (gradient-descent AHRS)
  * EKF       - ahrs.filters.EKF       (quaternion error-state Kalman filter)
  * SixAxis   - BeijingDynamics open_sprite_runtime native/src/imu_six_axis.hpp
                (proportional correction with a startup static gyro bias and an
                accelerometer trust window, no magnetometer)

Live view: 3-D body axes, roll/pitch/yaw comparison, world-frame linear
("motion") acceleration.

CAN backends (python-can):
  Linux   : SocketCAN   (-i socketcan -c can0), interface configured with
            `sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on`
  Windows : PCAN-Basic  (-i pcan -c PCAN_USBBUS1) for a PCAN-USB FD adapter

Requires: python-can, numpy, matplotlib, ahrs
    pip install python-can numpy matplotlib ahrs
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

try:
    from ahrs.filters import EKF, Madgwick, Mahony
except ImportError:  # pragma: no cover
    sys.exit("ahrs is required:  pip install ahrs")

# ---------------------------------------------------------------------------
# Frame / sensor constants (must match the firmware)
# ---------------------------------------------------------------------------
CAN_ID = 0x01
GRAVITY = 9.80665            # m/s^2
ACCEL_LSB_PER_G = 8192.0     # +/-4 g
GYRO_LSB_PER_DPS = 16.4      # +/-2000 dps
DEG = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

FILTER_NAMES = ("mahony", "madgwick", "ekf", "sixaxis")
FILTER_COLORS = {
    "mahony": "#d62728",
    "madgwick": "#2ca02c",
    "ekf": "#9467bd",
    "sixaxis": "#1f77b4",
}
NOMINAL_RATE = 800.0


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
    d = min(1.0, abs(float(np.dot(q1, q2))))
    return 2.0 * math.acos(d) * RAD2DEG


# ---------------------------------------------------------------------------
# Attitude estimators
# ---------------------------------------------------------------------------
class AhrsMahony:
    """Wrapper around ahrs.filters.Mahony (online IMU update)."""

    name = "mahony"

    def __init__(self, kp=1.0, ki=0.3):
        self.f = Mahony(frequency=NOMINAL_RATE, k_P=kp, k_I=ki)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def reset(self, accel, bias=None):
        self.q = quat_from_accel(accel)

    def update(self, accel, gyro, dt):
        self.q = self.f.updateIMU(self.q, gyro, accel, dt)
        return self.q


class AhrsMadgwick:
    """Wrapper around ahrs.filters.Madgwick (online IMU update)."""

    name = "madgwick"

    def __init__(self, gain=0.033):
        self.f = Madgwick(frequency=NOMINAL_RATE, gain_imu=gain)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def reset(self, accel, bias=None):
        self.q = quat_from_accel(accel)

    def update(self, accel, gyro, dt):
        self.q = self.f.updateIMU(self.q, gyro, accel, dt)
        return self.q


class AhrsEkf:
    """Wrapper around ahrs.filters.EKF (quaternion EKF, online update)."""

    name = "ekf"

    def __init__(self):
        self.f = EKF(frequency=NOMINAL_RATE)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def reset(self, accel, bias=None):
        self.q = quat_from_accel(accel)

    def update(self, accel, gyro, dt):
        self.q = self.f.update(self.q, gyro, accel, dt=dt)
        return self.q


class SixAxis:
    """Port of open_sprite_runtime native/src/imu_six_axis.hpp (SixAxis)."""

    name = "sixaxis"

    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)
        self.trust = 0.0

    def reset(self, accel, bias=None):
        roll = math.atan2(accel[1], accel[2])
        pitch = math.atan2(-accel[0], math.hypot(accel[1], accel[2]))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        self.q = np.array([cr * cp, sr * cp, cr * sp, -sr * sp])
        self.bias = np.array(bias if bias is not None else np.zeros(3), dtype=float)
        self.trust = 0.0

    def _up(self) -> np.ndarray:
        w, x, y, z = self.q
        return np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])

    def update(self, accel, gyro, dt):
        omega = gyro - self.bias
        mag = float(np.linalg.norm(accel))
        self.trust = min(1.0, max(0.0, (0.25 - abs(mag / GRAVITY - 1.0)) / 0.20))
        if self.trust > 0.0 and mag > 1e-9:
            measured = accel / mag
            omega = omega + self.trust * np.cross(measured, self._up())
        self.q = quat_integrate(self.q, omega, dt)
        return self.q


# ---------------------------------------------------------------------------
# Shared history: written by the acquisition thread, read by the display thread
# ---------------------------------------------------------------------------
class SharedHistory:
    """Fixed-size numpy ring buffer, lock-free in practice.

    The acquisition thread pushes one entry per IMU frame; the display thread
    only ever reads a snapshot. ``idx``/``count`` are plain ints (atomic under
    the GIL); a sample torn at the write boundary only shows up as one slightly
    wrong point in the plot, never as a stall.
    """

    def __init__(self, capacity, names):
        self.cap = int(capacity)
        self.t = np.zeros(self.cap)
        self.euler = {n: np.zeros((self.cap, 3)) for n in names}
        self.acc = np.zeros((self.cap, 4))
        self.idx = 0
        self.count = 0
        self.seq = 0

    def push(self, t, euler, acc):
        i = self.idx
        self.t[i] = t
        for n, e in euler.items():
            self.euler[n][i] = e
        self.acc[i] = acc
        i += 1
        self.idx = i % self.cap
        if self.count < self.cap:
            self.count += 1
        self.seq += 1

    def snapshot(self):
        """Samples currently in the ring, oldest first (seqlock read)."""
        data = None
        for _ in range(4):
            s0 = self.seq
            n = self.count
            if n == 0:
                return None
            end = self.idx
            start = (end - n) % self.cap
            if start < end:
                sel = np.arange(start, end)
            else:
                sel = np.concatenate((np.arange(start, self.cap), np.arange(0, end)))
            data = (self.t[sel],
                    {k: v[sel] for k, v in self.euler.items()},
                    {k: self.acc[sel, i] for i, k in enumerate(("ax", "ay", "az", "mag"))})
            if self.seq == s0:
                return data
        return data


# ---------------------------------------------------------------------------
# Acquisition + algorithm thread (never blocked by the display)
# ---------------------------------------------------------------------------
class _AlgoSource(threading.Thread):
    """Runs the static calibration and the four filters at full rate.

    Everything the display needs is published through ``history`` and the
    filters' current quaternions, so rendering cannot stall acquisition.
    """

    def __init__(self, calib_seconds, capacity, raw_gyro=False,
                 mahony_kp=1.0, mahony_ki=0.3, madgwick_gain=0.033):
        super().__init__(daemon=True)
        self.calib_seconds = float(calib_seconds)
        self.raw_gyro = bool(raw_gyro)
        self.history = SharedHistory(capacity, FILTER_NAMES)
        self.running = True
        self.error = None
        self.filters = None
        self.bias = np.zeros(3)
        self.accel0 = np.zeros(3)
        self.calib_count = 0
        self.last_seq = 0
        self.last_temp = 0.0
        self.ready = threading.Event()
        self._mahony_kp = mahony_kp
        self._mahony_ki = mahony_ki
        self._madgwick_gain = madgwick_gain
        self._calib = []
        self._t0 = None
        self._prev_t = None

    # -- calibration -----------------------------------------------------
    def _finish_calibration(self):
        accels = np.asarray([c[2] for c in self._calib])
        gyros = np.asarray([c[3] for c in self._calib])
        self.bias = gyros.mean(axis=0)
        self.accel0 = accels.mean(axis=0)
        self.calib_count = len(self._calib)
        self.filters = {
            "mahony": AhrsMahony(self._mahony_kp, self._mahony_ki),
            "madgwick": AhrsMadgwick(self._madgwick_gain),
            "ekf": AhrsEkf(),
            "sixaxis": SixAxis(),
        }
        for f in self.filters.values():
            # SixAxis keeps the calibrated bias only when the raw gyro is fed.
            f.reset(self.accel0, self.bias if self.raw_gyro else None)
        self._calib = []
        self._prev_t = None
        self.ready.set()

    # -- one IMU frame ---------------------------------------------------
    def _handle(self, t, seq, accel, gyro, temp):
        self.last_seq, self.last_temp = seq, temp
        if self.filters is None:
            if self._t0 is None:
                self._t0 = t
            self._calib.append((t, seq, accel, gyro, temp))
            if t - self._t0 >= self.calib_seconds:
                self._finish_calibration()
            return

        if self._prev_t is not None:
            dt = t - self._prev_t
            if 0.0 < dt < 0.1:
                gyro_in = gyro if self.raw_gyro else gyro - self.bias
                euler = {}
                for name, f in self.filters.items():
                    f.update(accel, gyro_in, dt)
                    euler[name] = np.array(quat_to_euler(f.q))
                rot = quat_to_rot(self.filters["sixaxis"].q)
                a_world = rot @ accel + np.array([0.0, 0.0, -GRAVITY])
                acc = np.array([a_world[0], a_world[1], a_world[2],
                                float(np.linalg.norm(a_world))])
                self.history.push(t, euler, acc)
        self._prev_t = t

    def stop(self):
        self.running = False


class CanReader(_AlgoSource):
    """CAN FD source: decodes ID 0x01 frames and feeds the algorithm thread."""

    def __init__(self, bus, calib_seconds, capacity, raw_gyro=False,
                 mahony_kp=1.0, mahony_ki=0.3, madgwick_gain=0.033, log_path=None):
        super().__init__(calib_seconds, capacity, raw_gyro,
                         mahony_kp, mahony_ki, madgwick_gain)
        self.bus = bus
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
                self._handle(t, seq, accel, gyro, temp)
                if self._log is not None:
                    self._log.write(
                        f"{t:.6f},{seq},{accel[0]:.4f},{accel[1]:.4f},{accel[2]:.4f},"
                        f"{gyro[0]:.5f},{gyro[1]:.5f},{gyro[2]:.5f},{temp:.2f}\n")
        except Exception as exc:
            self.error = exc
        finally:
            if self._log is not None:
                self._log.close()


class SimReader(_AlgoSource):
    """Synthetic 800 Hz source: static during calibration, then a slow tumble."""

    def __init__(self, calib_seconds, capacity, raw_gyro=False,
                 mahony_kp=1.0, mahony_ki=0.3, madgwick_gain=0.033):
        super().__init__(calib_seconds, capacity, raw_gyro,
                         mahony_kp, mahony_ki, madgwick_gain)
        self.seq = 0
        self.static_until = time.perf_counter() + calib_seconds

    def run(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        tumble = np.array([0.25, 0.12, 0.30])   # rad/s
        bias = np.array([0.010, -0.005, 0.015]) * DEG   # rad/s, realistic IMU bias
        last = time.perf_counter()
        while self.running:
            now = time.perf_counter()
            dt = now - last
            last = now
            if dt <= 0.0 or dt > 0.05:
                dt = 1.0 / 800.0
            omega = np.zeros(3) if self.filters is None else tumble
            q = quat_integrate(q, omega, dt)
            accel = quat_to_rot(q).T @ np.array([0.0, 0.0, GRAVITY])
            self.seq += 1
            self._handle(now, self.seq, accel, omega + bias, 25.0)
            time.sleep(1.0 / 800.0)


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
    def __init__(self, window=3.0):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        self.plt = plt
        self.window = float(window)
        self.max_points = 900          # ~1 point per pixel of the plot width
        plt.ion()
        self.fig = plt.figure("open_imu AHRS comparison (ahrs lib)", figsize=(12, 9))
        gs = self.fig.add_gridspec(3, 2, height_ratios=[1.25, 1.0, 1.0])
        self.ax3d = self.fig.add_subplot(gs[0, 0], projection="3d")
        self.ax_acc = self.fig.add_subplot(gs[0, 1])
        self.ax_roll = self.fig.add_subplot(gs[1, 0])
        self.ax_pitch = self.fig.add_subplot(gs[1, 1])
        self.ax_yaw = self.fig.add_subplot(gs[2, :])

        # 3-D attitude: world triad (grey) + body frame per filter.
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
            lz, = self.ax3d.plot([], [], [], color=color, lw=2.2, label=name)
            lx, = self.ax3d.plot([], [], [], color=color, lw=0.9, alpha=0.45)
            self.body_lines[name] = (lz, lx)
        self.ax3d.legend(loc="upper right", fontsize=6.5)

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

        # Roll / pitch / yaw comparison.
        for ax, title in ((self.ax_roll, "roll [deg]"), (self.ax_pitch, "pitch [deg]")):
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
        self.roll_lines = {n: self.ax_roll.plot([], [], color=FILTER_COLORS[n], lw=1.3,
                                                 label=n)[0] for n in FILTER_NAMES}
        self.pitch_lines = {n: self.ax_pitch.plot([], [], color=FILTER_COLORS[n], lw=1.3,
                                                  label=n)[0] for n in FILTER_NAMES}
        self.ax_roll.legend(loc="upper right", fontsize=7)

        self.ax_yaw.set_title("yaw [deg] (unwrapped; slope = gyro drift)")
        self.ax_yaw.set_xlabel("time [s]")
        self.ax_yaw.set_ylabel("deg")
        self.ax_yaw.grid(True, alpha=0.3)
        self.yaw_lines = {n: self.ax_yaw.plot([], [], color=FILTER_COLORS[n], lw=1.4,
                                              label=n)[0] for n in FILTER_NAMES}
        self.ax_yaw.legend(loc="upper left", fontsize=7)

        # Draw once now: otherwise the window stays white until the first data
        # frame arrives and it looks like the app has frozen.
        self.fig.suptitle("calibrating ... (keep the board still)", fontsize=9)
        try:
            self.fig.canvas.draw()
        except Exception:
            pass

    def update(self, snap, filters, seq, rate, temp):
        if snap is None:
            return
        t, euler_hist, acc_hist = snap
        n = len(t)
        if n < 2:
            return
        # Decimate to ~1 point/pixel: a 3 s window at 800 Hz is far more detail
        # than the screen can show, and it dominates the redraw cost.
        if n > self.max_points:
            idx = np.linspace(0, n - 1, self.max_points).astype(np.intp)
            t_plot = t[idx]
        else:
            idx = None
            t_plot = t

        def decimate(y):
            return y[idx] if idx is not None else y

        # Fixed sliding window: newest sample at `window`, older samples left.
        t_plot = (t_plot - t[-1]) + self.window

        for name, f in filters.items():
            rot = quat_to_rot(f.q)
            lz, lx = self.body_lines[name]
            lz.set_data_3d([0, rot[0, 2]], [0, rot[1, 2]], [0, rot[2, 2]])
            lx.set_data_3d([0, rot[0, 0]], [0, rot[1, 0]], [0, rot[2, 0]])

        acc_series = [decimate(acc_hist[k]) for k in ("ax", "ay", "az", "mag")]
        for i in range(4):
            self.acc_lines[i].set_data(t_plot, acc_series[i])

        yaw6 = 0.0
        yaw_unwrapped6 = None
        roll_s, pitch_s, yaw_s = [], [], []
        for name in FILTER_NAMES:
            e = np.degrees(euler_hist[name])
            rs = decimate(e[:, 0])
            ps = decimate(e[:, 1])
            yaw_unwrapped = np.degrees(np.unwrap(np.radians(e[:, 2])))
            ys = decimate(yaw_unwrapped)
            self.roll_lines[name].set_data(t_plot, rs)
            self.pitch_lines[name].set_data(t_plot, ps)
            self.yaw_lines[name].set_data(t_plot, ys)
            roll_s.append(rs)
            pitch_s.append(ps)
            yaw_s.append(ys)
            if name == "sixaxis":
                yaw6 = float(yaw_unwrapped[-1])
                yaw_unwrapped6 = yaw_unwrapped

        # Robust Y ranges: a single outlier must not flatten a curve for seconds.
        self._robust_ylim(self.ax_acc, acc_series)
        self._robust_ylim(self.ax_roll, roll_s)
        self._robust_ylim(self.ax_pitch, pitch_s)
        self._robust_ylim(self.ax_yaw, yaw_s)
        for ax in (self.ax_acc, self.ax_roll, self.ax_pitch, self.ax_yaw):
            ax.set_xlim(0.0, self.window)

        yaw_rate = 0.0
        if yaw_unwrapped6 is not None and n > 20:
            i2 = int(np.searchsorted(t, t[-1] - 2.0))
            i2 = max(0, min(i2, n - 2))
            span = t[-1] - t[i2]
            if span > 0.5:
                yaw_rate = (yaw_unwrapped6[-1] - yaw_unwrapped6[i2]) / span

        ref = filters["sixaxis"].q
        dqs = " / ".join(f"{quat_angle_deg(filters[nm].q, ref):.1f}"
                         for nm in ("mahony", "madgwick", "ekf"))
        self.fig.suptitle(
            f"seq {seq} | {rate:.0f} Hz | yaw {yaw6:+.2f} deg ({yaw_rate:+.3f} deg/s)"
            f" | dq vs SixAxis (Mahony/Madgwick/EKF): {dqs} deg",
            fontsize=9,
        )
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    @staticmethod
    def _robust_ylim(ax, series, pad=0.15):
        """Percentile Y range: one outlier cannot flatten the whole curve."""
        vals = np.concatenate([np.asarray(a)[np.isfinite(a)] for a in series])
        if vals.size == 0:
            return
        lo, hi = np.percentile(vals, [1, 99])
        span = max(hi - lo, 1e-6)
        ax.set_ylim(lo - pad * span, hi + pad * span)

    def alive(self) -> bool:
        try:
            return self.plt.fignum_exists(self.fig.number)
        except Exception:
            return False

    def pump(self):
        """Process GUI events without touching the data (keeps the window alive)."""
        try:
            self.fig.canvas.draw_idle()
            self.plt.pause(0.005)
        except Exception:
            pass

    def close(self):
        try:
            self.plt.close(self.fig)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="ICM-45686 CAN FD demo: Mahony / Madgwick / EKF / SixAxis comparison",
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
    p.add_argument("--mahony-ki", type=float, default=0.3)
    p.add_argument("--madgwick-gain", type=float, default=0.033)
    p.add_argument("--raw-gyro", action="store_true",
                   help="feed the raw (uncalibrated) gyro to every filter "
                        "(default: subtract the calibrated bias from all of them)")
    p.add_argument("--history", type=float, default=3.0,
                   help="sliding time window of the plots [s]")
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

    capacity = max(200, int(args.history * 1200))
    bus = None
    if args.simulate:
        reader = SimReader(args.calib_seconds, capacity, args.raw_gyro,
                           args.mahony_kp, args.mahony_ki, args.madgwick_gain)
    else:
        try:
            bus = open_bus(args)
        except Exception as exc:
            sys.exit(f"failed to open CAN bus: {exc}")
        reader = CanReader(bus, args.calib_seconds, capacity, args.raw_gyro,
                           args.mahony_kp, args.mahony_ki, args.madgwick_gain,
                           log_path=args.log)
    reader.start()
    print("waiting for CAN FD frames (ID 0x01) ..." if not args.simulate
          else "simulating a tumbling board at 800 Hz ...")

    # ---- GUI first, so the window is not a white rectangle -----------------
    # The plot is created before the calibration finishes; the main thread
    # pumps the GUI event loop while waiting, then starts rendering.
    plot = None if args.no_plot else LivePlot(args.history)

    # ---- acquisition + filters live in the reader thread -----------------
    # The main thread only waits for the calibration, then renders. It never
    # touches the sample stream, so a slow redraw cannot delay acquisition.
    t_wait = time.perf_counter()
    last_pump = 0.0
    while not reader.ready.wait(0.02):
        if plot is not None and time.perf_counter() - last_pump > 0.2:
            plot.pump()
            last_pump = time.perf_counter()
        if reader.error is not None:
            if plot is not None:
                plot.close()
            sys.exit(f"CAN read error: {reader.error}")
        if time.perf_counter() - t_wait > args.calib_seconds + 10.0:
            if plot is not None:
                plot.close()
            sys.exit("no/too few CAN frames received - check the bus and the interface setup")

    print(f"calibrated with {reader.calib_count} frames: "
          f"|a|={np.linalg.norm(reader.accel0):.2f} m/s^2, "
          f"gyro bias={np.round(np.degrees(reader.bias), 4)} deg/s")
    print(f"gyro input: {'raw' if args.raw_gyro else 'calibrated bias removed'}")

    filters = reader.filters
    last_redraw = 0.0
    last_print = 0.0
    running = True
    t_start = time.perf_counter()

    def current_rate(snap):
        if snap is None or len(snap[0]) < 10:
            return 0.0
        t = snap[0]
        i0 = max(0, len(t) - 1200)
        span = t[-1] - t[i0]
        return (len(t) - 1 - i0) / span if span > 0.0 else 0.0

    try:
        while running:
            now = time.perf_counter()
            snap = reader.history.snapshot()
            rate = current_rate(snap)

            if plot is not None:
                if not plot.alive():
                    # The user closed the window: stop cleanly.
                    running = False
                elif now - last_redraw > 0.1:
                    try:
                        plot.update(snap, filters, reader.last_seq, rate, reader.last_temp)
                    except Exception as exc:
                        # A redraw can fail while the window is being destroyed.
                        print(f"plot error: {exc}")
                        running = False
                    last_redraw = now
                else:
                    time.sleep(0.005)
            elif snap is not None and now - last_print > 0.2:
                e = {n: np.degrees(snap[1][n][-1]) for n in FILTER_NAMES}
                print(f"seq {reader.last_seq} {rate:5.0f} Hz | "
                      f"r/p/y {e['sixaxis'][0]:7.2f}/{e['sixaxis'][1]:7.2f}/{e['sixaxis'][2]:+8.2f} | "
                      f"mahony {e['mahony'][0]:7.2f}/{e['mahony'][1]:7.2f}/{e['mahony'][2]:+8.2f} | "
                      f"madgwick {e['madgwick'][0]:7.2f}/{e['madgwick'][1]:7.2f}/{e['madgwick'][2]:+8.2f} | "
                      f"ekf {e['ekf'][0]:7.2f}/{e['ekf'][1]:7.2f}/{e['ekf'][2]:+8.2f}")
                last_print = now
            else:
                time.sleep(0.005)

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
        if plot is not None:
            plot.close()
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass
        print("bye")


if __name__ == "__main__":
    main()
