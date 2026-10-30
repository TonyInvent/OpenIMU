#!/usr/bin/env python3
"""
open_imu 3-state attitude ESKF + ZARU (6-axis IMU, no magnetometer).

Estimator: the same architecture as tools/eskf.py - a 3-dimensional rotation
error on so(3), right perturbation, static gyro bias locked by the startup
calibration. That is the right choice for a pure 6-axis IMU: only roll/pitch
are observable (through the gravity direction), yaw is left to the gyroscope.

One addition on top of eskf.py: **ZARU (zero angular rate update)**. While the
board is still, the gyro reading *is* the bias - on all three axes, including
yaw - so the bias is low-passed towards the mean gyro of a short still window.
Unlike a Kalman bias state this cannot trade against the attitude error, so it
does not pollute roll/pitch/yaw; it only follows the temperature drift, which
is what makes yaw wander over minutes.

GUI + CAN-FD acquisition are reused unchanged from tools/eskf.py.

Run:
    python tools/eskf_zaru.py --simulate
    python tools/eskf_zaru.py -i pcan -c PCAN_USBBUS1
    python tools/eskf_zaru.py -i socketcan -c can0
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from eskf import ICM45686_ESKF, NOMINAL_DT, ODR_HZ, OpenIMUDashboard, OpenIMUReceiver


# ---------------------------------------------------------------------------
# zero angular rate update
# ---------------------------------------------------------------------------
class ZARU:
    """Detect standstill and expose the mean gyro of that window as the bias."""

    def __init__(self, gyro_th=0.05, accel_th=0.20, window=0.30, still_ratio=0.95):
        self.gyro_th = gyro_th          # rad/s
        self.accel_th = accel_th        # g
        self.window = window            # s
        self.still_ratio = still_ratio
        self.buf = []                   # (still, gyro, dt)
        self.mean_gyro = np.zeros(3)

    def push(self, gyro_rad, acc_g, dt):
        still = (np.linalg.norm(gyro_rad) < self.gyro_th
                 and abs(np.linalg.norm(acc_g) - 1.0) < self.accel_th)
        self.buf.append((still, gyro_rad.copy(), dt))
        total = sum(d for _, _, d in self.buf)
        while total > self.window and len(self.buf) > 1:
            total -= self.buf.pop(0)[2]
        if len(self.buf) < 20:
            return False
        n_still = sum(1 for s, _, _ in self.buf if s)
        if n_still < self.still_ratio * len(self.buf):
            return False
        self.mean_gyro = np.mean([g for s, g, _ in self.buf if s], axis=0)
        return True


# ---------------------------------------------------------------------------
# receiver: eskf.py's acquisition + ZARU bias tracking
# ---------------------------------------------------------------------------
class ZaruReceiver(OpenIMUReceiver):
    def __init__(self, *args, zaru_tau=5.0, **kwargs):
        super().__init__(*args, **kwargs)
        self._zaru = ZARU()
        self._zaru_tau = float(zaru_tau)
        self.zaru_active = False
        self.zaru_count = 0

    def _process_raw_sample(self, seq, acc_g, gyro_rad, temp_c):
        if self.last_seq is not None:
            diff = (seq - self.last_seq) & 0xFFFF
            if diff == 0:
                return
            if 1 < diff < 1000:
                self.dropped_frames += diff - 1
            dt = min(diff * NOMINAL_DT, 0.05)
        else:
            dt = NOMINAL_DT
        self.last_seq = seq
        self.total_frames += 1

        # -- startup static calibration ---------------------------------
        if not self.eskf.initialized:
            self._calib_acc.append(acc_g)
            self._calib_gyr.append(gyro_rad)
            self.calib_progress = len(self._calib_acc) / float(self.calib_frames)
            if len(self._calib_acc) >= self.calib_frames:
                self.eskf.align_initial_state(np.mean(self._calib_acc, axis=0),
                                              np.mean(self._calib_gyr, axis=0))
                self.sim_time_sec = 0.0
            return

        # -- propagation -------------------------------------------------
        self.sim_time_sec += dt
        self.eskf.predict(gyro_rad, dt)

        # -- ZARU: the safe way to track the bias on a 6-axis IMU --------
        self.zaru_active = self._zaru.push(gyro_rad, acc_g, dt)
        if self.zaru_active and self._zaru_tau > 0.0:
            alpha = min(1.0, dt / self._zaru_tau)
            self.eskf.bg += (self._zaru.mean_gyro - self.eskf.bg) * alpha
            self.zaru_count += 1

        # -- accelerometer: gravity direction ----------------------------
        if (self.total_frames & 0x01) == 0:
            self.eskf.update_accel(acc_g)
            r, p, y = self.eskf.get_euler_deg()
            bg_deg = np.degrees(self.eskf.bg)
            with self.lock:
                self.t_buf.append(self.sim_time_sec)
                self.roll_buf.append(r)
                self.pitch_buf.append(p)
                self.yaw_buf.append(y)
                self.bx_buf.append(bg_deg[0])
                self.by_buf.append(bg_deg[1])
                self.bz_buf.append(bg_deg[2])
                self.temp_c = temp_c
                self.acc_norm_g = float(np.linalg.norm(acc_g))


# ---------------------------------------------------------------------------
# main (same wiring as eskf.py)
# ---------------------------------------------------------------------------
def main():
    is_win = sys.platform.startswith("win")
    parser = argparse.ArgumentParser(description="open_imu 3-state ESKF + ZARU live monitor")
    parser.add_argument("-i", "--interface", default="pcan" if is_win else "socketcan")
    parser.add_argument("-c", "--channel", default="PCAN_USBBUS1" if is_win else "can0")
    parser.add_argument("--f-clock", type=int, default=80)
    parser.add_argument("--calib-seconds", type=float, default=1.5)
    parser.add_argument("--history", type=float, default=5.0)
    parser.add_argument("--zaru-tau", type=float, default=5.0,
                        help="ZARU bias time constant in seconds (0 = disable)")
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()

    from PyQt6 import QtWidgets

    app = QtWidgets.QApplication(sys.argv)
    eskf = ICM45686_ESKF()
    rx = ZaruReceiver(eskf, interface=args.interface, channel=args.channel,
                      f_clock_mhz=args.f_clock, calib_sec=args.calib_seconds,
                      window_sec=args.history, simulate=args.simulate,
                      zaru_tau=args.zaru_tau)
    rx.start()

    gui = OpenIMUDashboard(rx, window_sec=args.history)
    gui.setWindowTitle("open_imu (ICM-45686 @ 800 Hz) - 3-state ESKF + ZARU, 6-axis")
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
