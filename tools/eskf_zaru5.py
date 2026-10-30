#!/usr/bin/env python3
"""
open_imu 5-state ESKF + ZARU (6-axis IMU, no magnetometer).

ESKF state
    nominal : quaternion q, gyro bias bg (3 axes)
    error   : d_theta (3) + dbg_x, dbg_y (2)  -> 5 x 5 covariance

Why this split
    Roll/pitch (and therefore the X/Y gyro biases) are observable through the
    gravity direction, so dbg_x/dbg_y are proper Kalman states and are learned
    continuously. Yaw - and its bias dbg_z - are NOT observable on a 6-axis
    IMU, so bg_z is kept out of the state and handled by ZARU instead.

ZARU (zero angular rate update)
    While the board is still the gyro reading is the bias:
      * X/Y -> a proper Kalman bias measurement (H selects dbg_x/dbg_y)
      * Z   -> a direct low-pass on the locked bg_z

Dashboard + CAN-FD acquisition are reused unchanged from tools/eskf.py.

Run:
    python tools/eskf_zaru5.py --simulate
    python tools/eskf_zaru5.py -i pcan -c PCAN_USBBUS1
    python tools/eskf_zaru5.py -i pcan -c PCAN_USBBUS1 --zaru-tau 0   # ZARU off
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from eskf import NOMINAL_DT, ODR_HZ, OpenIMUDashboard, OpenIMUReceiver


class ICM45686_ESKF5:
    """3-dim rotation error + X/Y gyro bias errors."""

    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.bg = np.zeros(3, dtype=np.float64)  # bg_x, bg_y dynamic; bg_z locked
        self.P = np.diag([1e-3, 1e-3, 1e-3, 1e-4, 1e-4])

        self.var_g = (np.radians(0.01)) ** 2
        self.var_bg = (np.radians(0.001)) ** 2
        self.sigma_a = 0.035
        self.initialized = False

    @staticmethod
    def skew(v):
        return np.array([
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ], dtype=np.float64)

    @staticmethod
    def quat_mul(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ], dtype=np.float64)

    # -- startup ---------------------------------------------------------
    def align_initial_state(self, mean_acc_g, mean_gyro_rad):
        self.bg = mean_gyro_rad.copy()
        ax, ay, az = mean_acc_g / np.linalg.norm(mean_acc_g)
        roll0 = np.arctan2(ay, az)
        pitch0 = np.arctan2(-ax, np.sqrt(ay * ay + az * az))
        cr, sr = np.cos(roll0 * 0.5), np.sin(roll0 * 0.5)
        cp, sp = np.cos(pitch0 * 0.5), np.sin(pitch0 * 0.5)
        self.q = np.array([cr * cp, sr * cp, cr * sp, -sr * sp], dtype=np.float64)
        self.q /= np.linalg.norm(self.q)
        self.initialized = True

    # -- prediction ------------------------------------------------------
    def predict(self, gyro_rad, dt):
        omega_corr = gyro_rad - self.bg
        ox, oy, oz = omega_corr * dt
        angle = (ox * ox + oy * oy + oz * oz) ** 0.5
        if angle > 1e-7:
            half_a = 0.5 * angle
            s = np.sin(half_a) / angle
            dq = np.array([np.cos(half_a), ox * s, oy * s, oz * s], dtype=np.float64)
        else:
            dq = np.array([1.0, 0.5 * ox, 0.5 * oy, 0.5 * oz], dtype=np.float64)
        self.q = self.quat_mul(self.q, dq)
        self.q /= np.linalg.norm(self.q)

        F = np.eye(5, dtype=np.float64)
        F[0, 1] = oz; F[0, 2] = -oy
        F[1, 0] = -oz; F[1, 2] = ox
        F[2, 0] = oy; F[2, 1] = -ox
        F[0, 3] = -dt
        F[1, 4] = -dt

        Q = np.zeros((5, 5), dtype=np.float64)
        Q[0:3, 0:3] = np.eye(3) * (self.var_g * dt)
        Q[3:5, 3:5] = np.eye(2) * (self.var_bg * dt)
        self.P = F @ self.P @ F.T + Q

    # -- accelerometer: gravity direction --------------------------------
    def update_accel(self, acc_g):
        norm_a = np.linalg.norm(acc_g)
        if norm_a < 1e-3:
            return
        if abs(norm_a - 1.0) > 0.08:
            return
        a_unit = acc_g / norm_a
        R = np.eye(3) * (self.sigma_a ** 2)

        w, x, y, z = self.q
        g_pred = np.array([
            2.0 * (x * z - w * y),
            2.0 * (w * x + y * z),
            w * w - x * x - y * y + z * z,
        ], dtype=np.float64)
        y_res = a_unit - g_pred

        H = np.zeros((3, 5), dtype=np.float64)
        H[0:3, 0:3] = self.skew(g_pred)

        PHt = self.P @ H.T
        S = H @ PHt + R
        K = PHt @ np.linalg.inv(S)

        dx = K @ y_res
        d_theta, d_bg_xy = dx[0:3], dx[3:5]

        theta_norm = np.linalg.norm(d_theta)
        if theta_norm > 1e-7:
            dq_err = np.hstack([np.cos(0.5 * theta_norm),
                                (d_theta / theta_norm) * np.sin(0.5 * theta_norm)])
        else:
            dq_err = np.hstack([1.0, 0.5 * d_theta])
        self.q = self.quat_mul(self.q, dq_err)
        self.q /= np.linalg.norm(self.q)

        self.bg[0] += d_bg_xy[0]
        self.bg[1] += d_bg_xy[1]

        I_KH = np.eye(5, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        G = np.eye(5, dtype=np.float64)
        G[0:3, 0:3] -= 0.5 * self.skew(d_theta)
        self.P = G @ self.P @ G.T

    # -- ZARU: X/Y through the Kalman filter -----------------------------
    def update_bias_xy(self, mean_gyro_xy, sigma=0.001):
        H = np.zeros((2, 5), dtype=np.float64)
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        V = np.eye(2, dtype=np.float64) * (sigma * sigma)
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + V)
        dx = K @ (np.asarray(mean_gyro_xy, dtype=np.float64) - self.bg[0:2])
        self.bg[0] += dx[3]
        self.bg[1] += dx[4]
        I_KH = np.eye(5, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ V @ K.T

    # -- ZARU: Z directly (not observable, kept out of the state) --------
    def update_bias_z(self, mean_gyro_z, alpha):
        self.bg[2] += (mean_gyro_z - self.bg[2]) * alpha

    def get_euler_deg(self):
        w, x, y, z = self.q
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


class ZARU:
    """Detect standstill and expose the mean gyro of that window as the bias."""

    def __init__(self, gyro_th=0.05, accel_th=0.20, window=0.30, still_ratio=0.95):
        self.gyro_th = gyro_th
        self.accel_th = accel_th
        self.window = window
        self.still_ratio = still_ratio
        self.buf = []
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


class Zaru5Receiver(OpenIMUReceiver):
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

        if not self.eskf.initialized:
            self._calib_acc.append(acc_g)
            self._calib_gyr.append(gyro_rad)
            self.calib_progress = len(self._calib_acc) / float(self.calib_frames)
            if len(self._calib_acc) >= self.calib_frames:
                self.eskf.align_initial_state(np.mean(self._calib_acc, axis=0),
                                              np.mean(self._calib_gyr, axis=0))
                self.sim_time_sec = 0.0
            return

        self.sim_time_sec += dt
        self.eskf.predict(gyro_rad, dt)

        self.zaru_active = self._zaru.push(gyro_rad, acc_g, dt)
        if self.zaru_active and self._zaru_tau > 0.0:
            self.eskf.update_bias_xy(self._zaru.mean_gyro[0:2])
            self.eskf.update_bias_z(self._zaru.mean_gyro[2],
                                    min(1.0, dt / self._zaru_tau))
            self.zaru_count += 1

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


def main():
    is_win = sys.platform.startswith("win")
    parser = argparse.ArgumentParser(description="open_imu 5-state ESKF + ZARU live monitor")
    parser.add_argument("-i", "--interface", default="pcan" if is_win else "socketcan")
    parser.add_argument("-c", "--channel", default="PCAN_USBBUS1" if is_win else "can0")
    parser.add_argument("--f-clock", type=int, default=80)
    parser.add_argument("--calib-seconds", type=float, default=1.5)
    parser.add_argument("--history", type=float, default=5.0)
    parser.add_argument("--zaru-tau", type=float, default=5.0,
                        help="ZARU Z-bias time constant in seconds (0 = disable)")
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()

    from PyQt6 import QtWidgets

    app = QtWidgets.QApplication(sys.argv)
    eskf = ICM45686_ESKF5()
    rx = Zaru5Receiver(eskf, interface=args.interface, channel=args.channel,
                       f_clock_mhz=args.f_clock, calib_sec=args.calib_seconds,
                       window_sec=args.history, simulate=args.simulate,
                       zaru_tau=args.zaru_tau)
    rx.start()

    gui = OpenIMUDashboard(rx, window_sec=args.history)
    gui.setWindowTitle("open_imu (ICM-45686 @ 800 Hz) - 5-state ESKF + ZARU, 6-axis")
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
