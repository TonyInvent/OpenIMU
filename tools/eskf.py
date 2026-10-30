import sys
import time
import struct
import argparse
import queue
import threading
from collections import deque
import numpy as np
import pyqtgraph as pg
from PyQt6 import QtWidgets, QtCore

# =====================================================================
# Hardware Protocol Constants (STM32G431 + ICM-45686 CAN-FD Firmware)
# =====================================================================
CAN_ID_IMU   = 0x01          # Standard frame ID = 0x01
FRAME_LEN    = 16            # CAN-FD 16-byte payload
ODR_HZ       = 800.0         # ICM-45686 output data rate (800 Hz)
NOMINAL_DT   = 1.0 / ODR_HZ  # 1.25 ms nominal sample period
ACCEL_SCALE  = 8192.0        # +/- 4g   -> LSB/g
GYRO_SCALE   = 16.4          # +/- 2000 -> LSB/(deg/s)
TEMP_SCALE   = 132.48        # LSB/degC
TEMP_OFFSET  = 25.0          # degC


# =====================================================================
# 1. 3-State Attitude-Only ESKF (Eliminating Gyro Bias Pollution)
#    State     : Quaternion q [w, x, y, z] (4x1)
#    Error     : 3-DoF rotation error d_theta (3x1, rad)
#    Covariance: 3x3 matrix P
# =====================================================================
class ICM45686_ESKF:
    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.bg = np.zeros(3, dtype=np.float64)  # Locked static gyro bias
        self.P = np.diag([1e-4, 1e-4, 1e-4])     # 3x3 attitude error covariance

        self.var_g   = (np.radians(0.008)) ** 2
        self.sigma_a = 0.04
        self.initialized = False

    @staticmethod
    def skew(v):
        """Return 3x3 skew-symmetric cross-product matrix [v]_x."""
        return np.array([
            [ 0.0, -v[2],  v[1]],
            [ v[2],  0.0, -v[0]],
            [-v[1],  v[0],  0.0]
        ], dtype=np.float64)

    @staticmethod
    def quat_mul(q1, q2):
        """Hamilton quaternion multiplication q1 * q2."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ], dtype=np.float64)

    def align_initial_state(self, mean_acc_g, mean_gyro_rad):
        """Lock initial static gyro bias and initialize roll/pitch from gravity."""
        self.bg = mean_gyro_rad.copy()
        ax, ay, az = mean_acc_g / np.linalg.norm(mean_acc_g)
        roll0  = np.arctan2(ay, az)
        pitch0 = np.arctan2(-ax, np.sqrt(ay*ay + az*az))

        cr, sr = np.cos(roll0 * 0.5), np.sin(roll0 * 0.5)
        cp, sp = np.cos(pitch0 * 0.5), np.sin(pitch0 * 0.5)
        self.q = np.array([cr * cp, sr * cp, cr * sp, -sr * sp], dtype=np.float64)
        self.q /= np.linalg.norm(self.q)
        self.initialized = True

    def predict(self, gyro_rad, dt):
        """800 Hz pure gyro integration with fixed bias and 3x3 covariance propagation."""
        ox = (gyro_rad[0] - self.bg[0]) * dt
        oy = (gyro_rad[1] - self.bg[1]) * dt
        oz = (gyro_rad[2] - self.bg[2]) * dt
        angle = (ox*ox + oy*oy + oz*oz) ** 0.5

        if angle > 1e-7:
            half_a = 0.5 * angle
            s = np.sin(half_a) / angle
            dq = np.array([np.cos(half_a), ox * s, oy * s, oz * s], dtype=np.float64)
        else:
            dq = np.array([1.0, 0.5 * ox, 0.5 * oy, 0.5 * oz], dtype=np.float64)

        self.q = self.quat_mul(self.q, dq)
        self.q /= (self.q[0]**2 + self.q[1]**2 + self.q[2]**2 + self.q[3]**2) ** 0.5

        # State transition F = I - [omega*dt]_x (3x3)
        F = np.eye(3, dtype=np.float64)
        F[0, 1] =  oz; F[0, 2] = -oy
        F[1, 0] = -oz; F[1, 2] =  ox
        F[2, 0] =  oy; F[2, 1] = -ox

        self.P = F @ self.P @ F.T + np.eye(3, dtype=np.float64) * (self.var_g * dt)

    def update_accel(self, acc_g):
        """Gentle gravity measurement update for Roll/Pitch (Yaw is untouched)."""
        norm_a = np.linalg.norm(acc_g)
        if norm_a < 1e-3:
            return

        # Strict gating: only update when board is nearly static (accel magnitude close to 1.0g)
        if abs(norm_a - 1.0) > 0.08:
            return

        a_unit = acc_g / norm_a
        R = np.eye(3, dtype=np.float64) * (self.sigma_a ** 2)

        w, x, y, z = self.q
        g_pred = np.array([
            2.0 * (x*z - w*y),
            2.0 * (w*x + y*z),
            w*w - x*x - y*y + z*z
        ], dtype=np.float64)

        y_res = a_unit - g_pred
        H = self.skew(g_pred)  # 3x3 measurement Jacobian

        PHt = self.P @ H.T
        S = H @ PHt + R
        K = PHt @ np.linalg.inv(S)

        d_theta = K @ y_res

        # Inject error into quaternion
        theta_norm = np.linalg.norm(d_theta)
        if theta_norm > 1e-7:
            dq_err = np.hstack([np.cos(0.5*theta_norm), (d_theta/theta_norm)*np.sin(0.5*theta_norm)])
        else:
            dq_err = np.hstack([1.0, 0.5 * d_theta])

        self.q = self.quat_mul(self.q, dq_err)
        self.q /= np.linalg.norm(self.q)

        # Joseph form covariance reset
        I_KH = np.eye(3, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        G = np.eye(3, dtype=np.float64) - 0.5 * self.skew(d_theta)
        self.P = G @ self.P @ G.T

    def get_euler_deg(self):
        """Convert quaternion to Z-Y-X Euler angles (Roll, Pitch, Yaw) in degrees."""
        w, x, y, z = self.q
        roll  = np.arctan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
        sinp  = np.clip(2.0*(w*y - z*x), -1.0, 1.0)
        pitch = np.arcsin(sinp)
        yaw   = np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


# =====================================================================
# 2. CAN-FD Receiver Thread
# =====================================================================
class OpenIMUReceiver(threading.Thread):
    def __init__(self, eskf: ICM45686_ESKF, interface='pcan', channel='PCAN_USBBUS1',
                 f_clock_mhz=80, calib_sec=1.5, window_sec=5.0, simulate=False):
        super().__init__(daemon=True)
        self.eskf = eskf
        self.interface = interface
        self.channel = channel
        self.f_clock_mhz = f_clock_mhz
        self.calib_frames = int(calib_sec * ODR_HZ)
        self.window_sec = window_sec
        self.simulate = simulate
        self.running = True
        self.lock = threading.Lock()

        maxlen = int(window_sec * ODR_HZ) + 200
        self.t_buf     = deque(maxlen=maxlen)
        self.roll_buf  = deque(maxlen=maxlen)
        self.pitch_buf = deque(maxlen=maxlen)
        self.yaw_buf   = deque(maxlen=maxlen)
        self.bx_buf    = deque(maxlen=maxlen)
        self.by_buf    = deque(maxlen=maxlen)
        self.bz_buf    = deque(maxlen=maxlen)

        self.sim_time_sec = 0.0
        self.last_seq = None
        self.total_frames = 0
        self.dropped_frames = 0
        self.fps = 0.0
        self.temp_c = 25.0
        self.acc_norm_g = 1.0
        self.calib_progress = 0.0

    def _open_can_bus(self):
        import can
        if self.interface == 'pcan':
            if self.f_clock_mhz == 80:
                return can.Bus(
                    interface='pcan', channel=self.channel, fd=True, f_clock_mhz=80,
                    nom_brp=2,  nom_tseg1=31, nom_tseg2=8, nom_sjw=8,
                    data_brp=2, data_tseg1=5, data_tseg2=2, data_sjw=2
                )
            else:
                return can.Bus(
                    interface='pcan', channel=self.channel, fd=True, f_clock_mhz=self.f_clock_mhz,
                    nom_brp=1,  nom_tseg1=int(self.f_clock_mhz*0.8)-1, nom_tseg2=int(self.f_clock_mhz*0.2), nom_sjw=4,
                    data_brp=1, data_tseg1=int(self.f_clock_mhz/5*0.75)-1, data_tseg2=int(self.f_clock_mhz/5*0.25), data_sjw=2
                )
        elif self.interface == 'socketcan':
            return can.Bus(interface='socketcan', channel=self.channel, fd=True)
        else:
            return can.Bus(
                interface=self.interface, channel=self.channel, fd=True,
                bitrate=1000000, data_bitrate=5000000
            )

    def _process_raw_sample(self, seq, acc_g, gyro_rad, temp_c):
        if self.last_seq is not None:
            diff = (seq - self.last_seq) & 0xFFFF
            if diff == 0:
                return
            if diff > 1 and diff < 1000:
                self.dropped_frames += (diff - 1)
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
                mean_a = np.mean(self._calib_acc, axis=0)
                mean_g = np.mean(self._calib_gyr, axis=0)
                self.eskf.align_initial_state(mean_a, mean_g)
                self.sim_time_sec = 0.0
            return

        self.sim_time_sec += dt
        self.eskf.predict(gyro_rad, dt)

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

    def run(self):
        self._calib_acc = []
        self._calib_gyr = []

        if self.simulate:
            self._run_simulated_stream()
            return

        bus = None
        try:
            bus = self._open_can_bus()
            print(f"[CAN-FD] Connected to {self.interface}:{self.channel} (1M/5M BRS), waiting for ID=0x01...")
            while bus.recv(timeout=0.0) is not None:
                pass
        except Exception as e:
            print(f"[WARNING] Failed to open CAN-FD interface ({e}), falling back to simulation...")
            self._run_simulated_stream()
            return

        # Dedicated receive thread: it only does bus.recv() + decode + enqueue,
        # so the PCAN driver queue is drained at full speed and a slow ESKF or
        # GUI can never make the driver overflow and drop frames. The queue is
        # unbounded on purpose - losing a single sample would bias the accuracy
        # evaluation.
        self._raw_q = queue.Queue()
        recv_thread = threading.Thread(target=self._recv_loop, args=(bus,), daemon=True)
        recv_thread.start()

        rate_t0 = time.perf_counter()
        rate_cnt = 0

        try:
            while self.running:
                try:
                    seq, acc_g, gyro_rad, temp_c = self._raw_q.get(timeout=0.1)
                except queue.Empty:
                    continue
                self._process_raw_sample(seq, acc_g, gyro_rad, temp_c)
                rate_cnt += 1

                now = time.perf_counter()
                if now - rate_t0 >= 0.5:
                    with self.lock:
                        self.fps = rate_cnt / (now - rate_t0)
                    rate_cnt = 0
                    rate_t0 = now
        finally:
            self.running = False
            recv_thread.join(timeout=1.0)
            if bus is not None:
                bus.shutdown()

    def _recv_loop(self, bus):
        """Drain the CAN driver queue into the queue; never drop a frame."""
        while self.running:
            msg = bus.recv(timeout=0.05)
            if msg is None:
                continue
            if msg.arbitration_id != CAN_ID_IMU or len(msg.data) < FRAME_LEN:
                continue
            seq, ax, ay, az, gx, gy, gz, raw_temp = struct.unpack('>H7h', msg.data[:16])
            acc_g = np.array([ax, ay, az], dtype=np.float64) / ACCEL_SCALE
            gyro_rad = np.radians(np.array([gx, gy, gz], dtype=np.float64) / GYRO_SCALE)
            temp_c = raw_temp / TEMP_SCALE + TEMP_OFFSET
            self._raw_q.put((seq, acc_g, gyro_rad, temp_c))

    def _run_simulated_stream(self):
        seq = 0
        true_bg = np.radians([1.8, -1.2, 0.65])
        t_start = time.perf_counter()

        while self.running:
            now = time.perf_counter()
            t = now - t_start

            if t < 1.6:
                r_true, p_true, wx, wy, wz = 0.0, 0.0, 0.0, 0.0, 0.0
            else:
                tm = t - 1.6
                r_true = np.radians(28.0 * np.sin(1.5 * tm))
                p_true = np.radians(20.0 * np.cos(1.1 * tm))
                wx = np.radians(28.0 * 1.5 * np.cos(1.5 * tm))
                wy = np.radians(-20.0 * 1.1 * np.sin(1.1 * tm))
                wz = np.radians(15.0 * np.sin(0.6 * tm))

            cr, sr = np.cos(r_true), np.sin(r_true)
            cp, sp = np.cos(p_true), np.sin(p_true)
            acc_g = np.array([-sp, cp * sr, cp * cr]) + np.random.randn(3) * 0.015
            gyro_rad = np.array([wx, wy, wz]) + true_bg + np.random.randn(3) * np.radians(0.15)
            temp_c = 34.5 + 0.2 * np.sin(0.1 * t)

            self._process_raw_sample(seq, acc_g, gyro_rad, temp_c)
            seq = (seq + 1) & 0xFFFF
            with self.lock:
                self.fps = 800.0
            time.sleep(NOMINAL_DT)


# =====================================================================
# 3. PyQtGraph Live Dashboard
# =====================================================================
class OpenIMUDashboard(QtWidgets.QMainWindow):
    def __init__(self, receiver: OpenIMUReceiver, window_sec=5.0):
        super().__init__()
        self.receiver = receiver
        self.window_sec = window_sec
        self.setWindowTitle("open_imu (ICM-45686 @ 800Hz CAN-FD) — 3-State ESKF Live Monitor")
        self.resize(1100, 720)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        self.header = QtWidgets.QLabel("Waiting for CAN-FD stream & running static calibration (keep IMU still)...")
        self.header.setStyleSheet(
            "font-size: 15px; font-weight: bold; font-family: Consolas, monospace; "
            "padding: 10px; background-color: #1e1e2e; color: #f9e2af; border-radius: 6px;"
        )
        layout.addWidget(self.header)

        pg.setConfigOptions(antialias=True)

        self.plot_att = pg.PlotWidget(title="ESKF Euler Angles (Rightmost 0.0s = Latest Sample)")
        self.plot_att.setBackground('#181825')
        self.plot_att.showGrid(x=True, y=True, alpha=0.3)
        self.plot_att.setLabel('left', 'Angle', units='deg')
        self.plot_att.setYRange(-95, 95)
        self.plot_att.setXRange(-self.window_sec, 0.0, padding=0.0)
        self.plot_att.disableAutoRange()
        self.plot_att.addLegend(offset=(10, 10))
        layout.addWidget(self.plot_att, stretch=3)

        self.c_roll  = self.plot_att.plot(pen=pg.mkPen('#f38ba8', width=2.2), name="Roll (X)")
        self.c_pitch = self.plot_att.plot(pen=pg.mkPen('#a6e3a1', width=2.2), name="Pitch (Y)")
        self.c_yaw   = self.plot_att.plot(pen=pg.mkPen('#89b4fa', width=2.2), name="Yaw (Z)")

        self.plot_bias = pg.PlotWidget(title="Locked Static Gyro Bias bg (deg/s)")
        self.plot_bias.setBackground('#181825')
        self.plot_bias.showGrid(x=True, y=True, alpha=0.3)
        self.plot_bias.setLabel('left', 'Gyro Bias', units='deg/s')
        self.plot_bias.setLabel('bottom', 'Relative Time (0.0s = Latest)', units='s')
        self.plot_bias.setXRange(-self.window_sec, 0.0, padding=0.0)
        self.plot_bias.enableAutoRange(axis='y')
        self.plot_bias.addLegend(offset=(10, 10))
        layout.addWidget(self.plot_bias, stretch=2)

        self.c_bx = self.plot_bias.plot(pen=pg.mkPen('#fab387', width=1.8), name="bg_x")
        self.c_by = self.plot_bias.plot(pen=pg.mkPen('#cba6f7', width=1.8), name="bg_y")
        self.c_bz = self.plot_bias.plot(pen=pg.mkPen('#94e2d5', width=1.8), name="bg_z")

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(15)

    def closeEvent(self, event):
        self.receiver.running = False
        event.accept()

    def update_plots(self):
        if not self.receiver.eskf.initialized:
            pct = int(self.receiver.calib_progress * 100)
            self.header.setText(f"[Calibrating {pct:3d}%] Collecting 800Hz static samples for initial attitude & gyro bias...")
            return

        with self.receiver.lock:
            if not self.receiver.t_buf:
                return
            t  = np.array(self.receiver.t_buf)
            r  = np.array(self.receiver.roll_buf)
            p  = np.array(self.receiver.pitch_buf)
            y  = np.array(self.receiver.yaw_buf)
            bx = np.array(self.receiver.bx_buf)
            by = np.array(self.receiver.by_buf)
            bz = np.array(self.receiver.bz_buf)
            fps = self.receiver.fps
            dropped = self.receiver.dropped_frames
            temp_c = self.receiver.temp_c
            acc_norm = self.receiver.acc_norm_g

        t_rel = t - t[-1]

        self.c_roll.setData(t_rel, r)
        self.c_pitch.setData(t_rel, p)
        self.c_yaw.setData(t_rel, y)

        self.c_bx.setData(t_rel, bx)
        self.c_by.setData(t_rel, by)
        self.c_bz.setData(t_rel, bz)

        self.header.setStyleSheet(
            "font-size: 14px; font-weight: bold; font-family: Consolas, monospace; "
            "padding: 10px; background-color: #1e1e2e; color: #cdd6f4; border-radius: 6px;"
        )
        self.header.setText(
            f"Roll:{r[-1]:+7.2f} deg | Pitch:{p[-1]:+7.2f} deg | Yaw:{y[-1]:+7.2f} deg  ||  "
            f"|a|:{acc_norm:4.2f}g | Temp:{temp_c:4.1f} C | ODR:{fps:5.0f}Hz | Dropped:{dropped}"
        )


if __name__ == '__main__':
    is_win = sys.platform.startswith('win')
    parser = argparse.ArgumentParser(description="open_imu (ICM-45686 CAN-FD) ESKF Live Monitor")
    parser.add_argument('-i', '--interface', default='pcan' if is_win else 'socketcan',
                        help="CAN backend (default: pcan on Windows, socketcan on Linux)")
    parser.add_argument('-c', '--channel', default='PCAN_USBBUS1' if is_win else 'can0',
                        help="CAN channel (default: PCAN_USBBUS1 on Windows, can0 on Linux)")
    parser.add_argument('--f-clock', type=int, default=80, help="PCAN clock in MHz (default: 80)")
    parser.add_argument('--calib-seconds', type=float, default=1.5, help="Startup static calibration seconds")
    parser.add_argument('--history', type=float, default=5.0, help="Sliding window length in seconds")
    parser.add_argument('--simulate', action='store_true', help="Run in 800Hz simulation mode without hardware")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    eskf = ICM45686_ESKF()
    rx = OpenIMUReceiver(
        eskf,
        interface=args.interface,
        channel=args.channel,
        f_clock_mhz=args.f_clock,
        calib_sec=args.calib_seconds,
        window_sec=args.history,
        simulate=args.simulate
    )
    rx.start()

    gui = OpenIMUDashboard(rx, window_sec=args.history)
    gui.show()
    sys.exit(app.exec())