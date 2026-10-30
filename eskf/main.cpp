// SPDX-License-Identifier: MIT
//
// main.cpp - closed-loop demo for eskf.hpp.
//
// A point-mass "robot" drives a circle of radius R at speed v.  We synthesise
// an 800 Hz IMU stream (with constant gyro/accel biases and white noise) and a
// 10 Hz position observation (GNSS-like), feed both into the ESKF and compare
// it with naive IMU dead-reckoning.
//
// Build (needs Eigen 3):
//   g++ -O2 -std=c++17 -I<eigen> -I. main.cpp -o eskf_demo
//   ./eskf_demo
//
// With CMake: see CMakeLists.txt in this folder.

#include "eskf.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
#include <random>

using openimu::ESKF;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

struct Traj {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Matrix3d R;
};

// Truth trajectory: a circle in the X-Y plane, constant speed, nose pointing
// along the velocity (roll = pitch = 0, yaw = tangent angle).
Traj truth(double t, double radius, double speed) {
    const double w = speed / radius;
    const double th = w * t;
    Traj s;
    s.p = Eigen::Vector3d(radius * std::cos(th), radius * std::sin(th), 0.0);
    s.v = Eigen::Vector3d(-speed * std::sin(th), speed * std::cos(th), 0.0);
    const double yaw = th + 0.5 * kPi;
    s.R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return s;
}

// ZYX Euler angles in degrees, for the CSV / plot.
Eigen::Vector3d euler_deg(const Eigen::Matrix3d& R) {
    const double roll = std::atan2(R(2, 1), R(2, 2));
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw) * (180.0 / kPi);
}

}  // namespace

int main() {
    const double dt = 1.0 / 800.0;       // IMU rate
    const double duration = 60.0;        // seconds
    const double radius = 10.0;          // m
    const double speed = 3.0;            // m/s
    const int gnss_every = 80;           // 10 Hz position fixes

    // Biases the filter has to learn.
    const Eigen::Vector3d gyro_bias(0.010, -0.008, 0.012);    // rad/s
    const Eigen::Vector3d accel_bias(0.10, -0.05, 0.08);      // m/s^2
    const Eigen::Vector3d gravity(0.0, 0.0, -kGravity);

    std::mt19937 rng(42);
    std::normal_distribution<double> n_gyro(0.0, 0.002);      // rad/s
    std::normal_distribution<double> n_accel(0.0, 0.05);      // m/s^2
    std::normal_distribution<double> n_gnss(0.0, 0.10);       // m

    ESKF::Options opt;
    opt.gyro_var = 0.002 * 0.002;
    opt.accel_var = 0.05 * 0.05;
    opt.gyro_bias_var = 1e-10;   // the bias is constant here: keep it almost fixed
    opt.accel_bias_var = 1e-8;
    ESKF ekf(opt);

    const Traj t0 = truth(0.0, radius, speed);
    ESKF::State init;
    init.p = t0.p;
    init.v = t0.v;
    init.R = t0.R;
    init.g = gravity;
    ekf.reset(init);

    // Naive dead-reckoning (biases included, no observations).
    Eigen::Vector3d p_dr = t0.p;
    Eigen::Vector3d v_dr = t0.v;
    Eigen::Matrix3d R_dr = t0.R;

    double sq_err_ekf = 0.0;
    double sq_err_dr = 0.0;
    int steps = static_cast<int>(duration / dt);

    // CSV for the Python plot (tools/eskf_plot.py).
    std::FILE* csv = std::fopen("eskf_traj.csv", "w");
    if (csv != nullptr) {
        std::fprintf(csv, "t,px,py,pz,ex,ey,ez,dx,dy,dz,rx,ry,rz,"
                          "bgx,bgy,bgz,bax,bay,baz,sx,sy,sz\n");
    }

    for (int i = 0; i < steps; ++i) {
        const double t = i * dt;
        const Traj tr = truth(t, radius, speed);

        // --- synthetic IMU ------------------------------------------------
        const double w = speed / radius;
        const Eigen::Vector3d gyro =
            Eigen::Vector3d(0.0, 0.0, w) + gyro_bias + Eigen::Vector3d(n_gyro(rng), n_gyro(rng), n_gyro(rng));

        const Eigen::Vector3d a_world = -w * w * radius * Eigen::Vector3d(std::cos(w * t), std::sin(w * t), 0.0);
        const Eigen::Vector3d accel =
            tr.R.transpose() * (a_world - gravity) + accel_bias +
            Eigen::Vector3d(n_accel(rng), n_accel(rng), n_accel(rng));

        // --- ESKF ---------------------------------------------------------
        ekf.predict(gyro, accel, dt);

        // --- naive integration -------------------------------------------
        {
            const Eigen::Vector3d a = R_dr * accel + gravity;
            p_dr += v_dr * dt + 0.5 * a * dt * dt;
            v_dr += a * dt;
            R_dr = R_dr * openimu::expSO3(gyro * dt);
        }

        // --- 10 Hz position observation ----------------------------------
        if (i > 0 && i % gnss_every == 0) {
            const Eigen::Vector3d meas = tr.p + Eigen::Vector3d(n_gnss(rng), n_gnss(rng), n_gnss(rng));
            ekf.updatePosition(meas, 0.10);
        }

        sq_err_ekf += (ekf.state().p - tr.p).squaredNorm();
        sq_err_dr += (p_dr - tr.p).squaredNorm();

        if (csv != nullptr && (i % 8) == 0) {
            const auto& st = ekf.state();
            const auto sig = ekf.sigma();
            const Eigen::Vector3d e = euler_deg(st.R);
            std::fprintf(csv,
                         "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                         "%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                         t, tr.p.x(), tr.p.y(), tr.p.z(),
                         st.p.x(), st.p.y(), st.p.z(),
                         p_dr.x(), p_dr.y(), p_dr.z(),
                         e.x(), e.y(), e.z(),
                         st.bg.x(), st.bg.y(), st.bg.z(),
                         st.ba.x(), st.ba.y(), st.ba.z(),
                         sig(0), sig(1), sig(2));
        }
    }

    if (csv != nullptr) {
        std::fclose(csv);
    }

    const double rmse_ekf = std::sqrt(sq_err_ekf / steps);
    const double rmse_dr = std::sqrt(sq_err_dr / steps);
    const auto& s = ekf.state();

    std::printf("duration           : %.1f s\n", duration);
    std::printf("position RMSE      : ESKF %.3f m   dead-reckoning %.3f m\n", rmse_ekf, rmse_dr);
    std::printf("final position err : ESKF %.3f m   dead-reckoning %.3f m\n",
                (s.p - truth(duration, radius, speed).p).norm(),
                (p_dr - truth(duration, radius, speed).p).norm());
    std::printf("gyro bias  est/true: [%+.4f %+.4f %+.4f] / [%+.4f %+.4f %+.4f] rad/s\n",
                s.bg.x(), s.bg.y(), s.bg.z(), gyro_bias.x(), gyro_bias.y(), gyro_bias.z());
    std::printf("accel bias est/true: [%+.4f %+.4f %+.4f] / [%+.4f %+.4f %+.4f] m/s^2\n",
                s.ba.x(), s.ba.y(), s.ba.z(), accel_bias.x(), accel_bias.y(), accel_bias.z());
    std::printf("gravity            : [%+.3f %+.3f %+.3f] m/s^2\n", s.g.x(), s.g.y(), s.g.z());
    return 0;
}
