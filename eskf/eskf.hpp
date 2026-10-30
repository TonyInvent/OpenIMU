// SPDX-License-Identifier: MIT
//
// eskf.hpp - error-state Kalman filter for IMU + external pose/velocity
//            observations (18-dimensional error state, Eigen only).
//
// Why an ESKF instead of a plain EKF:
//   The attitude is a unit quaternion living on S^3.  Putting the 4 numbers
//   directly into a covariance matrix makes it singular (the norm is fixed,
//   rank 3).  The standard trick is to keep a *nominal* state that integrates
//   the quaternion without constraints, and let the Kalman filter estimate a
//   3-dimensional rotation error on the tangent space so(3).  That is what
//   this file does.
//
// Nominal state (integrated without constraints):
//     p   world position                     [m]
//     v   world velocity                     [m/s]
//     R   body -> world rotation             SO(3)
//     bg  gyro bias                          [rad/s]
//     ba  accel bias                         [m/s^2]
//     g   gravity in the world frame         [m/s^2]
//
// Error state (the Kalman state, 18x1):
//     dx = [ dp, dv, dtheta, dbg, dba, dg ]^T
//   with the rotation error defined on the right (body frame):
//     R_true = R * Exp(dtheta)
//
// References:
//   - J. Sola, "Quaternion kinematics for the error-state Kalman filter",
//     arXiv:1711.02508 (the canonical derivation of Exp/Log, F and reset).
//   - Gao Xiang, "slam_in_autonomous_driving", src/ch3/eskf.hpp (the same
//     18-state layout and the right-perturbation Jacobians used below).
//
// Deliberately dependency-free apart from Eigen: the so(3) helpers are
// implemented here, so there is no Sophus requirement.

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace openimu {

// ---------------------------------------------------------------------------
// so(3) helpers
// ---------------------------------------------------------------------------
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return m;
}

// Rodrigues / exponential map, so(3) -> SO(3).
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) {
    const double theta = w.norm();
    const Eigen::Matrix3d W = hat(w);
    if (theta < 1e-10) {
        return Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
    }
    return Eigen::Matrix3d::Identity()
         + (std::sin(theta) / theta) * W
         + ((1.0 - std::cos(theta)) / (theta * theta)) * W * W;
}

// Logarithmic map, SO(3) -> so(3).
inline Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
    const double c = std::min(1.0, std::max(-1.0, 0.5 * (R.trace() - 1.0)));
    const double theta = std::acos(c);
    const Eigen::Vector3d v(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    if (theta < 1e-10) {
        return 0.5 * v;
    }
    return (theta / (2.0 * std::sin(theta))) * v;
}

// ---------------------------------------------------------------------------
// ESKF
// ---------------------------------------------------------------------------
class ESKF {
public:
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;
    using Vec18 = Eigen::Matrix<double, 18, 1>;
    using Mat18 = Eigen::Matrix<double, 18, 18>;
    using Mat3x18 = Eigen::Matrix<double, 3, 18>;
    using Mat6x18 = Eigen::Matrix<double, 6, 18>;

    struct State {
        Vec3 p = Vec3::Zero();
        Vec3 v = Vec3::Zero();
        Mat3 R = Mat3::Identity();
        Vec3 bg = Vec3::Zero();
        Vec3 ba = Vec3::Zero();
        Vec3 g = Vec3(0.0, 0.0, -9.80665);
    };

    // Noise terms are *discrete-time* variances per IMU sample (that is how
    // the reference implementation defines them: they are already integrated
    // over the sample interval, so they must not be multiplied by dt again).
    struct Options {
        double gyro_var = 1e-5;        // rad/s
        double accel_var = 1e-2;       // m/s^2
        double gyro_bias_var = 1e-6;   // rad/s
        double accel_bias_var = 1e-4;  // m/s^2
        double gravity_var = 0.0;      // 0 = keep gravity fixed
        bool estimate_gyro_bias = true;
        bool estimate_accel_bias = true;
    };

    ESKF() { buildNoise(); }
    explicit ESKF(const Options& opt) : opt_(opt) { buildNoise(); }

    // Re-initialise the nominal state and the covariance.
    void reset(const State& state, double pos_var = 1e-4, double vel_var = 1e-4,
               double ang_var = 1e-4) {
        x_ = state;
        dx_.setZero();
        P_.setZero();
        P_.block<3, 3>(0, 0) = Mat3::Identity() * pos_var;
        P_.block<3, 3>(3, 3) = Mat3::Identity() * vel_var;
        P_.block<3, 3>(6, 6) = Mat3::Identity() * ang_var;
        P_.block<3, 3>(9, 9) = Mat3::Identity() * 1e-2;
        P_.block<3, 3>(12, 12) = Mat3::Identity() * 1e-2;
        P_.block<3, 3>(15, 15) = Mat3::Identity() * 1e-4;
    }

    // IMU prediction.  gyro [rad/s], accel [m/s^2] (specific force), dt [s].
    void predict(const Vec3& gyro, const Vec3& accel, double dt) {
        if (dt <= 0.0) {
            return;
        }

        // --- nominal state ------------------------------------------------
        const Vec3 a_world = x_.R * (accel - x_.ba) + x_.g;
        x_.p += x_.v * dt + 0.5 * a_world * dt * dt;
        x_.v += a_world * dt;
        x_.R = x_.R * expSO3((gyro - x_.bg) * dt);

        // --- error-state transition --------------------------------------
        // Sola eq. (269) / Gao ch3 (3.47), right perturbation.
        Mat18 F = Mat18::Identity();
        F.block<3, 3>(0, 3) = Mat3::Identity() * dt;                        // dp  <- dv
        F.block<3, 3>(3, 6) = -x_.R * hat(accel - x_.ba) * dt;              // dv  <- dtheta
        F.block<3, 3>(3, 12) = -x_.R * dt;                                  // dv  <- dba
        F.block<3, 3>(3, 15) = Mat3::Identity() * dt;                       // dv  <- dg
        F.block<3, 3>(6, 6) = expSO3(-(gyro - x_.bg) * dt);                 // dtheta <- dtheta
        F.block<3, 3>(6, 9) = -Mat3::Identity() * dt;                       // dtheta <- dbg

        dx_ = F * dx_;                       // stays zero between updates
        P_ = F * P_ * F.transpose() + Q_;
    }

    // External observation: full pose (position + attitude), e.g. LiDAR / VIO
    // / leg-kinematics odometry expressed in the world frame.
    void updatePose(const Mat3& R_meas, const Vec3& p_meas,
                    double pos_sigma, double ang_sigma) {
        Mat6x18 H = Mat6x18::Zero();
        H.block<3, 3>(0, 0) = Mat3::Identity();   // position
        H.block<3, 3>(3, 6) = Mat3::Identity();   // attitude (right error)

        Eigen::Matrix<double, 6, 6> V = Eigen::Matrix<double, 6, 6>::Zero();
        V.diagonal() << pos_sigma * pos_sigma, pos_sigma * pos_sigma, pos_sigma * pos_sigma,
                        ang_sigma * ang_sigma, ang_sigma * ang_sigma, ang_sigma * ang_sigma;

        const Eigen::Matrix<double, 18, 6> K =
            P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();

        Eigen::Matrix<double, 6, 1> innov;
        innov.head<3>() = p_meas - x_.p;
        innov.tail<3>() = logSO3(x_.R.transpose() * R_meas);

        dx_ = K * innov;
        P_ = (Mat18::Identity() - K * H) * P_;
        injectAndReset();
    }

    // External observation: position only (GNSS / RTK / mocap).
    void updatePosition(const Vec3& p_meas, double sigma) {
        Mat3x18 H = Mat3x18::Zero();
        H.block<3, 3>(0, 0) = Mat3::Identity();
        const Mat3 V = Mat3::Identity() * sigma * sigma;
        const Eigen::Matrix<double, 18, 3> K =
            P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();
        dx_ = K * (p_meas - x_.p);
        P_ = (Mat18::Identity() - K * H) * P_;
        injectAndReset();
    }

    // External observation: linear velocity (wheel odometry / leg kinematics).
    void updateVelocity(const Vec3& v_meas, double sigma) {
        Mat3x18 H = Mat3x18::Zero();
        H.block<3, 3>(3, 3) = Mat3::Identity();
        const Mat3 V = Mat3::Identity() * sigma * sigma;
        const Eigen::Matrix<double, 18, 3> K =
            P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();
        dx_ = K * (v_meas - x_.v);
        P_ = (Mat18::Identity() - K * H) * P_;
        injectAndReset();
    }

    // Zero-velocity update: a cheap "the robot is standing still" pseudo
    // measurement, very effective for legged robots.
    void updateZeroVelocity(double sigma = 0.01) {
        updateVelocity(Vec3::Zero(), sigma);
    }

    const State& state() const { return x_; }
    const Mat18& cov() const { return P_; }

    // Standard deviations of the current error state (diagnostics).
    Eigen::Matrix<double, 18, 1> sigma() const { return P_.diagonal().cwiseSqrt(); }

private:
    void buildNoise() {
        Q_.setZero();
        const double sv = opt_.accel_var;
        const double st = opt_.gyro_var;
        const double sg = opt_.gyro_bias_var;
        const double sa = opt_.accel_var * 0.0 + opt_.accel_bias_var;
        const double sgr = opt_.gravity_var;
        Q_.diagonal() << 0.0, 0.0, 0.0,
                         sv, sv, sv,
                         st, st, st,
                         sg, sg, sg,
                         sa, sa, sa,
                         sgr, sgr, sgr;
    }

    // Inject the error state into the nominal state, then reset it to zero and
    // propagate the covariance through the reset (Sola eq. 281 / Gao (3.63)).
    void injectAndReset() {
        x_.p += dx_.block<3, 1>(0, 0);
        x_.v += dx_.block<3, 1>(3, 0);
        x_.R = x_.R * expSO3(dx_.block<3, 1>(6, 0));
        if (opt_.estimate_gyro_bias) {
            x_.bg += dx_.block<3, 1>(9, 0);
        }
        if (opt_.estimate_accel_bias) {
            x_.ba += dx_.block<3, 1>(12, 0);
        }
        x_.g += dx_.block<3, 1>(15, 0);

        Mat18 J = Mat18::Identity();
        J.block<3, 3>(6, 6) = Mat3::Identity() - 0.5 * hat(dx_.block<3, 1>(6, 0));
        P_ = J * P_ * J.transpose();

        dx_.setZero();
    }

    Options opt_;
    State x_;
    Vec18 dx_ = Vec18::Zero();
    Mat18 P_ = Mat18::Identity() * 1e-4;
    Mat18 Q_ = Mat18::Zero();
};

}  // namespace openimu
