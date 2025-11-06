#include <iostream>
#include <Eigen/Dense>
#include <cmath>

struct Pose6D {
    double x{0}, y{0}, z{0};  // translation
    double h{0}, p{0}, r{0};  // yaw(Z), pitch(Y), roll(X) in radians
};

inline double wrapToPi(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}

// Basic rotation matrices
inline Eigen::Matrix3d Rz(double h) {
    const double c = std::cos(h), s = std::sin(h);
    Eigen::Matrix3d R;
    R <<  c, -s, 0,
          s,  c, 0,
          0,  0, 1;
    return R;
}

inline Eigen::Matrix3d Ry(double p) {
    const double c = std::cos(p), s = std::sin(p);
    Eigen::Matrix3d R;
    R <<  c, 0,  s,
          0, 1,  0,
         -s, 0,  c;
    return R;
}

inline Eigen::Matrix3d Rx(double r) {
    const double c = std::cos(r), s = std::sin(r);
    Eigen::Matrix3d R;
    R << 1,  0,  0,
         0,  c, -s,
         0,  s,  c;
    return R;
}

// Intrinsic ZYX = Rz(h)*Ry(p)*Rx(r)
inline Eigen::Matrix3d rotZYX(double h, double p, double r) {
    return Rz(h) * Ry(p) * Rx(r);
}

// Build 4x4 homogeneous transform from Pose6D
inline Eigen::Matrix4d T_from_poseZYX(const Pose6D& pose) {
    Eigen::Matrix3d R = rotZYX(pose.h, pose.p, pose.r);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = Eigen::Vector3d(pose.x, pose.y, pose.z);
    return T;
}

// Inverse of homogeneous transform: [R t; 0 1]^{-1} = [R^T -R^T t; 0 1]
inline Eigen::Matrix4d T_inverse(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    Eigen::Matrix3d Rt = R.transpose();
    Eigen::Matrix4d Tin = Eigen::Matrix4d::Identity();
    Tin.block<3,3>(0,0) = Rt;
    Tin.block<3,1>(0,3) = -Rt * t;
    return Tin;
}

// Robust ZYX (yaw, pitch, roll) extraction from rotation matrix
inline Eigen::Vector3d eulerZYX_from_R(const Eigen::Matrix3d& R) {
    // ZYX convention:
    // R(2,0) = -sin(pitch)
    const double sy = -R(2,0);
    const double pitch = std::asin(sy);
    const double cp = std::cos(pitch);
    double yaw, roll;

    const double eps = 1e-9;
    if (std::abs(cp) > eps) {
        roll = std::atan2(R(2,1), R(2,2));
        yaw  = std::atan2(R(1,0), R(0,0));
    } else {
        // Gimbal lock: cp ≈ 0 → pitch ≈ ±pi/2
        roll = 0.0;
        if (sy < 0) {
            // pitch ≈ +pi/2 (since sy = -sin(pitch) ≈ -1)
            yaw = std::atan2(-R(0,1), R(1,1));
        } else {
            // pitch ≈ -pi/2 (sy ≈ +1)
            yaw = std::atan2(R(0,1), -R(1,1));
        }
    }
    return Eigen::Vector3d(wrapToPi(yaw), wrapToPi(pitch), wrapToPi(roll));
}

// Convert homogeneous 4x4 back to Pose6D (ZYX)
inline Pose6D poseZYX_from_T(const Eigen::Matrix4d& T) {
    Pose6D out;
    out.x = T(0,3);
    out.y = T(1,3);
    out.z = T(2,3);
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d hpr = eulerZYX_from_R(R);
    out.h = hpr[0]; // yaw
    out.p = hpr[1]; // pitch
    out.r = hpr[2]; // roll
    return out;
}

// Core: transform object pose (world) into ego local frame using only matrix multiplications.
//   T_EO = inv(T_WE) * T_WO
inline Pose6D objectPoseInEgoZYX(const Pose6D& ego_in_world, const Pose6D& obj_in_world) {
    Eigen::Matrix4d T_WE = T_from_poseZYX(ego_in_world);
    Eigen::Matrix4d T_WO = T_from_poseZYX(obj_in_world);
    Eigen::Matrix4d T_EO = T_inverse(T_WE) * T_WO; // simple 4x4 multiply
    return poseZYX_from_T(T_EO);
}

// If you only need to transform a point (no orientation):
inline Eigen::Vector3d pointInEgo(const Pose6D& ego_in_world, const Eigen::Vector3d& p_world) {
    Eigen::Matrix4d T_WE = T_from_poseZYX(ego_in_world);
    Eigen::Matrix4d T_EW = T_inverse(T_WE);
    Eigen::Vector4d pw(p_world[0], p_world[1], p_world[2], 1.0);
    Eigen::Vector4d pe = T_EW * pw;
    return pe.head<3>();
}

int main() {
    // Ego pose in world: (10, 0, 0), yaw=+90° (pi/2), level
    // Pose6D egoW{10.0, 0.0, 0.0, M_PI/2, 0.0, 0.0};
    Pose6D egoW{1.828963f, -138.407091f, -0.006f, 1.571584f, -0.004266f, -0.000046f};

    // Object pose in world: (12, 3, 0), yaw=+45° (pi/4), level
    // Pose6D objW{12.0, 3.0, 0.0, M_PI/4, 0.0, 0.0};
    Pose6D objW{4.985880f, 112.518f, 0.0f, 0.f, 0.0, 0.0};

    // Transform to ego local frame
    Pose6D objE = objectPoseInEgoZYX(egoW, objW);

    std::cout << "Object pose in Ego frame:\n";
    std::cout << "  t = [" << objE.x << ", " << objE.y << ", " << objE.z << "]\n";
    std::cout << "  yaw(h) = " << objE.h << " rad, pitch(p) = " << objE.p
              << " rad, roll(r) = " << objE.r << " rad\n";

    return 0;
}
