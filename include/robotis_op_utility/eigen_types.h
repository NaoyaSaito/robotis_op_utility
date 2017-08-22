// copy from choreonoid

#ifndef ROBOTIS_OP_UTILITY_EIGEN_TYPES_H
#define ROBOTIS_OP_UTILITY_EIGEN_TYPES_H

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <Eigen/AlignedVector3>

namespace robotis{

using Eigen::Vector2i;
using Eigen::Matrix2f;
using Eigen::Vector2f;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Array2i;
using Eigen::Array2f;
using Eigen::Array2d;

using Eigen::Vector3i;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Array3i;
using Eigen::Array3f;
using Eigen::Array3d;

using Eigen::Matrix4f;
using Eigen::Vector4f;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Array4i;
using Eigen::Array4f;
using Eigen::Array4d;

using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::AngleAxisf;
using Eigen::Quaternionf;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;

using Eigen::Affine3f;
using Eigen::Affine3d;
using Eigen::Translation3f;
using Eigen::Translation3d;

typedef Eigen::Matrix2d Matrix2;
typedef Eigen::Vector2d Vector2;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Vector3d Vector3;
//typedef Eigen::AlignedVector3<double> Vector3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::Vector4d Vector4;
typedef Eigen::VectorXd VectorX;
typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Affine3d Affine3;
typedef Eigen::Translation3d Translation3;
typedef Eigen::AngleAxisd AngleAxis;

//! \deprecated
typedef Eigen::Quaterniond Quat;

typedef Eigen::Quaterniond Quaternion;

typedef Eigen::Transform<double, 3, Eigen::AffineCompact> EPosition;

// The followings should be removed later
using Eigen::Isometry3f;
using Eigen::Isometry3d;
typedef Eigen::Isometry3d EIsometry3;

constexpr double PI = 3.14159265358979323846;
constexpr double PI_2 = 1.57079632679489661923;

constexpr double TO_DEGREE = 180.0 / PI;
constexpr double TO_RADIAN = PI / 180.0;

inline double rad2deg(double rad) { return TO_DEGREE * rad; }
inline double deg2rad(double deg) { return TO_RADIAN * deg; }
inline double deg2rad(int deg) { return TO_RADIAN * deg; }

inline Quaternion rpy2q(double roll, double pitch, double yaw) {
  AngleAxisd rollAngle(roll, Vector3d::UnitZ());
  AngleAxisd yawAngle(yaw, Vector3d::UnitY());
  AngleAxisd pitchAngle(pitch, Vector3d::UnitX());

  Quaternion q = rollAngle * yawAngle * pitchAngle;
  return q;
}

inline Matrix3 rpy2mat(double roll, double pitch, double yaw) {
  AngleAxisd rollAngle(roll, Vector3d::UnitZ());
  AngleAxisd yawAngle(yaw, Vector3d::UnitY());
  AngleAxisd pitchAngle(pitch, Vector3d::UnitX());

  Quaternion q = rollAngle * yawAngle * pitchAngle;
  Matrix3 rotationMatrix = q.matrix();
  return rotationMatrix;
}

inline Vector3 rot2rqy(Matrix3 mat) { return mat.eulerAngles(0, 1, 2); }

class Pose {
    Vector3 pp;
    Quat qq;
public:
    Pose() { }
    Pose(const Vector3& translation, const Quat& rotation)
        : pp(translation), qq(rotation) { }
    Pose(const Vector3& translation, const Matrix3& rotation)
        : pp(translation), qq(rotation) { }

    void set(const Vector3& translation, const Quat& rotation) {
        this->pp = translation;
        this->qq = rotation;
    }
    void set(const Vector3& translation, const Matrix3& R) {
        this->pp = translation;
        this->qq = R;
    }
    Vector3& p() { return pp; }
    const Vector3& p() const { return pp; }
    Quat& q() { return qq; }
    const Quat& q() const { return qq; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif