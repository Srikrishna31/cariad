//
// Created by krishnachaa on 06.11.25.
//

#include "Transformation.h"
#include <Eigen/Dense>

#include "Object.h"

Eigen::Matrix4f Transformation::CreateTransformationMatrix(float x_e, float y_e, float z_e,
                                                           float roll, float pitch, float yaw)
{
    Eigen::Matrix3f Rx, Ry, Rz;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);

    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;

    Eigen::Matrix3f R = Rz * Ry * Rx;

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = R;
    T.block<1,3>(3,0) << x_e, y_e, z_e;

    return T;
}


Transformation::Transformation(float x_e, float y_e, float z_e, 
    float roll, float pitch, float yaw) 
{
    matrix_ = Transformation::CreateTransformationMatrix(x_e, y_e, z_e, roll, pitch, yaw);
}

auto Transformation::transform(const Object& point) const -> Object
{
    return point * (*this);
}

auto operator*(const Transformation& t) -> Transformation;
