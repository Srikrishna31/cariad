//
// Created by krishnachaa on 06.11.25.
//

#include "Transformation.h"
#include <Eigen/Dense>

#include "Object.h"

Eigen::Matrix4f CreateTransformationMatrix(float x_e, float y_e, float z_e,
                                           float yaw, float pitch, float roll)
{
      Eigen::Matrix3f Rx, Ry, Rz;
      Rx << 1, 0, 0,
          0, cos(yaw), -sin(yaw),
          0, sin(yaw), cos(yaw);

      Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);

      Rz << cos(roll), -sin(roll), 0,
          sin(roll), cos(roll), 0,
          0, 0, 1;

      Eigen::Matrix3f R = Rz * Ry * Rx;

      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block<3, 3>(0, 0) = R;
      T.block<3, 1>(0, 3) << x_e, y_e, z_e;

      return T;
}

Transformation::Transformation(float x_e, float y_e, float z_e, 
    float yaw, float pitch, float roll)
{
    matrix_ = CreateTransformationMatrix(x_e, y_e, z_e, yaw, pitch, roll);
}

auto Transformation::transform(const Object& point) const -> Object
{
    return point * (*this);
}

Transformation::Transformation(Eigen::Matrix4f matrix)
    : matrix_(std::move(matrix))
{}


auto Transformation::operator*(const Transformation& t) const -> Transformation
{
    Eigen::Matrix4f mat = t.matrix_ * matrix_;

    return Transformation{std::move(mat)};
}
