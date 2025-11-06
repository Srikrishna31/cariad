//
// Created by krishnachaa on 06.11.25.
//
#include <memory>
#include <Eigen/Dense>
#include <iostream>

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

int main(int argc, char **argv)
{

      // Eigen::Vector3f ego_pos(1.f, 0.f, 0.f);
      Eigen::Vector3f ego_pos(1.828963f, -138.407091f, -0.006f);
      Eigen::Vector3f ego_hpr(1.571584f, -0.004266f, -0.000046f);
      // Eigen::Vector3f ego_hpr(0.f, 0.f, 0.f);
      //  Eigen::Vector3f ego_hpr(-0.000046f, -0.004266f, 1.571584f);

      auto transformation_mtrx = CreateTransformationMatrix(ego_pos(0), ego_pos(1), ego_pos(2),
                                                            ego_hpr(2), ego_hpr(1), ego_hpr(0));

      std::cout << "Transformation Matrix:\n"
                << transformation_mtrx << std::endl;
      Eigen::Vector4f obj_pos(4.985880f, 112.518f, 0.0f, 1.0f);
      // Eigen::Vector4f obj_pos(0.f, 0.0f, 0.0f, 1.0f);

      auto transformed_pos = transformation_mtrx.inverse() * obj_pos;

      std::cout << "Original Point: " << obj_pos.transpose() << "\n";
      std::cout << "Transformed Point: " << transformed_pos.transpose() << "\n";
}
