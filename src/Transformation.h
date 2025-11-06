//
// Created by krishnachaa on 06.11.25.
//

#ifndef CARIAD_TRANSFORMATION_H
#define CARIAD_TRANSFORMATION_H

#include <Eigen/Eigen>

class Object;

class Transformation
{
public:
    Transformation(float x_e, float y_e, float z_e,
            float roll, float pitch, float yaw);
    Transformation(const Transformation&) = default;
    Transformation(Transformation&&) = default;
    Transformation& operator=(const Transformation&) = default;
    Transformation& operator=(Transformation&&) = default;
    ~Transformation() = default;

    auto transform(const Object& point) const -> Object;

    auto operator*(const Transformation& t) -> Transformation;

    static Eigen::Matrix4f CreateTransformationMatrix(float x_e, float y_e, float z_e,
            float roll, float pitch, float yaw);
private:
    Eigen::Matrix4f matrix_;
};


#endif //CARIAD_TRANSFORMATION_H