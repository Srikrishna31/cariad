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
    explicit Transformation(float x_e, float y_e, float z_e,
            float roll, float pitch, float yaw);
    explicit Transformation(Eigen::Matrix4f matrix);
        Transformation(const Transformation&) = default;
    Transformation(Transformation&&) = default;
    Transformation& operator=(const Transformation&) = default;
    Transformation& operator=(Transformation&&) = default;
    ~Transformation() = default;

    /**
     * Apply the current transformation to a given object, and return the
     * transformed object.
     * @param point : representing the object in global coordinates
     * @return transformed object.
     */
    // [[nodiscard]] auto transform(const Object& point) const -> Object;
    void transform( Object& point);

    /**
     * Convenience operator to stack multiple transformation objects together.
     * @param t: The transformation object which needs to be stacked upon.
     * @return combined transformation object.
     */
    auto operator*(const Transformation& t) const -> Transformation;

    /**
     * Get the underlying eigen matrix, for manipulation.
     * @return Eigen::Matrix4f
     */
    [[nodiscard]] auto matrix() const -> Eigen::Matrix4f {return matrix_;}

    static Eigen::Matrix4f CreateTransformationMatrix(float x_e, float y_e, float z_e,
                                           float yaw, float pitch, float roll);
private:

    Eigen::Matrix4f matrix_;
};

#endif //CARIAD_TRANSFORMATION_H