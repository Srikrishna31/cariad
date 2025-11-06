//
// Created by krishnachaa on 06.11.25.
//

#ifndef CARIAD_OBJECT_H
#define CARIAD_OBJECT_H

#include <Eigen/Eigen>

#include "types.h"

class Transformation;

class Object {
public:
    Object(Position pos, Movement rot, std::string name);
    Object(const Object&) = default;
    Object(Object&&) = default;
    Object& operator=(const Object&) = default;
    Object& operator=(Object&&) = default;
    ~Object() = default;

    /**
     * Get the transformation matrix, which encodes the position and orientation
     * information of the object wrt Global coordinates.
     * @return Transformation
     */
    [[nodiscard]] auto get_tranformation_matrix() const -> Transformation;

    /**
     * Mul operator, which applies the given transformation to the object, and returns
     * the transformed object.
     * @param t
     * @return Object
     */
    auto operator*(const Transformation& t) const -> Object;

    /**
     * Equality comparison of two objects
     * @param other
     * @return bool
     */
    auto operator==(const Object& other) const -> bool;
private:
    Eigen::Vector4f position;
    Eigen::Vector4f rotation;
    Position position_world;
    Movement rotation_world;
    std::string name;
};

#endif //CARIAD_OBJECT_H