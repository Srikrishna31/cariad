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
    Object(const Position& pos, const Movement& rot, std::string name);
    Object(const Object&) = default;
    Object(Object&&) = default;
    Object& operator=(const Object&) = default;
    Object& operator=(Object&&) = default;
    ~Object() = default;

    auto operator*(const Transformation& t) const -> Object;

private:
    Eigen::Vector4f position;
    Eigen::Vector4f rotation;
    std::string name;
};
#endif //CARIAD_OBJECT_H