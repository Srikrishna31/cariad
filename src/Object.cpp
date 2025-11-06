//
// Created by krishnachaa on 06.11.25.
//

#include "Object.h"
#include "Transformation.h"

Object::Object(const Position& pos, const Movement& rot, std::string name)
    : position(pos.x, pos.y, pos.z, 1.0), rotation(rot.x, rot.y, rot.z, 1.0), name(std::move(name))
{}

Object operator*(Eigen::Vector4f::Nested lhs, const Transformation& rhs);

auto Object::operator*(const Transformation& t) const -> Object
{
    return position * t;
}
