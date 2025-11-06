//
// Created by krishnachaa on 06.11.25.
//

#include "Object.h"
#include "Transformation.h"

Object::Object(Position pos, Movement rot, std::string name)
    : position(pos.x, pos.y, pos.z, 1.0)
    , rotation(rot.x, rot.y, rot.z, 1.0)
    , position_world(std::move(pos))
    , rotation_world(std::move(rot))
    , name(std::move(name))
{}

[[nodiscard]] auto Object::get_tranformation_matrix() const -> Transformation
{
    return Transformation{position_world.x, position_world.y, position_world.z,
                           position_world.h, position_world.p, position_world.r};
}

Object operator*(Eigen::Vector4f::Nested lhs, const Transformation& rhs);

auto Object::operator*(const Transformation& t) const -> Object
{
    auto mat = t.matrix();
    Eigen::Vector4f pos = mat * position;
    Eigen::Vector3f rot =
        {static_cast<float>(atan2(mat(1,0), mat(0,0))),
        static_cast<float>(asin(mat(2, 0))),
        static_cast<float>(atan2(mat(2,1), mat(2,2)))};

    const auto pos_world = Position{pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z()};

    return Object{pos_world, rotation_world, name};
}
