//
// Created by krishnachaa on 06.11.25.
//

#include "Object.h"

#include <iostream>

#include "Transformation.h"

Object::Object(Position pos, Movement move, std::string name)
    : position(pos.x, pos.y, pos.z, 1.0)
    , movement(move.x, move.y, move.z, 1.0)
    , position_world(std::move(pos))
    , rotation_world(std::move(move))
    , name(std::move(name))
{}

[[nodiscard]] auto Object::get_tranformation_matrix() const -> Transformation
{
    Eigen::Vector3f ego_pos(1.828963f, -138.407091f, -0.006f);
    Eigen::Vector3f ego_hpr(1.571584f, -0.004266f, -0.000046f);

    auto transformation = Transformation{Transformation::CreateTransformationMatrix(position_world.x, position_world.y, position_world.z,
        position_world.h, position_world.p, position_world.r)};
    std::cout << transformation.matrix() << std::endl;
    return Transformation{Transformation::CreateTransformationMatrix(position_world.x, position_world.y, position_world.z,
        position_world.h, position_world.p, position_world.r)};
    // return Transformation{position_world.x, position_world.y, position_world.z,
    //                        position_world.h, position_world.p, position_world.r};
}

auto Object::operator*(const Transformation& t) const -> Object
{
    const auto mat = t.matrix().inverse();
    Eigen::Vector4f pos = mat * position;
    Eigen::Vector3f rot =
        {static_cast<float>(atan2(mat(1,0), mat(0,0))),
        static_cast<float>(asin(-mat(2, 0))),
        static_cast<float>(atan2(mat(2,1), mat(2,2)))};

    const auto pos_world = Position{pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z()};

    return Object{pos_world, rotation_world, name};
}


auto Object::operator==(const Object& other) const -> bool
{
    return position == other.position
        && movement == other.movement
        && position_world == other.position_world
        && rotation_world == other.rotation_world
        && name == other.name;
}
