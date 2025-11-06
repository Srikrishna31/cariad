//
// Created by krishnachaa on 06.11.25.
//

#include "Object.h"

#include <iostream>

#include "Transformation.h"

Object::Object(Position pos, Movement move)//, std::string name)
    : position(pos.x, pos.y, pos.z, 1.0)
    , movement(move.x, move.y, move.z, 1.0)
    , position_world(std::move(pos))
    , rotation_world(std::move(move))
    // , name(std::move(name))
{}

[[nodiscard]] auto Object::get_tranformation_matrix() const -> Transformation
{
    return Transformation{Transformation::CreateTransformationMatrix(position_world.x, position_world.y, position_world.z,
        position_world.h, position_world.p, position_world.r)};
    // return Transformation{position_world.x, position_world.y, position_world.z,
    //                        position_world.h, position_world.p, position_world.r};
}

auto Object::operator*(const Transformation& t) const -> Object
{
    const Eigen::Matrix4f mat = t.matrix().inverse();
    Eigen::Vector4f pos = mat * position;

    auto transformation_obj_mtrx = Transformation::CreateTransformationMatrix(position_world.x, position_world.y, position_world.z,
        position_world.h, position_world.p, position_world.r);
    auto transform_hpr = mat * transformation_obj_mtrx;
    float yaw = atan2(transform_hpr(1, 0), transform_hpr(0, 0));
    float pitch = asin(-transform_hpr(2, 0));
    float roll = atan2(transform_hpr(2, 1), transform_hpr(2, 2));
    
    const auto pos_world = Position{pos.x(), pos.y(), pos.z(), yaw, pitch, roll};

    return Object{pos_world, rotation_world};//, name};
}


auto Object::operator==(const Object& other) const -> bool
{
    return position == other.position
        && movement == other.movement
        && position_world == other.position_world
        && rotation_world == other.rotation_world;
        // && name == other.name;
}
