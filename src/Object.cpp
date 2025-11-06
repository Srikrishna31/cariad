//
// Created by krishnachaa on 06.11.25.
//

#include "Object.h"

Object::Object(const Position& pos, const Movement& rot, std::string name)
    : position(pos.x, pos.y, pos.z), rotation(rot.x, rot.y, rot.z), name(std::move(name))
{}