//
// Created by krishnachaa on 06.11.25.
//

#ifndef CARIAD_TYPES_H
#define CARIAD_TYPES_H
#include <cfloat>

struct  Position
{
    float x, y, z;
    float h, p, r;
    auto operator==(const Position& position) const -> bool
    {
        return abs(x-position.x) <= FLT_EPSILON
        && abs(y - position.y) <= FLT_EPSILON
        && abs(z - position.z) <= FLT_EPSILON
        && abs(h - position.h) <= FLT_EPSILON
        && abs(p - position.p) <= FLT_EPSILON
        && abs(r - position.r) <= FLT_EPSILON;
    };
};

using Movement = Position;

#endif //CARIAD_TYPES_H