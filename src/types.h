//
// Created by krishnachaa on 06.11.25.
//

#ifndef CARIAD_TYPES_H
#define CARIAD_TYPES_H

constexpr float EPSILON = 1e-4;

struct  Position
{
    float x, y, z;
    float h, p, r;
    auto operator==(const Position& position) const -> bool
    {
        return abs(x-position.x) <= EPSILON
        && abs(y - position.y) <= EPSILON
        && abs(z - position.z) <= EPSILON
        && abs(h - position.h) <= EPSILON
        && abs(p - position.p) <= EPSILON
        && abs(r - position.r) <= EPSILON;
    };
};

using Movement = Position;

#endif //CARIAD_TYPES_H