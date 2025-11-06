//
// Created by krishnachaa on 06.11.25.
//

#ifndef CARIAD_OBJECT_H
#define CARIAD_OBJECT_H

#include <Eigen/Eigen>

class Object {
public:
    Object(float x, float y, float z, float h, float p, float r);
    Object(const Object&) = default;
    Object(Object&&) = default;
    Object& operator=(const Object&) = default;
    Object& operator=(Object&&) = default;
    ~Object() = default;

private:
    Eigen::Vector3f position;
    Eigen::Vector3f rotation;
};
#endif //CARIAD_OBJECT_H