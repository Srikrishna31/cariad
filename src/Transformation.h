//
// Created by krishnachaa on 06.11.25.
//

#ifndef CARIAD_TRANSFORMATION_H
#define CARIAD_TRANSFORMATION_H

#include <Eigen/Eigen>

class Transformation
{
public:
    Transformation();
private:
    Eigen::Matrix4f matrix;
};


#endif //CARIAD_TRANSFORMATION_H