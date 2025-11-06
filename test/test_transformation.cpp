//
// Created by krishnachaa on 06.11.25.
//
#include <memory>
#include <gtest/gtest.h>

#include "src/Object.h"
#include "src/Transformation.h"
#include <Eigen/Dense>

class TransformationShould: public ::testing::Test
{
protected:
    std::unique_ptr<Transformation> transformation;
    std::unique_ptr<Object> object;
};

TEST_F(TransformationShould, TransformEgo)
{
    auto counter = 0;

    EXPECT_EQ(counter, 0);

}

TEST_F(TransformationShould, TransformSingleObject)
{
    auto ego_pos = Position{1.828963f, -138.407091f, -0.006f, 1.571584f, -0.004266f, -0.00046f};
    auto ego_move = Movement{-0.101049, 13.055362, 0.055694, -0.000526, 0.000976, -0.0005126};
    auto ego = Object(ego_pos, ego_move, "ego");

    auto obj_pos = Position {4.985880, 112.5180, 0.0f, 6.234840f, 0.0f, 0.0f};
    auto obj_move = Movement {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    auto obj = Object(obj_pos, obj_move, "obj");

    auto mat = ego.get_tranformation_matrix();

    auto final_obj_pos = Position {250.920268, -3.354514, -1.064586, -1.619928, -0.000163, -0.004263};
    auto final_obj_move = Movement {-13.052681f, 0.132654f, 0.243177f, 0.000487f, 0.000554f, -0.000974f};
    auto final_obj = Object(final_obj_pos, final_obj_move, "obj");

    auto calc_obj = obj * mat;

    EXPECT_EQ(final_obj, calc_obj);
}

TEST_F(TransformationShould, TransformLotOfObjectsEasily)
{
    auto counter = 0;

    EXPECT_EQ(counter, 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}