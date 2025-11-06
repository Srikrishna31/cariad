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
    auto ego_pos = Position{1.828963f, -138.407091f, -0.006f};
    auto ego_rot = Movement{1.571584f, -0.004266f, -0.00046f};
    auto ego = Object(ego_pos, ego_rot, "ego");
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