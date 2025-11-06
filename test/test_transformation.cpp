//
// Created by krishnachaa on 06.11.25.
//
#include <memory>
#include <gtest/gtest.h>

#include "src/Object.h"
#include "src/Transformation.h"

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

TEST_F(TransformationShould, TransformLotOfObjectsEasily)
{

}
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}