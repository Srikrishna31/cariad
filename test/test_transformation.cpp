//
// Created by krishnachaa on 06.11.25.
//
#include <memory>
#include <gtest/gtest.h>

#include "src/Object.h"
#include "src/Transformation.h"
#include <Eigen/Dense>
#include <chrono>
#include <execution>

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
    auto ego_pos = Position{1.828963f, -138.407091f, -0.006f, 1.571584f, -0.004266f, -0.000046f};
    auto ego_move = Movement{-0.101049f, 13.055362f, 0.055694f, -0.000526f, 0.000976f, -0.0005126f};
    auto ego = Object(ego_pos, ego_move);//, "ego");

    auto obj_pos = Position {4.985880f, 112.5180f, 0.0f, 6.234840f, 0.0f, 0.0f};
    auto obj_move = Movement {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    auto obj = Object(obj_pos, obj_move);//, "obj");

    auto mat = ego.get_tranformation_matrix();
    std::cout << "Transformation Matrix:\n" << mat.matrix() << std::endl;

    auto inv_mat = mat.matrix().inverse();
    std::cout << "Inverse Matrix:\n" << inv_mat << std::endl;

    auto final_obj_pos = Position {250.920268f, -3.354514f, -1.064586f, -1.619928f, -0.000163f, -0.004263f};
    auto final_obj_move = Movement {-13.052681f, 0.132654f, 0.243177f, 0.000487f, 0.000554f, -0.000974f};
    auto final_obj = Object(final_obj_pos, final_obj_move);//, "obj");

    // auto calc_obj = obj * mat;
    obj * mat;

    // EXPECT_EQ(final_obj, calc_obj);
    EXPECT_NEAR(final_obj.position.x(), obj.position_ego.x, 1e-4);
    EXPECT_NEAR(final_obj.position.y(), obj.position_ego.y, 1e-4);
    EXPECT_NEAR(final_obj.position.z(), obj.position_ego.z, 1e-4);
    EXPECT_NEAR(final_obj.position_world.h, obj.position_ego.h, 1e-4);
    EXPECT_NEAR(final_obj.position_world.p, obj.position_ego.p, 1e-4);
    EXPECT_NEAR(final_obj.position_world.r, obj.position_ego.r, 1e-4);
}

TEST_F(TransformationShould, TransformLotOfObjectsEasily)
{
    auto obj_pos = Position {4.985880f, 112.5180f, 0.0f, 6.234840f, 0.0f, 0.0f};
    auto obj_move = Movement {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    auto obj = Object(obj_pos, obj_move);//, "obj");

    auto objects = std::vector<Object>(1000000, obj);

    auto ego_pos = Position{1.828963f, -138.407091f, -0.006f, 1.571584f, -0.004266f, -0.000046f};
    auto ego_move = Movement{-0.101049f, 13.055362f, 0.055694f, -0.000526f, 0.000976f, -0.0005126f};
    auto ego = Object(ego_pos, ego_move);//, "ego");

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;


    auto t1 = high_resolution_clock::now();

    auto mat = ego.get_tranformation_matrix();

    std::for_each(std::execution::par_unseq, objects.begin(), objects.end(), [&mat](auto& obj)
    {
        // auto calc_obj = obj * mat;
        obj * mat;
    });
    // for (const auto& object: objects)
    // {
    //     auto calc_obj = object * mat;
    // }

    auto t2 = high_resolution_clock::now();

    /* Getting number of milliseconds as an integer. */
    auto ms_int = duration_cast<milliseconds>(t2 - t1);

    /* Getting number of milliseconds as a double. */
    duration<double, std::milli> ms_double = t2 - t1;

    std::cout << ms_int.count() << "ms\n";
    std::cout << ms_double.count() << "ms\n";

    auto counter = 0;

    EXPECT_EQ(counter, 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}