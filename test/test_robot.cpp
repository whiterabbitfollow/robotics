/*
 * =====================================================================================
 *
 *       Filename:  test_robot.cpp
 *
*    Description:  
 *
 *        Version:  1.0
 *        Created:  2020-12-12 11:36:18
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#include <stdlib.h>
#include "robot.hpp"
#include <gtest/gtest.h>

TEST(Test, Robot){
	std::vector<LinkConf> confs = {
		{{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 10},
		{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 10}
	};
	auto rob = RevoluteRobot(confs, Vector3f(1.0f, 0.0f, 0.0f));
	float theta1 = -M_PI/4;
	float theta2 = theta1;
	float theta_tool = theta1 + theta2;
	std::vector<float> angles = {-M_PI/4, -M_PI/4};
	rob.set_angles(angles);
	SE3mat gd = rob.get_end_effector(); 
	Matrix3f R = SO3_XYZ(ROT_AXIS::ROT_Y, theta_tool);
	auto g = rob.get_end_effector();
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			ASSERT_LT(std::abs(R(i,j)-g.R(i,j)), 1e-7);
	ASSERT_LT(std::abs(g.p(0) - 1/std::sqrt(2)), 1e-7);
	ASSERT_EQ(g.p(1), 0);
	ASSERT_LT(std::abs(g.p(2) - (1+1/std::sqrt(2))), 1e-7);
}	

int
main ( int argc, char *argv[] )
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}	

