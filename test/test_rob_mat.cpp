/*
 * =====================================================================================
 *
 *       Filename:  test_rob_mat.cpp
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

TEST(Test, Exp){
	Vector3f q(1.0f,0.f,0.f);
	Vector3f w(0.0f,0.f,1.0f);
	UnitScrew S(w, -w.cross(q));
	float angle = M_PI/4;
	SE3mat g_0(Vector3f(2, 0, 0));
	SE3mat g = S.exp(angle) * g_0;
	Matrix3f R = SO3_XYZ(ROT_AXIS::ROT_Z, angle);
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			ASSERT_EQ(R(i,j), g.R(i,j));
	ASSERT_LT(std::abs(g.p(0) - (1+1/std::sqrt(2))), 1e-7);
	ASSERT_LT(std::abs(g.p(1) - 1/std::sqrt(2)), 1e-7);
	ASSERT_EQ(g.p(2), 0);
}	

int
main ( int argc, char *argv[] )
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}	


