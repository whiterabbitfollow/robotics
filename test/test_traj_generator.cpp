/*
 * =====================================================================================
 *
 *       Filename:  test_traj_generator.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020-12-12 09:05:34
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#include <stdlib.h>
#include "traj_generator.hpp"
#include <gtest/gtest.h>

TEST(CubicPolTest, Points){
	float T = 2;
	VectorXf s(2); s << 0, 0;
	VectorXf e(2); e << 1, 1;	
	VectorXf delta = e - s;
	VectorXf spd_max(2); spd_max<< (3/(2*T)) * delta;
	VectorXf acc_max(2); acc_max<< (6/(std::pow(T,2))) * delta.cwiseAbs();
	
	auto traj = CubicPolynomialTrajGenerator(s, e, T);
	
	traj.pos_speed_acc(0);

	ASSERT_EQ(traj.pos(0), 0);
	ASSERT_EQ(traj.pos(1), 0);
	ASSERT_EQ(traj.spd(0), 0);
	ASSERT_EQ(traj.spd(1), 0);
	ASSERT_EQ(traj.acc(0), acc_max(0));
	ASSERT_EQ(traj.acc(1), acc_max(1));
	
	traj.pos_speed_acc(T/2);
	
	ASSERT_EQ(traj.pos(0), 0.5);
	ASSERT_EQ(traj.pos(1), 0.5);
	ASSERT_EQ(traj.spd(0), spd_max(0));
	ASSERT_EQ(traj.spd(1), spd_max(1));
	ASSERT_EQ(traj.acc(0), 0);
	ASSERT_EQ(traj.acc(1), 0);
	
	traj.pos_speed_acc(T);

	ASSERT_EQ(traj.pos(0), 1);
	ASSERT_EQ(traj.pos(1), 1);
	ASSERT_EQ(traj.spd(0), 0);
	ASSERT_EQ(traj.spd(1), 0);
	ASSERT_EQ(traj.acc(0), -acc_max(0));
	ASSERT_EQ(traj.acc(1), -acc_max(1));
	
}

/*  TODO: Test other */


int
main ( int argc, char *argv[] )
{ 
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}				
/* ----------  end of function main  ---------- */
