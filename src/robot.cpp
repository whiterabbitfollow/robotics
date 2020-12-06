/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020-10-31 15:22:19
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */


#include <stdlib.h>
#include <iostream>
#include <vector>

#include "robot.hpp"


int
main ( int argc, char *argv[] )
{
	stanfordArmConfs confs = {0.1, 0.4, 1.0, 0.5, 0.6, 0.1, {0.0, 0.0, 1.0}};
	auto robot = stanford_arm_robot_init_from_confs(confs);
//	robot.add_point(PointX);
//	robot.add_point(PointY);
//	robot.execute();
	return EXIT_SUCCESS;
}	

/* ----------  end of function main  ---------- */
