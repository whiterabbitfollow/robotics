/*
 * =====================================================================================
 *
 *       Filename:  robot_system.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020-12-09 18:09:11
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "traj_generator.hpp"
#include "robot.hpp"

class RobotSystem {

	private:
		BaseRobot _robot;
	
	public:
		const float SAMPLE_TIME{0.048};

		RobotSystem(BaseRobot robot) : _robot(robot) {
		};
		
		void execute_instruction(BaseTrajGenerator traj_gen) {
			float t{0};
			// TODO could make this as generator??? 
			while(t < traj_gen.T){
				traj_gen.pos_speed_acc(t);
				// _robot.set_angles(traj_gen.pos);
				t += SAMPLE_TIME;
			}
		};

		void run(){
			// have a set of instructions. Execute them. Loop.
			while(true){
				// execute_instruction()
			}
		};
};


