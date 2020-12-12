/*
 * =====================================================================================
 *
 *       Filename:  robot.h
 *
 *    Description:  
*
 *        Version:  1.0
 *        Created:  2020-11-19 19:05:32
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <stdlib.h>
#include <vector>

#include "invkin.hpp"
#include "link.hpp"
#include "joint.hpp"

struct{
	Vector3f kinvek;
	Vector3f rot;
	double _mass;
} typedef LinkConf;

class BaseRobot
{

	protected:
		SE3mat _g_st_0, _g_mat;
		const size_t nr_actuated_joints;
		const size_t JOINT_START{1}, JOINT_END;

	public:
		std::vector<Link> links;
		std::vector<RevoluteJoint> joints;
		
		BaseRobot(size_t N) : 
			_g_st_0(),
			_g_mat(),
			nr_actuated_joints(N),
			JOINT_END{N}
			{
			};
		
		void set_angles(std::vector<float> angles){
			// product of expontentials
			// exp(psi1_hat theta1) ... exp(psiN_hat thetaN) 
			for(int joint_nr=JOINT_START; joint_nr <= JOINT_END; joint_nr++){	
				auto theta = angles[joint_nr-1];
				RevoluteJoint &joint = joints[joint_nr];
				joint.set_transform(theta);
				if(joint_nr==1)
					_g_mat = joint.g();
				else
					_g_mat = _g_mat *  joint.g();
				joint.set_global_transform(_g_mat);
			}
		}
		
		MatrixXf spatial_jacobian(){
			size_t N = size();
 			MatrixXf J_s(6, N);
			int col=0;
			for(int joint_nr=JOINT_START; joint_nr <= JOINT_END; joint_nr++){
 				SE3mat g;
 				for(int j=JOINT_START; j < joint_nr; j++){
 					g = g * joints[j].g();
 				}
 				J_s.block(0, col, 6, 1) = g.adj(joints[joint_nr].s_unit).flatten();
				col++;
 			}
			return J_s;
 		}

		MatrixXf body_jacobian(){
			size_t N = size();
 			MatrixXf J_b(6, N);
			int col=0;
			for(int joint_nr=JOINT_START; joint_nr <= JOINT_END; joint_nr++){
 				SE3mat g;
 				for(int j=joint_nr; j <= JOINT_END; j++){
 					g = g * joints[j].g();
 				}
				g = g * _g_st_0; 
 				J_b.block(0, col, 6, 1) = g.adj_inv(joints[joint_nr].s_unit).flatten();
				col++;
 			}
			return J_b;
		}

		SE3mat get_end_effector(){
			 //   Product of exponential formula to get end effector pos.
			 //   g(theata1, ..., thetaN) = exp(psi1_hat theta1) ... exp(psiN_hat thetaN) * g_st(0) = g_d 
			return _g_mat * _g_st_0;
		}
		
		SE3mat get_end_effector_intial(){
			return _g_st_0;
		}

		int size(){
			return nr_actuated_joints;
		}

};

class RevoluteRobot : public BaseRobot
{
	 // Link 0 is attaches ground (joint 0) with fist actuated joint (1).
	 // Link N is attached rigidly to the end-effector.
	public:
		RevoluteRobot(std::vector<LinkConf> confs, Vector3f tool_kinvek) : BaseRobot(confs.size())	
			{
				// could be improved...
				// have to add... rotation in kinvec....
				Vector3f q_abs(0.0f, 0.0f, 0.0f);
				int link_nr = 0, joint_nr = 1;
				// first joint is static ground joint
				joints.push_back(RevoluteJoint(0));
				for(auto conf : confs){
					q_abs = q_abs + conf.kinvek;
					joints.push_back(RevoluteJoint(joint_nr, conf.rot, q_abs));
					links.push_back(Link(link_nr, joints[joint_nr-1], joints[joint_nr]));
					link_nr += 1;
					joint_nr += 1;
				}
				// add link between axis N and tool
				q_abs += tool_kinvek;
				// set rot to zero vector, since dead joint
				Vector3f rot(0.0f, 0.0f, 0.0f);
				joints.push_back(RevoluteJoint(joint_nr, rot, q_abs));
				links.push_back(Link(link_nr, joints[joint_nr-1], joints[joint_nr]));
				_g_st_0.p = q_abs;
			}
};

struct stanfordArmConfs{
	float Lz1;
	float Lz2;
	float Lz3;
	float Lz4;
	float Lz5;
	float Lx6;
	Vector3f tool;
} typedef standfordArmConfs;


class StanfordArmRobot: public RevoluteRobot{
	
	public:
		StanfordArmRobot(std::vector<LinkConf> confs, Vector3f tool ) : 
			RevoluteRobot(confs, tool)
		{
		}

		std::vector<float> inv_kin(SE3mat gd){
			// pw is intersection point of wrist axis
			// pb is intersection point of base axis
			// calculate pw, and pb.
			Vector3f pw = joints[5].get_pos_raw();
			Vector3f pb = joints[2].get_pos_raw();
			// g_d = exp(theta1*psi1) ... exp(theta6*psi6) g_st(0)
			// g1 = g_d g_st^-1(0) = exp(theta1*psi1) ... exp(theta6*psi6) 
			// step 1, solve for elbow angle theta_3	
			SE3mat g1 = gd * get_end_effector_intial().inv();
			float delta = (g1 * pw  - pb).norm();
			Vector3f r3 = joints[3].get_pos_raw();
			// subproblem 3 p=pw, q=pb
			float theta31, theta32;
			subproblem_3(joints[3].rot, pw, pb, r3, delta, &theta31, &theta32);
			float theta_3 = std::min(theta31, theta32);
			// step 2, solve for the base joint angles
			float theta_1{0}, theta_2{0};
			Vector3f p2 = joints[3].transform(theta_3) * pw;
			Vector3f q2 = g1 * pw;
			subproblem_2(joints[1].rot, joints[2].rot, p2, q2, joints[2].get_pos_raw(), &theta_1, &theta_2);
			// step 3, solve for two of three wrist axis
			// p3 is a point on axis 6, but not on axis 4 and 5
			float theta_4{0}, theta_5{0};
			SE3mat g2 = joints[3].transform(-theta_3) * joints[2].transform(-theta_2) * joints[1].transform(-theta_1) * g1;
			Vector3f p3 = joints[6].get_pos_raw();
			Vector3f q3 = g2 * p3;
			subproblem_2(joints[4].rot, joints[5].rot, p3, q3, joints[5].get_pos_raw(), &theta_4, &theta_5);
			// step 4, solve for remaining wrist angle
			// p4 is any point which is not on axis 6
			Vector3f p4 = joints[4].get_pos_raw();
			Vector3f q4 = joints[5].transform(-theta_5) * joints[4].transform(-theta_4) * g2 * p4;
			float theta_6 = subproblem_1(joints[6].rot, p4, q4, joints[6].get_pos_raw());
			std::vector<float> angles  = {theta_1, theta_2, theta_3, theta_4, theta_5, theta_6};
			return angles; 
	}
};

StanfordArmRobot stanford_arm_robot_init_from_confs(stanfordArmConfs confs){
		std::vector<LinkConf> kin_confs = {
		{{0.0f, 0.0f, confs.Lz1}, {0.0f, 0.0f, 1.0f}, 1.0},
		{{0.0f, 0.0f, confs.Lz2}, {0.0f, 1.0f, 0.0f}, 1.0},
		{{0.0f, 0.0f, confs.Lz3}, {0.0f, 1.0f, 0.0f}, 1.0},
		{{0.0f, 0.0f, confs.Lz4}, {0.0f, 0.0f, 1.0f}, 1.0},
		{{0.0f, 0.0f, confs.Lz5}, {0.0f, 1.0f, 0.0f}, 1.0},
		{{confs.Lx6, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0}};
		return StanfordArmRobot(kin_confs, confs.tool);
}


// TODO add scara???
#endif
