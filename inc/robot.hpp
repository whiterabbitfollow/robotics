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


#include <stdlib.h>
#include <vector>

#include "rob_mat.hpp"
#include "invkin.hpp"

using namespace Eigen;


struct{
	Vector3f kinvek;
	Vector3f rot;
	double _mass;
} typedef LinkConf;



class RevoluteJoint{

	/*
	 *  A joint which only can rotate around its axis.
	 */

	private:
		SE3mat _g, _g_global, _g_initial;
		Vector3f _pos;

	public:
		UnitScrew s_unit;
		int joint_nr;
		float angle;
		Vector3f rot;

		RevoluteJoint(int nr) : 
			joint_nr{nr},
			s_unit(),
			_g(), 
			_g_global(),
			_pos(),
			angle{0},
			_g_initial(),
			rot(0.0f, 0.0f, 0.0)
		{
			_pos.setZero();
		}

		RevoluteJoint(int nr, Vector3f rot_vec, Vector3f q) : 
			joint_nr{nr},
			s_unit(),
			_g(), 
			_g_global(),
			_pos(q),
			_g_initial(q),
			angle{0},
			rot(rot_vec)
		{
			s_unit.set_from_revolute_axis(rot, q);
		}

		SE3mat transform(float angle){
		     return s_unit.exp(angle);
		}

		void set_transform(float angle){
			this-> angle = angle;
		     _g = s_unit.exp(angle);
		}

		SE3mat g(){
			return _g;
		}

		Matrix4f get_transform(){
			// legacy...
			return _g.to_mat4();
		}

		void set_global_transform(SE3mat g){
			_g_global = g;
		}

		Vector3f get_pos(){
			return _g_global * _pos;

		}

		Vector3f get_pos_raw(){
			return _pos;
		}

		Matrix4f get_global_transform(){
			return _g_global.to_mat4();
		}

		SE3mat get_SE3_pos(){
			return _g_global * _g_initial;
		}

};

class Link 
{

	/*  
	 * Link i connects joint i-1 and i
	 * joint i-1 is considered to be the "parent"
	 * joint i is the child, which it holds a r
	 *  
	 * A link has an angle? Which is the angle of the child.
	 * A link has a pos, which is the vector from child to parent coordinate frame.
	 *
	 */

	public:
		int link_nr;
		float mass;
		RevoluteJoint &_parent, &_child;

		Link(int link_nr, RevoluteJoint &parent, RevoluteJoint &child) : 
			link_nr{link_nr},
			mass{0},
			_parent(parent),
			_child(child)
		{ 	
		}

		void get_link_pos(Vector3f &start, Vector3f &end){
			start = _parent.get_pos();
			end = _child.get_pos();
		}

};

class RevoluteRobot
{
	 // Link 0 is base.
	 // Link N is attached rigidly to the end-effector.
	
	private:
		SE3mat _g_st_0, _g_mat;
		const size_t nr_actuated_joints;
		const size_t JOINT_START, JOINT_END;

	public:
		std::vector<Link> links;
		std::vector<RevoluteJoint> joints;

		RevoluteRobot(std::vector<LinkConf> confs, Vector3f tool_kinvek) : 
			links(), 
			_g_st_0(),
			_g_mat(),
			joints(),
			nr_actuated_joints(confs.size()),
			JOINT_START{1}, 
			JOINT_END{confs.size()}
			{
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

		/*  		
		MatrixXf spatial_jacobian(){
			// TODO: verify
			size_t N = size();
 			MatrixXf J_s(6, N);
			for(int joint_nr=JOINT_START; joint_nr <= JOINT_END; joint_nr++){
 				SE3mat g{};
 				for(int j=1; j<joint_nr; j++){
 					g = g * joints[j].get_transform();
 				}
 				J_s.block(0, joint_nr, N, joint_nr) = g.adj(joints[joint_nr].s_unit).flatten();
 			}
			return J_s;
 		}
		 */

		SE3mat get_end_effector(){
			 //   Product of exponential formula to get end effector pos.
			 //   g(theata1, ..., thetaN) = exp(psi1_hat theta1) ... exp(psiN_hat thetaN) * g_st(0) = g_d 
			return _g_mat * _g_st_0;
		}
		
		SE3mat get_end_effector_intial(){
			return _g_st_0;
		}

		int size(){
			return links.size();
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


