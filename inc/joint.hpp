/*
 * =====================================================================================
 *
 *       Filename:  joint.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020-12-12 14:18:46
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#ifndef JOINT_HPP 
#define JOINT_HPP 

#include "rob_mat.hpp"

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
			rot()
		{
			_pos.setZero();
			rot.setZero();
		}

		RevoluteJoint(int nr, Vector3f rot_vec, Vector3f q) :
			RevoluteJoint(nr)
		{
			rot = rot_vec;
			_g_initial = q;
			_pos = q,
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


#endif
