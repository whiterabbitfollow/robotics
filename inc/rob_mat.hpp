/*
 * =====================================================================================
 *
 *       Filename:  so_mat.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020-11-08 14:44:31
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef SO_MAT_HPP
#define SO_MAT_HPP

#include <stdlib.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <math.h>

using namespace Eigen;

class SE3mat;
class SO3mat;
class so3mat;

float SO3_log_theta(Matrix3f R){
	float R_trace = R.trace();
	float theta = acos((R_trace - 1) * 0.5);
	return theta;
}

Vector3f SO3_log(Matrix3f R, float theta){
	Vector3f w;
	if( theta != 0 ){
		float r21 = R(2, 1), r12 = R(1,2), r02 = R(0,2), r20 = R(2,0), r01 = R(0,1), r10 = R(1,0);
		w[0] = r21-r12;
		w[1] = r02-r20;
		w[2] = r10-r01;
		w *= 1/(2 * sin(theta));
	}
	return w;
}

Matrix3f SO3_exp(Matrix3f w_skew, double theta)
{
	/*
	 *  Rodrigues formula:
	 *  exp(w_skew theta) = eye(3) + w_skew sin(theta) + w_skew^2( 1 - cos(theta))
	 */
	auto I = Matrix3f::Identity();
	Matrix3f R = I + w_skew * sin(theta) + w_skew * w_skew * (1-cos(theta));
	return R;
}

Matrix3f so3_from_vec(Vector3f w)
{
	double w0 = w[0], w1 = w[1], w2 = w[2];
	Matrix3f w_hat; w_hat <<  0, -w2,  w1,
							  w2, 0 , -w0,
							 -w1, w0,   0;
	return w_hat;
}


class se3mat;

class SO3mat{

	public:
		Matrix3f R;
		SO3mat() {
			R = Matrix3f();
			R = Matrix3f::Identity(); 
		}
		SO3mat(Matrix3f rot) : R(rot) { }
		SO3mat inv();
		so3mat log();
		void set_rot_z(float angle){
			R << cos(angle), -sin(angle), 0.0f,
				 sin(angle),  cos(angle), 0.0f,
				 0.0f	   ,		0.0f, 1.0f;
		}
};

class so3mat{
	
	public:
		Matrix3f w_skew;
		Vector3f w;
		so3mat(Vector3f rot){
			w = rot;
			w_skew = so3_from_vec(rot);
		}
		SO3mat exp(float angle);

};

class SE3mat{

	public:
		
		Matrix3f R;
		Vector3f p;
		SE3mat() : R(), p(0,0,0) {R.setIdentity();}
		SE3mat(Vector3f trans) : R(), p(trans) {}
		SE3mat(SO3mat rot, Vector3f trans) : R(rot.R), p(trans) { }
		SE3mat(Matrix3f rot, Vector3f trans) : R(rot), p(trans) { }
		SE3mat(Matrix4f T) : R(T.block(0,0,3,3)), p(T.col(3).segment(0, 3)) { }
 		
		SE3mat inv(){
			Matrix3f R_new;
			R_new = R.transpose();
			Vector3f p_new = - R_new * p;
			return SE3mat(R_new, p_new);
		}

		void set_from_mat4(Matrix4f T_new){
			R = T_new.block(0,0,3,3);
			p = T_new.col(3).segment(0,3);
		}

		Matrix4f to_mat4(){
			auto T = Matrix4f();
			T.block(0, 0, 3, 3) = R;
			T.col(3).segment(0, 3) = p;
			T(3,0) = T(3,1) = T(3,2) = 0;
			T(3,3) = 1;
			return T;
		}

		Vector3f operator*(Vector3f pp){
			return R * pp + p;
		}
		se3mat adj_inv(se3mat psi);
		se3mat adj(se3mat psi);
		friend SE3mat operator*(const SE3mat &g_1, const SE3mat &g_2);

};


class se3mat{
	
	public:
		// twist coordinates
		Vector3f w, v;
		
		se3mat() : w(), v() {}
		se3mat(Vector3f rot, Vector3f vel) : w(rot) , v(vel) {}
		SE3mat exp(float angle);
		VectorXf flatten_twist(){
			// returns the twist coordinates.
			auto psi = VectorXf(6);
			for(int i=0; i<3; i++){
				psi[i] = v[i];
				psi[i+3] = w[i];
			}
			return psi;
		}

		Matrix4f get_twist_in_homogeneous_coordinates(){
			auto psi_hat = Matrix4f();
			psi_hat.block(0, 0, 3, 3) = so3_from_vec(w);
			psi_hat.col(3).segment(0, 3) = v;
			for(int i=0; i<4; i++)
				psi_hat(3, i) = 0;
			return psi_hat;
		}

		MatrixXf flatten(){
			MatrixXf psi(6,1);
			psi.block(0, 0, 3, 1) = v;
			psi.block(3, 0, 3, 1) = w;
			return psi;
		}
};

SE3mat se3mat::exp(float theta){
	auto R = so3mat(w).exp(theta).R;
	Vector3f p = (Matrix3f::Identity() - R) * w.cross(v) + w * w.dot(v) * theta;
	return SE3mat(R, p);
}

se3mat SE3mat::adj(se3mat psi){
	auto v_prime =  R * psi.v  + so3_from_vec(p) * R * psi.w;
	auto w_prime = R * psi.w;
 	return se3mat(v_prime, w_prime);
}

se3mat SE3mat::adj_inv(se3mat psi){
	auto v = R.transpose() * psi.v - so3_from_vec(R.transpose() * p) * R.transpose() * psi.w;
	auto w = R.transpose() * psi.w;
	return se3mat(v, w);
}

SE3mat operator*(const SE3mat &g_1, const SE3mat &g_2){
	return SE3mat(g_1.R * g_2.R, g_1.R * g_2.p + g_1.p); 
}

// TODO: Add coordinate transforms....

class UnitScrew : public se3mat{

	public:
		Vector3f l;
		float h;
		bool has_rotation;

		UnitScrew() : 
			se3mat(),
			l(),
			h{0},
			has_rotation{false}
			{ }

		UnitScrew(Vector3f rot, Vector3f vel) : se3mat(rot, vel), h{0}{
			has_rotation = w.sum()!=0;
			bool has_unit_rotation = w.norm() == 1;
			bool has_unit_velocity = v.norm() == 1;
			if(has_rotation){
				float w_norm_sq = w.dot(w);
				h = w.dot(v) / w_norm_sq;
				l = w.cross(v) / w_norm_sq + w;
			}
			else{
				h = 0; // the pitch is infinte, so only translation...
				l = v; 
			}
		}

		SE3mat exp(float angle){
			return get_unit_twist().exp(angle);
		}

		void set_from_revolute_axis(Vector3f rot_unit, Vector3f q){
			// set w, v from a revolute axis with direction unit direction vector rot_unit.
			// q is point on the axis.
			w = rot_unit;
			v = -rot_unit.cross(q);
			h = 0;
			l = q;
			has_rotation = true;
		}
		
		se3mat get_unit_twist(){
			// TODO: necessary??? 
			Vector3f v_out, w_out;
			if(has_rotation){
				v_out = -w.cross(l) + h * w;
				w_out = w;
			}
			else{
				v_out = v;
				w_out = Vector3f(0,0,0);
			}
			return se3mat(w_out, v_out);
		}

};

class ZPUnitScrew: public UnitScrew{
	public:
		// Zero Pitch Unit Screw
		ZPUnitScrew(Vector3f rot) : UnitScrew(rot, Vector3f(0.0f, 0.0f, 0.0f)) { }
};

SO3mat so3mat::exp(float theta){
	Matrix3f A = SO3_exp(w_skew, theta);
	return SO3mat(A);
}

SO3mat SO3mat::inv(){
	return SO3mat(R.transpose());
}

so3mat SO3mat::log(){
	auto theta = SO3_log_theta(R);
	Vector3f w = SO3_log(R, theta);
	// TODO: what with theta???
	return so3mat(w);
}

#endif
