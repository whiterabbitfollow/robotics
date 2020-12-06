/*
 * =====================================================================================
 *
 *       Filename:  invkin.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020-11-21 12:48:50
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include <eigen3/Eigen/Core>
#include "rob_mat.hpp"
#include <cmath>

/* solutions to the inverse kinematic problem by using Paden-Kahan subproblems  */


float subproblem_1(Vector3f w, Vector3f p, Vector3f q, Vector3f r){
	/* Let psi be a zero-pitch twist with unit magnitude and p, q in R^3 two points.
	 * Find theta such that
	 * 		exp(psi_hat theta) p = q
	 */
	Vector3f u = p - r; // r point on rotation axis w
	Vector3f v = q - r;
	Vector3f u_prime = u - w * w.dot(u);
	Vector3f v_prime = v - w * w.dot(v);
	if(u_prime.sum() == 0){
		throw "infinite number of solutions";
	}
	if(!( std::abs(w.dot(u)-w.dot(v)) < 1e-4 && (std::abs(u_prime.norm() - v_prime.norm()) < 1e-4))){
		std::cout << "u: "<< u << std::endl;
		std::cout << "v: "<< v << std::endl; 
		std::cout << "u: "<< std::abs(u_prime.norm() - v_prime.norm()) << std::endl;
		throw "condition not forefilled";
	}
	return std::atan2(w.dot(u_prime.cross(v_prime)), u_prime.dot(v_prime));
}

void subproblem_2(Vector3f w1, Vector3f w2, Vector3f p, Vector3f q, Vector3f r, float *theta1, float *theta2){
	/*  Let psi1, psi2 be two zero-pitch, unit magnitude twists with intersecting axes and p, q in R^3 be two points.
	 *  Find theta1 and theta2, such that
	 *  	exp(psi1_hat theta1) exp(psi2_hat theta2) p = q 
	*/
	Vector3f u = p - r;
	Vector3f v = q - r;
	float w1_dot_w2 = w1.dot(w2);
	float den = (std::pow(w1_dot_w2, 2) - 1);
	float alpha = (w1_dot_w2 * w2.dot(u) - w1.dot(v)) /den;
	float beta =  (w1_dot_w2 * w1.dot(v) - w2.dot(u)) /den;
	float gamma_pow2 = (std::pow(u.norm(), 2) - std::pow(alpha, 2) - std::pow(beta,2) - 2*alpha*beta*w1.dot(w2)) / std::pow((w1.cross(w2)).norm(), 2);
	if(gamma_pow2 < 0 ){
		throw "imaginary solution";
	}
	float gamma = std::sqrt(gamma_pow2);
	/*
	 *  get two solutions... one is a bad root.... have to find out which one to use...
	 */
	for(int i=0; i<2; i++){
		int sign_gamma = (i == 1) ? 1 : -1; 
		Vector3f c = (alpha * w1 + beta * w2 + sign_gamma * gamma * w1.cross(w2)) + r;
		try{
			*theta1 = subproblem_1(-w1, q, c, r);
		}
		catch(const char* msg){
			continue;
		}
		try{
			*theta2 = subproblem_1(w2, p, c, r);
		}
		catch(const char* msg){
			continue;
		}
		auto c_check_1 = ZPUnitScrew(w1).exp(-*theta1) * q;
		auto c_check_2 = ZPUnitScrew(w2).exp(*theta2) * p;		
		if((c_check_1-c_check_2).norm() < 1e-4)
			// Have valid solution. Done!
			return;
	}
	throw "no solution found";
}

void subproblem_3(Vector3f w, Vector3f p, Vector3f q, Vector3f r, float delta, float *theta1, float *theta2){
	/* Let psi be a zero-pitch, unit magnitude twist: p, q in R^3 two points; and delta a real number > 0.
	 * Find theta such that
	 *   	||q - exp(psi_hat theta) p || = delta
	 */
	Vector3f u = p - r; // r point on rotation axis w
	Vector3f v = q - r;
	Vector3f u_prime = u - w * w.dot(u);
	Vector3f v_prime = v - w * w.dot(v);
	float delta_prime_pow_2 = pow(delta,2) - pow(fabs(w.dot(p-q)),2);
	float theta_0 = std::atan2(w.dot(u_prime.cross(v_prime)), u_prime.dot(v_prime));
	// law of cosines c^2 = a^2 + b^2 - 2ab cos(gamma)
	float u_prime_norm = u_prime.norm();
	float v_prime_norm = v_prime.norm();
	float phi = acos((pow(u_prime_norm, 2) + pow(v_prime_norm, 2) - delta_prime_pow_2)/ (2*u_prime_norm*v_prime_norm));
	// two solutions, are all valid? TODO 
	*theta1 = theta_0 + phi;
	*theta2 = theta_0 - phi;
}



