/*
 * =====================================================================================
 *
 *       Filename:  traj_generator.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020-12-07 20:39:17
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include <stdlib.h>
#include "rob_mat.hpp"
#include <cmath>

class BaseTrajGenerator
{
	protected:
		VectorXf _start, _end;
	public:
		VectorXf pos, spd, acc;
		float T;

		BaseTrajGenerator(VectorXf start, VectorXf end, float time) : 
			_start{start}, 
			_end{end}, 
			T{time},
			pos(start.size()),
			spd(start.size()),
			acc(start.size())
			{
				pos.setZero();
				spd.setZero();
				acc.setZero();
			
			}
		virtual void pos_speed_acc(float t) {};
};

class CubicPolynomialTrajGenerator : public BaseTrajGenerator
{	
	/*  s(t) = a0 + a_1 * t + a_2 * t^2 + a_3 * t_3 
	 *  theta(t)      = theta_start +	(3*t^2/T^2 - 2*T^3/T^3) * (theta_end - theta_start)
	 *  theta_dot(t)  =  				(6*t/T^2 - 6*t^2/T^3) * (theta_end - theta_start)
	 *  theta_dot2(t) =  				(6/T - 12*t/T^3) * (theta_end - theta_start)
	*/
	
	private:
		const float _pos_a_2, _pos_a_3, _spd_a_2, _spd_a_3, _acc_a_2, _acc_a_3;
		VectorXf _delta;

	public:
		CubicPolynomialTrajGenerator(VectorXf start, VectorXf end, float time) : 
			BaseTrajGenerator(start, end, time),
			_pos_a_2{3.0f / (float) pow(T, 2)},
			_pos_a_3{2.0f / (float) pow(T, 3)},
			_spd_a_2{_pos_a_2 * 2},
			_spd_a_3{_pos_a_3 * 3},
			_acc_a_2{_spd_a_2},
			_acc_a_3{_spd_a_3 * 2},
			_delta(end-start)
		{
		}
		
		void pos_speed_acc(float t){
			pos = _start + (_pos_a_2 * std::pow(t, 2) - _pos_a_3 * std::pow(t, 3)) * _delta;
			spd =  (_spd_a_2 * t - _spd_a_3 * std::pow(t, 2)) * _delta;
			acc =  (_acc_a_2 - _acc_a_3 * t) * _delta;
		}
};

class TrapzPolynomialTrajGenerator : BaseTrajGenerator{
	
	/*  
	 * 
	 */
	public:
		VectorXf v, a;
		TrapzPolynomialTrajGenerator(VectorXf start, VectorXf end, float T) :
				BaseTrajGenerator(start, end, T),
				v(start.size()),
				a(start.size())
		{
			// TODO: Implement other options... 
			/* Implementation 2nd option in 'Modern Robotics; Mechanics, Planning and Control.' which states:
			 * 		"Choose v and T such that 2 >= vT > 1"
			 * Implementation:
			 * 		Given T -> v = 2/T
			 *		a = v^2 / (v * T - 1 ) = (2/T)^2 / ((2/T) * T - 1) 
			 *		  = 4 / ( T^2 * (2 - 1)) = 4 / T^2
			 * */
			float v_val= 2/T;
			float a_val= 4/std::pow(T, 2);
			// set each joint... could fix this independetly...
			for(int i=0; i<v.size(); i++){
				v(i) = v_val;
				a(i) = a_val;
			}
		}

		void pos_speed_acc(float t){
			// could loop over joints
			for(int i=0; i<v.size(); i++){
				float v_a_ratio = v(i)/a(i);
				if(0<=t<=v_a_ratio){
					pos(i) = 0.5 * a(i) * std::pow(t, 2);
					spd(i) = a(i) * t;
					acc(i) = a(i);
				}
				else if( v_a_ratio < t <= T - v_a_ratio){
					pos(i) = v(i) * t - v_a_ratio * v(i) * 0.5;
					spd(i) = v(i);
					acc(i) = 0;
				}
				else
				{
					pos(i) = (2 * a(i) * v(i) * T - 2 * std::pow(v(i),2) - std::pow(a(i) * (t-T), 2)) / (2 * a(i));
					spd(i) = a(i) * (T - t);
					acc(i) = -a(i);
				}
			}
		}
};

// Base class doesn't suite this case...

class ViaPointTrajectory{
	
	public:
		
		MatrixXf ps;
		MatrixXf vs;
		VectorXf Ts;
		VectorXf dTs;

		ViaPointTrajectory(MatrixXf points, MatrixXf velocities, VectorXf times) :
			vs(velocities),
			Ts(times),
			ps(points),
			dTs(Ts.block(1, 0, Ts.size()-1, 1) - Ts.block(0,0,Ts.size()-1, 1)) 
		{
		}

		void pos_speed_acc(float t){
			for(int j=0; j<ps.rows(); j++){
				int i; // costly...
				for(i=0; i < Ts.size() && t < Ts(i); i++) { }
				if(i==Ts.size())
					throw "Time is out of bounce";
				float deltaT = Ts(i+1) - Ts(i); 
				// delta Tj
				float a_j0 = ps(i);
				float a_j1 = vs(i);
				float a_j2 = (3 * ps(j, i+1) - 3 * ps(j, i) - 2 * vs(j,i) * deltaT - vs(j,i+1) * deltaT);
				a_j2 = a_j2 / std::pow(deltaT, 2);
				float a_j3 = (2 * ps(j, i) + (vs(j,i)+vs(j,i+1)) * deltaT - 2 * ps(j, i+1));
				a_j3 = a_j3 / std::pow(deltaT, 3); 
				float B_j = a_j0 + a_j1 * deltaT + a_j2 * std::pow(deltaT, 2) + a_j3 * std::pow(deltaT, 3); 
			}
		}
};




