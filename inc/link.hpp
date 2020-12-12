/*
 * =====================================================================================
 *
 *       Filename:  link.hpp
 *
 *    Description: 
 *
 *        Version:  1.0
 *        Created:  2020-12-12 14:19:46
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef LINK_HPP 
#define LINK_HPP 

#include "rob_mat.hpp"
#include "joint.hpp"


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
#endif
