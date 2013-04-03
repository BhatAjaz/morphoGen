/* 
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author:  Elena Ceseracciu
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "iCub/DMP/DMPstate.h"

DMPstate::DMPstate()
{
    x=-1.0;
}
DMPstate::DMPstate(int dof):dof_(dof), y(dof, 0.0), z(dof, 0.0)
{
    resetPhase();
}

DMPstate::~DMPstate()
{}

void DMPstate::set_phase(double newPhase)
{
    if (newPhase>1.0)
        newPhase=1.0;
    if (newPhase<0.0)
        newPhase=0.0;
    this->x=newPhase;
}

bool DMPstate::set_position(const yarp::sig::Vector& newPosition)
{
    if (newPosition.size() == this->y.size())
	{
        this->y=newPosition;
		return true;
	}
	else return false;
};

bool DMPstate::set_velocity(const yarp::sig::Vector& newVelocity)
{
        if (newVelocity.size() == this->z.size())
        {
        this->z=newVelocity;
                return true;
        }
        else return false;
    
}

void DMPstate::set_dof(int dof)
{
    dof_=dof;
    y.resize(dof); y.zero();
    z.resize(dof); z.zero();
    x=-1.0;
    
}
double DMPstate::get_phase()
{
    return x;
}
yarp::sig::Vector DMPstate::get_position()
{
    return y;
}
yarp::sig::Vector DMPstate::get_velocity()
{
    return z;
}

double DMPstate::get_position_at(size_t ind)
{
    return y(ind);
}

double DMPstate::get_velocity_at(size_t ind)
{
    return z(ind);
}

void DMPstate::resetPhase()
{
    x=1.0;
}

int DMPstate::get_dof()
{
    return dof_;
}

bool DMPstate::isValid()
{
    return x>0.0;
}
