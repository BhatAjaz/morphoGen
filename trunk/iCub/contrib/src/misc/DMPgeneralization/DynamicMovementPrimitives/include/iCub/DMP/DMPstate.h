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

#ifndef _DMP_DMP_STATE_H_

#define _DMP_DMP_STATE_H_

#include <string>

#include <fstream>
#include <sstream>
#include <iostream>

#include <yarp/sig/Vector.h>

class DMPstructure; //forward-declaration
class DMPstate
{
protected:
  double x; //phase
  yarp::sig::Vector y; //position for each degree of freedom
  yarp::sig::Vector z;//velocity for each degree of freedom
  int dof_;
public:
  DMPstate();
  DMPstate(int dof);
  ~DMPstate();
  void set_dof(int dof);
  void set_phase(double newPhase);
  bool set_position(const yarp::sig::Vector& newPosition);
  bool set_velocity(const yarp::sig::Vector& newVelocity);
  double get_phase();
  yarp::sig::Vector get_position();
  yarp::sig::Vector get_velocity();
  double get_position_at(size_t ind);
  double get_velocity_at(size_t ind);
  void resetPhase();
  int get_dof();
  bool isValid();
  friend class DMPstructure;
};
#endif
