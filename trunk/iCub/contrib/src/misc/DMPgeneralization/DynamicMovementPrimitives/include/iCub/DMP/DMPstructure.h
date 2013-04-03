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

#ifndef _DMP_DMP_STRUCTURE_H_

#define _DMP_DMP_STRUCTURE_H_

#include <string>

#include <fstream>
#include <sstream>
#include <iostream>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include "iCub/DMP/DMPstate.h"

/**
* Dynamic Motion Primitive representation for a trajectory
*/
class DMPstructure {
private:
  // DMP parameters
  size_t dof, N;
  double alpha_x, alpha_z, beta_z;
  yarp::sig::Vector goal;
  double tau;
  yarp::sig::Vector c;
  yarp::sig::Vector sigma2;
  yarp::sig::Matrix w;
  // int periodic;

public:
   DMPstructure();
  /**
  * Constructor.
  *
  * @param[in] dof degrees of freedom of original trajectories 
  * @param[in] N number of basis functions
  * @param[in] alpha_x alpha_x time constant in DMP dynamic system
  * @param[in] alpha_z alpha_x time constant in DMP dynamic system
  * @param[in] beta_z beta_z time constant in DMP dynamic system
  * 
  */
  DMPstructure(int dof, int N, double alpha_x, double alpha_z, double beta_z);
  ~DMPstructure();

  /**
  * Print DMP parameters to screen
  */
  void print(void);
  /**
  * Integrate the dynamical equations to update the provided DMPstate
  * @param[in,out] state DMPstate that is updated
  * @param[in] dt length of integration step
  * @param[in] scaling set to true to normalize trajectory
  * @param[in] servo_rate rate of actual integration steps
  * @param[in] verbose print information to screen while executing 
  * @return true if successful
  */
  bool integrate(DMPstate &state, double dt, bool scaling = false, int servo_rate=500, bool verbose=false);
  /**
  * Estimate DMP parameters from an example trajectory
  * @param[in] traj trajectory matrix. First column is the time, then each column represents a degrees of freedom. 
  * The matrix has as many rows as the number of samples, and as many columns as the number of degrees of freedom +1
  * @param[in] back length of integration step
  * @param[in] scaling set to true to use normalized trajectory
  * @param[in] verbose print information to screen while executing 
  * @return true if successful
  */
  bool estimate_from_example(yarp::sig::Matrix &traj, bool back = false, bool scaling = false, bool verbose=false);



 // bool save(std::string file_name);
  /**
  * Get goal for the i-th degree of freedom
  */
  double get_goal(int i) const;
  /**
  * Get goal vector (each element is a degree of freedom)
  */
  yarp::sig::Vector get_goal();
  /**
  * Set goal vector
  */
  void set_goal(const yarp::sig::Vector &newGoal);
  /**
  * Get tau (temporal scaling factor) value
  */
  double get_tau() const { return tau; };
  /**
  * Get the number of degrees of freedom of the original trajectory
  */
  int get_dof() const { return dof; };
  /**
  * Get the number of basis functions of the non-linear phase-dependent term
  */
  int get_num_basis() const { return N; };
  //void get_back_conf(int i, double *conf);
  //yarp::sig::Vector get_back_conf(int i);
  /**
  * Get the weight associated with the k-th basis function of the j-th degree of freedom
  */
  double get_weight (int j, int k) const;
  /**
  * Get the weight associated to the basis functions for the j-th degree of freedom
  */
  yarp::sig::Vector get_weights(int j) const;
  /**
  * Set the weight associated to the basis functions for the j-th degree of freedom
  */
  bool set_weights(int j, yarp::sig::Vector &input);
  /**
  * Set tau (temporal scaling factor) value
  */
  void set_tau(double tau_) { tau = tau_; }
  /**
  * Get value of alpha_x time constant
  */
  double get_alphax() const {return alpha_x;};
  /**
  * Get value of alpha_z time constant
  */
  double get_alphaz() const {return alpha_z;};
  /**
  * Get value of beta_z time constant
  */
  double get_betaz() const {return beta_z;};
  
  bool isValid();
  
private:
  void define_basis_functions(int n, double factor);
  double distance_kernel(yarp::sig::Vector &query, yarp::sig::Vector &goal, double scale);
  void make_linear();
};

#endif
