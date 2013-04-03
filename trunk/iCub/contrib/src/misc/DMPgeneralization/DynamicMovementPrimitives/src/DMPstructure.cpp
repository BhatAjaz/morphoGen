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

#include <cmath>
#if defined(_MSC_VER)
  #define copysign _copysign
#endif 
#include <stddef.h>
#include <string.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/Time.h>

#include "iCub/DMP/DMPstructure.h"



//const int servo_rate = 500;
//const int servo_rate = 100;
using namespace std;
using namespace yarp::sig;
using namespace yarp::math;

DMPstructure::DMPstructure()
{
  dof = 0; 
  N = 0;
  alpha_x = alpha_z = beta_z = 0.0;
  tau = 0.0;
}

DMPstructure::DMPstructure(int dof_, int N_, double alpha_x_, double alpha_z_, double beta_z_)
: goal(dof_, 0.0), c(N_, 0.0), sigma2(N_, 0.0), w(dof_, N_)
{
  dof = dof_; N = N_;
  alpha_x = alpha_x_; alpha_z = alpha_z_; beta_z = beta_z_;
  tau = 0.0;

  w.zero();

  define_basis_functions(N, 1);
}


DMPstructure::~DMPstructure()
{}

bool DMPstructure::isValid()
{
    return dof>0 && N >0 && tau>0;
}
void DMPstructure::print(void)
{
  int i, j;

  printf("\ndof = %ld, N = %ld\n", dof, N);
  printf("alpha_x = %.2f, alpha_z = %.2f, beta_z = %.2f\n", alpha_x, alpha_z, beta_z);
  printf("tau = %.3f\n", tau);
  printf("goal = ( %s )\n", goal.toString().c_str());
  printf("c = ( %s )\n", c.toString().c_str());
  printf("sigma2 = ( %s )\n", sigma2.toString().c_str());
  printf("w = ( %s )\n", w.toString().c_str());
}

static const double epsilon = 1.0e-16;

bool DMPstructure::integrate(DMPstate &state, double dt, bool scaling, int servo_rate, bool verbose)
{
    if (state.dof_!=dof)
        return false;
    size_t steps = (size_t) (dt *servo_rate + 0.5);
  
     if (verbose)
        printf("dt: %f, steps:%ld\t", dt, steps); fflush(stdout);
     
     if (steps <=0)
         return false;
    double stepdt = dt / steps;
   
    
    // Integrate DMP variables using Euler's method
    for (size_t k = 0; k < steps; k++)
    {
        double sum_psi=0.0;
        Vector fx(dof); //should put it before FOR cycle?
        fx.zero();
        for (size_t i = 0; i < N; i++)
        {
           // double psi = exp( -0.5 * pow((state.x-c(i))/sigma2(i), 2));
           double psi=exp(-0.5* pow((state.x - c(i)), 2) / sigma2(i));
            fx+= w.getCol(i)*psi;

            sum_psi += psi;
        }
        
        if (state.x < c(N-1) || fabs(sum_psi) < epsilon)
            fx.zero();
        else
            fx *= state.x/sum_psi;
        
        if (scaling)
        {
            for (size_t j = 0; j < dof; j++)
            {
                
                double EPS=1.0e-4;
                double g=goal(j);
                g=(abs(g)>EPS ? g : copysign(EPS,g));        
                 fx(j) *= g;
            }
        }

        Vector dz = alpha_z * (beta_z * ( goal - state.y) -  state.z) + fx;

        // Euler integration: new positions (y) and scaled velocities (z)
        state.y+=state.z*stepdt/tau;
        state.z+=dz*stepdt/tau;

        // phase integration
        double dx = -alpha_x * state.x / tau;
        state.x += dx*stepdt;
        
    }
    
    if (verbose)
        printf("phase: %f\n", state.x);
    return true;
}


bool DMPstructure::set_weights(int j,yarp::sig::Vector &input)
{  
    return w.setRow(j, input);;
}

double DMPstructure::get_weight(int j, int k) const
{
   return w (j, k);
}

yarp::sig::Vector DMPstructure::get_weights(int j)  const
{
    return w.getRow(j);
}

double DMPstructure::get_goal(int i)  const
{
    return goal(i);
    
}

yarp::sig::Vector DMPstructure::get_goal()
{
    return goal;
}

void DMPstructure::set_goal(const yarp::sig::Vector &newGoal)
{
    this->goal=newGoal;
    
}

// bool DMPstructure::save(char *file_name)
// {
// FILE *f = NULL;
// int i, j;
// 
//   if (file_name != NULL)
//     f = fopen(file_name, "w");
// 
//   if (f != NULL)
//   {
//     fprintf (f, "%d %d %d\n", dof, N, periodic);
// 
//     fprintf(f, "%le %le %le\n", alpha_x, alpha_z, beta_z);
// 
//     fprintf(f, "%le\n", tau);
// 
//     for (j = 0; j < dof; j++)
//       fprintf(f, "%le ", goal[j]);
//     fprintf(f, "\n");
// 
//     for (i = 0; i < N; i++)
//       fprintf(f, "%le ",c[i]);
//     fprintf(f, "\n");
// 
//     for (i = 0; i < N; i++)
//       fprintf(f, "%le ",sigma2[i]);
//     fprintf(f, "\n");
// 
//     for (j = 0; j < dof; j++)
//     {
//       for (i=0; i < N; i++)
//         fprintf(f, "%le ", w[j][i]);
//       fprintf(f, "\n");
//     }
//     fclose(f);
//   }
// }

bool DMPstructure::estimate_from_example(yarp::sig::Matrix &traj, bool back, bool scaling, bool verbose)
{
    if (traj.cols() !=dof+1)
        return false;
    size_t nSamples = traj.rows();
    Matrix traj_data(nSamples, 3*dof+1);
    traj_data.zero();
    
    
   double start, end;    
    if (verbose)
    {
        printf("DMP estimation ... "); fflush(stdout);
        start=yarp::os::Time::now();
    }

    if (!back)
    {
        traj_data.setSubmatrix(traj, 0, 0);
    }        
    else // compute back trajectory instead
    {
        for (size_t l = 0; l <= dof; l++) 
        {
            for (size_t j = 0; j < traj.rows(); j++)
                traj_data(j, l)= traj (traj.rows()-1-j, l);
            for (size_t j = 1; j < traj.rows(); j++)
                traj_data(j, 0)=  traj_data(0,0) - traj_data( j,0);
            traj_data(0, 0)= 0.0;
        }
    }
    traj_data.setCol(0, traj_data.getCol(0)-traj_data(0, 0));

    //velocities (central differences)
    traj_data.setSubmatrix(traj_data.submatrix(2, nSamples-1, 1, dof)-traj_data.submatrix(0, nSamples-3, 1,  dof), 1, dof+1);
    //accelerations
    traj_data.setSubmatrix(traj_data.submatrix(2, nSamples-1, dof+1, 2*dof)-traj_data.submatrix(0, nSamples-3, dof+1, 2*dof), 1, 2*dof+1);

    
    Vector timeDiffs=traj_data.subcol(1,0,nSamples-2)- traj_data.subcol(0, 0, nSamples-2);
    for (size_t dofCt = 0; dofCt < dof; ++dofCt)
    {
        traj_data.setSubcol(traj_data.getCol(dofCt+dof+1).subVector(1, nSamples-2)/timeDiffs, 1, dofCt+dof+1);
        traj_data.setSubcol(traj_data.getCol(dofCt+2*dof+1).subVector(1, nSamples-2)/timeDiffs, 1, dofCt+2*dof+1);
    }

    Matrix A(nSamples, N);
    tau = traj_data(nSamples-1,0);
    goal=traj_data.subrow(nSamples-1, 1, dof);
    
    for (size_t j = 0; j < nSamples; ++j)
    {
        double x = exp(-alpha_x / tau * traj_data(j,0));
        std::cout << "x :" << x << std::endl; fflush(stdout);
        double psi_sum = 0;
        for (size_t k = 0; k < N; k++)
        {
          //  double psi=exp(-0.5 * pow((x - c(k)) / sigma2(k), 2));
          double psi=exp(-0.5* pow((x - c(k)), 2) / sigma2(k));
            A (j, k)= psi;
            psi_sum += psi;
        }
        A.setRow(j, A.getRow(j)*x/psi_sum); // should check that psi_sum !=0 ?
    
    }
    Matrix f(dof, nSamples);
    for (size_t j = 0; j < nSamples; j++)
    {
        for (size_t k = 0; k < dof; k++)
        {
            f(k, j)= tau* tau * traj_data(j,2*dof+1+k) - alpha_z * (beta_z * (goal (k) - traj_data(j, k+1)) -
                             tau * traj_data(j,dof+1+k));
         }
    }

    Matrix Ainv=pinv(A);
    //check rank?
    
    for (size_t k = 0; k < dof; k++)
    {
        w.setRow(k, Ainv*f.getRow(k));
    }
    
           
//   if (scaling) //ATTN to add
//   {
//   double g;
//     for (k = 1; k <= dof; k++)
//     {
//       if (goal[k-1] > EPS || goal[k-1] < -EPS)
//         g = goal[k-1];
//       else if (goal[k-1] < 0)
//         g = -EPS;
//       else
//         g = EPS;
//       for (i = 1; i <= N; i++)
//         w_t[k][i] = w_t[k][i] / g;
//     }
//   }

    if (scaling)
    {
        for (size_t j = 0; j < dof; j++)
        {
            
            double EPS=1.0e-4;
            double g=goal(j);
            g=(abs(g)>EPS ? g : copysign(EPS,g));        
            for (size_t k=0; k<w.cols(); ++k)
                w[j][k]/=g;
        }
    }

    if (verbose)
    {
        printf("tau: %.3f\n", tau);
        end=yarp::os::Time::now();
        double mtime=(end-start)*1000;
        printf("finished (elapsed time = %f milliseconds).\n", mtime);
    }

    return (nSamples > 0);
}

void DMPstructure::make_linear()
{
   for (size_t i = 0; i < dof; i++)
     for (size_t j = 0; j < N; j++)
         w(i, j)=0.0;
}


double DMPstructure::distance_kernel(yarp::sig::Vector &query, yarp::sig::Vector &goal, double scale)
{
size_t j;
double d;
  d = 0;
  for (j = 0; j < query.size(); j++)
  {
    d += pow(query(j) - goal(j), 2);
  }
  d = sqrt(d) / scale;

  if (d < 1)
    d = (1 - d*d*d)*(1 - d*d*d)*(1 - d*d*d);
  else
    d = 0;
  return d;
}

void DMPstructure::define_basis_functions(int n, double factor)
{
size_t i;

  if (n != N)
  {
    N = n;
    
    c.resize(N);
    sigma2.resize(N);
    w.resize(dof, N);
  }

  for (i = 0; i < N; i++)
      c(i)= exp(-alpha_x*i/(N-1.0));
  for (i = 0; i < N - 1; i++)
      sigma2(i)=  pow((c(i+1)-c(i))*factor, 2);
  sigma2 (N-1)= sigma2(N-2);

  w.zero();
}
