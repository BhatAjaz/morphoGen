/* 
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "iCub/DMP/DmpGPR.h"
#include <iostream>
#include <yarp/math/Math.h>


void DmpGPR::reset()
{
    this->init=false;
    for (std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin(); gpIt!=gprs.end(); ++gpIt)
           gpIt->reset();
    
}

bool DmpGPR::checkConstants(const DMPstructure &y)
{
    if(!init)
    {
        alpha_x=y.get_alphax();
        alpha_z=y.get_alphaz();
        beta_z=y.get_betaz();
        N_=y.get_num_basis();
        dof_=y.get_dof();
        init=true;
        return true;
    }
    else
    {
        return y.get_alphax()==this->alpha_x && y.get_alphaz()==this->alpha_z && y.get_betaz()==this->beta_z && y.get_num_basis() ==this->N_ && y.get_dof()==this->dof_;
    }
    
}

bool DmpGPR::feedSample(const yarp::sig::Vector &x, const DMPstructure &y)
{   
    if (!checkConstants(y))
        return false; //should print out message as well?
    if (x.size()!=m_)
        return false; //should print out message as well?
    yarp::sig::Vector dmpParams(0);
    for (int i=0; i<y.get_dof(); ++i)
    {
        dmpParams = yarp::math::cat(dmpParams, y.get_weights(i));
    }
    for (int i=0; i<y.get_dof(); ++i)
    {
        dmpParams.push_back(y.get_goal(i));
    }
   // dmpParams=yarp::math::cat(dmpParams, y.get_goal()); // complains that: error: passing ‘const DMPstructure’ as ‘this’ argument of ‘yarp::sig::Vector DMPstructure::get_goal()’ discards qualifiers [-fpermissive]
    dmpParams.push_back(y.get_tau());
    for (size_t j=0; j<dmpParams.length(); ++j)
        gprs.at(j).feedSample(x, yarp::sig::Vector(1, dmpParams(j)));  
    sampleCount++;
    return true;
}

bool DmpGPR::inference()
{
    for (std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin(); gpIt!=gprs.end(); ++gpIt)
           gpIt->train();
    return true; //should add "bool train" method to GPRLearner ?
}

bool DmpGPR::inference(const std::vector <yarp::sig::Vector> & x, const std::vector<DMPstructure> & y, bool optimizeHyperparameters)
{
    if (x.size()!=y.size())
    {
        std::cout<< "vector of targets x and vector of dmp parameters y must have the same size!"<< std::endl;
        return false;        
    }
    // feed samples
    std::vector<yarp::sig::Vector>::const_iterator xIt=x.begin();
    for ( std::vector<DMPstructure>::const_iterator yIt=y.begin();xIt<x.end() && yIt<y.end(); ++xIt, ++yIt)
    {
        feedSample(*xIt, *yIt);

        //feedSample(*xIt, );
    }
   if (optimizeHyperparameters)
   {
       double tol= 0.01;
       int max_iter=100;
       unsigned int verb=0;
       bool useHessian=false;
       for (std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin(); gpIt!=gprs.end(); ++gpIt)
           gpIt->optimize(tol, max_iter, verb, useHessian);
    
   }
   
   return inference();
}


std::vector<DMPstructure> DmpGPR::generalize(const std::vector <yarp::sig::Vector> & xs)
{
    std::vector<DMPstructure> ys;
    for (std::vector <yarp::sig::Vector>::const_iterator xsIt=xs.begin(); xsIt!=xs.end(); ++xsIt)
    {
       
        ys.push_back(generalize(*xsIt));
    }
    return ys;
}

DMPstructure DmpGPR::generalize(const yarp::sig::Vector &xs)
{
    DMPstructure ys_i(dof_, N_,alpha_x, alpha_z, beta_z);
    yarp::sig::Vector weightsPred(0);
    for (std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin(); gpIt!=gprs.end(); ++gpIt)
    {
        iCub::learningmachine::Prediction pred =gpIt->predict(xs);
        weightsPred= yarp::math::cat(weightsPred, pred.getPrediction());      
    }
    
    for (int dofCt=0; dofCt<dof_; ++dofCt)
    {
       // std::cout << "dof: " << dofCt;
        yarp::sig::Vector dofWeights=weightsPred.subVector(dofCt*N_, (dofCt+1)*N_ -1);
       // std::cout << "weights " << dofWeights.toString();
        ys_i.set_weights(dofCt, dofWeights);
      //  std::cout << "set_weights " << std::endl;
    }
    ys_i.set_goal(weightsPred.subVector(dof_*N_,dof_*(N_+1)-1));
    ys_i.set_tau(weightsPred(weightsPred.length()-1));    
    return ys_i;
}

DmpGPR::DmpGPR(int N, int dof, int m, double sf2) : gprs((N+1)*dof+1, iCub::learningmachine::GPRLearner(m, 1, sf2))
//DmpGPR::DmpGPR(int N, int dof, double sf2) : gprs(dof, iCub::learningmachine::GPRLearner(N*dof+1, 1, sf2))
{
    
    dof_ = dof;
    N_=N;
    m_=m;
   // gprs.resize(dof, GPR(D, n));
    verbose=true;
    alpha_x=0.0;
    alpha_z=0.0;
    beta_z=0.0;
    init=false;
    sampleCount=0;
    
}



DmpGPR::~DmpGPR()
{}


void DmpGPR::print()
{
for(std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin();gpIt<gprs.end(); ++gpIt) 
    {
        std::cout <<gpIt->toString()<<std::endl;
    }    
}

int DmpGPR::getNumberOfSamples()
{
    return sampleCount;
}