/*
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
 * email:  elena.ceseracciu@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
#include <sstream>
#include <stdio.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include "iCub/module.h"

#include <gsl/gsl_math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define RET_INVALID     -1

#define CMD_TRAIN               VOCAB4('t','r','a','i')
#define CMD_EXECUTE             VOCAB4('e','x','e','c')
#define CMD_TOOLEXTEND          VOCAB4('e','x','t','d')

/************************************a**********************/
bool DmpLearner::attach(yarp::os::Port &source)
{
    return this->yarp().attachAsServer(source);
}
/**********************************************************/
void DmpLearner::resetCurrent()
{
    currentMutex.wait();
    currentTrajectory.resize(0,0);
   // if (currentDMP)
   //     delete currentDMP;
   // currentDMP=NULL;
   // currentDMP=new DMPstructure(dof, N, alpha_x, alpha_z, beta_z);
    actionName="";
    currentTarget.clear();
    currentMutex.post();
}
/**********************************************************/
void DmpLearner::resetMap()
{
    for (std::map<std::string, DmpGPR*>::iterator mapIt=learnersMap.begin(); mapIt!=learnersMap.end(); ++mapIt)
    {
        if(mapIt->second)
            delete mapIt->second;
    }
    learnersMap.clear();
}
/**********************************************************/
bool DmpLearner::sync_opc()
{
    std::vector<int32_t> actionIds=opcPort.get_all_actions_with_traj_endeff();
    bool ok= true;
    for (size_t actCt=0; actCt<actionIds.size(); ++actCt)
    {
        ok= estimate_DMP(actionIds.at(actCt)) && ok;
    }
    return ok;
}
/**********************************************************/
bool DmpLearner::estimate_DMP(const int32_t id)
{
   
    if (!opcPort.get_information_for(id))
        return false;

    if (currentTrajectory.rows()==0) // TODO check what other checks are needed
        return false;

     currentDMP=DMPstructure(dof, N, alpha_x, alpha_z, beta_z);
    if (!currentDMP.estimate_from_example(currentTrajectory, false, false, true))
        return false;
    // TODO should add to learnersMap? how do I retrieve the actions' name?
    if (learnersMap.find(actionName)==learnersMap.end())
        learnersMap[actionName]=new DmpGPR(N, dof, currentTarget.size());

    learnersMap.at(actionName)->feedSample(currentTarget, currentDMP);
    return opcPort.addActionDMP(id, currentDMP);
    //newDMP.print(); //can be commented out...
}
/**********************************************************/
bool DmpLearner::train_ids(const std::vector<int32_t> & trainInputIds)
{
    //resetMap();
    if (learnersMap.find("CUSTOM")!=learnersMap.end())
        delete learnersMap.at("CUSTOM");
    learnersMap["CUSTOM"]=new DmpGPR(N, dof, currentTarget.size());
    bool ok=true;
    for (size_t actCt=0; actCt<trainInputIds.size(); ++actCt)
    {
        ok= !opcPort.get_information_for(trainInputIds.at(actCt)) && ok;
        if(ok && currentDMP.isValid() )
        {
            learnersMap.at("CUSTOM")->feedSample(currentTarget, currentDMP);
        }
    }
    ok=learnersMap.at("CUSTOM")->inference() && ok;
    return false;
}
/**********************************************************/
bool DmpLearner::train_action(const std::string& actionName)
{
    if (learnersMap.find(actionName)!=learnersMap.end())
        return false;
    else 
    {
        return learnersMap.at(actionName)->inference();
    }
}
/**********************************************************/
bool DmpLearner::generalize_DMP(const int32_t id)
{
    if (!opcPort.get_information_for(id))
        return false;
    if ( actionName== "" || learnersMap.find(actionName)==learnersMap.end())
        return false;
    
     DMPstructure newDMP= learnersMap.at(actionName)->generalize(currentTarget);
     return opcPort.addActionDMP(id, newDMP);
}
/**********************************************************/
void DmpLearner::set_num_basis_functions(const int32_t N)
{
    this->N=N;
    resetCurrent();
    resetMap();  
}
/**********************************************************/
void DmpLearner::set_alphax(const double alphax)
{
    this->alpha_x=alphax;
    resetCurrent();
    resetMap();     
}
/**********************************************************/
void DmpLearner::set_alphaz(const double alphaz)
{
    this->alpha_z=alphaz;
    resetCurrent();
    resetMap(); 
}
/**********************************************************/
void DmpLearner::set_betaz(const double betaz)
{
    this->beta_z=betaz;
    resetCurrent();
    resetMap();
}

/**********************************************************/
DmpLearner::DmpLearner()
{
    alpha_x=0.0;
    alpha_z=0.0;
    beta_z=0.0;
    N=0;
    dof=0;
   // currentDMP=NULL;
}

/**********************************************************/
DmpLearner::~DmpLearner()
{
    resetCurrent();
    resetMap();
}

/**********************************************************/
bool DmpLearner::configure(ResourceFinder &rf)
{
    name=rf.find("name").asString().c_str();
    //incoming
    opcPort.open(("/"+name+"/opc:rpc").c_str());
    opcPort.setLearner(this);

    //outgoing
    N=rf.check("N", Value(27)).asInt();
    alpha_x= rf.check("alphax", Value(10.0)).asDouble();
    alpha_z=rf.check("alphaz", Value(1.0)).asDouble();
    beta_z=rf.check("betaz", Value(1.0)).asDouble();

    //rpc 
    thriftPort.open(("/"+name+"/rpc").c_str());   
    attach(thriftPort);

    return true;
}

/**********************************************************/
bool DmpLearner::interruptModule()
{
    thriftPort.interrupt();
    opcPort.interrupt();

    return true;
}
/**********************************************************/
bool DmpLearner::close()
{
    thriftPort.close();
    opcPort.close();

    return true;
}

/**********************************************************/
bool DmpLearner::updateModule()
{
    if (isStopping())
        return false;

    return true;
}

/**********************************************************/
double DmpLearner::getPeriod()
{
    return 0.1;
}


