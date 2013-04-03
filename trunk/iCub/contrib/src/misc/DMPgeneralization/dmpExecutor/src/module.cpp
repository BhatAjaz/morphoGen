/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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
#include <iostream>
#include <stdio.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include "iCub/module.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

#define RET_INVALID     -1
YARP_DECLARE_DEVICES(icubmod)
/**********************************************************/
DmpExecutor::DmpExecutor()
{
    YARP_REGISTER_DEVICES(icubmod)
    running=false;
    rightCartCtrl=NULL;
    leftCartCtrl=NULL;
   // currentState=NULL;
   // currentDMP=NULL;
  //  chooseBestHand=true;
    defaultHand=iCub::INDIFF;
    currentHand=iCub::RIGHT;
    
}
/**********************************************************/
bool DmpExecutor::attach(yarp::os::Port &source)
{
    return this->yarp().attachAsServer(source);
}
/**********************************************************/
bool DmpExecutor::run()
{
    //activate interface
    running=true;
    
}
/**********************************************************/
bool DmpExecutor::is_running()
{
    return running;
}
/**********************************************************/
bool DmpExecutor::stop()
{
    //stop interface
    running=false;
    bool ok= rightCartCtrl->stopControl();
    ok= leftCartCtrl->stopControl() && ok;
    //return ok; // need to check meaning of stopControl() return value
    return true;
}
/**********************************************************/
bool DmpExecutor::execute_OPC(const int32_t id)
{
    mutex.wait();
    currentDMP=opcPort.get_information_for(id);
    if (!currentDMP.isValid())
    {
        std::cout << "OPC returned an invalid DMP" << std::endl; fflush(stdout);
       // delete currentDMP;
        mutex.post();
        return false;
    }
    currentState.set_dof(currentDMP.get_dof());
    
   // std::cout << "DMP: "; currentDMP->print(); fflush(stdout);
    
    Vector x0,o0,v0,a0;
    
    
    
    if (defaultHand==iCub::INDIFF)
    {
        currentHand= ((currentDMP.get_goal())(1)<0) ? iCub::LEFT : iCub::RIGHT; // checking for position of target along Y coord of root frame
    }
        
    bool ok=true;
    switch (currentHand)
    { // TODO: ADD CHECKS ON CARTESIAN INTERFACE SUCCESS
        case iCub::LEFT:
            leftCartCtrl->getPose(x0,o0);
            leftCartCtrl->getTaskVelocities(v0,a0);
            ok=currentState.set_position(yarp::math::cat(x0, o0)) && ok;
            ok=currentState.set_velocity(yarp::math::cat(v0, a0)) && ok;
            break;
        case iCub::RIGHT:
            rightCartCtrl->getPose(x0,o0);
            rightCartCtrl->getTaskVelocities(v0,a0);
            ok=currentState.set_position(yarp::math::cat(x0, o0)) && ok;
            ok=currentState.set_velocity(yarp::math::cat(v0, a0)) && ok;
            break;
            
        default:
            ok=false;
    }

    if (ok)
        currentState.resetPhase();
    mutex.post();
    return ok;
}
/**********************************************************/    
void  DmpExecutor::set_hand(const iCub::Hand newHand)
{
    defaultHand=newHand;
    if (newHand != iCub::INDIFF)
        currentHand=newHand;
    
}
/**********************************************************/
iCub::Hand  DmpExecutor::get_hand()
{
    return defaultHand; //or current?? need to decide..
}
/**********************************************************/
bool DmpExecutor::waitMotionDone(const double period, const double timeout)
{
    double start=yarp::os::Time::now();
    while (currentState.get_phase()>0.0001 && (yarp::os::Time::now() -start) < timeout)
    {
        yarp::os::Time::delay(period);
    }
    
    if ((yarp::os::Time::now() -start) >timeout)
        return false;
    else
        return true;
}
/**********************************************************/
bool DmpExecutor::configure(ResourceFinder &rf)
{
    bool configOk=true;
    setName(rf.find("name").asString().c_str());
    string robot=rf.check("robot", yarp::os::Value("icub")).asString().c_str();
    
    period=rf.check("period", Value(1)).asDouble();
    std::cout << "period " << period <<std::endl;
    string slash="/";
 
    thriftPort.open((slash+string(getName().c_str())+"/thrift:rpc").c_str());   
    configOk = attach(thriftPort) && configOk;
    
    opcPort.open((slash+string(getName().c_str())+"/opc:rpc").c_str());
    

    Property optionR;
    optionR.put("device","cartesiancontrollerclient");
    optionR.put("remote",(slash + robot +"/cartesianController/right_arm").c_str());
    optionR.put("local",(slash + getName().c_str() +"/client/right_arm").c_str());
    

    rightCartCtrlPD.open(optionR);

    if (rightCartCtrlPD.isValid()) 
    {
        configOk=rightCartCtrlPD.view(rightCartCtrl) && configOk;
    }
    else configOk=false;

    Property optionL;
    optionL.put("device","cartesiancontrollerclient");
    optionL.put("remote",(slash + robot +"/cartesianController/left_arm").c_str());
    optionL.put("local",(slash + getName().c_str() +"/client/left_arm").c_str());
    
    leftCartCtrlPD.open(optionL);

    if (leftCartCtrlPD.isValid()) 
    {
        configOk=leftCartCtrlPD.view(leftCartCtrl) && configOk;
    }
    else configOk=false;
    
    
    return configOk;
}

/**********************************************************/
bool DmpExecutor::interruptModule()
{

    opcPort.interrupt();
    thriftPort.interrupt();

    return true;
}
/**********************************************************/
bool DmpExecutor::close()
{
    opcPort.close();
    thriftPort.close();
    leftCartCtrlPD.close();
    rightCartCtrlPD.close();
    return true;
}
/**********************************************************/
bool DmpExecutor::updateModule()
{
    if (isStopping())
        return false;
    mutex.wait();
    if(!running || !currentDMP.isValid() || !currentState.isValid())
    {
        mutex.post();
        return true;
    }
    currentDMP.integrate(currentState,getPeriod(), false, 500,false);
    
    if (currentState.get_phase()<1.0e-4)
        currentState.set_phase(0.0);
    
    Vector xodotd=currentState.get_velocity();
    Vector xod=currentState.get_position();
    switch (currentHand)
    { // TODO: ADD CHECKS ON CARTESIAN INTERFACE SUCCESS
        case iCub::LEFT:
            leftCartCtrl->setTaskVelocities(xodotd.subVector(0,2), xodotd.subVector(3,6));
            leftCartCtrl->goToPose (xod.subVector(0,2), xod.subVector(3,6));
            break;
        case iCub::RIGHT:
            rightCartCtrl->setTaskVelocities(xodotd.subVector(0,2), xodotd.subVector(3,6));
            rightCartCtrl->goToPose (xod.subVector(0,2), xod.subVector(3,6));
            break;
            
        default:
            {}
    }

    mutex.post();
    return true;
}
/**********************************************************/
double DmpExecutor::getPeriod()
{
   
    return period;
}