/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
 * website: www.robotcub.org
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

#include "iCub/torqueCtrlTest/controlThread.h"
#include "iCub/skinDynLib/common.h"

#include <yarp/os/Time.h>
#include <yarp/math/SVD.h>
#include <yarp/math/NormRand.h>
#include <iCub/ctrl/math.h>
#include <iostream>

using namespace yarp::math;
using namespace std;
using namespace iCub::skinDynLib;
using namespace iCub::torqueCtrlTest;

	
controlThread::controlThread(string _moduleName, string _robotName, int _period,
                             BodyPart _bodyPart, VerbosityLevel _verbose) 
: RateThread(_period), bodyPart(_bodyPart), verbose(_verbose)
{   
    name                = "torqueCtrlThread";
	ctrlMode            = NO_CONTROL;
    ctrlModeChanged     = true;
    cmdMode             = SIMULATION;
    interfaces          = new robot_interfaces(_moduleName.c_str(), _robotName.c_str(), bodyPart);    
    /*cmdMode             = REAL;*/    

    try{
        //---------------------PORTS-------------------------//	
        string slash = "/";
	    port_torques        = new BufferedPort<Vector>;
        port_monitor        = new BufferedPort<Vector>;
	    if(!port_torques->open((slash+_moduleName+"/torques:o").c_str()))
            throw runtime_error("It was not possible to open the torques:o port");    
	    if(!port_monitor->open((slash+_moduleName+"/monitor:o").c_str()))
            throw runtime_error("It was not possible to open the monitor:o port");

        durations.resize(12, 0.0);
        timestamps.resize(12, 0.0);

        pwm = pwmD = 0.0;
	    tao.resize(1);
        taod.resize(1);
        alpha = DEFAULT_ALPHA;
        jointId = DEFAULT_JOINT_ID;

        Vector Kp(1), Ki(1), Kd(1);
        Vector Wp(1), Wi(1), Wd(1);
        Vector N(1),  Tt(1);
        Matrix satLim(1,2);
        Kp=DEFAULT_PID_KP[0];
        Ki=DEFAULT_PID_KI[0];
        Kd=DEFAULT_PID_KD[0];
        Wp=Wi=Wd=1.0;
        N=10.0;
        Tt=1.0;
        satLim(0,0)= DEFAULT_SAT_LIM_DOWN;
        satLim(0,1)= DEFAULT_SAT_LIM_UP;
        torquePid = new parallelPID(_period/1000.0,Kp,Ki,Kd,Wp,Wi,Wd,N,Tt,satLim);
    }
    catch(runtime_error){
        this->threadRelease();
        throw;
    }
}

bool controlThread::threadInit(){
	thread_status = STATUS_OK;
    if(interfaces->init())
        return true;
    
    thread_status = STATUS_DISCONNECTED;
    return false;
}

void controlThread::threadRelease(){
    fprintf(stderr,"Control thread release.\n");
    fprintf(stderr, "Setting position control mode.\n");
    interfaces->icmd[bodyPart]->setPositionMode(jointId);

	Time::delay(0.5);

    if(port_torques)    { port_torques->interrupt(); port_torques->close(); delete port_torques;  port_torques = 0;}
    if(port_monitor)    { port_monitor->interrupt(); port_monitor->close(); delete port_monitor;  port_monitor = 0;}
}

void controlThread::run(){
    updateDurations(0);
    if(timestamps[0]-timestamps[1]> 0.050*100 && thread_status!=STATUS_DISCONNECTED && getIterations()>0){
        printf("Too much time between updateRobotStatus() and sanityCheck(): %3.3f\n", timestamps[0]-timestamps[1]);
        for(unsigned int i=0;i<durations.size();i++)
            printf("duration %d: %3.3f\n", i, durations[i]);
        for(unsigned int i=0;i<timestamps.size();i++)
            printf("timestamp %d: %3.3f\n", i, timestamps[i]);
    }
    // check everything is working fine
    if(thread_status==STATUS_DISCONNECTED || sanityCheck()==false){
        thread_status = STATUS_DISCONNECTED;
        return;
    }    

    // read encoders, compute e.e. pose and force, jacobian, etc
    updateRobotStatus();
    updateDurations(1);

    // CONTROL LOOP
    ctrlModeSem.wait();
    updateDurations(2);

    ControlMode currentCtrlMode = ctrlMode;
    if(ctrlModeChanged){
        printf("Control mode just changed to %s.\n", ControlMode_desc[currentCtrlMode].c_str());
        // if the ctrl mode has changed, initialize the new ctrl mode
        initCtrlMode(currentCtrlMode);
        ctrlModeChanged = false;
    }
    ctrlModeSem.post();
    updateDurations(5);

    computePwm(currentCtrlMode);
    updateDurations(6);

    sendPwm(currentCtrlMode);
    updateDurations(8);

    // send monitoring data   
    prepareMonitorData(currentCtrlMode);
    port_monitor->prepare() = monitorData;
    updateDurations(9);

    port_monitor->write();
    updateDurations(10);
}
void controlThread::updateDurations(int index){
    double now = Time::now();
    timestamps[index] = now;
    durations[index] = now-timePre;
    timePre = now;
}

void controlThread::prepareMonitorData(const ControlMode &cm){
    // *** PID monitoring ***
    monitorData.resize(3);
    monitorData[0]  = taod[0];
    monitorData[1]  = tao[0];
    monitorData[2]  = pwm;
}

void controlThread::updateRobotStatus(){    
    // *** JOINT TORQUES *** 
    double temp[20];
    if(!interfaces->itrq[bodyPart]->getTorques(temp)){
        printf("Get joint torques failed!\n");
    }
    else{
        tao[0] = temp[jointId];
    }
}

void controlThread::initCtrlMode(const ControlMode &cm){
    switch(cm){
        case NO_CONTROL:
            updateDurations(3);
            interfaces->icmd[bodyPart]->setPositionMode(jointId);
            updateDurations(4);
            break;
        case PID_CTRL:
        case OPEN_CTRL:
            if(cmdMode == REAL){
                updateDurations(3);
                printf("setting open loop ctrl mode\n");
                interfaces->icmd[bodyPart]->setOpenLoopMode(jointId);
                updateDurations(4);
            }
            break;
        default:
            fprintf(stderr, "[initCtrlMode] Unknown control mode: %d\n", cm);
    }
}
void controlThread::computePwm(const ControlMode &cm){    
    double newPwm;
    switch(cm){
        case NO_CONTROL:
            break;

        case PID_CTRL:
            {
                newPwm = torquePid->compute(taod, tao).operator ()(0);
                if(newPwm==DEFAULT_SAT_LIM_UP || newPwm==DEFAULT_SAT_LIM_DOWN)
                    printf("WARNING: pid is saturating!\n");
                break;
            }
        
        case OPEN_CTRL:
            {
                newPwm = pwmD;
                break;
            }

        default:
            fprintf(stderr, "[computePwm] Unknown control mode: %d\n", cm);
    }
    // low pass filter
    pwm = (1.0-alpha)*newPwm + alpha*pwm;
}
void controlThread::sendPwm(const ControlMode &cm){    
    if(cmdMode == REAL){
        //send the torque commands to the motors
        switch(cm){
            case PID_CTRL:
            case OPEN_CTRL:
                interfaces->icmd[bodyPart]->setOpenLoopMode(jointId);
                interfaces->iopl[bodyPart]->setOutput(jointId, pwm);
                break;
            case NO_CONTROL:
                break;
            default:
                fprintf(stderr, "[sendPwm] Unknown control mode: %d \n", cm);
        }       
    }

    // write torques on the output port
    port_torques->prepare() = pwm;
    updateDurations(7);
	port_torques->write();
}
bool controlThread::sanityCheck(){
    return true;
}
