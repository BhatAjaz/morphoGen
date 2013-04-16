// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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

/**
 * @file postureControlThread.cpp
 * @brief Implementation of the eventDriven thread (see postureControlThread.h).
 */

#include <iCub/postureControlThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

postureControlThread::postureControlThread() {
    robot = "icub";        
}

postureControlThread::postureControlThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

postureControlThread::~postureControlThread() {
    // do nothing
}

bool postureControlThread::threadInit() {

    //initialization of the controller
    initController();
    
   
    if (!inputPort.open(getName("/posture:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    

    return true;
    

}

bool postureControlThread::initController() {
    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/postureControl/client");                 //local port names
    string remoteRightArm("/");
    remoteRightArm.append(robot.c_str());
    remoteRightArm.append("/right_arm");
    options.put("remote",(const char*)remoteRightArm.c_str());         //where we connect to
    
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
      printf("Device not available.  Here are the known devices:\n");
      printf("%s", Drivers::factory().toString().c_str());
      return 0;
    }

    bool ok;
    ok = robotDevice.view(posRightArm);
    ok = ok && robotDevice.view(encsRightArm);
    ok = ok && robotDevice.view(ictrlRightArm);
    ok = ok && robotDevice.view(iimpRightArm);
    ok = ok && robotDevice.view(itrqRightArm);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int jnts = 0;
    posRightArm->getAxes(&jnts);
    
    Vector tmp;
    
    
    Vector command_velocity;

    tmp.resize(jnts);
    encodersRightArm.resize(jnts);

    int i;
    for (i = 0; i < jnts; i++) {
        tmp[i] = 50.0;
    }    
    posRightArm->setRefAccelerations(tmp.data());
    for (i = 0; i < jnts; i++) {
        tmp[i] = 10.0;
        posRightArm->setRefSpeed(i, tmp[i]);
    }
    
    encsRightArm->getEncoders(encodersRightArm.data());
}

void postureControlThread::setName(string str) {
    this->name=str;
    printf("name: %s \n", name.c_str());
}


std::string postureControlThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void postureControlThread::setInputPortName(string InpPort) {
    
}

void postureControlThread::run() {    
    while (isStopping() != true) {
        if (inputPort.getInputCount()) {
            printf("Checking the port \n");
            Bottle* receivedBottle = inputPort.read(true);
            if(receivedBottle!=NULL){
                printf("Bottle %s \n", receivedBottle->toString().c_str());
                Vector command_position;
                //bool ok = posRightArm->positionMove(command_position.data());
            }
        }
        Time::delay(0.1);
    }               
}

void postureControlThread::threadRelease() {
    // nothing
    printf("postureControlThread::threadRelease \n");
     
}

void postureControlThread::onStop() {
    printf("postureControlThread::onStop \n");
    inputPort.interrupt();
    printf("interrupted the port \n");
    inputPort.close();
    printf("postureControlThread::onStop \n");
}

