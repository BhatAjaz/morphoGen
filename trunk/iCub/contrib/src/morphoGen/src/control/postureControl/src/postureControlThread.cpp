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


const double postureControlThread::armMax[] = {-10.0, 70.0,  30.0, 80.0,  50.0,   0.0 };
const double postureControlThread::armMin[] = {-70.0, 20.0, -20.0, 20.0, -50.0, -30.0 };  
const double postureControlThread::torsoMin[] = {-50.0, -30.0, -10.0};  
const double postureControlThread::torsoMax[] = {50.0, 30.0, 70.0};

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
    bool ok = initController();
    if(!ok){
        return false;
    }
    
    if (!interfaceIn.open(getName("/interface:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    interfaceIn.setStrict(true);
    
    if (!inputLeftArm.open(getName("/leftArm:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
   
    if (!inputRightArm.open(getName("/rightArm:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    return true;
    

}

bool postureControlThread::initController() {
    Property options;

    // ================== instantiate right arm ==================================
    
    options.put("device", "remote_controlboard");
    options.put("local", "/postureControl/client/rightarm");                 //local port names
    string remoteRightArm("/");
    remoteRightArm.append(robot.c_str());
    remoteRightArm.append("/right_arm");
    options.put("remote",(const char*)remoteRightArm.c_str());      //where we connect to
    
    robotDeviceRightArm = new PolyDriver(options);
   
    if (!robotDeviceRightArm->isValid()) {
      printf("Device Right Arm not available.  Here are the known devices:\n");
      printf("%s", Drivers::factory().toString().c_str());
      return 0;
    }

    bool okRight;
    okRight = robotDeviceRightArm->view(posRightArm);
    okRight = okRight && robotDeviceRightArm->view(encsRightArm);
    okRight = okRight && robotDeviceRightArm->view(ictrlRightArm);
    okRight = okRight && robotDeviceRightArm->view(iimpRightArm);
    okRight = okRight && robotDeviceRightArm->view(itrqRightArm);
    if (!okRight) {
        printf("Problems acquiring interfaces from the right arm\n");
        return 0;
    }
    else {
        printf("Ok. proceed! \n");
    }

    
    Time::delay(0.01);
    cout<<"after"<<endl;

    // checking the readings
    encodersRightArm.resize(16);
    bool getRightCorrect = encsRightArm->getEncoders(encodersRightArm.data());
    printf("initial encoders position (%s) \n",encodersRightArm.toString().c_str());
    
    
    if(!getRightCorrect){
        printf("just read crap from encoders \n");
        return 0;
    }
    else{
        printf("correct encoders \n");
    }

    // ================== instantiate left arm ==================================
    
    options.put("device", "remote_controlboard");
    options.put("local", "/postureControl/client/leftarm");                 //local port names
    string remoteLeftArm("/");
    remoteLeftArm.append(robot.c_str());
    remoteLeftArm.append("/left_arm");
    options.put("remote",(const char*)remoteLeftArm.c_str());      //where we connect to
    
    robotDeviceLeftArm = new PolyDriver(options);
   
    if (!robotDeviceLeftArm->isValid()) {
      printf("Device Left Arm not available.  Here are the known devices:\n");
      printf("%s", Drivers::factory().toString().c_str());
      return 0;
    }

    bool okLeft;
    okLeft = robotDeviceLeftArm->view(posLeftArm);
    okLeft = okLeft && robotDeviceLeftArm->view(encsLeftArm);
    okLeft = okLeft && robotDeviceLeftArm->view(ictrlLeftArm);
    okLeft = okLeft && robotDeviceLeftArm->view(iimpLeftArm);
    okLeft = okLeft && robotDeviceLeftArm->view(itrqLeftArm);
    if (!okLeft) {
        printf("Problems acquiring interfaces from the left arm\n");
        return 0;
    }

    // ================== instantiate torso =====================================
    
    options.put("device", "remote_controlboard");
    options.put("local", "/postureControl/client/torso");                 //local port names
    string remoteTorso("/");
    remoteTorso.append(robot.c_str());
    remoteTorso.append("/torso");
    options.put("remote",(const char*)remoteTorso.c_str());      //where we connect to
    
    robotDeviceTorso = new PolyDriver(options);
   
    if (!robotDeviceTorso->isValid()) {
      printf("Device Torso not available.  Here are the known devices:\n");
      printf("%s", Drivers::factory().toString().c_str());
      return 0;
    }

    bool okTorso;
    okTorso = robotDeviceTorso->view(posTorso);
    okTorso = okTorso && robotDeviceLeftArm->view(encsTorso);
    okTorso = okTorso && robotDeviceLeftArm->view(ictrlTorso);
    okTorso = okTorso && robotDeviceLeftArm->view(iimpTorso);
    okTorso = okTorso && robotDeviceLeftArm->view(itrqTorso);
    if (!okTorso) {
        printf("Problems acquiring interfaces from the left arm\n");
        return 0;
    }

    // ================= interfacing to robot parts ==========================

    int jnts = 0;
    posRightArm->getAxes(&jnts);
    jntsRightArm = jnts;
    
    Vector tmp;
    Vector command_velocity;

    tmp.resize(jnts);
    encodersRightArm.resize(jnts);

    // setting reference acceleration
    int i;
    for (i = 0; i < jnts; i++) {
        tmp[i] = 50.0;
    }    
    posRightArm->setRefAccelerations(tmp.data());
    // setting reference speed
    for (i = 0; i < jnts; i++) {
        tmp[i] = 10.0;
        posRightArm->setRefSpeed(i, tmp[i]);
    }
    //==================== checking the readings ===================================
     

    Vector command_position;
    posRightArm->getAxes(&jntsRightArm);
    printf("got the number of axis initialization %d \n", jntsRightArm);
    Vector torques;
    torques.resize(jntsRightArm);
    itrqRightArm->getTorques(torques.data());
    printf("got the reading of the torques %s \n", torques.toString().c_str());

    // =================== setting impedence control  ===============================

    double stiffness = 0.081;       // stiffness coefficient, units are Nm/deg
    double damping   = 0.020;       // damping coefficient,   units are Nm/(deg/s)
    double offset    = 0.0;         // torque offset,         units are Nm
    bool okImpP, okImpO, okImp;
    
    if(strcmp(robot.c_str(),"icubSim")){
        okImpP = iimpRightArm->setImpedance(3, stiffness, damping);  
        okImpO = iimpRightArm->setImpedanceOffset(3,offset);
        okImp  = ictrlRightArm->setImpedancePositionMode(3);
        
        if(okImpP & okImp & okImpO) {
            printf("success in sending switching to impedence mode control \n");
        }
        else {
            printf("Error! in sending switching to impedence mode control \n");
        return false;
        }
    }
        
    // =================== setting torque control ==================================

    //ictrlLeftArm->setTorqueMode(3);
    //double jnt_torque= 0.0; //expressed in Nm
    //itrqLeftArm->setRefTorque(3,jnt_torque); 

    if(strcmp(robot.c_str(),"icubSim")){

        okImpP = iimpLeftArm->setImpedance(3, stiffness, damping);  
        okImpO = iimpLeftArm->setImpedanceOffset(3,offset);
        okImp  = ictrlLeftArm->setImpedancePositionMode(3);
        
        if(okImpP & okImp & okImpO) {
            printf("success in sending switching to impedence mode control \n");
        }
        else {
            printf("Error! in sending switching to impedence mode control \n");
            return false;
        }
    
    }

    //==================== moving to default position ==============================
    
    command_position.resize(jntsRightArm);
    command_position[0]  = -30;
    command_position[1]  = 30;
    command_position[2]  = 0;
    command_position[3]  = 45;
    command_position[4]  = 0;
    command_position[5]  = 0;
    command_position[6]  = 0;
    command_position[7]  = 15;
    command_position[8]  = 30;
    command_position[9]  = 4;
    command_position[10] = 1;
    command_position[11] = 9;
    command_position[12] = 0;
    command_position[13] = 4;
    command_position[14] = 1;
    command_position[15] = 1;
    
    printf("sending command %s \n", command_position.toString().c_str());
    
    
    posRightArm->positionMove(command_position.data());
    
    return true;
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


const bool postureControlThread::checkTorso(const Bottle* b) {
    printf("dimension of the received bottle %d \n", b->size());
        
    if (b->size() > 3) {
        printf("out of limit size \n");
        return false;
    }
    
    for(int i = 0; i < b->size() ; i++){
        double d = b->get(i).asDouble();
        if((d < torsoMin[i]) || (d > torsoMax[i])) {
            printf("d value %f out of limit \n", d);
            return false;        
        }
    }
        
    return true;
}

const bool postureControlThread::checkArm(const Bottle* b) {
    printf("dimension of the received bottle %d \n", b->size());
    printf("componets of the received bottle  \n");
    //Bottle* values =   b->get(0).asList();

    //printf("content of the bottle %s \n", values->toString().c_str());
    //printf("content comprises %d values \n", values->size());
        
    if (b->size() > 16) {
        printf("out of limit size \n");
        return false;
    }
    
    for(int i = 0; i < b->size() ; i++){
        double d = b->get(i).asDouble();
        if((d < armMin[i]) || (d > armMax[i])) {
            printf("d value %f out of limit \n", d);
            return false;        
        }
    }
        
    return true;
}

Vector postureControlThread::parseBottle(Bottle* b, int dim) {
    //Bottle* values =   b->get(0).asList();
    Vector result(dim); 
    encsRightArm->getEncoders(encodersRightArm.data());
    printf("encoders position \n (%s) \n",encodersRightArm.toString().c_str());
    for (int i = 0; i < dim ; i++) {
        if( i < b->size()) {
            result[i] = b->get(i).asDouble();
        }
        else{            
            result[i] = encodersRightArm(i);
        }
    }    
    return result;
}

void postureControlThread::setInputPortName(string InpPort) {
    
}

void postureControlThread::run() {
    while (isStopping() != true) {

        Bottle* receivedBottle;

        //**********************************************************
        if (interfaceIn.getInputCount()) {
            
            
            receivedBottle = interfaceIn.read(false);
            if(receivedBottle != NULL){
                
                int bodypart  = receivedBottle->get(0).asVocab();
                Bottle* value = receivedBottle->get(1).asList();
                //Bottle lvalue;
                //Bottle& rvalue = lvalue.addList();
                //rvalue = *value;
                                
                switch(bodypart){
                case COMMAND_RIGHT_ARM: {
                    printf("################################### \n");
                    printf("pending reads %d \n",interfaceIn.getPendingReads());
                    printf("right_arm: %s \n", value->toString().c_str());
                    //printf("right_arm: %s \n", lvalue.toString().c_str());
                    bool rightArmOk = checkArm(value);

                    if(rightArmOk) {

                        int jnts = 0;
                        //Vector command_position;
                        Vector command_position = parseBottle(value, 16);                                            
                        command_position.resize(jntsRightArm);
                        printf("jnt dimension %d \n", jntsRightArm);
                        
                        printf("sending command\n %s \n", command_position.toString().c_str());
                        //printf("temporary command\n %s \n", command_position_temp.toString().c_str());
                        
                        bool ok; 
                        //ok = posRightArm->positionMove(command_position.data());
                        if(!ok){
                            break;
                        }
                    }
                    else {
                        printf("detected out of limits control \n");
                    }
                    
                } break;
                case COMMAND_LEFT_ARM:  {
                    printf("################################### \n");
                    printf("pending reads %d \n",interfaceIn.getPendingReads());
                    printf("left_arm: %s \n", value->toString().c_str());
                    bool leftArmOk = checkArm(value);

                    if(leftArmOk) {

                        int jnts = 0;
                        //Vector command_position;
                        Vector command_position = parseBottle(value, 16);                                            
                        command_position.resize(jntsLeftArm);
                        printf("jnt dimension %d \n", jntsLeftArm);
                        
                        printf("sending command\n %s \n", command_position.toString().c_str());
                        //printf("temporary command\n %s \n", command_position_temp.toString().c_str());
                        
                        bool ok;
                        //ok = posLeftArm->positionMove(command_position.data());
                        if(!ok){
                            break;
                        }
                    }
                    else {
                        printf("detected out of limits control \n");
                    }
                    
                } break;
                case COMMAND_TORSO_ARM: {
                    printf("################################### \n");
                    printf("pending reads %d \n",interfaceIn.getPendingReads());
                    printf("torso: %s \n", value->toString().c_str());
                    bool torsoOk = checkTorso(value);

                    if(torsoOk) {

                        int jnts = 0;
                        //Vector command_position;
                        Vector command_position = parseBottle(value, 3);                                            
                        command_position.resize(jntsTorso);
                        printf("jnt dimension %d \n", jntsTorso);
                        
                        printf("sending command\n %s \n", command_position.toString().c_str());
                        //printf("temporary command\n %s \n", command_position_temp.toString().c_str());
                        
                        bool ok;
                        //ok = posTorso->positionMove(command_position.data());
                        if(!ok){
                            break;
                        }
                    }
                    else {
                        printf("detected out of limits control \n");
                    }
                    
                } break;
                case COMMAND_RIGHT_HAND:{
                    printf("################################### \n");
                    printf("pending reads %d \n",interfaceIn.getPendingReads());
                    printf("right_hand: %s \n", value->toString().c_str());
                    bool rightArmOk = checkArm(value);

                    if(rightArmOk) {

                        int jnts = 0;
                        //Vector command_position;
                        Vector command_position = parseBottle(value, 16);                                            
                        command_position.resize(jntsRightArm);
                        printf("jnt dimension %d \n", jntsRightArm);
                        
                        printf("sending command\n %s \n", command_position.toString().c_str());
                        //printf("temporary command\n %s \n", command_position_temp.toString().c_str());
                        
                        bool ok;
                        //ok = posRightArm->positionMove(command_position.data());
                        if(!ok){
                            break;
                        }
                    }
                    else {
                        printf("detected out of limits control \n");
                    }
                    
                } break;
                case COMMAND_LEFT_HAND:  {
                    printf("################################### \n");
                    printf("pending reads %d \n",interfaceIn.getPendingReads());
                    printf("left_hand: %s \n", value->toString().c_str());
                    bool rightArmOk = checkArm(value);
                    
                    if(rightArmOk) {
                        
                        int jnts = 0;
                        //Vector command_position;
                        Vector command_position = parseBottle(value, 16);                                            
                        command_position.resize(jntsRightArm);
                        printf("jnt dimension %d \n", jntsRightArm);
                        
                        printf("sending command\n %s \n", command_position.toString().c_str());
                        //printf("temporary command\n %s \n", command_position_temp.toString().c_str());
                        
                        bool ok;
                        //ok = posRightArm->positionMove(command_position.data());
                        if(!ok){
                            break;
                        }
                    }
                    else {
                        printf("detected out of limits control \n");
                    }
                    
                } break;
                default:{
                    printf("command not recognized \n");
                }
                }
            }
        }
        

        //**********************************************************
        if (inputRightArm.getInputCount()) {
            
            receivedBottle = inputRightArm.read(false);
            if(receivedBottle != NULL){
                printf("Bottle %s \n", receivedBottle->toString().c_str());

                bool rightArmOk = checkArm(receivedBottle);

                if(rightArmOk) {

                    int jnts = 0;
                    //Vector command_position;
                    Vector command_position = parseBottle(receivedBottle, 16);                    
                    
                    command_position.resize(jntsRightArm);
                    
                    printf("jnt dimension %d \n", jntsRightArm);
                    
                    /*
                    command_position[0]  = -45;
                    command_position[1]  = 65;
                    command_position[2]  = 0;
                    command_position[3]  = 15;
                    command_position[4]  = 0;
                    command_position[5]  = 0;
                    command_position[6]  = 0;
                    command_position[7]  = 15;
                    command_position[8]  = 30;
                    command_position[9]  = 4;
                    command_position[10] = 1;
                    command_position[11] = 9;
                    command_position[12] = 0;
                    command_position[13] = 4;
                    command_position[14] = 1;
                    command_position[15] = 1;
                    */
                    
                    printf("sending command\n %s \n", command_position.toString().c_str());
                    //printf("temporary command\n %s \n", command_position_temp.toString().c_str());

                    bool ok = posRightArm->positionMove(command_position.data());
                    if(!ok){
                        break;
                    }
                }
                else {
                    printf("detected out of limits control \n");
                }
            }
        }

        //**************************************************************

        if (inputLeftArm.getInputCount()) {
            
            receivedBottle = inputLeftArm.read(false);
            if(receivedBottle != NULL){
                printf("Bottle %s \n", receivedBottle->toString().c_str());

                bool leftArmOk = checkArm(receivedBottle);

                if(leftArmOk) {

                    int jnts = 0;
                    //Vector command_position;
                    Vector command_position = parseBottle(receivedBottle, 16);                    
                    
                    command_position.resize(jntsRightArm);
                    
                    printf("jnt dimension %d \n", jntsRightArm);
                    
                    /*
                    command_position[0]  = -45;
                    command_position[1]  = 65;
                    command_position[2]  = 0;
                    command_position[3]  = 15;
                    command_position[4]  = 0;
                    command_position[5]  = 0;
                    command_position[6]  = 0;
                    command_position[7]  = 15;
                    command_position[8]  = 30;
                    command_position[9]  = 4;
                    command_position[10] = 1;
                    command_position[11] = 9;
                    command_position[12] = 0;
                    command_position[13] = 4;
                    command_position[14] = 1;
                    command_position[15] = 1;
                    */
                    
                    printf("sending command\n %s \n", command_position.toString().c_str());
                    //printf("temporary command\n %s \n", command_position_temp.toString().c_str());

                    bool ok = posLeftArm->positionMove(command_position.data());
                    if(!ok){
                        break;
                    }
                }
                else {
                    printf("detected out of limits control \n");
                }
            }
        }

        Time::delay(0.1);
    }               
}

void postureControlThread::threadRelease() {
    robotDeviceRightArm->close();
    robotDeviceLeftArm->close();
    printf("postureControlThread::threadRelease \n");  
}

void postureControlThread::onStop() {
    
    ictrlRightArm->setPositionMode(3);
    ictrlLeftArm->setPositionMode(3);

    printf("postureControlThread::onStop \n");
    inputRightArm.interrupt();
    inputLeftArm .interrupt();
    interfaceIn  .interrupt();
    printf("interrupted the port \n");
    inputRightArm .close();
    inputLeftArm  .close();
    interfaceIn   .close();
    printf("postureControlThread::onStop \n");
}

