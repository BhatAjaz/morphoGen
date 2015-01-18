// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Ajaz Ahmad Bhat
  * email: ajaz.bhat@iit.it
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
 * @file visualizationControlModule.cpp
 * @brief Implementation of the visualizationControlModule (see header file).
 */

#include "visualizationControlModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define COMMAND_VOCAB_HELP               VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_OK                 VOCAB2('o','k')
#define COMMAND_VOCAB_FAILED             VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_CSH                VOCAB3('C','S','H')
#define COMMAND_VOCAB_REM                VOCAB3('R','E','M')
#define COMMAND_VOCAB_NEU                VOCAB3('N','E','U')
#define COMMAND_VOCAB_ON                 VOCAB2('O','N')
#define COMMAND_VOCAB_OFF                VOCAB3('O','F','F')
#define COMMAND_VOCAB_VIS                VOCAB3('V','I','S')
#define COMMAND_VOCAB_OFF                VOCAB3('O','F','F')
#define COMMAND_VOCAB_IIT                VOCAB3('I','I','T')
#define COMMAND_VOCAB_NEU                VOCAB3('N','E','U')
#define COMMAND_VOCAB_REM                VOCAB3('R','E','M')
#define COMMAND_VOCAB_CSH                VOCAB3('C','S','H')
#define COMMAND_VOCAB_GRA                VOCAB3('G','R','A')
#define COMMAND_VOCAB_VIS                VOCAB3('V','I','S')

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool visualizationControlModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */   

    
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/visualizationControl"), 
                           "module name (string)").asString();
     
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";

    inputPortName           = rf.check("inputPortName",
			                Value(":i"),
                            "Input port name (string)").asString();
    

    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 
    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port
    if (rf.check("config")) {
        configFile=rf.findFile(rf.find("config").asString().c_str());
        if (configFile=="") {
            return false;
        }
    }
    else {
        configFile.clear();
    }

    Bottle*     partialBottle[5], *remBottle[5], *hubObject,*hubBody, *hubBottom[5], *planA, *planB,*weight,*hubBodyObs,*hubObjectObs,*hubActionObs,*hubCWS;
    Semaphore*  pmutex[5], *rmutex[5], *mutexObject, *mutexBody, *mutexBottom[5], *mutexA, *mutexB, *weightMutex,*mutexObjectObs,*mutexBodyObs, *mutexActionObs, *mutexCWS;
    
    for (int i = 0; i < 5; i++) {
        partialBottle[i]    = new Bottle();
        remBottle[i]        = new Bottle();
        hubBottom[i]        = new Bottle();
        pmutex[i]           = new Semaphore();
        rmutex[i]           = new Semaphore();
        mutexBottom[i]      = new Semaphore();
    }
    
    hubObject       = new Bottle();
    hubBody		    = new Bottle();
    planA           = new Bottle();
    planB           = new Bottle();
    hubObjectObs    = new Bottle();
    hubBodyObs		= new Bottle();
	hubActionObs	= new Bottle();
	hubCWS			= new Bottle();
    weight			= new Bottle();

	
	mutexObject     = new Semaphore();
    mutexBody	    = new Semaphore();
    mutexA          = new Semaphore();
    mutexB          = new Semaphore();
	mutexObjectObs  = new Semaphore();
    mutexBodyObs    = new Semaphore();
	mutexActionObs	= new Semaphore();
	mutexCWS		= new Semaphore();
	weightMutex     = new Semaphore();

    if (!emPlotterPort.open(getName("/emPlotter/interface:o").c_str())) {
        cout << ": unable to open port to receive partial cue bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    if (!kclPort.open(getName("/kcl/interface:o").c_str())) {
        cout << ": unable to open port to receive remembered bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!cvutForthPort.open(getName("/cvutForth/interface:o").c_str())) {
        cout << ": unable to open port to receive remembered bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!neuralPort.open(getName("/neuralVis/interface:o").c_str())) {
        cout << ": unable to open port to receive remembered bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    /* create the thread and pass pointers to the module parameters */
    visualizationControlRatethread* rThreadPointer;
    for(int i = 0; i< dimensionControl; i++){
        rThreadPointer = new visualizationControlRatethread(robotName, configFile);
        string nameStr("input");
        sprintf((char*)nameStr.c_str(), "input%d",i );
        rThreadPointer->setName(getName(nameStr.c_str()).c_str());
        rThread[i] = rThreadPointer;
        rThread[i]->start();
    }
     
    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool visualizationControlModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool visualizationControlModule::close() {
    handlerPort.close();
    emPlotterPort.close();
    kclPort.close();
    cvutForthPort.close();

    /* stop the thread */
    printf("stopping the threads \n");

    for(int i = 0; i < dimensionControl; i++){
        printf("stopping the thread %d \n", i);
        rThread[i]->stop();
    }
    
    printf("success in stopping the threads \n");

    return true;
}

bool visualizationControlModule::respond(const Bottle& command, Bottle& reply) 
{
    

    bool ok = false;
    bool rec = false; // is the command recognized?

    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    
    mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("many");
            reply.addString("help");

            //reply.addString();
            reply.addString("set fn \t: general set command ");
            reply.addString("get fn \t: general get command ");
            //reply.addString();

            
            //reply.addString();
            reply.addString("seek red \t : looking for a red color object");
            reply.addString("seek rgb \t : looking for a general color object");
            reply.addString("sus  \t : suspending");
            reply.addString("res  \t : resuming");
            //reply.addString();


            ok = true;
        }break;
        case COMMAND_VOCAB_CSH:
        rec = true;
        {
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_ON:{
                printf("CSH ON \n");
                Bottle send, response;
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_CSH);
                send.addVocab(COMMAND_VOCAB_ON);
                emPlotterPort.write(send,reply);
                
            }break;
                
            case COMMAND_VOCAB_OFF:{
                printf("CSH OFF \n");
                Bottle send, response;
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_CSH);
                send.addVocab(COMMAND_VOCAB_OFF);
                emPlotterPort.write(send,reply);
            }break;
            }
            
            ok = true;
        }break;
        case COMMAND_VOCAB_NEU:
        rec = true;
        {
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_ON:{
                printf("NEU ON \n");
                Bottle send, response;
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_ON);
                neuralPort.write(send,reply);
            }break;
                
            case COMMAND_VOCAB_OFF:{
                printf("NEU OFF \n");
                Bottle send, response;
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_OFF);
                neuralPort.write(send,reply);
            }break;
            }
            
            ok = true;
        }break;
        case COMMAND_VOCAB_REM:
        rec = true;
        {
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_ON:{
                printf("REM ON \n");
                Bottle send, response;
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_REM);
                send.addVocab(COMMAND_VOCAB_ON);
                emPlotterPort.write(send,reply);
            }break;
                
            case COMMAND_VOCAB_OFF:{
                printf("REM OFF \n");
                 Bottle send, response;
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_REM);
                send.addVocab(COMMAND_VOCAB_OFF);
                emPlotterPort.write(send,reply);
            }break;
            }
            
            ok = true;
        }break;
        case COMMAND_VOCAB_GRA:
        rec = true;
        {
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_ON:{
                printf("GRA ON \n");
                Bottle send, response;
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_ON);
                kclPort.write(send,reply);
            }break;
                
            case COMMAND_VOCAB_OFF:{
                printf("GRA OFF \n");
                Bottle send, response;
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_OFF);
                kclPort.write(send,reply);
            }break;
            }
            
            ok = true;
        }break;
        case COMMAND_VOCAB_VIS:
        rec = true;
        {
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_ON:{
                printf("VIS ON \n");
                Bottle& send = cvutForthPort.prepare();
                send.clear();
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_ON);
                cvutForthPort.write();
            }break;
                
            case COMMAND_VOCAB_OFF:{
                printf("VIS OFF \n");
                Bottle& send = cvutForthPort.prepare();
                send.clear();
                send.addVocab(COMMAND_VOCAB_VIS);
                send.addVocab(COMMAND_VOCAB_OFF);
                cvutForthPort.write();
            }break;
            }
            
            ok = true;
        }break;
    default: {            
    }break;    
    }

    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool visualizationControlModule::updateModule()
{
    return true;
}

double visualizationControlModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}

