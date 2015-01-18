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
 * @file neuroVisModule.cpp
 * @brief Implementation of the neuroVisModule (see header file).
 */

#include "neuroVisModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool neuroVisModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */   

    
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/neuronsVisualization"), 
                           "module name (string)").asString();
    /* get the module name which will form the stem of all module port names */
    width            = rf.check("width", 
                           Value(320), 
                           "image width size (int)").asInt();
    /* get the module name which will form the stem of all module port names */
    height            = rf.check("height", 
                           Value(240), 
                           "image height size (int)").asInt();                       
    
     
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
    
 

    /* create the thread and pass pointers to the module parameters */
    rThread = new neuroVisRatethread(robotName, configFile);
    rThread->setName(getName().c_str());
    //rThread->setInputPortName(inputPortName.c_str());
     

    /* now start the thread to do the work */
    
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()
    //sThread->start();
  
        
    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool neuroVisModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool neuroVisModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the threads \n");

    
    rThread->stop();

    return true;
}

bool neuroVisModule::respond(const Bottle& command, Bottle& reply) 
{
    bool ok = false;
    bool rec = false; // is the command recognized?

    string helpMessage =  string(getName().c_str()) + 
                " commands are: \n" +  
                "help \n" +
                "quit \n";
    reply.clear(); 

    //if (command.get(0).asString()=="quit") {
    //    reply.addString("quitting");
    //    return false;     
    //}
    //else if (command.get(0).asString()=="help") {
    //    cout << helpMessage;
    //    reply.addString("ok");
    //}
 respondLock.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("help");
            reply.addString("commands are:");
            reply.addString(" help    : to get help");
            reply.addString(" quit    : to quit the module");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" sus     : to suspend the processing");
            reply.addString(" res     : to resume  the processing");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" VIS ON  : to enable  the visualization");
            reply.addString(" VIS OFF : to disable the visualization");
            reply.addString("    ");
            reply.addString(" test    : automatic test of the features of the module");
            reply.addString(" ");
            reply.addString(" ");
            //reply.addString(helpMessage.c_str());
            ok = true;
        }
        break;
    case COMMAND_VOCAB_QUIT:
        rec = true;
        {
            reply.addString("quitting");
            
            ok = true;
        }
        break;
    case COMMAND_VOCAB_VIS:
        rec = true;
        {
            reply.addString("vision");
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_ON:{
                reply.addString("on");
                rThread->switchVis(true);
                printf("VIS ON \n");
            }break;
            case COMMAND_VOCAB_OFF:{
                reply.addString("off");
                rThread->switchVis(false);
                printf("VIS OFF \n");
            }break;
            }
            
            ok = true;
        }
        break;
    default:
        rec = false;
        ok  = false;
    }

    respondLock.post();

    if (!rec){
        ok = RFModule::respond(command,reply);
    }
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);
    
    return true;
    
}

/* Called periodically every getPeriod() seconds */
bool neuroVisModule::updateModule()
{
    return true;
}

double neuroVisModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}

