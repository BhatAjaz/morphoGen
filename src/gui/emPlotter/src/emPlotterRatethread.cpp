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
 * @file emPlotterRatethread.cpp
 * @brief Implementation of the eventDriven thread (see emPlotterRatethread.h).
 */

#include <emPlotterRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100 //ms

emPlotterRatethread::emPlotterRatethread():RateThread(THRATE) {
    robot = "icub";        
}

emPlotterRatethread::emPlotterRatethread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

emPlotterRatethread::~emPlotterRatethread() {
    // do nothing
}

bool emPlotterRatethread::threadInit() {

    //printf("emPlotterRatethread::threadInit \n");
    idle = false;    

    if (!inputPortcue.open(getName("/cue:i").c_str())) {
        cout << ": unable to open port to receive partial cue bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!inputPortrem.open(getName("/rememberedX:i").c_str())) {
        cout << ": unable to open port to receive remembered bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    if (!inputPorthub.open(getName("/hub:i").c_str())) {
        cout << ": unable to open port to receive hub bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  
    
    if (!inputPortplan.open(getName("/plan:i").c_str())) {
        cout << ": unable to open port to receive plan bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!inputPorthubType.open(getName("/observerHub:i").c_str())) {
        cout << ": unable to open port to receive plan bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    //printf("ports opened\n");
    
    /*if (!Network::connect("/MyRemembered:o","/emPlotter/rememberedX:i"))
        return false;    
   //printf("Connection to rememberedX");
    if (!Network::connect("/Useful/PastXperiences:o","/emPlotter/cue:i"))
        return false;
    //if (!Network::connect("/Hub:o","/emPlotter/hub:i"))
    //   return false;
    */
    //Network::connect("/PlanXplore:o","/emPlotter/plan:i");
        //printf("connection NOT successful\n");
       
       
           
    return true;
    

}

void emPlotterRatethread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string emPlotterRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void emPlotterRatethread::setInputPortName(string InpPort) {
        
}

void emPlotterRatethread::setSharingBottle(Bottle *partialBot[], Bottle *remBot[], Bottle *hubBot[], Bottle* object, Bottle *body, Bottle *a, Bottle *b,Bottle *w, Bottle* x, Bottle *y, Bottle *z, Bottle *cws ) {

    for (int i = 0; i < 5; i++) {
        pbot[i] = partialBot[i];
        rbot[i] = remBot[i];
        hbot[i] = hubBot[i];
    }
    hubObject          =   object;
    hubBody    =   body;
    planA           =   a;
    planB           =   b;
	weight			=	w;
	hubObjectObs       =   x;
    hubBodyObs			=   y;
	hubActionObs		=   z;
	hubCWS               = cws;
}

void emPlotterRatethread::setSemaphore(Semaphore *pmu[], Semaphore *rmu[], Semaphore *hmu[], Semaphore *c, Semaphore *d, Semaphore *a, Semaphore *b, Semaphore *w, Semaphore *e, Semaphore *f, Semaphore *g, Semaphore *h) {
    
    for (int i = 0; i < 5; i++) {
        pmutex[i] = pmu[i];
        rmutex[i] = rmu[i];
        hmutex[i] = hmu[i];
    }
    mutexObject        =   c;
    mutexBody  =   d;
    mutexA          =   a;
    mutexB          =   b;     
    weightMutex		=	w;
	mutexObjectObs     =   e;
    mutexBodyObs  =   f;
	mutexActionObs  =   g;
	mutexCWS  =   h;
}

void emPlotterRatethread::run() {    

    cueIncoming     = inputPortcue.read(false);
    remIncoming     = inputPortrem.read(false);
    hubIncoming     = inputPorthub.read(false);
    planIncoming    = inputPortplan.read(false);
    hubTypeIncoming     = inputPorthubType.read(false);

    if (cueIncoming!=NULL){
        //printf(" bottle %s \n",cueIncoming->toString().c_str()); 
        string name;
        name = cueIncoming->get(0).asString();
                   
        if(!strcmp(name.c_str(), "cue0")) {
			printf("correctly detected the cue0 \n");
            if (cueIncoming->size() > 0){
                pmutex[0]->wait();
                *pbot [0] = *cueIncoming->get(1).asList();
                pmutex[0]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "cue1")) {
            printf("correctly detected the cue1 \n");    
            if (cueIncoming->size() > 0){
                pmutex[1]->wait();
                *pbot [1] = *cueIncoming->get(1).asList();
                //printf("bot1 %08x got secondary list %s \n",pbot[1], pbot[1]->toString().c_str());
                pmutex[1]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "cue2")) {
			printf("correctly detected the cue2 \n");
            if (cueIncoming->size() > 0){
                pmutex[2]->wait();
                *pbot [2] = *cueIncoming->get(1).asList();
                pmutex[2]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "cue3")) {
			printf("correctly detected the cue3 \n");
            if (cueIncoming->size() > 0){
                pmutex[3]->wait();
                *pbot [3] = *cueIncoming->get(1).asList();
                pmutex[3]->post();
            }                   
        }
                
        else if(!strcmp(name.c_str(), "cue4")) {
			printf("correctly detected the cue4 \n");
            if (cueIncoming->size() > 0){
                pmutex[4]->wait();
                *pbot [4] = *cueIncoming->get(1).asList();
                pmutex[4]->post();
            }                 
        }        
        else 
            printf("Please provide a partial cue with a name like \"cue0\"   \"cue1\"   \"cue2\"   \"cue3\"    \"cue4\" \n");          
    }

          
    if (remIncoming!=NULL){           
        //printf(" \n remembered bottle %s \n",remIncoming->toString().c_str());               
        string name;
        name = remIncoming->get(0).asString();      
        if(!strcmp(name.c_str(), "rem0")) {
			printf("correctly detected the rememberedExp0 \n");
            if (remIncoming->size() > 0){
                rmutex[0]->wait();
                *rbot [0] = *remIncoming->get(1).asList();
                rmutex[0]->post();
            }               
        }
                
        else if(!strcmp(name.c_str(), "rem1")) {
			printf("correctly detected the rememberedExp1 \n");
            if (remIncoming->size() > 0){
                rmutex[1]->wait();
                *rbot [1] = *remIncoming->get(1).asList();
                rmutex[1]->post();
            }                    
        }
        
        else if(!strcmp(name.c_str(), "rem2")) {
			printf("correctly detected the rememberedExp2 \n");
            if (remIncoming->size() > 0){
                rmutex[2]->wait();
                *rbot [2] = *remIncoming->get(1).asList();
                rmutex[2]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "rem3")) {
			printf("correctly detected the rememberedExp3 \n");
            if (remIncoming->size() > 0){
                rmutex[3]->wait();
                *rbot [3] = *remIncoming->get(1).asList();
                rmutex[3]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "rem4")) {
			printf("correctly detected the rememberedExp4 \n");
            if (remIncoming->size() > 0){
                rmutex[4]->wait();
                *rbot [4] = *remIncoming->get(1).asList();
                rmutex[4]->post();
            }                    
        }
                               
        else 
            printf("Please provide a remembered experience with a name like \"rem0\"   \"rem1\"   \"rem2\"   \"rem3\"    \"rem4\" \n");
    }
    
    
    
    if (planIncoming!=NULL){
        //printf(" bottle %s \n",planIncoming->toString().c_str()); 
        string name;
        name = planIncoming->get(0).asString();
                   
       if(!strcmp(name.c_str(), "plan1")) {
		   printf("correctly detected the plan1 \n");
            if (planIncoming->size() > 0){
                mutexA->wait();
                *planA = *planIncoming->get(1).asList();
                mutexA->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "plan2")) {
			printf("correctly detected the plan2 \n");
            if (planIncoming->size() > 0){
                mutexB->wait();
                *planB = *planIncoming->get(1).asList();
                mutexB->post();
            }                   
        } 

		else if(!strcmp(name.c_str(), "weight")) {
			printf("correctly detected the weights \n");
            if (planIncoming->size() > 0){
                weightMutex->wait();
                *weight = *planIncoming->get(1).asList();
                weightMutex->post();
            }                   
        }
               
        else 
            printf("Please provide a  plan with a name like   \"planA\"   \"planB\"   \"weight\" \n");
        }  
        
        
        
        
        
        
        
        if (hubIncoming!=NULL){           
        //printf(" hub bottle %s \n",hubIncoming->toString().c_str());               
        string name;
        name = hubIncoming->get(0).asString();      
        if(!strcmp(name.c_str(), "hub0")) {
			printf("correctly detected the hub0 \n");
            if (hubIncoming->size() > 0){
                hmutex[0]->wait();
                *hbot [0] = *hubIncoming->get(1).asList();
                hmutex[0]->post();
            }               
        }
                
        else if(!strcmp(name.c_str(), "hub1")) {
			printf("correctly detected the hub1 \n");
            if (hubIncoming->size() > 0){
                hmutex[1]->wait();
                *hbot [1] = *hubIncoming->get(1).asList();
                hmutex[1]->post();
            }                    
        }
        
        else if(!strcmp(name.c_str(), "hub2")) {
			printf("correctly detected the hub2 \n");
            if (hubIncoming->size() > 0){
                hmutex[2]->wait();
                *hbot [2] = *hubIncoming->get(1).asList();
                hmutex[2]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "hub3")) {
			printf("correctly detected the hub3 \n");
            if (hubIncoming->size() > 0){
                hmutex[3]->wait();
                *hbot [3] = *hubIncoming->get(1).asList();
                hmutex[3]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "hub4")) {
			printf("correctly detected the hub4 \n");
            if (hubIncoming->size() > 0){
                hmutex[4]->wait();
                *hbot [4] = *hubIncoming->get(1).asList();
                hmutex[4]->post();
            }                    
        }
        
       /* else if(!strcmp(name.c_str(), "hubTop")) {
			printf("correctly detected the hubTop \n");
            if (hubIncoming->size() > 0){
                mutexTop->wait();
                *hubTop = *hubIncoming->get(1).asList();
                mutexTop->post();
            }                    
        }*/
                
        else if(!strcmp(name.c_str(), "hubObject")) {
            printf("correctly detected the object hub from memory \n");    
            if (hubIncoming->size() > 0){
                mutexObject->wait();
                *hubObject = *hubIncoming->get(1).asList();
                //printf("hubObject %08x got secondary list %s \n",hubObject, hubObject->toString().c_str());
                mutexObject->post();
            }                    
        }
		else if(!strcmp(name.c_str(), "hubBody")) {
            printf("correctly detected the body hub from memory \n");    
            if (hubIncoming->size() > 0){
                mutexBody->wait();
                *hubBody = *hubIncoming->get(1).asList();
                //printf("hubBody %08x got secondary list %s \n",hubBody, hubBody->toString().c_str());
                mutexBody->post();
            }                    
        }          
                               
        else 
            printf("Please provide a hub with a name like \"hub0\"  \"hub1\"  \"hub2\"  \"hub3\"  \"hub4\"  \"hubTop\" \"hubBottom\" \n");
    }
        
	if (hubTypeIncoming!=NULL){
        //printf(" bottle %s \n",hubTypeIncoming->toString().c_str()); 
        string name;
        name = hubTypeIncoming->get(0).asString();
                   
       if(!strcmp(name.c_str(), "object")) {
		   printf("correctly detected the object hub from observer \n");
            if (hubTypeIncoming->size() > 0){
                mutexObjectObs->wait();
                *hubObjectObs = *hubTypeIncoming->get(1).asList();
                mutexObjectObs->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "body")) {
			printf("correctly detected the body hub from observer \n");
            if (hubTypeIncoming->size() > 0){
                mutexBodyObs->wait();
                *hubBodyObs = *hubTypeIncoming->get(1).asList();
                mutexBodyObs->post();
            }                   
        }
		else if(!strcmp(name.c_str(), "action")) {
			printf("correctly detected the action hub from observer \n");
            if (hubTypeIncoming->size() > 0){
                mutexActionObs->wait();
                *hubActionObs = *hubTypeIncoming->get(1).asList();
                mutexActionObs->post();
            }                   
        }

		else if(!strcmp(name.c_str(), "CWS")) {
			printf("correctly detected the CWS hub from observer \n");
            if (hubTypeIncoming->size() > 0){
                mutexCWS->wait();
                *hubCWS = *hubTypeIncoming->get(1).asList();
                mutexCWS->post();
            }                   
        }

		 else 
            printf("Please provide a Observer hub with a name like \"object\"  \"body\" \"action\"  \"CWS\"  \n");
    }
                        
        //printf("Finishing the rate thread run \n");          
}

void emPlotterRatethread::threadRelease() {


    inputPortcue.interrupt();
    inputPortcue.close();
    inputPortrem.interrupt();
    inputPortrem.close();
    inputPorthub.interrupt();
    inputPorthub.close();
    inputPortplan.interrupt();
    inputPortplan.close();
    inputPorthubType.interrupt();
    inputPorthubType.close();
}


