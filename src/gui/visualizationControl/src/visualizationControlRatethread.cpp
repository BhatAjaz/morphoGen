// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2014  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco.Rea
  * email: francesco.rea@iit.it
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
 * @file visualizationControlRatethread.cpp
 * @brief Implementation of the eventDriven thread (see visualizationControlRatethread.h).
 */

#include <visualizationControlRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100 //ms

visualizationControlRatethread::visualizationControlRatethread():RateThread(THRATE) {
    robot = "icub";        
}

visualizationControlRatethread::visualizationControlRatethread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

visualizationControlRatethread::~visualizationControlRatethread() {
    // do nothing
}

bool visualizationControlRatethread::threadInit() {

    //printf("visualizationControlRatethread::threadInit \n");
    idle = false;    

    if (!inputPort.open(getName("/image:i").c_str())) {
        cout << ": unable to open port to receive partial cue bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!outputPort.open(getName("/image:o").c_str())) {
        cout << ": unable to open port to receive remembered bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
              
           
    return true;
    

}

void visualizationControlRatethread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string visualizationControlRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void visualizationControlRatethread::setInputPortName(string InpPort) {
        
}

void visualizationControlRatethread::setSharingBottle(Bottle *partialBot[], Bottle *remBot[], Bottle *hubBot[], Bottle* object, Bottle *body, Bottle *a, Bottle *b,Bottle *w, Bottle* x, Bottle *y, Bottle *z, Bottle *cws ) {

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

void visualizationControlRatethread::setSemaphore(Semaphore *pmu[], Semaphore *rmu[], Semaphore *hmu[], Semaphore *c, Semaphore *d, Semaphore *a, Semaphore *b, Semaphore *w, Semaphore *e, Semaphore *f, Semaphore *g, Semaphore *h) {
    
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

void visualizationControlRatethread::run() {    
    if((inputPort.getInputCount() > 0) && (outputPort.getOutputCount() > 0)) {
        
        ImageOf<PixelRgb>* readImage = inputPort.read(true);
        int width  = readImage->width();
        int height = readImage->height();
        int padding = readImage->getPadding();
        int rowSize = readImage->getRowSize();
        ImageOf<PixelRgb>& sentImage = outputPort.prepare();
        sentImage.resize(width, height);
        //memcpy (&sentImage, readImage, rowSize * height * sizeof(unsigned char));
        unsigned char* psource = readImage->getRawImage();
        unsigned char* pdest   = sentImage.getRawImage();
        for(int r = 0; r < height; r++) {
            for (int c = 0; c < width; c++) {
                *pdest++ = *psource++;
                *pdest++ = *psource++;
                *pdest++ = *psource++;
            }
            pdest   += padding;
            psource += padding;
        }
        
        outputPort.write();
    }
         
}

void visualizationControlRatethread::threadRelease() {

    inputPort.interrupt();
    inputPort.close();
    outputPort.interrupt();
    outputPort.close();
    
}


