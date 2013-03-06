// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Ajaz Ahamd Bhat
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
 * @file colorQuantizatorThread.cpp
 * @brief Implementation of the eventDriven thread (see colorQuantizatorThread.h).
 */

#include <iCub/colorQuantizatorThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

colorQuantizatorThread::colorQuantizatorThread() {
    robot = "icub";        
}

colorQuantizatorThread::colorQuantizatorThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

colorQuantizatorThread::~colorQuantizatorThread() {
    // do nothing
}

bool colorQuantizatorThread::threadInit() {

    if (!inputPort.open(getName("/img:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
   
    if (!outputPort.open(getName("/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
    

}

void colorQuantizatorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string colorQuantizatorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void colorQuantizatorThread::setInputPortName(string InpPort) {
    
}

void colorQuantizatorThread::run() {    
    while (isStopping() != true) {

		if(inputPort.getInputCount()){
			ImageOf<PixelRgb>* inputImage = inputPort.read(false);
		    	   
            if (outputPort.getOutputCount()) {

                ImageOf<PixelMono>& processedImage = outputPort.prepare();

                if(inputImage!=NULL){

                    //processing
                    int width   = inputImage->width();
                    int height  = inputImage->height();

                    processedImage.resize(width, height);

                    int paddingInput = inputImage->getPadding();
                    int paddingProc  = processedImage.getPadding();

                    unsigned char* pinput = inputImage->getRawImage();
                    unsigned char* pproc  = processedImage.getRawImage();

                    //printf("input %d processed %d \n", paddingInput, paddingProc);
                    //printf("width %d height %d \n", width, height);
 
                    
                    //processedImage.zero();

                    
                    for (int r = 0; r < height; r++) {
                        for (int c = 0; c < width; c++) {
                            
                            *pproc = *pinput;
                            //*pproc = (unsigned char) 0;
                            //pproc[c] = 0;
                            pinput+=3; //pinput++; pinput++; pinput++;
                            
                            pproc++;
                            
                        }
                        pinput += paddingInput;
                        pproc  += paddingProc;
                    }
                    
                                        
                    
                    //outputPort.prepare() = *processedImage;
                    outputPort.write();  
                }
            }
        }
    }               
}

void colorQuantizatorThread::threadRelease() {
    // nothing
     
}

void colorQuantizatorThread::onStop() {
    inputPort.interrupt();
    outputPort.interrupt();
	inputPort.close();
    outputPort.close();
}

