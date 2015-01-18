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
 * @file planThread.cpp
 * @brief Implementation of the eventDriven thread (see planThread.h).
 */

#include <planThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 33 //ms

planThread::planThread():RateThread(THRATE) {
    robot = "icub";      
    switchVisFlag = true;
}

planThread::planThread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
    switchVisFlag = true;
}

planThread::~planThread() {
    // do nothing
}

bool planThread::threadInit() {

    idle = false;
    for (int i = 0; i < 20; i++)
        for (int j = 0; j < 50; j++)
            plan[i][j]=0.0;

	for (int i = 0; i < 1000; i++)
        for (int j = 0; j < 1000; j++)
            weight[i][j]=0.0;
            
    if (!outputPort[0].open(getName("/plan1/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
    
    if (!outputPort[1].open(getName("/plan2/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
 
	if (!weightPort.open(getName("/weight/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  

    return true;
    

}

void planThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string planThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void planThread::setInputPortName(string InpPort) {
    
}

void planThread::setSharingBottle(Bottle *a, Bottle *b,Bottle *c) {

    planA1       = a;
    planB1       = b;
	weightBottle = c;
}


void planThread::setSemaphore(Semaphore *a, Semaphore *b, Semaphore *c) {

    mutexA1      =   a;
    mutexB1      =   b; 
	weightMutex	 =	 c;
}

void planThread::updatePlan(Bottle* hubBottle) {

     printf("%s \n", hubBottle->toString().c_str());
    
    
     for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 50; j++) {
            int index = i * 50 + j;
            double x  = hubBottle->get(index).asDouble();
            if(x <= 1 && x >= 0) {
                plan[i][j] = x;
            }
            else {
                printf("bad input bottle\n");
                return;
                }
              //printf("%f is the value of plan \n",*plan[i][j]);
        }
     
     }
        
}


void planThread::updateWeight(Bottle* weightBottle) {

     printf("%s \n", weightBottle->toString().c_str());
    
    
     for (int i = 0; i < 1000; i++) {
        for (int j = 0; j < 1000; j++) {
            int index = i * 1000 + j;
            int x  = weightBottle->get(index).asInt();
            //if(x == 1 || x == 0) {
                weight[i][j] = x;
           // }
           // else {
             //   printf("bad input bottle\n");
               // return;
                //}
              //printf("%f is the value of plan \n",*plan[i][j]);
        }
     
     }
        
}



void planThread::planPlotting(int i) {

    if ((outputPort[i].getOutputCount()) && (switchVisFlag)) {
        yarp::sig::ImageOf<yarp::sig::PixelMono> &outputImage = outputPort[i].prepare();
        
        // processing of the outputImage
        int height = 20;
        int width  = 50;
        int scale  = 10;
            
        outputImage.resize(width*scale, height*scale);
        int padding = outputImage.getPadding();

        unsigned char* oproc = outputImage.getRawImage();
        unsigned char* temp = oproc;
    
        for (int r = 0; r < height; r++) {
            temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {  
                    *oproc++ = plan[r][c] * 255;
               }               
            }
            oproc+=padding;
            for (int l = 0; l < scale-1; l++){
                for (int y = 0; y < outputImage.width(); y++){
                    *oproc++ = *(temp+y);
                }
                oproc+=padding;
            }                                   
        }
   //outputPort[i].prepare() = *outputImage;
            
                
        outputPort[i].write();  
    } 
 
}






void planThread::weightPlotting() {

    if ((weightPort.getOutputCount()) && (switchVisFlag)) {
        yarp::sig::ImageOf<yarp::sig::PixelMono> &outputImage = weightPort.prepare();
        
        // processing of the outputImage
        int height = 1000;
        int width  = 1000;
        //int scale  = 10;
            
        outputImage.resize(width, height);
        int padding = outputImage.getPadding();

        unsigned char* oproc = outputImage.getRawImage();
        unsigned char* temp = oproc;
    
        for (int r = 0; r < 1000; r++) {
            for (int c = 0; c < 1000; c++) {
               
                    *oproc++ = weight[c][r] * 255;

				//if(r%2 == 0 || c%2 == 0)
				//{
				//	 *oproc++ = 255;
				//}

				//else
				//{
				//	 *oproc++ = 0;
				//}

               }
			oproc+=padding;
            }
                
        weightPort.write();  
    } 
 
}















void planThread::run() {    
       
 /*       if (!idle) {
             printf(" planA pointer is %08x \n", planA1);
             idle = true;
        }
       
 */           
             mutexA1->wait();          
            if(planA1->size() > 0){
                printf("received  valid data as plan \n");  
                this->updatePlan(planA1);
            }
            planA1->clear();
            if(planA1->size() != 0){
                printf("Error\n");
            }
            mutexA1->post();
            this->planPlotting(0);
        
            
            mutexB1->wait();          
            if(planB1->size() > 0){
                printf("received  valid data as plan \n");  
                this->updatePlan(planB1);
            }
            planB1->clear();
            if(planB1->size() != 0){
                printf("Error\n");
            }
            mutexB1->post();
            this->planPlotting(1); 
//////////////////////////////////////////////////////////////////////

			weightMutex->wait();          
            if(weightBottle->size() > 0){
                printf("received  valid data as plan \n");  
                this->updateWeight(weightBottle);
            }
            weightBottle->clear();
            if(weightBottle->size() != 0){
                printf("Error\n");
            }
            weightMutex->post();
            this->weightPlotting();   


}

void planThread::threadRelease() {
    for (int i = 0; i < 2; i++) {
        outputPort[i].interrupt();
        outputPort[i].close();
    }
}

