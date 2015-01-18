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
 * @file hubTypeThread.cpp
 * @brief Implementation of the eventDriven thread (see hubTypeThread.h).
 */

#include <hubTypeThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 33 //ms

hubTypeThread::hubTypeThread():RateThread(THRATE) {
    robot = "icub";
    switchVisFlag = false;
}

hubTypeThread::hubTypeThread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
    switchVisFlag = false;
}

hubTypeThread::~hubTypeThread() {
    // do nothing
}

bool hubTypeThread::threadInit() {

    idle = false;
    for (int i = 0; i < 6; i++)
		for (int j = 0; j < 7; j++){
		bodyHub[i][j]=0.0;
		objectHub[i][j]=0.0;
		}
            

            
    if (!outputPort[0].open(getName("/observerObject/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
    
    if (!outputPort[1].open(getName("/observerBody/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
 if (!outputPort[2].open(getName("/observerAction/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
	if (!outputPort[3].open(getName("/observerColorWordShape/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 

}

void hubTypeThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string hubTypeThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void hubTypeThread::setInputPortName(string InpPort) {
    
}

void hubTypeThread::setSharingBottle(Bottle *a, Bottle *b,Bottle *c, Bottle *d ) {

    object		= a;
    body	    = b;
	action		= c;
	colorWordShape = d;
}


void hubTypeThread::setSemaphore(Semaphore *a, Semaphore *b,Semaphore *c, Semaphore *d ) {

    mutexObject     =   a;
    mutexBody		=   b; 
	mutexAction		=	c;
	mutexcolorWordShape = d;
}

void hubTypeThread::updateHub(Bottle* hubBottle, int p) {

     printf("%s \n", hubBottle->toString().c_str());
    
    
	 if(p==0) {
					

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 7; j++) {
				int index = i * 7 + j;
				double x  = hubBottle->get(index).asDouble();
				if(x <= 1 && x >= 0) {
					objectHub[i][j] = x;
				}
				else {
					printf("bad input bottle\n");
					return;
					}
				//printf("%f is the value of plan \n",*plan[i][j]);
			}
     
		}

	 }

	 else if(p==1) {
					

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 7; j++) {
				int index = i * 7 + j;
				double x  = hubBottle->get(index).asDouble();
				if(x <= 1 && x >= 0) {
					bodyHub[i][j] = x;
				}
				else {
					printf("bad input bottle\n");
					return;
					}
				//printf("%f is the value of plan \n",*plan[i][j]);
			}
     
		}

	 }

	 else if(p==2) {
					

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				int index = i * 4 + j;
				double x  = hubBottle->get(index).asDouble();
				if(x <= 1 && x >= 0) {
					actionHub[i][j] = x;
				}
				else {
					printf("bad input bottle\n");
					return;
					}
				//printf("%f is the value of plan \n",*plan[i][j]);
			}
     
		}

	 }

	 else if(p==3) {
					

		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 18; j++) {
				int index = i * 18 + j;
				double x  = hubBottle->get(index).asDouble();
				if(x <= 1 && x >= 0) {
					colorWordShapeHub[i][j] = x;
				}
				else {
					printf("bad input bottle\n");
					return;
					}
				//printf("%f is the value of plan \n",*plan[i][j]);
			}
     
		}

	 }

	 else 
		 printf("Wrong input port number");

        
}





void hubTypeThread::hubPlotting(int hb) {



/*
    if (outputPort[hb].getOutputCount()) {
        yarp::sig::ImageOf<yarp::sig::PixelMono> &outputImage = outputPort[hb].prepare();
        int height,width,scale;
        // processing of the outputImage
		if (hb == 0 || hb== 1) {
         height = 6;
         width  = 7;
		 scale  = 20;
		}

		if (hb == 2) {
         height = 3;
         width  = 4;
		 scale  = 40;
		}

		if (hb == 3) {
         height = 5;
         width  = 18;
		 scale  = 10;
		}
        
            
        outputImage.resize(width*scale, height*scale);
        int padding = outputImage.getPadding();

        unsigned char* oproc = outputImage.getRawImage();
        unsigned char* temp = oproc;
    
        for (int r = 0; r < height; r++) {
            temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {
				   if (hb==0)
						*oproc++ = objectHub[r][c] * 255;
				   else if (hb==1)
					   *oproc++ = bodyHub[r][c] * 255;
				   else if (hb==2)
					   *oproc++ = actionHub[r][c] * 255;
				   else
					   *oproc++ = colorWordShapeHub[r][c] * 255;
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
   //outputPort[hb].prepare() = *outputImage;
            
                
        outputPort[hb].write();  
    } 
*/ 
	
	if (outputPort[hb].getOutputCount()) {
		yarp::sig::ImageOf<yarp::sig::PixelMono> &outputImage = outputPort[hb].prepare();
        int height,width,scale;
        // processing of the outputImage
		if (hb == 0 ){
		height = 6;
		width  = 7;
		scale  = 20;
		outputImage.resize(width*scale, height*scale);
		int padding = outputImage.getPadding();

		unsigned char* oproc = outputImage.getRawImage();
		unsigned char* temp = oproc;
		for (int r = 0; r < height; r++) {
        temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {
						*oproc++ = objectHub[r][c] * 255;   
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
   //outputPort[hb].prepare() = *outputImage;
		}

		else if (hb == 1 ){
		height = 6;
		width  = 7;
		scale  = 20;
		outputImage.resize(width*scale, height*scale);
		int padding = outputImage.getPadding();

		unsigned char* oproc = outputImage.getRawImage();
		unsigned char* temp = oproc;
		for (int r = 0; r < height; r++) {
        temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {
						*oproc++ = bodyHub[r][c] * 255;   
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
   //outputPort[hb].prepare() = *outputImage;
		}


		else if (hb == 2 ){
		height = 3;
		width  = 4;
		scale  = 40;
		outputImage.resize(width*scale, height*scale);
		int padding = outputImage.getPadding();

		unsigned char* oproc = outputImage.getRawImage();
		unsigned char* temp = oproc;
		for (int r = 0; r < height; r++) {
        temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {
						*oproc++ = actionHub[r][c] * 255;   
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
   //outputPort[hb].prepare() = *outputImage;
		}

		else if (hb == 3 ){
		height = 5;
		width  = 18;
		scale  = 10;
		outputImage.resize(width*scale, height*scale);
		int padding = outputImage.getPadding();

		unsigned char* oproc = outputImage.getRawImage();
		unsigned char* temp = oproc;
		for (int r = 0; r < height; r++) {
        temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {
						*oproc++ = colorWordShapeHub[r][c] * 255;   
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
   //outputPort[hb].prepare() = *outputImage;
		}

		outputPort[hb].write(); 
	
	}




}


void hubTypeThread::run() {    
       
 /*       if (!idle) {
             printf(" planA pointer is %08x \n", planA1);
             idle = true;
        }
       
 */           
             mutexObject->wait();          
            if(object->size() > 0){
                printf("received  valid data as object hub \n");  
                this->updateHub(object,0);
            }
            object->clear();
            if(object->size() != 0){
                printf("Error\n");
            }
            mutexObject->post();
            this->hubPlotting(0);
        
            
            mutexBody->wait();          
            if(body->size() > 0){
                printf("received  valid data as body hub \n");  
                this->updateHub(body,1);
            }
            body->clear();
            if(body->size() != 0){
                printf("Error\n");
            }
            mutexBody->post();
            this->hubPlotting(1); 




			mutexAction->wait();          
            if(action->size() > 0){
                printf("received  valid data as action hub \n");  
                this->updateHub(action,2);
            }
            action->clear();
            if(action->size() != 0){
                printf("Error\n");
            }
            mutexAction->post();
            this->hubPlotting(2);
        
            
            mutexcolorWordShape->wait();          
            if(colorWordShape->size() > 0){
                printf("received  valid data as colorwordshape hub \n");  
                this->updateHub(colorWordShape,3);
            }
            colorWordShape->clear();
            if(colorWordShape->size() != 0){
                printf("Error\n");
            }
            mutexcolorWordShape->post();
            this->hubPlotting(3); 
//////////////////////////////////////////////////////////////////////

}

void hubTypeThread::threadRelease() {
    for (int i = 0; i < 4; i++) {
        outputPort[i].interrupt();
        outputPort[i].close();
    }
}

