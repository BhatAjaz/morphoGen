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
 * @file neuroVisRatethread.cpp
 * @brief Implementation of the eventDriven thread (see neuroVisRatethread.h).
 */

#include <neuroVisRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;

#define THRATE 100 //ms

neuroVisRatethread::neuroVisRatethread():RateThread(THRATE) {
    robot = "icub";        
    visFlag = true;
}

neuroVisRatethread::neuroVisRatethread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
    visFlag = true;
}

neuroVisRatethread::~neuroVisRatethread() {
    // do nothing
}

bool neuroVisRatethread::threadInit() {

    
    if (!inputPort.open(getName("/data:i").c_str())) {
        cout << ": unable to open port to receive neuronal activation data\n"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!outputPort.open(getName("/image:o").c_str())) {
        cout << ": unable to open port to send output image\n"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    for (int i =0;i<12;i++)
        for(int j=0;j<12;j++)
            activations[i][j] = 0;

    return true;
}

void neuroVisRatethread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string neuroVisRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void neuroVisRatethread::setInputPortName(string InpPort) {
        
}


void neuroVisRatethread::run() {


    if ((outputPort.getOutputCount()) && (inputPort.getInputCount()) && (true)) {
    
        inputData  =   inputPort.read(false);            
        
        if (inputData!=NULL)   {

            for (int i =0;i<12;i++)
                for(int j=0;j<12;j++)
                    activations[i][j] = 0;

        
            int height = 12; 
            int width  = 12;                                   
            int scale  = 20;

        
                //fill the first layer
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < width; j++) {
                    int index = i * width + j;
                    double x  = inputData->get(index).asDouble();
                    if(x <= 1 && x >= 0) {
                        activations[i][j] = x;
                    }
                    else {
                        printf("bad input bottle\n");
                        return;
                        }                   
                }
     
            }
                //make a gap between two layers
            for (int j = 0; j < width; j++) {
                activations[4][j] = 0;
                activations[5][j] = 0;
                activations[6][j] = 0;
            }
                //fill the second layer
            for (int i = 0; i < (height-7); i++) {
                for (int j = 0; j < (width-1); j++) {
                    int index = width*4 + i * (width-1) + j;
                    double x  = inputData->get(index).asDouble();
                    if(x <= 1 && x >= 0) {
                        activations[i+7][j] = x;
                    }
                    else {
                        printf("bad input bottle\n");
                        return;
                        }                   
                }
     
            }
                //fill the last column of second layer with zeros
            for (int i = 0; i < height-7; i++) 
                activations[i+7][width-1] = 0;



                //for (int i = 0; i<height;i++)
                 //   for(int j=0;j<width;j++)
                 //       cout << activations[i][j];

           
            
           /* inputIplImage   =   *((IplImage*) inputImage->getIplImage());  
            temp = & inputIplImage;
            thumbnail = Mat(outputHeight,outputWidth,CV_8UC3, CV_RGB(0,0,0));
            cv::resize(temp, thumbnail,Size(),0.5,0.5,CV_INTER_LINEAR);
            outputIplImage  =   thumbnail;          
            outputImage.wrapIplImage(&outputIplImage);
            */
            
            
            
                    // processing of the outputImage
        ImageOf<PixelMono>& outputImage =  outputPort.prepare();    
        outputImage.resize(width*scale, height*scale);
        int padding = outputImage.getPadding();

        unsigned char* oproc = outputImage.getRawImage();
        unsigned char* temp = oproc;
    
        for (int r = 0; r < height; r++) {
            temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {  
                    *oproc++ = activations[r][c] * 255;
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
        outputPort.write();         
        }        
        
    }                
}

void neuroVisRatethread::threadRelease() {

   
        outputPort.interrupt();
        outputPort.close();
        inputPort.interrupt();
        inputPort.close();
    
    
}


