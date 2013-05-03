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
 * @file shapeSelectorRatethread.cpp
 * @brief Implementation of the eventDriven thread (see shapeSelectorRatethread.h).
 */

#include <shapeSelectorRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;

#define THRATE 100 //ms

shapeSelectorRatethread::shapeSelectorRatethread():RateThread(THRATE) {
    robot = "icub";        
}

shapeSelectorRatethread::shapeSelectorRatethread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

shapeSelectorRatethread::~shapeSelectorRatethread() {
    // do nothing
}

bool shapeSelectorRatethread::threadInit() {

    
        

    if (!inputBottlePort[0].open(getName("/leftBottle:i").c_str())) {
        cout << ": unable to open port to receive left input "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!inputBottlePort[1].open(getName("/rightBottle:i").c_str())) {
        cout << ": unable to open port to receive right input image"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
    
    if (!inputImagePort[0].open(getName("/leftImage:i").c_str())) {
        cout << ": unable to open port to receive left input image"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!inputImagePort[1].open(getName("/rightImage:i").c_str())) {
        cout << ": unable to open port to receive right input "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
     

    if (!outputImagePort[0].open(getName("/leftImage:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
    
    if (!outputImagePort[1].open(getName("/rightImage:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
        
    
    
    
    /*if (!Network::connect("/MyRemembered:o","/shapeSelector/rememberedX:i"))
        return false;    
   //printf("Connection to rememberedX");
    if (!Network::connect("/Useful/PastXperiences:o","/shapeSelector/cue:i"))
        return false;
    //if (!Network::connect("/Hub:o","/shapeSelector/hub:i"))
    //   return false;
    */
    //Network::connect("/PlanXplore:o","/shapeSelector/plan:i");
        //printf("connection NOT successful\n");
       

    
    
    height = 240; //default value;
    width  = 320; //default value;
    for (int j = 0; j < 2; j++)
        for(int i = 0; i < 3; i++)  {        // initialize the coordinates
            xLeft[j][i]     =   -1;
            yTop[j][i]      =   -1;
            xWidth[j][i]    =   0;
            yHeight[j][i]   =   0;
            objectID[j][i]  =   -1;
        }
         
    return true;
    

}

void shapeSelectorRatethread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string shapeSelectorRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void shapeSelectorRatethread::setInputPortName(string InpPort) {
        
}

void shapeSelectorRatethread::setSharingBottle(Bottle *lBot, Bottle *rBot) {

   
       leftBot  =   lBot;
       rightBot =   rBot;
}



void shapeSelectorRatethread::updateObjects(ImageOf<PixelMono>* inputImage, ImageOf<PixelMono>* outputImage, int k, int x) {
    
     
    for (int i = 0; i < 3; i++) {
        oproc       =   outputImage->getRawImage();   
        if (x) {
            inproc      =   inputImage->getRawImage();
            for (int r = 0; r < height; r++) {
                    for (int c = 0; c < width; c++) {
                        if ( ( r >= yTop[k][i]) && (r <= yHeight[k][i]) && (c >= xLeft[k][i]) && (c <= xWidth[k][i] ) ) {
                            *oproc = *inproc;
                            
                        }    
                        oproc++;
                        inproc++;
                             
                    }
                    oproc+=padding;
                    inproc+=padding;                                  
            }
        }
        else {
            for (int r = 0; r < height; r++) {
                    for (int c = 0; c < width; c++) {
                        if ( ( r >= yTop[k][i]) && (r <= yHeight[k][i]) && (c >= xLeft[k][i]) && (c <= xWidth[k][i] ) ) {
                            *oproc = 255;
                        }    
                        oproc++;                 
                    }
                    oproc+=padding;                                
            }
        }
    }
}

void shapeSelectorRatethread::run() {
    
    for (int j = 0 ; j < 2; j++)  {
    
        // Bottle handling
        if( inputBottlePort[j].getInputCount()){
            incomingBottle    =   inputBottlePort[j].read(false);
            if (incomingBottle  !=  NULL)   {
                for(int i = 0; i < 3; i++)  {        // initialize the coordinates for every new bottle
                    xLeft[j][i]     =   -1;
                    yTop[j][i]      =   -1;
                    xWidth[j][i]    =   0;
                    yHeight[j][i]   =   0;
                    objectID[j][i]  =   -1;
                    
                    
                }
                objectCount = incomingBottle->get(0).asInt();
                if (incomingBottle->size() > 0) {
                    
                    for (int i = 0; i < 3; i++) {
                        Bottle * tempBottle =   incomingBottle->get(i+1).asList();
                        if(tempBottle != NULL){
                            xLeft[j][i]         =   abs(tempBottle->get(0).asInt());
                            yTop[j][i]          =   abs(tempBottle->get(1).asInt());
                            xWidth[j][i]        =   abs(tempBottle->get(2).asInt());
                            yHeight[j][i]       =   abs(tempBottle->get(3).asInt());
                            objectID[j][i]      =   tempBottle->get(4).asInt();
                        }
                        printf("handling object %d: %d %d %d %d with ID %d \n", i, xLeft[j][i], yTop[j][i], xWidth[j][i], yHeight[j][i], objectID[j][i]);                   
                    }
                    incomingBottle->clear();
                    if(incomingBottle->size() != 0) {
                        printf("Error\n");   
                    }
                }  
            }
        }
        else {
            for(int i = 0; i < 3; i++)  {        // initialize in case of no input bottle ports
                xLeft[j][i]     =   -1;
                yTop[j][i]      =   -1;
                xWidth[j][i]    =   0;
                yHeight[j][i]   =   0;
                objectID[j][i]  =   -1;
                }
        } 
        
        
       
        // image handling
        if (outputImagePort[j].getOutputCount()) {
            //printf("%d\n",foreground.at<int>(0, 0));                           
            ImageOf<PixelMono>& outputImage =  outputImagePort[j].prepare();
            outputImage.resize(width, height);
            padding = outputImage.getPadding();
            int x   =   inputImagePort[j].getInputCount();
            if (x)  {
                tempImage  =   inputImagePort[j].read(false);
                if (tempImage  !=  NULL)   {
                    inputImage[j]   =   tempImage; 
                    width           =   inputImage[j]->width();
                    height          =   inputImage[j]->height();
                    outputImage.resize(width, height);
                    outputImage.zero();
                    foreground      =   Mat(height,width,CV_8UC1,cv::Scalar(0));
                    mask            =   Mat(height,width,CV_8UC1,cv::Scalar(0));
                    inputIplImage   =   *((IplImage*) inputImage[j]->getIplImage());   
                    temp            =   &inputIplImage;
                    cv::Mat in[]    =   {temp, temp, temp};
                    cv::merge(in, 3, img0);
                    int i = 0;
                    cv::Mat bgdModel, fgdModel;
                    for (int i = 0; i < 3; i++) {
                        rect = Rect (xLeft[j][i], yTop[j][i], xWidth[j][i], yHeight[j][i]);
                        //printf("rect values %d: %d %d %d %d \n", i, xLeft[j][i], yTop[j][i], xWidth[j][i], yHeight[j][i]);
                        if (rect.area() > 0)    {
                            grabCut( img0, mask, rect, bgdModel, fgdModel, 10 ,cv::GC_INIT_WITH_RECT);
                            // Get the pixels marked as likely foreground
                            //cv::compare(mask,cv::GC_PR_FGD,mask,cv::CMP_EQ);
                            // Generate output image                        
                            img0.copyTo(foreground,mask);
                            //printf("this is the first character %c",foreground.at<unsigned char>(0,0));
                            outputIplImage = foreground;
                            outputImage.wrapIplImage(&outputIplImage);
                            
                        }
                        
                        
                    }
                    outputImagePort[j].write();               
                }
                else {
                    outputImage.resize(width, height);
                    //padding = outputImage.getPadding();   
                    outputImage.zero();
                    /* if(!foreground.empty()) {
                        outputIplImage = foreground;
                        //IplImage *im_gray = cvCreateImage(cvGetSize(&outputIplImage),IPL_DEPTH_8U,1);
                        //cvCvtColor(&outputIplImage,im_gray,CV_RGB2GRAY);
                        outputImage.wrapIplImage(&outputIplImage);
                    
                    }*/
                    //this->updateObjects(inputImage[j], &outputImage, j, x); //update while keeping the previous
                    outputImagePort[j].write();
                }
            }
            else{   
                outputImage.zero();
                for (int i = 0; i < 3; i++) {
                    oproc       =   outputImage.getRawImage();
                    for (int r = 0; r < height; r++) {
                        for (int c = 0; c < width; c++) {
                            if ( ( r >= yTop[j][i]) && (r <= (yTop[j][i] + yHeight[j][i])) && (c >= xLeft[j][i]) && (c <= (xLeft[j][i] + xWidth[j][i] )) ) {
                                *oproc = 255;
                            }    
                            oproc++;                 
                        }
                        oproc+=padding;                                
                    }
                }
                outputImagePort[j].write();
                oproc = NULL;
            }
            
        }
        
        
        

         
    }// for loop ends here                   
    //printf("Finishing the rate thread run \n");
    //watershed();          
}

void shapeSelectorRatethread::threadRelease() {

    for (int i = 0; i < 2; i++) {
        outputImagePort[i].interrupt();
        outputImagePort[i].close();
        inputImagePort[i].interrupt();
        inputImagePort[i].close();
        inputBottlePort[i].interrupt();
        inputBottlePort[i].close();
    }
    
}


