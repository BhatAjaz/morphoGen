// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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
 * @file plotterThread.cpp
 * @brief Implementation of the thread that represent image (see header plotterThread.h)
 */

#include <iCub/plotterThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 5

plotterThread::plotterThread() : RateThread(THRATE) {
    synchronised = false;
    count  = 0;
    width  = 320;   //default dimension width 
    height = 240;   //default dimension height
}

plotterThread::~plotterThread() {

}

bool plotterThread::threadInit() {
    printf("\n starting the thread.... \n");
    // opening ports 
    leftPort.open      (getName("/left:o").c_str());
    rightPort.open     (getName("/right:o").c_str());
    leftIntPort.open   (getName("/leftInt:o").c_str());
    rightIntPort.open  (getName("/rightInt:o").c_str());
    leftGrayPort.open  (getName("/leftGrey:o").c_str());
    rightGrayPort.open (getName("/rightGrey:o").c_str());
    eventPort.open     (getName("/event:o").c_str());

    // initialising images
    imageLeft      = new ImageOf<PixelRgb>;
    imageLeft->resize(width,height);
    /*
    imageRight     = new ImageOf<PixelRgb>;
    imageRight->resize(retinalSize,retinalSize);
    imageLeftInt   = new ImageOf<PixelMono>;
    imageLeftInt->resize(retinalSize,retinalSize);
    imageLeftInt->zero();
    imageRightInt  = new ImageOf<PixelMono>;
    imageRightInt->resize(retinalSize,retinalSize);
    imageRightInt->zero();
    imageLeftBW    = new ImageOf<PixelMono>;
    imageLeftBW->resize(retinalSize,retinalSize);
    imageLeftBW->zero();
    imageRightBW   = new ImageOf<PixelMono>;
    imageRightBW->resize(retinalSize,retinalSize);
    imageRightBW->zero();
    imageLeftGrey  = new ImageOf<PixelMono>;
    imageLeftGrey->resize(retinalSize,retinalSize);
    imageLeftGrey->zero();
    imageRightGrey = new ImageOf<PixelMono>;
    imageRightGrey->resize(retinalSize,retinalSize);
    imageRightGrey->zero();
    imageLeftThreshold  = new ImageOf<PixelMono>;
    imageLeftThreshold->resize(retinalSize,retinalSize);
    imageLeftThreshold->zero();
    imageRightThreshold = new ImageOf<PixelMono>;
    imageRightThreshold->resize(retinalSize,retinalSize);
    imageRightThreshold->zero();
    */
    
    printf("initialization in plotter thread correctly ended \n");
    return true;
}

void plotterThread::interrupt() {
    leftPort.interrupt();
    leftIntPort.interrupt();
    rightPort.interrupt();
    rightIntPort.interrupt();
    leftGrayPort.interrupt();
    rightGrayPort.interrupt();
    eventPort.interrupt();

}

void plotterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string plotterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void plotterThread::resize(int widthp, int heightp) {
}

void plotterThread::copyLeft(ImageOf<PixelMono>* image) {
    //printf("retinalSize in plotterThread %d \n",retinalSize);
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft  = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < height; r++) {
            for(int c = 0; c < width; c++) {                
                if(r%2 == 0){
                    *pleft++ = 0;
                }
                else{
                    *pleft++ = 255;
                }
            }
            pleft  += padding;
            pimage += padding;
        }
    }
}

void plotterThread::copyLeft(ImageOf<PixelRgb>* image) {
    int padding= imageLeft->getPadding();
    printf("retinalSize in plotterThread %d %d %d \n",padding, width, height);
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft  = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < height; r++) {
            for(int c = 0; c < width; c++) {                
                if(r%2 == 0) {
                    *pleft++ = 0;
                    *pleft++ = 0;
                    *pleft++ = 0;
                }
                else{
                    *pleft++ = 255;
                    *pleft++ = 255;
                    *pleft++ = 255;
                }
            }
            pleft  += padding;
            pimage += padding;
        }
    }
}


void plotterThread::run() {
    count++;
    imageLeft  = &leftPort.prepare();
    imageLeft->resize(width, height);
    imageRight = &rightPort.prepare();
    imageRight->resize(width, height);
    synchronised = true;

    
    if(leftPort.getOutputCount()) {
        leftPort.write();
    }
    if(rightPort.getOutputCount()) {
        rightPort.write();
    }

    /*
    // obtaining the left and right integrated images
    int positionLeft = 0, positionRight = 0;
    int ul = 0, vl = 0, ur = 0, vr = 0;
    if ((leftIntPort.getOutputCount())||(leftGrayPort.getOutputCount())) {
        ImageOf<PixelMono>& leftInt = leftIntPort.prepare();
        ImageOf<PixelMono>& leftGrey = leftGrayPort.prepare();
        leftInt.resize(imageLeftInt->width(), imageLeftInt->height());
        if(count % 500 == 0) {
            imageLeftInt->copy(*imageLeft);
            imageLeftGrey->zero();
        }
        else {
            positionLeft = integrateImage(imageLeft, imageLeftInt,imageLeftBW, imageLeftGrey, imageLeftThreshold);
        }
        leftInt.copy(*imageLeftBW);
        leftIntPort.write();
        leftGrey.copy(*imageLeftThreshold);
        leftGrayPort.write(); 
        vl = floor(positionLeft/retinalSize);
        ul = positionLeft%retinalSize;       
    }
    
    if ((rightIntPort.getOutputCount())||(rightGrayPort.getOutputCount())) {
        ImageOf<PixelMono>& rightInt = rightIntPort.prepare();
        ImageOf<PixelMono>& rightGrey = rightGrayPort.prepare();
        rightInt.resize(imageRightInt->width(), imageRightInt->height());
        if(count % 500 == 0) {
            imageRightInt->copy(*imageRight);
            imageRightGrey->zero();
        }
        else {
            positionRight = integrateImage(imageRight, imageRightInt,imageRightBW, imageRightGrey, imageRightThreshold);
        }
        rightInt.copy(*imageRightBW);
        rightIntPort.write();
        rightGrey.copy(*imageRightThreshold);
        rightGrayPort.write();
        vr = floor(positionRight/retinalSize);
        ur = positionRight%retinalSize;
    }
    */
    

    /*
    // extracting the centroid of the integrated image
    //Bottle& eventBottle = eventPort.prepare();
    Vector& centroidStereo = eventPort.prepare();
    if (( ul!= 0) && ( vl!= 0) && ( ur!= 0) && ( vr!= 0)) {
        //eventBottle.addInt(ul);
        //eventBottle.addInt(vl);
        //eventBottle.addInt(ur);
        //eventBottle.addInt(vr);
        //eventPort.write();
        ul = (((((ul - 64)/ 128.0)/ 7.4) * 4) * 320) + 160;
        vl = (((((vl - 64)/ 128.0)/ 7.4) * 4) * 240) + 120;
        ur = (((((ur - 64)/ 128.0)/ 7.4) * 4) * 320) + 160;
        vr = (((((vr - 64)/ 128.0)/ 7.4) * 4) * 240) + 120;
        centroidStereo.clear();
        
        centroidStereo.push_back(ul);
        centroidStereo.push_back(vl);
        centroidStereo.push_back(ur);
        centroidStereo.push_back(vr);
        eventPort.write();
        
        //printf("positionLeft %d-%d, positionRight %d \n", (int) floor(positionLeft/128.0),positionLeft%128, positionRight);
    }
    */
}




void plotterThread::threadRelease() {
    printf("plotterThread: portClosing \n");  
    leftPort.close();
    leftIntPort.close();
    rightPort.close();
    rightIntPort.close();
    leftGrayPort.close();
    rightGrayPort.close();
    eventPort.close();

    printf("freeing memory \n");
    //if(imageLeft!=0)
    //    delete imageLeft;
    //if(imageRight!=0)
    //    delete imageRight;
    printf("freed images \n");
    delete imageLeftInt;
    delete imageRightInt;
    printf("freed integrated images \n");
    delete imageLeftBW;
    delete imageRightBW;
    printf("freed bw images \n");
    delete imageLeftGrey;
    delete imageRightGrey;
    printf("freed grey images \n");
    delete imageLeftThreshold;
    delete imageRightThreshold;

    printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
