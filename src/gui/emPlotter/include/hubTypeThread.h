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
 * @file hubTypeThread.h
 * @brief Definition of a thread that receives a bottle from input port and sends the corresponding plans and hubs to the output port.
 */


#ifndef _HUB_TYPE_THREAD_H_
#define _HUB_TYPE_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <sstream> 
#include <time.h>


class hubTypeThread : public yarp::os::RateThread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string name;
    
    
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outputPort[4];     // output port to plot event
  
    yarp::os::Bottle  *body, *object, *action,*colorWordShape;
    yarp::os::Semaphore *mutexBody, *mutexObject, *mutexAction, *mutexcolorWordShape;
    double bodyHub[6][7],objectHub[6][7],actionHub[3][4],colorWordShapeHub[5][18]; // matrix to represent the hubs 
	
    bool switchVisFlag;
    bool idle;                      // flag that indicates whether the thread is active

public:
    /**
    * constructor default
    */
    hubTypeThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    hubTypeThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~hubTypeThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run(); 

    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /* 
    * function that receives the input bottles and assigns them to the corresponding thread
    */
    void setSharingBottle(yarp::os::Bottle *c, yarp::os::Bottle *d, yarp::os::Bottle *e,yarp::os::Bottle *f);
    
    
    /*
    * function that sets the semaphores to the corresponding thread
    */
    void setSemaphore(yarp::os::Semaphore *a, yarp::os::Semaphore *b,yarp::os::Semaphore *c, yarp::os::Semaphore *d);

    /*
    * function that updates when every new cue comes in
    */
    void updateHub(yarp::os::Bottle *queueBottle, int i);
    

	/*
    * function that switch plots activation
    */
    void switchVis(bool value){switchVisFlag = value;};
    

    /*
    * function that plots the contents of the cue
    */
    void hubPlotting(int i);



	
};

#endif  //_HUB_TYPE_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

