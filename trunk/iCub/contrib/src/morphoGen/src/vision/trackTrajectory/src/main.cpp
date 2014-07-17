// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2014  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Ajaz Bhat
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
 * @file main.cpp
 * @brief main code for the trackTrajectory module. Uses the output from the colorVision modules
		 to file down the trajectory coordinates of a moving color object/blob.
 */


#include <iostream>
#include <fstream> 
#include <cstdlib>
#include <yarp/os/all.h>

using namespace yarp::os;
using namespace std;

int main(int argc, char * argv[])
{
    
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setDefaultConfigFile("trajectVisualize.ini");    //overridden by --from parameter
    rf.setDefaultContext("morphoGenApp/conf");    //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);  
    //yarp::os::ConstString path = rf.findPath("trajPoints.txt");
    //printf("File found! path %s \n", path.c_str());

    
    yarp::os::BufferedPort<yarp::os::Bottle> input;
	yarp::os::Bottle *incomingBottle;
    input.open("/trackTrajectory/data:i");
    Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/data:i");
    int x1,y1,x2,y2,x,y,count,ID;
    // Bottle handling
	FILE * trajFile;
	trajFile = fopen ("trajectory.txt","w+");
	while (true){
        if( input.getInputCount()){
			//fprintf (trajFile,"running the module%d %d %d",i,j);
            incomingBottle    =   input.read(false);
            if (incomingBottle  !=  NULL)   {           
                count = incomingBottle->get(0).asInt();
                if (incomingBottle->size() > 0) {                 
                        Bottle * tempBottle =   incomingBottle->get(1).asList();                     
                        if (tempBottle != NULL){
                            x1  =   abs(tempBottle->get(0).asInt()); 
                            y1  =   abs(tempBottle->get(1).asInt()); 
                            x2  =   abs(tempBottle->get(2).asInt()); 
                            y2  =   abs(tempBottle->get(3).asInt());
                            ID  =   tempBottle->get(4).asInt();  
							x	=	(x1+x2)/2;
							y	=	(y1+y2)/2;
							printf("handling object with ID %d: and centroid at %d %d  \n", ID, x,y);
							fprintf (trajFile, "%d %d\n",x,y);
                        }						
                }
                incomingBottle->clear();
                if(incomingBottle->size() != 0) {
                    printf("Error\n");   
                }
            }  
        }
	}
		fclose(trajFile);

    return 0;
}


