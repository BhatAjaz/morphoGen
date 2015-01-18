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
		 to file down the trajectory coordinates of a moving color object/blob; smoothens out the
		 trajectory and finally finds the critical points on it.
 */


#include <iostream>
#include <fstream> 
#include <cstdlib>
#include <yarp/os/all.h>
#include <math.h>
#include "stdafx.h"
#include "interpolation.h"

using namespace yarp::os;
using namespace std;
using namespace alglib;
#define COMMAND_VOCAB_STRT   VOCAB4('S','T','R','T')
#define COMMAND_VOCAB_BUMP   VOCAB4('B','U','M','P')
#define COMMAND_VOCAB_CUSP   VOCAB4('C','U','S','P')
#define COMMAND_VOCAB_END   VOCAB3('E','N','D')



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


    
    yarp::os::BufferedPort<yarp::os::Bottle> input,imitPort,outputPort;
	yarp::os::Bottle *incomingBottle,*imitBottle;
	//outputPort.open("/trackTrajectory/data:o");
    input.open("/trackTrajectory/data:i");
	imitPort.open("/trackTrajectory/imitationCtrl:i");
	if (!Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/data:i"))
	{
		Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/data:i");
	}
	double xPoint[10000],yPoint[10000];
	float backward_slope=0, forward_slope=0, vecA[2],vecB[2],cosRatio=-1;
    int x1,y1,x2,y2,x,y,count,ID;
	int sameData=0;
	bool moving =false;
    // Bottle handling
	
	std::ofstream outFile,trajFile;
	trajFile.open("trajectory.txt");
	outFile.open("data1.txt");
	int i = 0;

	while( !imitPort.getInputCount()){
		imitBottle    =   imitPort.read(true);
		if (imitBottle  !=  NULL)   {           
			int ctrl = 1234;
			ctrl = imitBottle->get(0).asInt();	
			if (ctrl==1){
				moving=true;
			}
			imitBottle->clear();
		}
	}


	while (moving){
        if( input.getInputCount()){
			//fprintf (trajFile,"running the module%d %d %d",i,j);
            incomingBottle    =   input.read(false);
            if (incomingBottle  !=  NULL)   {             
                if (incomingBottle->size() > 0) {
					count = incomingBottle->get(0).asInt();
					Bottle * tempBottle =   incomingBottle->get(1).asList();                     
					if (tempBottle != NULL){
						x1  =   abs(tempBottle->get(0).asInt()); 
						y1  =   abs(tempBottle->get(1).asInt()); 
						x2  =   abs(tempBottle->get(2).asInt()); 
						y2  =   abs(tempBottle->get(3).asInt());
						ID  =   tempBottle->get(4).asInt();  
						x	=	(x1+x2)/2;
						y	=	(y1+y2)/2;
						printf("Found object with ID \t %d: and centroid at \t %d \t %d  \n", ID, x,y);
						if(ID==2) {
							if (i==0){
								xPoint[i]=x;
								yPoint[i]=y;
								printf("Saving object at new location with ID \t %d: and centroid at \t %d \t %d  \n", ID, x,y);
								//fprintf (trajFile, "%d %d\n", x,y);
								trajFile <<x << "\t"<<y << "\t"<<std::endl;
								i++;
							}
						
							if (i>0&& i<10000){
								if ( (fabs(x - xPoint[i-1]) >=1) && (fabs(y - yPoint[i-1]) >= 1) ) {
									xPoint[i]=x;
									yPoint[i]=y;
									printf("Saving object at new location with ID \t %d: and centroid at \t %d \t %d  \n", ID, x,y);
									//fprintf (trajFile, " %d %d\n", x,y);
									trajFile <<x << "\t"<<y << "\t"<<std::endl;
									i++;
									//sameData=0;
								}
							}
						}
					}						
                }
                incomingBottle->clear();
                if(incomingBottle->size() != 0) {
                    printf("Error\n");   
                }
            }  
        }
		if( imitPort.getInputCount()){
			imitBottle    =   imitPort.read(false);
			if (imitBottle  !=  NULL)   {           
				int ctrl = 1234;
				ctrl = imitBottle->get(0).asInt();	
				if (ctrl==0){
					moving=false;
				}
			}
		}
	}
	trajFile.close();
	int dataSize = i;
	float startPx=xPoint[0];
	float startPy=yPoint[0];
	float endPx=xPoint[dataSize-1];
	float endPy=yPoint[dataSize-1];
	//Trajectory saved
	//Smoothening the trajectory follows
	double timeVec[10000]={0};
	for (int i=0;i<dataSize;i++)
	{
		timeVec[i]=i+1;
	}
	 real_1d_array ttr,xtr,ytr;
    //real_1d_array xtr = "[117,113,108,102,94,91,85,81,80,78,77,76,76]";
	//real_1d_array ytr = "[111,110,109,107,103,101,97,92,91,87,84,81,80]";
	 ttr.setcontent(dataSize,timeVec);
	 xtr.setcontent(dataSize,xPoint);
	 ytr.setcontent(dataSize,yPoint);
    ae_int_t info;
    spline1dinterpolant s1,s2;
    spline1dfitreport rep1,rep2;
    double rho= 0;//2;///between -5 and 10

    spline1dfitpenalized(ttr, xtr,dataSize, 5, rho, info, s1, rep1);
    //printf("%d\n", int(info)); // EXPECTED: 1
	for (int i=0;i<dataSize;i++)
	{
		xPoint[i] = spline1dcalc(s1, double(i+1));
		//printf("%f\t %.1f\n",i, double(v)); // EXPECTED: 0.10
	}
	spline1dfitpenalized(ttr, ytr,dataSize, 5, rho, info, s2, rep2);
    //printf("%d\n", int(info)); // EXPECTED: 1
	for (int i=0;i<dataSize;i++)
	{
		yPoint[i] = spline1dcalc(s2, double(i+1));
		printf("%.1f\t %.1f\n", double(xPoint[i]),double(yPoint[i])); // EXPECTED: 0.10
	}

	

	//Finding the critical points follows
	int probableindex[210]={-1};
	int probableshape[210]={-1};
	float probableCos[210]={-1};
	float bumpLimit=-0.99; //(was -0.94)when angle between two vectors is less than 160
	float cuspLimit = 0.6; //0.7 when angle between two vectors is less than 45 degrees
	int slope_distance = 1;
	int counter = 0;
	
	float dist=0;
	for (int i =1; i <dataSize; i++){
		dist = dist + sqrt(pow((xPoint[i]-xPoint[i-1]),2) +  pow((yPoint[i]-yPoint[i-1]),2) );
	}
	dist = dist/dataSize;
	std::cout<<"avg dist\t" <<dist<<std::endl; 
	xPoint[0]=startPx;
	yPoint[0]=startPy;
	xPoint[dataSize-1]=endPx;
	yPoint[dataSize-1]= endPy;

	////////////////writing output////////////////////
	//Bottle &outBottle = outputPort.prepare();
	//outBottle.addVocab(COMMAND_VOCAB_STRT);
	//Bottle& listBot2 = outBottle.addList();
 //   listBot2.addDouble(xPoint[0]);
 //   listBot2.addDouble(yPoint[0]);
	////listBot2.addDouble(100);//z value or height fixed..please CHECK
	//outputPort.write();   
	std::cout <<"start location" <<"\t"<< xPoint[0] <<"\t"<< yPoint[0] <<std::endl;
    /////////////////////////////////////////////////////////////

	outFile <<xPoint[0] << "\t" << yPoint[0] <<"\t" << backward_slope << "\t" << forward_slope << "\t" << cosRatio << std::endl; 	
	for (int i = slope_distance; i< dataSize-slope_distance; i++)
	{		
		vecA[0] = (xPoint[i-slope_distance]-xPoint[i]);
		vecA[1] = (yPoint[i-slope_distance]-yPoint[i]);
		vecB[0] = (xPoint[i+slope_distance]-xPoint[i]);
		vecB[1] = (yPoint[i+slope_distance]-yPoint[i]);
		if (vecA[0]!=0)
			backward_slope = vecA[1]/vecA[0];
		else
			backward_slope = 0;

		if (vecB[0]!=0)
			forward_slope =  vecB[1]/vecB[0];
		else
			forward_slope = 0;
		cosRatio = ((vecA[0]*vecB[0])+(vecA[1]*vecB[1])) / (sqrt((vecA[0]*vecA[0])+(vecA[1]*vecA[1])) * sqrt((vecB[0]*vecB[0])+(vecB[1]*vecB[1])));
		outFile <<xPoint[i] << "\t" << yPoint[i] <<"\t" << backward_slope << "\t" << forward_slope << "\t" << cosRatio << std::endl; 
	

		 if (cosRatio >cuspLimit)
		 { 
			probableindex[counter]=i;
			probableshape[counter]=1;
			probableCos[counter] = cosRatio;
			counter++;
			//std::cout <<"cusp at location" <<"\t"<< trajPoints[i][0] <<"\t"<< trajPoints[i][1] <<std::endl;
		}
		if (cosRatio >bumpLimit && cosRatio <cuspLimit)
		{
			probableindex[counter]=i;
			probableshape[counter]=0;
			probableCos[counter] = cosRatio;
			//std::cout <<"bump indices" <<"\t"<< probableindex[counter]<<"\t"<< counter <<std::endl;
			counter++;	
		}
	}
	outFile <<xPoint[dataSize-1] << "\t" << yPoint[dataSize-1] <<"\t" << backward_slope << "\t" << forward_slope << "\t" << cosRatio << std::endl; 
	outFile.close();
	i = 0;
	while (i<counter){
		float maxCos=probableCos[i];
		int idx=i;
		int j=i+1;
		while((j<counter)&&(sqrt(pow((xPoint[probableindex[j]]-xPoint[probableindex[j-1]]),2) +  pow((yPoint[probableindex[j]]-yPoint[probableindex[j-1]]),2)))<(2*dist)){
			if(probableCos[j]>maxCos){
				maxCos=probableCos[j];
				idx=j;
			}
			j++;
		}
	
		if (probableshape[idx]==1){
			////////////////writing output////////////////////
			//outBottle = outputPort.prepare();
			//outBottle.clear();
			//outBottle.addVocab(COMMAND_VOCAB_CUSP);
			//listBot2.clear();
			//listBot2 = outBottle.addList();
			//listBot2.addDouble(xPoint[probableindex[idx]]);
			//listBot2.addDouble(yPoint[probableindex[idx]]);
			////listBot2.addDouble(100);//z value or height fixed..please CHECK
			//outputPort.write();			
			std::cout <<"Cusp found at" <<"\t"<< xPoint[probableindex[idx]] <<"\t"<< yPoint[probableindex[idx]] <<std::endl;
			/////////////////////////////////////////////////////////////
		}
		if (probableshape[idx]==0){
			////////////////writing output////////////////////
			//outBottle = outputPort.prepare();
			//outBottle.clear();
			//outBottle.addVocab(COMMAND_VOCAB_BUMP);
			//listBot2.clear();
			//listBot2 = outBottle.addList();
			//listBot2.addDouble(xPoint[probableindex[idx]]);
			//listBot2.addDouble(yPoint[probableindex[idx]]);
			////listBot2.addDouble(100);//z value or height fixed..please CHECK
			//outputPort.write();	
			std::cout <<"Bump found at" <<"\t"<< xPoint[probableindex[idx]] <<"\t"<< yPoint[probableindex[idx]] <<std::endl;
			/////////////////////////////////////////////////////////////
		}
		i=j;
	}
    
	////////////////writing output////////////////////
	//outBottle = outputPort.prepare();
	//outBottle.clear();
	//outBottle.addVocab(COMMAND_VOCAB_END);
	//listBot2.clear();
	//listBot2 = outBottle.addList();
	//listBot2.addDouble(xPoint[dataSize-1]);
	//listBot2.addDouble(yPoint[dataSize-1]);
	////listBot2.addDouble(100);//z value or height fixed..please CHECK
	//outputPort.write();	
	std::cout <<"stop location" <<"\t"<< xPoint[dataSize-1] <<"\t"<< yPoint[dataSize-1] <<std::endl;
	/////////////////////////////////////////////////////////////
	


    return 0;
}

