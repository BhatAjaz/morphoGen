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
#include <yarp/dev/all.h>

#include <math.h>
#include "stdafx.h"
#include "interpolation.h"
#include <algorithm>    // std::sort


using namespace yarp::os;
using namespace std;
using namespace alglib;

#define COMMAND_VOCAB_STRT   VOCAB4('S','T','R','T')
#define COMMAND_VOCAB_BUMP   VOCAB4('B','U','M','P')
#define COMMAND_VOCAB_CUSP   VOCAB4('C','U','S','P')
#define COMMAND_VOCAB_END   VOCAB3('E','N','D')

//****************** global variables *******************************************
double xPoint[10000],yPoint[10000],xLeft[10000],yLeft[10000],xRight[10000],yRight[10000], a[3],locBegin[3],shapesArray[20][3],shapeType[20],shapeIndex[20];
int dataSize,originalSize,shapesCount=0;
int inBmp;
std::ofstream shapesFile;
yarp::os::BufferedPort<yarp::os::Bottle> outputPort;
//yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > plotPort;
yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > plotPort;
yarp::dev::IGazeControl *igaze; // Ikin controller of the gaze
yarp::dev::PolyDriver* clientGazeCtrl;          // polydriver for the gaze controller
int originalContext;                    // original context for the gaze Controller
YARP_DECLARE_DEVICES(icubmod)
bool FIX= true; 
bool visionON = true;
bool saccadeON=true;
bool Find3D = false;



//**************************************************************************

int signum(double n) { return (n < 0) ? -1 : (n > 0) ? +1 : 0; }

int round_int( double r ) {
    return (r > 0.0) ? (r + 0.5) : (r - 0.5); 
}
int maxOf(int a, int b) {
	if (a>b)
		return a;
	else
		return b;
}
inline double cbrt(double x) {

  if (fabs(x) < DBL_EPSILON) return 0.0;

  if (x > 0.0) return pow(x, 1.0/3.0);

  return -pow(-x, 1.0/3.0);
}

void function_cx_1_func(const real_1d_array &c, const real_1d_array &x, double &func, void *ptr) 
{
    // this callback calculates f(c,x)=exp(-c0*sqr(x0))
    // where x is a position on X-axis and c is adjustable parameter
    //func = exp(-c[0]*pow(x[0],2));
	func = c[0]+c[1]*pow((x[0]-c[2]),3)+c[3]*cbrt(pow((x[0]-c[4]),2));
}
void initGaze(){                
	//initializing gazecontrollerclient
    printf("initialising gazeControllerClient \n");
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
    //localCon.append(getName(""));
    option.put("local",localCon.c_str());
    clientGazeCtrl=new yarp::dev::PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    igaze->storeContext(&originalContext);
}
void releaseGaze(){
	igaze->restoreContext(originalContext);
    delete clientGazeCtrl;
}
bool isZero(double a)
{
	if (fabs(a)>0.000001)
		return false;
	else
		return true;
}
void useSaccade3D(int i){
	yarp::sig::Vector pxL(2);
	pxL[0] = xLeft[i];
	pxL[1] = yLeft[i];
	yarp::sig::Vector pxR(2);
	pxR[0] = xRight[i];
	pxR[1] = yRight[i];
	yarp::sig::Vector position(3);
	//yarp::sig::Vector positionR(3);
	//yarp::sig::Vector plane(4);
	//plane[0]=0;
	//plane[1]=0;
	//plane[2]=1;
	//plane[3]=-0.135;
	igaze->lookAtStereoPixels(pxL,pxR);
	Time::delay(2);
	igaze->getFixationPoint(position);
	//igaze->triangulate3DPoint(pxL,pxR,position);
	//igaze->get3DPointOnPlane(0,pxL,plane,position);
	locBegin[0] = position[0]*1000;
	locBegin[1] = position[1]*1000;
	locBegin[2] = position[2]*1000;
	printf("Corresponding 3D location using Saccade \t %.1f \t %.1f \t %.1f \n", locBegin[0],locBegin[1],locBegin[2]);	
	//igaze->blockNeckPitch();
	//igaze->blockNeckRoll();
	//igaze->blockNeckYaw();
}
void GetBothEyesVision()
{
	/////////////////using input data from vision::: control input by writing "/trackTrajectory/imitationCtrl:i" port with 1 (start) or 0 (stop)  
	yarp::os::BufferedPort<yarp::os::Bottle> inputLeft,inputRight,imitPort; 
	yarp::os::Bottle *incomingBottleLeft,*incomingBottleRight,*imitBottle;
	std::ofstream visionFile;
	visionFile.open("visionData.txt");
    inputLeft.open("/trackTrajectory/left/data:i");
	inputRight.open("/trackTrajectory/right/data:i");
	imitPort.open("/trackTrajectory/imitationCtrl:i");
	if (!Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/left/data:i"))
	{
		Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/left/data:i");
	}
	if (!Network::connect("/colorVisionRight/colordata:o","/trackTrajectory/right/data:i"))
	{
		Network::connect("/colorVisionRight/colordata:o","/trackTrajectory/right/data:i");
	}
	/*if (!Network::connect("/leftDetector/shapeData:o","/trackTrajectory/left/data:i"))
	{
		Network::connect("/leftDetector/shapeData:o","/trackTrajectory/left/data:i");
	}
	if (!Network::connect("/rightDetector/shapeData:o","/trackTrajectory/right/data:i"))
	{
		Network::connect("/rightDetector/shapeData:o","/trackTrajectory/right/data:i");
	}*/
	/*if (!Network::connect("/commandImitato:o", "/trackTrajectory/imitationCtrl:i"))
	{
		Network::connect("/commandImitato:o", "/trackTrajectory/imitationCtrl:i");
	}*/
	

	int i = 0;
	int x1,y1,x2,y2,x3,y3,x4,y4,countL,IDL,IDR,countR; //move these DOWN
	double uL,vL,uR,vR;
	int sameData=0;
	bool moving =false;
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
        if( inputLeft.getInputCount() && inputRight.getInputCount()){
            incomingBottleLeft    =   inputLeft.read(true);
			incomingBottleRight   =   inputRight.read(true);
            if ((incomingBottleLeft  !=  NULL) && (incomingBottleRight  !=  NULL))  {             
                if ( (incomingBottleLeft->size() > 0) && (incomingBottleRight->size() > 0) ) {

					countL = incomingBottleLeft->get(0).asInt();
					int indxLeft=1;
					bool objLeftFound=false;
					while((!objLeftFound) && (countL>0))
					{
						Bottle * tempBottleLeft =   incomingBottleLeft->get(indxLeft).asList();
						int tempID = tempBottleLeft->get(4).asInt();
						if (tempID==2) //green object
						{
							objLeftFound=true;
						}
						else
						{
							indxLeft++;
							countL--;
						}
					}
					/////
					countR = incomingBottleRight->get(0).asInt();
					int indxRight=1;
					
					bool objRightFound=false;
					while((!objRightFound) && (countR>0))
					{
						Bottle * tempBottleRight =   incomingBottleRight->get(indxRight).asList();
						int tempID = tempBottleRight->get(4).asInt();
						if (tempID==2)
						{
							objRightFound=true;
						}
						else
						{
							indxRight++;
							countR--;
						}
					}
					/////


					if (objLeftFound && objRightFound){
						Bottle * tempBottleLeft =   incomingBottleLeft->get(indxLeft).asList();
						Bottle * tempBottleRight =   incomingBottleRight->get(indxRight).asList();
						x1  =   abs(tempBottleLeft->get(0).asInt()); 
						y1  =   abs(tempBottleLeft->get(1).asInt()); 
						x2  =   abs(tempBottleLeft->get(2).asInt()); 
						y2  =   abs(tempBottleLeft->get(3).asInt());
						IDL  =   tempBottleLeft->get(4).asInt();  
						uL	=	x1+ x2/2.0;
						vL	=	y1+ y2/2.0;
						//printf("Left Eye Found object with ID\t %d: and centroid at \t %f \t %f  \n", ID, uL,vL);

						x3  =   abs(tempBottleRight->get(0).asInt()); 
						y3  =   abs(tempBottleRight->get(1).asInt()); 
						x4  =   abs(tempBottleRight->get(2).asInt()); 
						y4  =   abs(tempBottleRight->get(3).asInt());
						IDR  =   tempBottleRight->get(4).asInt();  
						uR	=	x3+ x4/2.0;
						vR	=	y3+ y4/2.0;
						//printf("Right Eye Found object with ID \t %d: and centroid at \t %f \t %f  \n", ID, uR,vR);
						visionFile <<uL <<"\t"<<vL <<"\t"<<uR <<"\t"<<vR <<"\t"<<std::endl;
						if((IDL==2) && (IDR==2)) {
							if (i==0){
								xLeft[i]=uL;
								yLeft[i]=vL;
								xRight[i]=uR;
								yRight[i]=vR;
								printf("Saving object at new location with ID \t %d: and centroid at \t %.1f \t %.1f  \n", IDL, uL,vL);
								visionFile <<uL <<"\t"<<vL <<"\t"<<uR <<"\t"<<vR <<"\t"<<std::endl;
								//useSaccade3D(i);

								//////////////// saccade again ends
								i++;

							}
						
							if (i>0&& i<5000){
								if ( (fabs(uL - xLeft[i-1]) >=1) || (fabs(vL - yLeft[i-1]) >= 1) ) {								
									xLeft[i]=uL;
									yLeft[i]=vL;
									xRight[i]=uR;
									yRight[i]=vR;
									if (i==1){
										xLeft[0]=uL;
										yLeft[0]=vL;
										xRight[0]=uR;
										yRight[0]=vR;
									}
									printf("Saving object at new location with ID \t %d: and centroid at \t %.1f \t %.1f  \n", IDL, uL,vL);
									visionFile <<uL <<"\t"<<vL <<"\t"<<uR <<"\t"<<vR <<"\t"<<std::endl;
									i++;
								}
							}
						}
						else
						{
						printf("Object ID not Green");
						}
					}
					else
					{
					printf ("No Object Found:::::Bottles are NULL \n");
					}
                }
				else {
				printf("Bottle size less than 0 \n");
				}
                incomingBottleLeft->clear();
				incomingBottleRight->clear();
                if( (incomingBottleLeft->size() != 0) || (incomingBottleRight->size() != 0)) {
                    printf("Error\n");   
                }
            }
			else
			{
				printf("Either of input bottles is NULL \n");
			}
        }
		else
		{
			printf("Missing connections trying reconnection\n");
			if (!Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/left/data:i"))
			{
				Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/left/data:i");
			}
			if (!Network::connect("/colorVisionRight/colordata:o","/trackTrajectory/right/data:i"))
			{
				Network::connect("/colorVisionRight/colordata:o","/trackTrajectory/right/data:i");
			}
			/*if (!Network::connect("/leftDetector/shapeData:o","/trackTrajectory/left/data:i"))
			{
				Network::connect("/leftDetector/shapeData:o","/trackTrajectory/left/data:i");
			}
			if (!Network::connect("/rightDetector/shapeData:o","/trackTrajectory/right/data:i"))
			{
				Network::connect("/rightDetector/shapeData:o","/trackTrajectory/right/data:i");
			}*/
			if (!Network::connect("/commandImitato:o", "/trackTrajectory/imitationCtrl:i"))
			{
				Network::connect("/commandImitato:o", "/trackTrajectory/imitationCtrl:i");
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
	visionFile.close();
	dataSize = i;
	originalSize=i;
	for (int i=0;i<dataSize;i++){
		xPoint[i]=xLeft[i];
		yPoint[i]=yLeft[i];
	}
	
}
void getColorTrajectory(){
	/////////////////using input data from vision::: control input by writing "/trackTrajectory/imitationCtrl:i" port with 1 (start) or 0 (stop)  
	yarp::os::BufferedPort<yarp::os::Bottle> inputLeft,inputRight,imitPort; 
	yarp::os::Bottle *incomingBottleLeft,*incomingBottleRight,*imitBottle;
	std::ofstream visionFile;
	visionFile.open("visionDataLeft.txt");
    inputLeft.open("/trackTrajectory/left/data:i");
	inputRight.open("/trackTrajectory/right/data:i");
	imitPort.open("/trackTrajectory/imitationCtrl:i");
	if (!Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/left/data:i"))
	{
		Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/left/data:i");
	}
	/*if (!Network::connect("/colorVisionRight/colordata:o","/trackTrajectory/right/data:i"))
	{
		Network::connect("/colorVisionRight/colordata:o","/trackTrajectory/right/data:i");
	}*/
	
	int i = 0;
	int x1,y1,x2,y2,x3,y3,x4,y4,countL,IDL,IDR,countR; //move these DOWN
	double uL,vL,uR,vR;
	int sameData=0;
	bool moving =false;
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
        if( inputLeft.getInputCount()){
            incomingBottleLeft    =   inputLeft.read(true);
            if ((incomingBottleLeft  !=  NULL))  {             
                if ( (incomingBottleLeft->size() > 0)) {

					countL = incomingBottleLeft->get(0).asInt();
					int indxLeft=1;
					bool objLeftFound=false;
					while((!objLeftFound) && (countL>0))
					{
						Bottle * tempBottleLeft =   incomingBottleLeft->get(indxLeft).asList();
						int tempID = tempBottleLeft->get(4).asInt();
						if (tempID==2) //green object
						{
							objLeftFound=true;
						}
						else
						{
							indxLeft++;
							countL--;
						}
					}
					/////


					if (objLeftFound){
						Bottle * tempBottleLeft =   incomingBottleLeft->get(indxLeft).asList();
						x1  =   abs(tempBottleLeft->get(0).asInt()); 
						y1  =   abs(tempBottleLeft->get(1).asInt()); 
						x2  =   abs(tempBottleLeft->get(2).asInt()); 
						y2  =   abs(tempBottleLeft->get(3).asInt());
						IDL  =   tempBottleLeft->get(4).asInt();  
						uL	=	x1+ x2/2.0;
						vL	=	y1+ y2/2.0;
						//printf("Left Eye Found object with ID\t %d: and centroid at \t %f \t %f  \n", ID, uL,vL);
						if((IDL==2)) {//green object
							if (i==0){
								xLeft[i]=uL;
								yLeft[i]=vL;
								printf("Saving object at new location with ID \t %d: and centroid at \t %.1f \t %.1f  \n", IDL, uL,vL);
								visionFile <<uL <<"\t"<<vL<<std::endl;
								i++;
							}
						
							if (i>0&& i<5000){
								if ( (fabs(uL - xLeft[i-1]) >=1) || (fabs(vL - yLeft[i-1]) >= 1) ) {								
									xLeft[i]=uL;
									yLeft[i]=vL;
									if (i==1){
										xLeft[0]=uL;
										yLeft[0]=vL;
									}
									printf("Saving object at new location with ID \t %d: and centroid at \t %.1f \t %.1f  \n", IDL, uL,vL);
									visionFile <<uL <<"\t"<<vL <<std::endl;
									i++;
								}
							}
						}
						else
						{
						printf("Object ID not Green");
						}
					}
					else
					{
					printf ("No Object Found:::::Bottles are NULL \n");
					}
                }
				else {
				printf("Bottle size less than 0 \n");
				}
                incomingBottleLeft->clear();
                if( (incomingBottleLeft->size() != 0)) {
                    printf("Error\n");   
                }
            }
			else
			{
				printf("Either of input bottles is NULL \n");
			}
        }
		else
		{
			printf("Missing connections trying reconnection\n");
			if (!Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/left/data:i"))
			{
				Network::connect("/colorVisionLeft/colordata:o","/trackTrajectory/left/data:i");
			}
			if (!Network::connect("/commandImitato:o", "/trackTrajectory/imitationCtrl:i"))
			{
				Network::connect("/commandImitato:o", "/trackTrajectory/imitationCtrl:i");
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
	visionFile.close();
	dataSize = i;
	originalSize=i;
	for (int i=0;i<dataSize;i++){
		xPoint[i]=xLeft[i];
		yPoint[i]=yLeft[i];
	}
}
double computeY(double x)
{
	double radius=175;///FIND radius
	double y = locBegin[1] - sqrt(abs(pow(radius,2) - pow((x-locBegin[0]+radius),2)) );
	return y;
}
void get3D(int i){
	yarp::sig::Vector pxL(2);
	pxL[0] = xLeft[i];
	pxL[1] = yLeft[i];
	yarp::sig::Vector pxR(2);
	pxR[0] = xRight[i];
	pxR[1] = yRight[i];
	yarp::sig::Vector position(3),positionTri(3),positionGaze(3);
	yarp::sig::Vector positionR(3);
	yarp::sig::Vector plane(4);
	plane[0]=0;
	plane[1]=0;
	plane[2]=1;
	plane[3]=-0.135;
	
	igaze->triangulate3DPoint(pxL,pxR,positionTri);
	printf("Corresponding 3D location using Triangulation \t %.1f \t %.1f \t %.1f \n", positionTri[0]*1000,positionTri[1]*1000,positionTri[2]*1000);
	igaze->get3DPointOnPlane(0,pxL,plane,position);
	printf("Corresponding 3D location using Left eye wrt plane \t %.1f \t %.1f \t %.1f \n", position[0]*1000,position[1]*1000,position[2]*1000);
	a[0] = position[0]*1000;
	a[1] = position[1]*1000;
	a[2] = position[2]*1000;
	igaze->get3DPointOnPlane(1,pxR,plane,positionR);
	printf("Corresponding 3D location using Right eye wrt plane \t %.1f \t %.1f \t %.1f \n", positionR[0]*1000,positionR[1]*1000,positionR[2]*1000);	
	igaze->lookAtStereoPixels(pxL,pxR);
	Time::delay(0.1);
	igaze->getFixationPoint(positionGaze);
	//printf("%f %f %f", positionGaze[0], positionGaze[1], positionGaze[2]);
	printf("Corresponding 3D location using Gaze control \t %.1f \t %.1f \t %.1f \n", positionGaze[0]*1000,positionGaze[1]*1000,positionGaze[2]*1000);
	
	//igaze->blockNeckPitch();
	//igaze->blockNeckRoll();
	//igaze->blockNeckYaw();

}
int findClosestPoint(double u,double v){
	double min=1000;
	int minIndex=0;
	for (int i=0;i<originalSize;i++) {
		double dist = sqrt(pow((u-xLeft[i]),2) +  pow((v-yLeft[i]),2));
		if(dist < min)
		{
			min= dist;
			minIndex= i;
		}
	}
	return minIndex;
}
double curvaTure(int i)
{
	double xD	=	(xPoint[i+1]-xPoint[i-1])/2;
	double xDD	=	(xPoint[i+1]-2*xPoint[i]+xPoint[i-1]);
	double yD	=	(yPoint[i+1]-yPoint[i-1])/2;
	double yDD	=	(yPoint[i+1]-2*yPoint[i]+yPoint[i-1]);
	double numer = ((xD*yDD) - (yD*xDD));
	double denom = sqrt(pow (((xD*xD)+(yD*yD)),3));
	double kappa =  numer/denom;
	return kappa;
}
int shapeEndPoint(int shapePt, int maxEnd)
{
	///finds next point in curve where direction of curvature changes
	int i = shapePt;
	if (i < maxEnd)
	{
		if (curvaTure(i) <=curvaTure(i+1))
		{
			while ((i < maxEnd) && (curvaTure(i) <=curvaTure(i+1)))
			{
				i++;
			}
			return i+1;
		}
		else 
		{
			while ((i < maxEnd) && (curvaTure(i) > curvaTure(i+1)))
			{
				i++;
			}
			return i+1;
		}

	}
	else
		return i;
}
void FindShape(int startPt, int shapePt, int endPt)
{
	//finds if cusp or bump based on angle
	double backward_slope=0, forward_slope=0, vecA[2],vecB[2],cosRatio=-1;
	double bumpLimit=-0.89; //(was -0.94)when angle between two vectors is less than 160
	double cuspLimit = 0.6; //0.7 when angle between two vectors is less than 45 degrees
			vecA[0] = (xPoint[shapePt]-xPoint[startPt]);
			vecA[1] = (yPoint[shapePt]-yPoint[startPt]);
			vecB[0] = (xPoint[shapePt]-xPoint[endPt]);
			vecB[1] = (yPoint[shapePt]-yPoint[endPt]);
			if (vecA[0]!=0)
				backward_slope = vecA[1]/vecA[0];
			else
				backward_slope = 0;

			if (vecB[0]!=0)
				forward_slope =  vecB[1]/vecB[0];
			else
				forward_slope = 0;
			cosRatio = ((vecA[0]*vecB[0])+(vecA[1]*vecB[1])) / (sqrt((vecA[0]*vecA[0])+(vecA[1]*vecA[1])) * sqrt((vecB[0]*vecB[0])+(vecB[1]*vecB[1])));
			//shapesFile <<xPoint[shapePt] << "\t" << yPoint[shapePt] <<"\t" << backward_slope << "\t" << forward_slope << "\t" << cosRatio << std::endl; 
			if (cosRatio >cuspLimit)
			 { 
				
				shapesFile<<"Cusp through findshape\t" <<xPoint[shapePt] <<"\t" << yPoint[shapePt] <<std::endl;
				int iN=findClosestPoint(xPoint[shapePt],yPoint[shapePt]);
				std::cout<< "Cusp at \t" <<xLeft[iN] <<"\t" << yLeft[iN] <<std::endl;
				if (Find3D)
				{
					get3D(iN);		
					std::cout<< "Corresponding x y locations in robot frame \t" <<a[0] <<"\t" << a[1] <<std::endl;
					std::cout<<std::endl;
				}
				if (!FIX){
					shapesArray[shapesCount][0]=a[0];
					shapesArray[shapesCount][1]=a[1];
					shapesArray[shapesCount][2]=a[2];
					shapeIndex[shapesCount]=iN;
					shapeType[shapesCount]=2;
					shapesCount++;
				}
			}
			if (cosRatio >bumpLimit && cosRatio <cuspLimit)
			{
				
				shapesFile<<"Bump \t" <<xPoint[shapePt] <<"\t" << yPoint[shapePt] <<std::endl;
				int iN=findClosestPoint(xPoint[shapePt],yPoint[shapePt]);
				inBmp=iN;
				std::cout<< "Bump at \t" <<xLeft[iN] <<"\t" << yLeft[iN] <<std::endl;
				if (Find3D)
				{
					get3D(iN);		
					std::cout<< "Corresponding x y locations in robot frame \t" <<a[0] <<"\t" << a[1] <<std::endl;
					std::cout<<std::endl;
				}
				if (FIX){
				shapesArray[shapesCount][0]=-320;//a[0];
				shapesArray[shapesCount][1]=-245;//a[1];
				shapesArray[shapesCount][2]=45;//a[2];
				shapeIndex[shapesCount]=iN;
				shapeType[shapesCount]=1;
				shapesCount++;
				}
			}
}
void removeJunkData()
{
		//////////////////Removing the first and last few points within a fixed distance from origin and end of the curve..
///////////// specific to vision data on icub
	double junkLength=7;
	int lc = 1;
	while (lc<dataSize/4)
	{
		if( sqrt(pow((xPoint[0]-xPoint[lc]),2) +  pow((yPoint[0]-yPoint[lc]),2) ) < junkLength)
			lc++;
		else
			break;
	}
	int beginD=lc;
	lc=dataSize;
	while (lc>dataSize/4)
	{
		if( sqrt(pow((xPoint[dataSize-1]-xPoint[lc-1]),2) +  pow((yPoint[dataSize-1]-yPoint[lc-1]),2) ) < junkLength)
			lc--;
		else
			break;
	}
	int endD=dataSize-lc;

	dataSize=dataSize -  (beginD + endD) ;
	for(int i = 0; i<dataSize;i++)
	{
		xPoint[i]= xPoint[i+beginD];
		yPoint[i]= yPoint[i+beginD];
	}	

}
void pixelIncrease()
{

//////////////////////Preprocessing of pixels to generate continuous data////////////////////////////
	double tempX[10000],tempY[10000];
	int j=0;
	tempX[j]=xPoint[0];
	tempY[j]=yPoint[0];
	j++;
	for (int i=1;i<dataSize-1;i++)
	{
		if (abs(xPoint[i]-xPoint[i-1]) <=1)
		{
			if (abs(yPoint[i]-yPoint[i-1]) <=1) 
			{
				tempX[j]=xPoint[i];
				tempY[j]=yPoint[i];
				j++;
			}
			else
			{
				double tempDiff = (yPoint[i]-yPoint[i-1]);
				int tD= abs(tempDiff);
				for (int k = 1; k < tD; k++)
				{
					tempX[j]=xPoint[i];
					tempY[j]=yPoint[i-1] + k*signum(tempDiff);
					j++;
				}
				tempX[j]=xPoint[i];
				tempY[j]=yPoint[i];
				j++;
			}
		}
		else 
		{
			if (abs(yPoint[i]-yPoint[i-1]) <=1) 
			{
				double tempDiff = (xPoint[i]-xPoint[i-1]);
				int tD= abs(tempDiff);
				for (int k = 1; k < tD; k++)
				{
					tempX[j]=xPoint[i-1] + k*signum(tempDiff);
					tempY[j]=yPoint[i];
					j++;
				}
				tempX[j]=xPoint[i];
				tempY[j]=yPoint[i];
				j++;
			}
			else
			{
				double xDiff = (xPoint[i]-xPoint[i-1]);
				double yDiff = (yPoint[i]-yPoint[i-1]);
				double AbsX = abs(xDiff);
				double AbsY = abs(yDiff);
				if (AbsX>AbsY)
				{
					for (int k = 1; k < int(AbsX); k++)
					{
						tempX[j]=xPoint[i-1] + k*signum(xDiff); 
						tempY[j]=yPoint[i-1] + round_int (k* AbsY/AbsX)*signum(yDiff);
						j++;
					}
					tempX[j]=xPoint[i];
					tempY[j]=yPoint[i];
					j++;
				}
				else
				{
					for (int k = 1; k < int(AbsY); k++)
					{
						tempX[j]=xPoint[i-1] + round_int (k* AbsX/AbsY)*signum(xDiff);
						tempY[j]=yPoint[i-1] + k*signum(yDiff);
						j++;
					}
					tempX[j]=xPoint[i];
					tempY[j]=yPoint[i];
					j++;

				}
			}

		}
	}
	dataSize = j;
	for (int i = 0; i< dataSize;i++)
	{
		xPoint[i] = tempX[i];
		yPoint[i] = tempY[i];
	}
}
void smooothenCurve()
{
		double timeVec[10000]={0};
		for (int i=0;i<dataSize;i++)
		{
			timeVec[i]=i+1;
		}
//////////////////////////smooting using spline///////////////////
		 real_1d_array ttr,xtr,ytr;
		//real_1d_array xtr = "[117,113,108,102,94,91,85,81,80,78,77,76,76]";
		//real_1d_array ytr = "[111,110,109,107,103,101,97,92,91,87,84,81,80]";
		 ttr.setcontent(dataSize,timeVec);
		 xtr.setcontent(dataSize,xPoint);
		 ytr.setcontent(dataSize,yPoint);
		ae_int_t info;
		spline1dinterpolant s1,s2;
		spline1dfitreport rep1,rep2;
		double rho= -2;//0//2;///between -5 and 10

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
			//printf("%.1f\t %.1f\n", double(xPoint[i]),double(yPoint[i])); // EXPECTED: 0.10
		}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////polynomial fitting
	//real_1d_array x = "[0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]";
    //real_1d_array y = "[0.00,0.05,0.26,0.32,0.33,0.43,0.60,0.60,0.77,0.98,1.02]";
		//
    // Fitting without individual weights
    //
    // NOTE: result is returned as barycentricinterpolant structure.
    //       if you want to get representation in the power basis,
    //       you can use barycentricbar2pow() function to convert
    //       from barycentric to power representation (see docs for 
    //       POLINT subpackage for more info).
    //
/*		real_1d_array ttr,xtr,ytr;
		ttr.setcontent(dataSize,timeVec);
		xtr.setcontent(dataSize,xPoint);
		ytr.setcontent(dataSize,yPoint);
		ae_int_t m = 4;
		ae_int_t info;
		barycentricinterpolant p1,p2;
		polynomialfitreport rep1,rep2;

    
		polynomialfit(ttr, xtr, m, info, p1, rep1);
		for (int i=0;i<dataSize;i++)
		{
			//v = barycentriccalc(p1, t);
			xPoint[i] = barycentriccalc(p1, double(i+1));
			//printf("%.2f\n", double(v)); // EXPECTED: 2.011
		}
		polynomialfit(ttr, ytr, m, info, p2, rep2);
		for (int i=0;i<dataSize;i++)
		{
			//v = barycentriccalc(p1, t);
			yPoint[i] = barycentriccalc(p2, double(i+1));
			//printf("%.2f\n", double(v)); // EXPECTED: 2.011
		}

*/
////////////////////////smooting using least square fit ///////////////////////////////////////////////////
/*
		real_2d_array ttr;
		real_1d_array xtr , ytr;
		ttr.setcontent(dataSize,1,timeVec);
		xtr.setcontent(dataSize,xPoint);
		ytr.setcontent(dataSize,yPoint);

	//real_2d_array x = "[[-1],[-0.8],[-0.6],[-0.4],[-0.2],[0],[0.2],[0.4],[0.6],[0.8],[1.0]]";
    //real_1d_array y = "[0.223130, 0.382893, 0.582748, 0.786628, 0.941765, 1.000000, 0.941765, 0.786628, 0.582748, 0.382893, 0.223130]";
    real_1d_array c = "[1,1,1,1,1]";
    double epsf = 0;
    double epsx = 0.01;
    ae_int_t maxits = 0;
    ae_int_t info;
    lsfitstate state;
    lsfitreport rep;
    double diffstep = 0.00001;

    //
    // Fitting without weights
    //
    lsfitcreatef(ttr, xtr, c, diffstep, state);
    lsfitsetcond(state, epsf, epsx, maxits);
    alglib::lsfitfit(state, function_cx_1_func);
    lsfitresults(state, info, c, rep);
    printf("%d\n", int(info)); // EXPECTED: 2
    printf("%s\n", c.tostring(1).c_str()); // EXPECTED: [1.5]
	double cxArr[5]={0};
	double *pppp= c.getcontent();
	for (int hh = 0; hh<5;hh++)
	{
		cxArr[hh]= *pppp;
		pppp++;
	}
	for (int k =0;k<dataSize;k++)
	{
		tempX[k] = c[0]+c[1]*pow((k-c[2]),3)+c[3]*cbrt(pow((k-c[4]),2));
	}
	//////////////////////////
	lsfitcreatef(ttr, ytr, c, diffstep, state);
    lsfitsetcond(state, epsf, epsx, maxits);
    alglib::lsfitfit(state, function_cx_1_func);
    lsfitresults(state, info, c, rep);
    printf("%d\n", int(info)); // EXPECTED: 2
    printf("%s\n", c.tostring(1).c_str()); // EXPECTED: [1.5]

	for (int hh = 0; hh<5;hh++)
	{
		cxArr[hh]= *pppp;
		pppp++;
	}
	for (int k =0;k<dataSize;k++)
	{
		tempY[k] = c[0]+c[1]*pow((yPoint[k]-c[2]),3)+c[3]*cbrt(pow((yPoint[k]-c[4]),2));
	}
	*/
	
}
double tricube(double x)
{
	 /**
     * Compute the 
     * <a href="http://en.wikipedia.org/wiki/Local_regression#Weight_function">tricube</a>
     * weight function
     *
     * @param x the argument
     * @return (1-|x|^3)^3
     */
    double tmp = 1 - x * x * x;
    return tmp * tmp * tmp;
}
void updateBandwidthInterval(double xval[], int i, int bandwidthInterval[])
{
	   /**
     * Given an index interval into xval that embraces a certain number of
     * points closest to xval[i-1], update the interval so that it embraces
     * the same number of points closest to xval[i]
     *
     * @param xval arguments array
     * @param i the index around which the new interval should be computed
     * @param bandwidthInterval a two-element array {left, right} such that: <p/>
     * <tt>(left==0 or xval[i] - xval[left-1] > xval[right] - xval[i])</tt>
     * <p/> and also <p/>
     * <tt>(right==xval.length-1 or xval[right+1] - xval[i] > xval[i] - xval[left])</tt>.
     * The array will be updated.
     */
    int left = bandwidthInterval[0];
    int right = bandwidthInterval[1];
    // The right edge should be adjusted if the next point to the right
    // is closer to xval[i] than the leftmost point of the current interval
    if (right < dataSize - 1 && //dataSize likely xval.Length CHECK
        xval[right + 1] - xval[i] < xval[i] - xval[left])
    {
        bandwidthInterval[0]++;
        bandwidthInterval[1]++;
    }
}
double* smooth( double yval[], int bandwidth) //fix return type double[] CHECK
{
	/**
    * Compute a loess fit on the data at the original abscissae.
    *
    * @param xval the arguments for the interpolation points
    * @param yval the values for the interpolation points
    * @return values of the loess fit at corresponding original abscissae
    * @throws MathException if some of the following conditions are false:
    * <ul>
    * <li> Arguments and values are of the same size that is greater than zero</li>
    * <li> The arguments are in a strictly increasing order</li>
    * <li> All arguments and values are finite real numbers</li>
    * </ul>
    */
    double xval[10000]={0};
		for (int i=0;i<dataSize;i++)
		{
			xval[i]=i+1;
		}
	int n = dataSize; //xval.Length; CHECK
    int bandwidthInPoints = (int)(bandwidth * n);
	int robustnessIters=0;


    double *res = new double[n];
    double* residuals = new double[n];
    double* sortedResiduals = new double[n];
    double* robustnessWeights = new double[n];

    // Do an initial fit and 'robustnessIters' robustness iterations.
    // This is equivalent to doing 'robustnessIters+1' robustness iterations
    // starting with all robustness weights set to 1.
    for (int i = 0; i < n; i++) robustnessWeights[i] = 1; //robustnessWeights.Length
    for (int iter = 0; iter <= robustnessIters; ++iter)
    {
        int bandwidthInterval[] = { 0, bandwidthInPoints - 1 };
        // At each x, compute a local weighted linear regression
        for (int i = 0; i < n; ++i)
        {
            double x = xval[i];

            // Find out the interval of source points on which
            // a regression is to be made.
            if (i > 0)
            {
                updateBandwidthInterval(xval, i, bandwidthInterval);
            }

            int ileft = bandwidthInterval[0];
            int iright = bandwidthInterval[1];

            // Compute the point of the bandwidth interval that is
            // farthest from x
            int edge;
            if ((xval[i] - xval[ileft]) > (xval[iright] - xval[i]))
            {
                edge = ileft;
            }
            else
            {
                edge = iright;
            }

            // Compute a least-squares linear fit weighted by
            // the product of robustness weights and the tricube
            // weight function.
            // See http://en.wikipedia.org/wiki/Linear_regression
            // (section "Univariate linear case")
            // and http://en.wikipedia.org/wiki/Weighted_least_squares
            // (section "Weighted least squares")
            double sumWeights = 0;
            double sumX = 0, sumXSquared = 0, sumY = 0, sumXY = 0;
            double denom = abs(1.0 / (xval[edge] - x));
            for (int k = ileft; k <= iright; ++k)
            {
                double xk = xval[k];
                double yk = yval[k];
                double dist;
                if (k < i)
                {
                    dist = (x - xk);
                }
                else
                {
                    dist = (xk - x);
                }
                double w = tricube(dist * denom) * robustnessWeights[k];
                double xkw = xk * w;
                sumWeights += w;
                sumX += xkw;
                sumXSquared += xk * xkw;
                sumY += yk * w;
                sumXY += yk * xkw;
            }

            double meanX = sumX / sumWeights;
            double meanY = sumY / sumWeights;
            double meanXY = sumXY / sumWeights;
            double meanXSquared = sumXSquared / sumWeights;

            double beta;
            if (meanXSquared == meanX * meanX)
            {
                beta = 0;
            }
            else
            {
                beta = (meanXY - meanX * meanY) / (meanXSquared - meanX * meanX);
            }

            double alpha = meanY - beta * meanX;

            res[i] = beta * x + alpha;
            residuals[i] = abs(yval[i] - res[i]);
        }

        // No need to recompute the robustness weights at the last
        // iteration, they won't be needed anymore
        if (iter == robustnessIters)
        {
            break;
        }

        // Recompute the robustness weights.

        // Find the median residual.
        // An arraycopy and a sort are completely tractable here, 
        // because the preceding loop is a lot more expensive
		for(int i=0; i <n;i++){
			sortedResiduals[i]=residuals[i];
		}
        //System.Array.Copy(residuals, sortedResiduals, n); CHECK
        //System.arraycopy(residuals, 0, sortedResiduals, 0, n);
        //Array.Sort<double>(sortedResiduals);// CHECK
		sort(sortedResiduals,sortedResiduals+n);
        double medianResidual = sortedResiduals[n / 2];

        if (medianResidual == 0)
        {
            break;
        }

        for (int i = 0; i < n; ++i)
        {
            double arg = residuals[i] / (6 * medianResidual);
            robustnessWeights[i] = (arg >= 1) ? 0 : pow(1 - arg * arg, 2);
        }
    }

    return res;
}
bool MaxMinLocation(int i)
{
		double xD	=	(xPoint[i+1]-xPoint[i-1])/2;
		double yD	=	(yPoint[i+1]-yPoint[i-1])/2;
		double xDPrev	=	(xPoint[i]-xPoint[i-2])/2;
		double yDPrev	=	(yPoint[i]-yPoint[i-2])/2;
		if ( (xD!=0 && xDPrev!=0) && ( (((yD/xD) > 0) && ((yDPrev/xDPrev) <0))   || (((yD/xD) < 0) && ((yDPrev/xDPrev) > 0)) ) )
		{
			return true;
		}
		else
		{
			return false;
		}
}
void writeOutput(){
	if (!Network::connect("/trackTrajectory/shapePoints:o", "/shapesImitato:i"))
	{
		Network::connect("/trackTrajectory/shapePoints:o", "/shapesImitato:i");
	}
	 //outBottle;
	bool recieverFound=false;
	Bottle* listBot= new Bottle[20];
	while(!recieverFound)
	{
		if(outputPort.getOutputCount()) {
			recieverFound=true;
			Time::delay(0.1);
			Bottle& outBottle = outputPort.prepare();
			outBottle.clear();
			outBottle.addInt(shapesCount);
			for(int i =0;i<shapesCount;i++) 
			{
				if (shapeType[i] == 0){
				////////////////writing start////////////////////
							outBottle.addVocab(COMMAND_VOCAB_STRT);
						
							Bottle &tmp = outBottle.addList();
							tmp.clear();
							tmp.addDouble(shapesArray[i][0]);
							tmp.addDouble(shapesArray[i][1]);
							tmp.addDouble(shapesArray[i][2]);
				}
				else if (shapeType[i] == 1){
				////////////////writing bumps////////////////////
							outBottle.addVocab(COMMAND_VOCAB_BUMP);
							Bottle &tmp = outBottle.addList();
							tmp.clear();
							tmp.addDouble(shapesArray[i][0]);
							tmp.addDouble(shapesArray[i][1]);
							tmp.addDouble(shapesArray[i][2]);
				}
				else if (shapeType[i] == 2){
				////////////////writing cusps////////////////////
							outBottle.addVocab(COMMAND_VOCAB_CUSP);
							Bottle &tmp = outBottle.addList();
							tmp.clear();
							tmp.addDouble(shapesArray[i][0]);
							tmp.addDouble(shapesArray[i][1]);
							tmp.addDouble(shapesArray[i][2]);
				}
				else if (shapeType[i] == 3){
				////////////////writing end////////////////////
							outBottle.addVocab(COMMAND_VOCAB_END);
							Bottle &tmp = outBottle.addList();
							tmp.clear();
							tmp.addDouble(shapesArray[i][0]);
							tmp.addDouble(shapesArray[i][1]);
							tmp.addDouble(shapesArray[i][2]);
				}
			}
			outputPort.write();
		}
		else
		{
			if (!Network::connect("/trackTrajectory/shapePoints:o", "/shapesImitato:i"))
			{
				Network::connect("/trackTrajectory/shapePoints:o", "/shapesImitato:i");
			}
		}

	}
}

void trajPlotter()
{
	if (!Network::connect("/trackTrajectory/plotPoints:o","/trajView")){
		Network::connect("/trackTrajectory/plotPoints:o","/trajView");
	}
	if (plotPort.getOutputCount()) {
         //yarp::sig::ImageOf<yarp::sig::PixelMono> &outputImage = plotPort.prepare();
		 yarp::sig::ImageOf<yarp::sig::PixelRgb> &outputImage = plotPort.prepare();
        // processing of the outputImage
        int height = 240;
        int width  = 320;
        //int scale  = 10;
		int maxX =0;
		int minX = 320;
		int maxY = 0;
		int minY = 240;
		for(int i=0;i<originalSize;i++)
		{
			if (xLeft[i]>maxX){
				maxX=xLeft[i];
			}
			else if (xLeft[i]<minX){
				minX=xLeft[i];
			}

			if (yLeft[i]>maxY){
				maxY=yLeft[i];
			}
			else if (yLeft[i]<minY){
				minY=yLeft[i];
			}
		}
		if(maxX<310) maxX+=10;
		if(maxY<230) maxY+=10;
		if(minX>10) minX-=10;
		if(minY>10) minY-=10;

		int imgMat[320][240]={0};

		int shapesEncoded=0;
		for(int i=0;i<originalSize;i++)
		{

			if((shapesEncoded<=shapesCount) && (shapeIndex[shapesEncoded]==i)){
				if(shapeType[shapesEncoded]==1) //BUMP
				{
					imgMat[round_int(xLeft[i])][round(yLeft[i])]=5;
					imgMat[round_int(xLeft[i])-1][round(yLeft[i])]=5;
					imgMat[round_int(xLeft[i])+1][round(yLeft[i])]=5;
					imgMat[round_int(xLeft[i])][round(yLeft[i])-1]=5;
					imgMat[round_int(xLeft[i])][round(yLeft[i])+1]=5;
				}
				else if(shapeType[shapesEncoded]==2) //CUSP
				{
					imgMat[round_int(xLeft[i])][round(yLeft[i])]=6;
					imgMat[round_int(xLeft[i])-1][round(yLeft[i])]=6;
					imgMat[round_int(xLeft[i])+1][round(yLeft[i])]=6;
					imgMat[round_int(xLeft[i])][round(yLeft[i])-1]=6;
					imgMat[round_int(xLeft[i])][round(yLeft[i])+1]=6;
				}
				else//End points
				{
					imgMat[round_int(xLeft[i])][round(yLeft[i])]=7;
					imgMat[round_int(xLeft[i])-1][round(yLeft[i])]=7;
					imgMat[round_int(xLeft[i])+1][round(yLeft[i])]=7;
					imgMat[round_int(xLeft[i])][round(yLeft[i])-1]=7;
					imgMat[round_int(xLeft[i])][round(yLeft[i])+1]=7;
				}

				shapesEncoded++;
			}
			else
				imgMat[round_int(xLeft[i])][round(yLeft[i])]=1;
		}

		width  = maxX-minX;
		height = maxY-minY;
       	
        outputImage.resize(width, height);

		 for (int c = minX; c < maxX; c++) {
			for (int r = maxY; r > minY; r--) {                   
                yarp::sig::PixelRgb& pix = outputImage.pixel(c-minX,maxY-r);
				if (imgMat[c][r]==7){// Yellow start and end
					pix.r = 255;
					pix.g = 255;
					pix.b = 0;
				}
				else if (imgMat[c][r]==6){//Blue Cusps
					pix.r = 0;
					pix.g = 0;
					pix.b = 255;
				}
				else if (imgMat[c][r]==5){//Red Bumps
					pix.r = 255;
					pix.g = 0;
					pix.b = 0;
				}
				else
				{//green trajectory and black background
					pix.r = 0;
					pix.g = imgMat[c][r] * 255;
					pix.b = 0;
				}
            }                                   
        }

        plotPort.write();  
    }

}
void main(int argc, char * argv[])
{
    
    Network yarp;
	YARP_REGISTER_DEVICES(icubmod)
    ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setDefaultConfigFile("traject.ini");    //overridden by --from parameter
    rf.setDefaultContext("morphoGenApp/conf");    //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);  
    //yarp::os::ConstString path = rf.findPath("trajPoints.txt");
    //printf("File found! path %s \n", path.c_str());
	plotPort.open("/trackTrajectory/plotPoints:o");
	outputPort.open("/trackTrajectory/shapePoints:o");
	std::ofstream smoothFile;//,visionFile;
	smoothFile.open("smoothData.txt");
	if (visionON){
		if (saccadeON) initGaze();
					//locBegin[0]=-272;
					//locBegin[1]=-146;
					//
		if (saccadeON) {GetBothEyesVision();}
		else {getColorTrajectory();}
		}
//////////////////////////data from vision loaded///////////////////////////////////////////////////////
	if (!visionON)
	{
/////////////// ALTERNATE USE INPUT DATA FROM FILE ::testing purpose ...generally this section is commmented out/////////////////////////////
		std::ifstream tempFile;
		tempFile.open("trajFile.txt");
		int i=0;
		while(!tempFile.eof())
		{
			double waste;
			double x,y;
			tempFile >> x;
			xPoint[i]=x;
			xLeft[i]=x;
			tempFile >> y;
			yPoint[i]=y;
			yLeft[i]=y;
			//tempFile >> waste;
			//tempFile >> waste;
			printf("loaded object location centroid as \t %f \t %f  \n", xPoint[i],yPoint[i]);
			i++;
		}
		i=i-1;
		tempFile.close();
		dataSize = i;
		originalSize=i;
	}
	//////////////////////////////////////Trajectory saved/////////////////////////////////////////////////////////////////
		
		if (originalSize >0) {
			shapesFile.open("shapesData.txt");
			double startPx=xPoint[0];
			double startPy=yPoint[0];
			shapesFile<<"Start \t" <<startPx <<"\t" << startPy <<std::endl;
			std::cout<< "Start at \t" <<xLeft[0] <<"\t" << yLeft[0] <<std::endl;
			if (Find3D)
			{
				get3D(0);
				a[1]=computeY(a[0]);
				shapesFile<<"3D \t" <<a[0] <<"\t" << a[1] <<std::endl;
				std::cout<< "Corresponding x y locations in robot frame \t" <<a[0] <<"\t" << a[1] <<"\t" << a[2]<<std::endl;
				std::cout<<std::endl;
			}

			if (FIX)
			{
			shapesArray[shapesCount][0]=-300;//a[0];
			shapesArray[shapesCount][1]=-100;//a[1];
			shapesArray[shapesCount][2]=45;//a[2];
			}
			else
			{
			shapesArray[shapesCount][0]=a[0];
			shapesArray[shapesCount][1]=a[1];
			shapesArray[shapesCount][2]=a[2];
			}
			shapeIndex[shapesCount]=0;
			shapeType[shapesCount]=0;
			shapesCount++;
			double endPx=xPoint[originalSize-1];
			double endPy=yPoint[originalSize-1];
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////Removing the first and last few points within a fixed distance from origin and end of the curve..
			removeJunkData();
		//////////////////////Preprocessing of pixels to generate continuous data////////////////////////////
			//pixelIncrease();
			

			if (dataSize>2) {
				//////////////////////////////////////////////Smoothening the trajectory follows/////////////////////////////
				smooothenCurve();
				/*double* tmp1 =smooth(xPoint,5);
				double* tmp2 =smooth(yPoint,5);
				for (int i = 1; i< dataSize-1;i++){
					xPoint[i]=tmp1[i];
					yPoint[i]=tmp2[i];
				}*/
				


		///////////////////////New Code
			double dist=0;
			for (int i = 1; i< dataSize-1;i++)
			{
				double cc = curvaTure(i);
				double xD	=	(xPoint[i+1]-xPoint[i-1])/2;
				double xDD	=	(xPoint[i+1]-2*xPoint[i]+xPoint[i-1]);
				double yD	=	(yPoint[i+1]-yPoint[i-1])/2;
				double yDD	=	(yPoint[i+1]-2*yPoint[i]+yPoint[i-1]);
				dist = dist + sqrt(pow((xPoint[i]-xPoint[i-1]),2) +  pow((yPoint[i]-yPoint[i-1]),2) );
				//smoothFile<< xPoint[i] <<"\t" << yPoint[i] << "\t" << cc <<std::endl;
				smoothFile<<xPoint[i] <<"\t" << yPoint[i] << "\t" << xD<< "\t" << xDD<< "\t" << yD << "\t" << yDD << "\t"<<curvaTure(i)<<"\t"<<yD/xD <<std::endl;
			}
			smoothFile.close();
			dist = dist/dataSize;
			double cuspLimit = (1.0/dist);
			int indexJump=maxOf(ceil(dataSize/20.0),5);
			std::cout<<"avg dist\t"<<dist <<"\n indexJump\t"<<indexJump <<"\n cuspLimit\t"<<cuspLimit<<std::endl; 
			double shapeSpace= 2*dist;
			double xPrev=startPx;
			double yPrev=startPy;
			bool oneFound= false;
			for (int i = 2;i <dataSize-2;)
			{
				///start somewhat far from beginnig n end use datasize/smthing
				int inc=1;
				double xD	=	(xPoint[i+1]-xPoint[i-1])/2;
				double xDD	=	(xPoint[i+1]-2*xPoint[i]+xPoint[i-1]);
				double yD	=	(yPoint[i+1]-yPoint[i-1])/2;
				double yDD	=	(yPoint[i+1]-2*yPoint[i]+yPoint[i-1]);
				if (abs(curvaTure(i)-curvaTure(i-1)) > cuspLimit)
				{
					if ( (sqrt(pow((xPrev-xPoint[i]),2) +  pow((yPrev-yPoint[i]),2)) > shapeSpace) && (sqrt(pow((endPx-xPoint[i]),2) +  pow((endPy-yPoint[i]),2)) > shapeSpace)){
						shapesFile<<"Cusp \t" <<xPoint[i] <<"\t" << yPoint[i] << "\t" << xD<< "\t" << xDD<< "\t" << yD << "\t" << yDD << "\t"<<curvaTure(i)<<"\t"<<yD/xD <<std::endl;
						//inc=indexJump;
						xPrev = xPoint[i];
						yPrev = yPoint[i];
						oneFound=true;
						int iN=findClosestPoint(xPoint[i],yPoint[i]);
						std::cout<< "Cusp at \t" <<xLeft[iN] <<"\t" << yLeft[iN] <<std::endl;
						if (Find3D)
						{
							get3D(iN);
							a[1]=computeY(a[0]);
							shapesFile<<"3D \t" <<a[0] <<"\t" << a[1] <<std::endl;
							std::cout<< "Corresponding x y locations in robot frame \t" <<a[0] <<"\t" << a[1] <<"\t" << a[2]<<std::endl;
							std::cout<<std::endl;
						}
						
						
						if(!FIX){
						shapesArray[shapesCount][0]=a[0];
						shapesArray[shapesCount][1]=a[1];
						shapesArray[shapesCount][2]=a[2];
						
						shapeIndex[shapesCount]=iN;
						shapeType[shapesCount]=2;
						shapesCount++;
						}
					}
				}

				else if (MaxMinLocation(i))
				{
					if ( (sqrt(pow((xPrev-xPoint[i]),2) +  pow((yPrev-yPoint[i]),2)) >shapeSpace) && (sqrt(pow((endPx-xPoint[i]),2) +  pow((endPy-yPoint[i]),2)) >shapeSpace)){
						shapesFile<<"Bump \t" <<xPoint[i] <<"\t" << yPoint[i] << "\t" << xD<< "\t" << xDD<< "\t" << yD << "\t" << yDD << "\t"<<curvaTure(i)<<"\t"<<yD/xD <<std::endl;
						//inc=indexJump;
						xPrev = xPoint[i];
						yPrev = yPoint[i];
						oneFound=true;
						int iN=findClosestPoint(xPoint[i],yPoint[i]);
						inBmp=iN;
						std::cout<< "Bump at \t" <<xLeft[iN] <<"\t" << yLeft[iN] <<std::endl;
						if (Find3D)
						{
							get3D(iN);	
							a[1]=computeY(a[0]);
							shapesFile<<"3D \t" <<a[0] <<"\t" << a[1] <<std::endl;
							std::cout<< "Corresponding x y locations in robot frame \t" <<a[0] <<"\t" << a[1]<<"\t" << a[2] <<std::endl;
							std::cout<<std::endl;
						}
						if (FIX){
						shapesArray[shapesCount][0]=-320;//a[0];
						shapesArray[shapesCount][1]=-245;//a[1];
						shapesArray[shapesCount][2]=45;//a[2];
						}
						else{
						shapesArray[shapesCount][0]=a[0];
						shapesArray[shapesCount][1]=a[1];
						shapesArray[shapesCount][2]=a[2];					
						}
						shapeIndex[shapesCount]=iN;
						shapeType[shapesCount]=1;
						shapesCount++;
						if (FIX) i=dataSize-4;
					}
		
				}
				/*if ( ( (isZero(xD)) && (!isZero(xDD)) && (!isZero(yD)) ) || ( (isZero(yD)) && (!isZero(yDD)) && (!isZero(xD)) ))
				{
					std::cout <<"bump location" <<"\t"<< xPoint[i] <<"\t"<< yPoint[i] <<std::endl;
					shapesFile << "bump \t"<< xPoint[i] <<"\t" << yPoint[i]   <<std::endl;
				}

				if ( (isZero(xD)) && (!isZero(xDD)) && (isZero(yD)) && (!isZero(yDD)) ) 
				{
					std::cout <<"cusp location" <<"\t"<< xPoint[i] <<"\t"<< yPoint[i] <<std::endl;
					shapesFile << "cusp \t"<< xPoint[i] <<"\t" << yPoint[i] <<std::endl;
				}*/
				//smoothFile<<xPoint[i] <<"\t" << yPoint[i] << "\t" << xD<< "\t" << xDD<< "\t" << yD << "\t" << yDD << "\t"<<curvaTure(i)<<"\t"<<yD/xD <<std::endl;
				i=i+inc;
			}
			//smoothFile.close();


			if (!oneFound)
			{	
				xPoint[0]=startPx;
				yPoint[0]=startPy;
				xPoint[dataSize-1] =endPx;
				yPoint[dataSize-1]=endPy;
				FindShape(0,dataSize/2,dataSize-1);	

			}
		/*	
			i = 1;
			int startPoint=i;
			int shapePoint=i;
			int endPoint=i;
			if (i<dataSize-2)
			{
				shapesFile.open("shapesData.txt");
				//send out first point
				if ((curvaTure(i) <=curvaTure(i+1))) //&& (curvaTure(i+1) <=curvaTure(i+2)) && (curvaTure(i+2) <=curvaTure(i+3)) )
				{
					while ((i<dataSize-2) && (curvaTure(i) <=curvaTure(i+1))) //&& (curvaTure(i+1) <=curvaTure(i+2)) && (curvaTure(i+2) <=curvaTure(i+3)) )
					{
						i++;
					}
					shapePoint = i+1;
					while ((shapePoint+1) < dataSize-2)
					{
						endPoint = shapeEndPoint(shapePoint, dataSize-2);
						FindShape(startPoint,shapePoint, endPoint);
						startPoint=shapePoint;
						shapePoint=endPoint;
					}
				}

				else if ((curvaTure(i) > curvaTure(i+1))) //&& (curvaTure(i+1) >=curvaTure(i+2)) && (curvaTure(i+2) >=curvaTure(i+3)))
				{
					while ((i<dataSize-2)  && (curvaTure(i) >curvaTure(i+1)) )
					{
						i++;
					}
					shapePoint = i;
					while ((shapePoint+1) < dataSize-2)
					{
						endPoint = shapeEndPoint(shapePoint, dataSize-2);
						FindShape(startPoint,shapePoint, endPoint);
						startPoint=shapePoint;
						shapePoint=endPoint;
					}
				}
				///send out last point
				shapesFile.close();
			}
			*/
			}
			shapesFile<<"End \t" <<endPx <<"\t" << endPy <<std::endl;
			std::cout<< "End (in vision frame)at \t" <<xLeft[originalSize-1] <<"\t" << yLeft[originalSize-1] <<std::endl;
			if (Find3D)
			{
				get3D(originalSize-1);	
				a[1]=computeY(a[0]);
				shapesFile<<"3D \t" <<a[0] <<"\t" << a[1] <<std::endl;
				std::cout<< "Corresponding x y locations in robot frame \t" <<a[0] <<"\t" << a[1] <<"\t" << a[2]<<std::endl;
				std::cout<<std::endl;
			}
			if (FIX){
				shapesArray[shapesCount][0]=-440;//a[0];
				shapesArray[shapesCount][1]=-270;//a[1];
				shapesArray[shapesCount][2]=45;//a[2];
			}
			else{
				shapesArray[shapesCount][0]=a[0];
				shapesArray[shapesCount][1]=a[1];
				shapesArray[shapesCount][2]=a[2];
			}
			shapeIndex[shapesCount]=originalSize-1;
			shapeType[shapesCount]=3;
			shapesCount++;
			shapesFile.close();
			trajPlotter();
			if (saccadeON) writeOutput();
			outputPort.close();
			plotPort.close();
			
		}
		else
		{
			std::cout <<"No valid input data:: exiting the program "<<std::endl;
		}
	
	
	if (saccadeON)	releaseGaze();
    return;
	
}




















	////////old code
/*
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
		//xPoint[0]=startPx;
		//yPoint[0]=startPy;
		//xPoint[dataSize-1]=endPx;
		//yPoint[dataSize-1]= endPy;

		/////////////////sending output////////////////////
		Bottle &outBottle = outputPort.prepare();
		outBottle.addVocab(COMMAND_VOCAB_STRT);
		Bottle& listBot2 = outBottle.addList();
		listBot2.addDouble(xPoint[0]);
		listBot2.addDouble(yPoint[0]);
		listBot2.addDouble(100);//z value or height fixed..please CHECK
		outputPort.write();
		/////////////////////////////////////////////////////////////

		processedFile <<startPx << "\t" <<startPy <<"\t" << backward_slope << "\t" << forward_slope << "\t" << cosRatio << std::endl; 	
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
			processedFile <<xPoint[i] << "\t" << yPoint[i] <<"\t" << backward_slope << "\t" << forward_slope << "\t" << cosRatio << std::endl; 
	

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
		processedFile << endPx << "\t" <<  endPy <<"\t" << backward_slope << "\t" << forward_slope << "\t" << cosRatio << std::endl; 
		processedFile.close();
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
*/

/*
///////////////////ploy x y z on yarpscope
			if (yarpScopePort.getOutputCount() >0) {
				Bottle & PlotPoints = yarpScopePort.prepare();
				PlotPoints.clear();
				PlotPoints.addDouble(Gam);
				if (HandAct==0){
					PlotPoints.addDouble( X_pos[0]);
					PlotPoints.addDouble( X_pos[1]);
					PlotPoints.addDouble( X_pos[2]);
					//PlotPoints.addDouble(sqrt(pow(X_pos[0]-x_ini,2)+pow(X_pos[1]-y_ini,2)+pow(X_pos[2]-z_ini,2)));
				}
				if (HandAct==1){
					PlotPoints.addDouble(X_posL[0]);
					PlotPoints.addDouble(-X_posL[1]);
					PlotPoints.addDouble(X_posL[2]);
					//PlotPoints.addDouble(sqrt(pow(X_posL[0]-x_iniL,2)+pow(X_posL[1]-y_iniL,2)+pow(X_posL[2]-z_iniL,2)));
				}
				yarpScopePort.write(true);
				Time::delay(0.02);
			}
/////////////////////////////////////
*/