// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Vishwanathan Mohan
  * email: Vishwanathan Mohan@iit.it
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
 * @file tutorialThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */

#pragma once

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include "segment.h"
#include <math.h>
#include <vector>
#include "MessageFormats/VocabDefinitions.h"
#include "MessageFormats/DarwinMessages.h"
#include "../include/Kinematics.h"
#include "../include/KinematicsTX90.h"
#include "../include/KinematicsRX130.h"
#include "../include/vecmath3D.h"
#include "KinematicsAccess.h"
#include "PMPAccess.h"

#define COMMAND_VOCAB_REACH   VOCAB3('R','E','A')

class PMPThread : public yarp::os::Thread,public darwin::reach::PMPAccess {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string weights1Path, weights2Path, weights3Path, biases1Path, biases2Path, biases3Path;  // paths to files containing weights and biases   
	std::string weights4Path, weights5Path, weights6Path, biases4Path, biases5Path, biases6Path; 
	bool verboseFile, verboseTerm;
	yarp::os::BufferedPort<yarp::os::Bottle > MotCom;     // output port to command the robot
	//yarp::os::BufferedPort<yarp::os::Bottle > MotComRX;     //ports for context information
	//yarp::os::BufferedPort<yarp::os::Bottle > MotComTX;
	yarp::os::BufferedPort<yarp::os::Bottle > Inp3D;      // input port to receive 3d information 
	yarp::os::BufferedPort<yarp::os::Bottle > Inpjoints;
	yarp::os::BufferedPort<yarp::os::Bottle > activationsPort;
	yarp::os::BufferedPort<JointsContext> MotComRX ;
	yarp::os::BufferedPort<JointsContext> MotComTX ;
	//PMPServer to Observer
	yarp::os::RpcServer PMPResponse; //server responding to Observer Client Port(/BodySchemaSim/io), with the result
	//and motor commands

	//a pointer to the KCL KinematicsThread
	darwin::reach::KinematicsAccess* _kinematicsAccess;
	yarp::os::Stamp _kinematicsStamp;
	int _kinematicsMovesStarted;

	//a bottle to contain the reply to the Observer
	yarp::os::Bottle bottleReq;
	darwin::msg::PMPResult ObsResp;
   	
	std::string name;           // rootname of all the ports opened by this thread

    	double Jan[6];
		double x_ini; 
		double y_ini;
		double z_ini; 
		double x_fin,y_fin,z_fin;
    	double Gam_Arr[20000],Gam_Arry[20000],Gam_Arrz[20000],q1[20000],q2[20000],q3[20000],q4[20000],q5[20000];
		int inputL,hiddenL1,hiddenL2,outputL;
		double w1[48][6],w2[55][48],w3[3][55],b1[48][1],b2[55][1],b3[3][1]; 
		double janini0,janini3,janini4,janini5,janini6,janini1,janini2, csi_dot;
		double q6[20000];
		double ang1,ang2,ang3,ang4,ang5,ang6,konst;
		std::vector<double> movingjointsTX,movingjointsRX;
		std::vector<double> movingjoints;
		double target[3];
		double X_pos[3],ffield[3],JoVel[6],X_posIniObRx[3],X_posIniObTx[3];
		double x_iniIC,y_iniIC,z_iniIC;
		double *ptr;
		double KFORCE,ITERATION,RAMP_KONSTANT,t_dur,KOMP_JANG,J2H,J3H;
		float s[5000];
		int ParamStiff;
		int ResPM;
		double AngUp[6];
		double VTGSIN[14], ColAvoidOld[6]; // Comment VM 27 Feb: this has to be updated if we are usign both robots and making a choice which one will move
        double x_fin1,y_fin1,z_fin1,x_fin2,y_fin2,z_fin2;
		double KXA,KXB,KYA,KYB,x_off1,z_off1,x_off2,z_off2,STARTini,FrameRef,XinitTarg,ZinitTarg,csi_dot1;
		int dividen,MSimExec,TrajType,TSEC;
		int Wr_Iorient;
		int Context;
		int robCmd;
		int AvoidTray;
		int MSFlag,NSet[2],NsetJ;
		double JStoreTx[10][6],JStoreRx[10][6];
		double YRobRx, YRobTx,CJA[12],CJATx[6], CJARx[6],wrioTx;
		double Gam_Arr1[5000],Gam_Arry1[5000],Gam_Arrz1[5000];
		double Gam_Arr2[5000],Gam_Arry2[5000],Gam_Arrz2[5000];
         FILE *Weight1,*Weight2,*Weight3,*bias1,*bias2,*bias3;
		 FILE *Weight4,*Weight5,*Weight6,*bias4,*bias5,*bias6;
		 bool flag_openfile;
		
public:
    /**
    * constructor default
    */
    PMPThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    PMPThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~PMPThread();

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

    /**
    *  on stopping of the thread
    */
    void onStop();

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
    * function that sets the inputPort name
    */
//    void setColorPath(std::string inp) { colorMapPath = inp; };

       /*
    * function that sets the model path
    */
  //  void setModelPath(std::string inp) { modelPath = inp; };

	//calls the KCL Kinematics to execute a reach command
	void handleKinematicsCommand(ReachCommand&);

	//set the KinematicsThread
	void setKinematicsAccess(darwin::reach::KinematicsAccess*);

	void writeReachResult(darwin::msg::ReachResult&,yarp::os::Stamp&);

	//function that calls the PMP to execute a reach command
	void handlePMPCommand(PMPCommand&);

    	void InitializeJan();

		void InitializeJanObst();

		void LoadANN();
		void CloseANN();

		void initializeANN(int ANNswitch);

		double* forward_Kinematics(double *u , int l);

		double* forward_KinematicsLRH(double *uLRH , int lefRH);

		double* forcefield(double *w, double*v);

		int VTGS(double XT1, double YT2, double ZT3,double XO1, double YO2, double ZO3,int ChoiceAct,int MentalSim, double WristGraspPose);

		double* PMPJack(double *force);
		
		double Gamma_Int(double *Gar,int n);

		double Gamma_IntDisc(double *Gar,int n);

		double GammaDisc(int _Time);

		double Gamma(int _Time);

		double Gamma1(int _Time1);

		void MotCon(double T1, double T2, double T3,int time, double Gam);

		void Kompliance(int tagK);
		
		void MessagePassR(int roboname);

		void VariableSpeedCtrl(int roboname);

        void initTX();

		void initRX();

		double ComputeTheta5(double Fx, double Fy,double Bx, double By);

		void IndReachCheck(int robcmd);
		void ReachCheck();

		int FindSmoothPath(std::vector<float>* PreviousPosition, std::vector<std::vector<float>>* NextPosition);

		void AngUpStore();

		void AngUpLoad();

		int sizz;

		void Interpret(int CCode,int PtCode,double AmplificationX,double AmplificationZ);

		void ReadCurrentJoints();
    
		void setRobotName(std::string robotname);
		 /*
    * functions that set path variables to corresponding paths of weight and bias files
    */
		void setWeightsPath(std::string s, int i);
		void setBiasesPath(std::string s, int i);
		void ChooseANN(double Thrish);
		void TrayAvoidanceON();
		bool detectCollision();
		void Paraleelism();
		void sendParalell();

		 /*
    * function that enables/disables the verbose terminal and file
		 */
		 void setVerbose(bool vFile, bool vTerm) {verboseFile = vFile; verboseTerm = vTerm;};

		 std::vector<LibRoboMove::Kinematics::vecType> getBodyConfig(std::vector<float> joints, int robot);

};

//----- end-of-file --- ( next line intentionally left blank ) ------------------

