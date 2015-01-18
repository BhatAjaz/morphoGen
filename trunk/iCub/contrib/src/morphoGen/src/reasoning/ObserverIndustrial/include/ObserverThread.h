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

#ifndef _OBSERVER_THREAD_H_
#define _OBSERVER_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <math.h>

#include "MessageFormats/VocabDefinitions.h"
#include "MessageFormats/DarwinMessages.h"
#include "EffectorAccess.h"
#include "AffordanceAccess.h"
#include "AttentionAccess.h"
#include "WorkspaceCalculations.h"

#define COMMAND_VOCAB_REQ          VOCAB3('R','E','Q') //with episodic mem module
#define COMMAND_VOCAB_ACK          VOCAB3('A','C','K')
//#define COMMAND_VOCAB_REACH        VOCAB3('R','E','A') //with body schema module
#define COMMAND_VOCAB_FIND         VOCAB4('F','I','N','D')  //with OPC module
#define COMMAND_VOCAB_GRAZP        VOCAB5('G','R','A','Z') //with KCL grasper

//shared vocabulary with user interface module
#define COMMAND_VOCAB_XPLOR          VOCAB4('X','P','L','R')
#define COMMAND_VOCAB_LEARNACT       VOCAB8('L','E','A','R','N','A','C','T')
#define COMMAND_VOCAB_LEARNOBJ       VOCAB8('L','E','A','R','N','O','B','J')
#define COMMAND_VOCAB_STACK          VOCAB5('S','T','A','C','K')
#define COMMAND_VOCAB_PUSH           VOCAB4('P','U','S','H')
#define COMMAND_VOCAB_PICK           VOCAB4('P','I','C','K')
#define COMMAND_VOCAB_WHERE          VOCAB5('W','H','E','R','E')
#define COMMAND_VOCAB_HELP           VOCAB4('H','E','L','P')

#define COMMAND_VOCAB_INIT   VOCAB4('I','N','I','T')
#define COMMAND_VOCAB_CACT   VOCAB4('C','A','C','T')
#define COMMAND_VOCAB_BCHA   VOCAB4('B','C','H','A')
#define COMMAND_VOCAB_MSIM   VOCAB4('M','S','I','M')
#define COMMAND_VOCAB_TRAT   VOCAB4('T','R','A','T')
#define COMMAND_VOCAB_GRIG   VOCAB4('G','R','I','G')
#define COMMAND_VOCAB_GLEF   VOCAB4('G','L','E','F')
#define COMMAND_VOCAB_OLEF   VOCAB4('O','L','E','F')
#define COMMAND_VOCAB_ORIG   VOCAB4('O','R','I','G')
#define COMMAND_VOCAB_WRIO   VOCAB4('W','R','I','O')
#define COMMAND_VOCAB_FAILED VOCAB4('F','A','I','L')
#define COMMAND_VOCAB_REA    VOCAB3('R','E','A')
#define COMMAND_VOCAB_OK     VOCAB2('O','K')

//enum RobotBodySide {
//	BODY_SIDE_LEFT,
//	BODY_SIDE_RIGHT,
//	BODY_WHOLE
//};

class ObserverThread : public yarp::os::Thread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

   	yarp::os::BufferedPort<yarp::os::Bottle > PlanEx;     // output port to command the robot
	yarp::os::BufferedPort<yarp::os::Bottle > bodyPlot,objectPlot,actionPlot,cwsPlot;		// output port to plot the hubs

	yarp::os::BufferedPort<yarp::os::Bottle > QueryEPIM;      // input port to receive 3d information
	yarp::os::BufferedPort<yarp::os::Bottle > AknowEpim;     // output port to command the robot

	yarp::os::Port Planxx;
	yarp::os::RpcClient EpimCtrlPort; //connection to the server : Episodic memory and learning
	yarp::os::RpcClient OPCCtrlPort; //connection to the server Object properties collector
	yarp::os::RpcClient BodySchemaCtrlPort; //connection to the server PMP.BS
	yarp::os::BufferedPort<darwin::msg::ReachCommand> kinematicsPort; //connection to the server PMP.kin
	yarp::os::BufferedPort<GraspCommand> GraspPort; //connection to the server PMP.BS
	yarp::os::BufferedPort<GraspResult> GraspResultPort; //connection to the GraspModule for the result
	yarp::os::RpcServer UserServices;  //Connects to external clients: User
	yarp::os::BufferedPort<ExpectedObjects> objport;
	yarp::os::BufferedPort<VisualScene> VScene;

	yarp::os::Stamp GraspTimeStamp;
	yarp::os::Stamp ReachTimeStamp;
	darwin::observer::EffectorAccess* effectorAccess;
	darwin::observer::AffordanceAccess* affordanceAccess;
	darwin::observer::AttentionAccess* attentionAccess;
	darwin::observer::WorkspaceCalculations* workspaceAccess;

	VisualScene *scene,*scene2,scene1;
	VisualObject fuseTX, fuseRX,fuseTXg, fuseRXg,currentHole;
	std::ofstream Report,HubA,MapA,WorABin,Present;
	std::string name;           // rootname of all the ports opened by this thread
    std::string pathPrefix,fileName;
	int KeySend;
	int state;                  //state that represents the action to perform
	int Strata[1000];
	int GiD; //this can be transformed into a shared vocabulary between user and observer
	double NumberofObs, NumberofObsE;
	int SeqAc[20], SeqAcP[20],SeqAcInterp[20],SeqAcInterpR[20],SeqAcNum[20],Encapsulate[20],numcu,numcy,nummu,PastPlan[10], Replan,numberpast;
	int SeqAcXl[2];
	int PickMicro, PlaceMicro, NPiCs,findsuccess,findsuccess2,PPiCo;
	double PlaceMap[10][18]; //col-shap-x-y-z-constraint
	double ObjIDEE[20], ObjIDEEEpim[20], XPosition[3];
	double NumObjectsinScene;
	int GetObjIDs[2], largeness,cannotfindXLoc,CannotFind,Cumulate,AlignFlag,AlignHandFlag;
	double PMPresp[12],cannotfindX,NumCubID[3],NumCylID[2],NumMushID[2],Graspability[10];
	double XlatTrackP[10],XlatTrackPl[10],StaticLoc[3];
	int PtOfReplan,PtOfReplann;
	double Col[30],Word[30],Shape[30],ProvHub[42],OCHub[42],LocalMapAct[90],ipCol[3],ipShap[3],RdisActW[2][30],BodyHub[42],ActionHub[12],ActionHubXplore[12],ComboCoordinate[6];
	int MapstoProvHub[36][90],ProvHubtoMaps[90][36],ColW[30][3], WorW[30][120],ShapW[30][3],BodyHub2Acn[42][12],ActH[12],NXploreAct;
	int WordIn[2][120],inhib[30][30],NumWords, WActPrim[12][120],WActionPrim[12][120];
	int WordAIn[120],Goal_AcID,HubID_EPIM, GContext;
	double Rdis,RdisAct[90],pr_Co,RdisS,RdisActS[90],pr_CoS,LoCAlign[3],LoCAlignFuseeD[10][3],FBoxPos2D[10][10],FuseePos2D[10][3],LoCAlignHand[3],LoCAlignFusee[3],FuseObst[3],FBObst[3],ViaPoint[3],ChosenFB[3];
    double WorActiv[2][30],pr_CoW,RdisW,RdistempW,MaxiAct,MaxiActW,MaxiActS,SerialHole[10][3];
	int GlobalWorkSpace[10][50],PlaceMapHub[10][42], BottomUPTrace[20][50],	Behavior[50][50],PosHandOcc[3],numfuseFBoxe, ProtoPlan[20][50], FuseBoXPos;
	int GoalPointer,pointRew,pointIntersect,ActionPointer,GoalStack[10][50],GoalStackPtr;
	int BodyHubBU[42],BodyHubTD[42],iterMicro,iterMicroN, TerminateFlag,NActs,RootGoalFlag,ActionChoice, RealFusee;
	int CMicroSub, FuseFilled[10], HoleFilled[10];
	int stateMM,FFindex1,FFindex2, FFindexRem, RemRemFuse, inDexPMPParalell;
	int MergePlans;
	int PlaceMapPos;
	int GWSPtr;
	int UGPush, NullObj, PushIntersect;
	int PeriPersonalF, PeriPersonalFB,HandOccupancy,PushOID,fuseBoxPos,numfusee, roboIndexTx,roboIndexRx;
	int ActionRecorder[20],ifXplore,psFlag, isFBinScene; //OnlineLR
	int Gres;
    int Protol;
	int RobotMovePMP,FusePointer,FusePointerTX,FusePointerRX,HolePointerRX,HolePointerTX;
	int locHole;
	int fusesRX[5][3],fusesTX[5][3],distan[20];
	FILE *WeightsRX,*WeightsTX;
	int FuseIndexRX, FuseIndexTX,invGrspInitFlag,JJSh, minObjectsCount;
	int activeHole, FuseOrient,holeCount;
	//Variables from OPC
	double Eighteens[10][18]; //face coordinates
	bool testfuse;
public:
    /**
    * constructor default
    */
    ObserverThread();

    /**
    * constructor
    * @param robotname name of the robot
    */
    ObserverThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~ObserverThread();

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

	//Allows access to current effector state
	void setEffectorAccess(darwin::observer::EffectorAccess*);

	//Allows access to AffordanceModule
	void setAffordanceAccess(darwin::observer::AffordanceAccess*);

	//Allows access to attentional part of the vision system
	void setAttentionAccess(darwin::observer::AttentionAccess*);

	//Allows access to calculations about workspace and objects
	void setWorkspaceAccess(darwin::observer::WorkspaceCalculations*);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

	void Interpret();  //plan interpreter

	double RefreshPlacemap(); // speak with vision

	double PrimBodySchema(int PMPGoalCode,int OIDinPM,int PIdentifier, int MsimFlag, int WristOrient, int TrajType);

	int PrimGrasp(darwin::msg::GraspTypeType GraspReq,int BodyChain);
	int PrimGraspIndustrial(darwin::msg::GraspTypeType GraspReq,int BodyChain);
	//int PrimGraspIndustrial(GraspTypeType GraspReq,int BodyChain);

	int PickandPlace(int pick, int place, int seqNumber);

	double PrimSearch(int obj1, int obj2, int goalidentity);

	double PrimPush(int ReachSide, int PushTarget);

	void Xlator();
	void ProtoLtoEpim();

	int ifObstacle();

	int ifObstacleMultiFuse();

	void ComputeViaPoint();

	void ChooseFuseBox();
	int VerifyFuseBox();

	void Mergence();

	double PrimBodySchemaIndustrial(int PMPGoalCode,int OIDinPM,int PIdentifier, int MsimFlag, int WristOrient, int TrajType);

	void InvXlator(int pi, int pl);

	int UserInterface(int GoalLearn) ;

	void InitializeSW();

	void WordEncode(int numu, int hubenc);

	int WordEncodeA(int numu, int hubenc);

	void GetLocalAct(int numW);

	void LoadGWSArgument();

	void Retroactivate(int PropWC);

	int RetroactivateBodyHub(int PropWB);

	int RetroactivateAcHub();

	void InitializeWorkingMemory(int GoalPointer);

	int MicroMonitor(int stateMicormonitor);

	void initBodyHub();

	int MaintainTrace(int PtRe);
	//void HubRep();

    // Function that sets path for files
    void setPath(std::string inP);

	bool Align(double& AlignF);

	double AlignHand();
	int ChooseFuse();
	bool SceneChanged(double threshold);
	void FuseAllocatorDyn();
	void FusePosession();
	// Function for neurally learnt space representation
	void PPSAdvisor(double target[][3],int length);
	int GetMaxProv();
	void RecursiveAlignment(int robCtrl);
	void GraspInitCombo();
	void InverseGraspInitCombo();
	void DoubleGraspCombo();
	void InsertReachCombo(int robCtrl);
	VisualObject findFuseInScene(VisualScene* scene, double x, double y,double threshold);
	VisualObject findFuseboxInScene( VisualScene* scene,double x, double y);
	int chooseFromSequence(ActionSequence sequence);
	void reorientAndInsert(double x, double y);
	int writeToKinematics(ReachCommand &kinreach);
	void askAffordance();
	void Restorer();
	/*
    * function that sets the inputPort name
    */
//    void setColorPath(std::string inp) { colorMapPath = inp; };

       /*
    * function that sets the model path
    */
  //  void setModelPath(std::string inp) { modelPath = inp; };


};

#endif  //_OBSERVER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

