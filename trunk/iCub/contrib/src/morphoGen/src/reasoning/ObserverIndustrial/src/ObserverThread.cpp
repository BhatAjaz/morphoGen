
#include <ObserverThread.h>
#include <cstring>
#include <string>
#include<time.h>
#include <math.h>
//#include <windows.h>
#include <iostream>
#include <fstream>
#include <vector>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

//using namespace darwin;
//using namespace observer;
using namespace darwin::msg;

ObserverThread::ObserverThread() {
  robot = "icub";
}

ObserverThread::ObserverThread(string _robot, string _configFile) : GraspTimeStamp() {
  robot = _robot;
  configFile = _configFile;
  GraspTimeStamp.update();
}

ObserverThread::~ObserverThread() {
  // do nothing
}

bool ObserverThread::threadInit() {

  if (!QueryEPIM.open(getName("/what-to-do:o").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }

  if (!PlanEx.open(getName("/Strategy:i").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }

  if (!EpimCtrlPort.open(getName("/EpimCtrl:io").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }

  if (!UserServices.open(getName("/UserServer:io").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }

  if (!OPCCtrlPort.open(getName("/SmallWorldsOPC:io").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }

  if (!BodySchemaCtrlPort.open(getName("/BodySchemaSim:io").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }

  if (!GraspPort.open(getName("/GraspCtrl:o").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }
  if (!GraspResultPort.open(getName("/GraspResult:i").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }
  GraspResultPort.setStrict(true);
  if (!bodyPlot.open(getName("/bodyPlot:o").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }

  if (!objectPlot.open(getName("/objectPlot:o").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }
  if (!actionPlot.open(getName("/actionPlot:o").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }
  if (!cwsPlot.open(getName("/cwsPlot:o").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }
  if (!objport.open(getName("/observer/expectedobjs:o").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }
  if (!VScene.open(getName("/observer/VisionScene:i").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }

	if (!kinematicsPort.open(getName("/observer/kinematicsPort:o").c_str())) {
    cout << ": unable to open port to send unmasked events "  << endl;
    return false;  // unable to open; let RFModule know so that it won't run
  }
  

  Network::connect("/observer/kinematicsPort:o", "/kinematics/reach:i");
  

  fileName = pathPrefix;
  fileName.append("Report.txt");
  Report.open(fileName.c_str());

  fileName = pathPrefix;
  fileName.append("HubA.txt");
  HubA.open(fileName.c_str());

  fileName = pathPrefix;
  fileName.append("MapA.txt");
  MapA.open(fileName.c_str());

  fileName = pathPrefix;
  fileName.append("AcInitActiv.txt");
  WorABin.open(fileName.c_str());

  fileName = pathPrefix;
  fileName.append("ThePresent.txt");
  Present.open(fileName.c_str());
  return true;
}

void ObserverThread::setName(string str) {
  this->name=str;
  printf("name: %s", name.c_str());
}


std::string ObserverThread::getName(const char* p) {
  string str(name);
  str.append(p);
  return str;
}

void ObserverThread::setPath(string inS) {
  pathPrefix	= inS.c_str();
}

void ObserverThread::setInputPortName(string InpPort) {

}

void ObserverThread::setEffectorAccess(darwin::observer::EffectorAccess* e) {
	effectorAccess = e;
}

void ObserverThread::setAffordanceAccess(darwin::observer::AffordanceAccess* a) {
	affordanceAccess = a;
}

void ObserverThread::setAttentionAccess(darwin::observer::AttentionAccess* a) {
	attentionAccess = a;
}

void ObserverThread::setWorkspaceAccess(darwin::observer::WorkspaceCalculations* w) {
	workspaceAccess = w;
}

void ObserverThread::askAffordance()
{
	//VisualScene *scene;

	//if (VScene.getInputCount())
	//{
	//	scene=VScene.read(true);

		if(scene->size())
		{
			IntVector filled;
			filled.clear();
			filled.add(1);
			filled.add(2);
			ActionSequence sequence = affordanceAccess->insert(GRASP_LEFT,(*scene)[3],(*scene)[0],0,filled);

			std::cout<<":::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
	//		std::cout<<":::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
	//		std::cout<<sequence.toString()<<std::endl;
	//		std::cout<<":::::::::::::::::::::::::::::::::::::::::::"<<std::endl;

	//		
	//		for(int i=0;i<sequence.size();i++)
	//		{
	//			if(sequence[i].subID() == Action::rchID::value)
	//			{
	//				if(sequence[i].rch().kin()[0].context() == (int) CONTEXT_INSERT)
	//				{
	//					std::cout<<":::::::::::::::::::::::::::"<<std::endl;
	//					std::cout<<"Found the insertion context in reach type"<<std::endl;
	//					std::cout<<"THe index is "<<i<<std::endl;
	//					std::cout<<":::::::::::::::::::::::::::"<<std::endl;
	//				}
	//			}
	//		}
	//		std::cout<<"Sequence size is "<<sequence.size()<<std::endl;

	//	//	for(int i=0;i<sequence.size();i++)
	//	//	{
	//	//		Action act = sequence[i];

	//	//		//ReachDescriptor des = act;

	//	//		if(sequence[i].subID() == Action::rchID::value)
	//	//		{
	//	//			PMPKINCommand command = act.rch();
	//	//			ReachCommand Reach = command.kin();
	//	//			
	//	//		}



	//	//}
	//}

	//}
		}
}



void ObserverThread::run()
{
  KeySend=0;
  int AckMesEp=0;
  //state = 1; //VM made state as 2 for checking with TX190L 14 feb 2013
  state = 0;

  while (isStopping() != true) {

    switch(state) {
    case 0 : {         //Now retrocatives the small world: with learnt proto language: Tallest stack is retained/integrated :)
	  int iCo;
      cout<<" Waiting for user goal " <<endl;
      Report<<" Waiting for user goal " <<endl;
      GContext=50;
      HubID_EPIM=50;
      GoalPointer=0;
      iterMicro=0;
      iterMicroN=0;
      TerminateFlag=50;
      RootGoalFlag=14;
      MergePlans=0;
	  ifXplore=0;
      InitializeSW();
	  Gres=-1;
	  Protol=1;
	  psFlag=0;
      UGPush=0;
	  isFBinScene=14;
	  RemRemFuse=0;
	  FuseBoXPos=-1;
	  RobotMovePMP=0;
	  inDexPMPParalell=14; //VM2011
	  
      int interfaceRes=UserInterface(14);
      // to test communication with prim body schema industrial
      //int PMPIT=MicroMonitor(0);
	  
      //==============================================
      if((interfaceRes==1)&&(Protol!=14))
      {
       //  RefreshPlacemap();
		//cout<<" From User goal to anticipated Neural Hub activations " <<endl;
        GetLocalAct(NumWords);
        int PropWeightC=1; // this can come as a result of the elimination/growth rule
        Retroactivate(PropWeightC);

	    RefreshPlacemap();
        cout<<"provH"<<ProvHub[0]<<endl;
        //cout<<ProvHub[20]<<endl;
        //RetroactivateBodyHub(1);  //not needed here will become active based on failiures
        cout<<" Initializing DARWIN Working memory " <<endl;
        InitializeWorkingMemory(GoalPointer);
        if(Goal_AcID==4)
        {
          cout<<"Enter the Object on which to Place " <<endl;
          LoadGWSArgument();
        }
        GContext=14;
        Replan=1400; //made as 140
        pointIntersect=0; // this helps thread/plan binding in time
        //////////////////////////////////////////////////////////////////////////////////////////
        // GWSPtr=0; // this is not needed
        //  PrimPush(0);
        //change
        if(Goal_AcID==3){
          UGPush=1;
        }
        //change
      }

      GiD=100;  //made this 100 to divert the loop

      PlaceMapPos = 0;
      // This is to close the loop without micromonitoring.........................
      //	PosHandOcc[0]=40;
      //	PosHandOcc[0]=5;
      //PosHandOcc[0]=-15;
      /*PlaceMapPos=0;
      PrimPush(-77,77);*/
      state=1; // was state 1, made to 0 for testing opc vision loop
      // This is to close the loop without micromonitoring.........................

      //cin >> GiD; //not distrubing this for the time being
      if(GiD==14){
        Replan=0;
        //RefreshPlacemap(); // need to Switch this on so as to initialize both the Global workspace and Placemap
        //	state=1; // commented this but u need to take actions: Very OK!
      }

      if(GiD==32)
      {
        NPiCs=1;
        Replan=0;
        for(iCo=0; iCo<10; iCo++)
        {
          SeqAcP[iCo]=50;
        }
        RefreshPlacemap(); // commented for testing 10-03-2014 ********************************
        for(iCo=0; iCo<NumberofObs; iCo++)
        {
          SeqAcP[iCo]=ObjIDEE[iCo];
          //cout<<"Oject id is "<<SeqAcP[iCo]<<endl;
          NPiCs=NPiCs+1;
        }
        state = 5;
      }

	  if (Protol==14)
	  {
	    state=0;
	  
	  }
             }
             break;


    case 1: {
      Network::connect("/EpimCtrl:io", "/strategy:io");
      for(int i=0;i<1000;i++)
      {
        Strata[i]= 0;
      }
      //cout << "Sending snapshot to EPIM "<< endl;
      Report << "Sending snapshot to EPIM "<< endl;

      // for tests with Xlate and Inv-Xlate// this should come from Refresh Place Map that issues OPC the microgoal to FORTH

      for(int i=0;i<10;i++)
      {
        ObjIDEE[i]=50; //null object
        XlatTrackP[i]=50;
        XlatTrackPl[i]=50;
      }
      if(Replan==0){
        /* NumberofObs=3;
        NumberofObsE=2*/;

      NumberofObsE=NumberofObs;
      largeness=0;
      //                  ObjIDEE[0]=5; //mush
      //				ObjIDEE[1]=0;
      //ObjIDEE[2]=3; //cyli

      for(int i=0;i<NumberofObs;i++)
      {
        XlatTrackP[i]=ObjIDEE[i];
        XlatTrackPl[i]=ObjIDEE[i];
      }
      //	Xlator(); //*******************************************************************************
      }

      if(Replan==1){
        cout<<"Micromonitoring Failed: Transimiting Body Hub activations to the Episodic memory"<<endl;
        NumberofObsE=42;
        largeness=0;
        GContext=50;
        HubID_EPIM=2;
      }

      if((Replan==140)||(Replan==1400)){
        cout<<"Transimiting Object Hub activations to the Episodic memory"<<endl;
        NumberofObsE=42;
        largeness=0;
        GContext=50;
		HubID_EPIM=1;
      } 

	  if(Replan==1400)
	  {
	     GContext=1400;
	  }
      //==========================================================================================
      Bottle cmd, response;
      cmd.addVocab(COMMAND_VOCAB_REQ);
      cmd.addInt(GContext);
      cmd.addInt(HubID_EPIM);
      cmd.addInt(Goal_AcID+1);
      //cout<<"Goal Context  "<<GContext << "with Hub Query  "<< HubID_EPIM << "and Action Hubactivation" <<Goal_AcID+1<< endl;

      cmd.addDouble(NumberofObsE);
      cmd.addInt(largeness);
      cmd.addInt(Replan);
      for(int i=0;i<NumberofObsE;i++)
      {
        if(Replan==0)
        {
          cmd.addDouble(ObjIDEEEpim[i]); //this should be replaced with ObjIDEE[i] coming from OPC server
        }
        if(Replan==1)
        {
          cmd.addDouble(BodyHub[i]);
        }
        if((Replan==140)||(Replan==1400))
        {
          cmd.addDouble(ProvHub[i]); 
        }
      }

      EpimCtrlPort.write(cmd,response);
      printf("%s \n",response.toString().c_str());

      int responsecode = response.get(0).asVocab();
      //cout<<responsecode<<endl;

      if(responsecode == 123 /*COMMAND_VOCAB_ACK*/) {
        fileName = pathPrefix;
        fileName.append("PXper.txt");
        ofstream PXper(fileName.c_str());
        //cout << "Receiving Plan from server" << endl;
        //Report <<"Receiving Plan from server" << endl;
        for(int i=0;i<1000;i++)
        {
          Strata[i]= response.get(i+1).asInt();
          PXper << Strata[i] <<endl;
        }
        pointRew=response.get(1001).asInt();
        //cout<<"Point of Chunk Termination is"<<pointRew<<endl;
        //cout << "Minimal energy Plan or MemoChunk recd sucessfully: Requesting EPIM to wait for next event" << endl;
        //Report << "Minimal energy Plan or MemoChunk recd sucessfully: Requesting EPIM to wait for next event" << endl;
		if(Replan==1400)
		{
		  Replan=0;
		  GContext=14;
		  NumberofObsE=NumberofObs;
		}
        state=6;
      }
      if(responsecode == 41 /*COMMAND_VOCAB_ACK*/) {
        cout << "No relevant past experience exists in the Neural Episodic memory: Triggering exploration" << endl;
        Report << "No relevant past experience exists in the Neural Episodic memory: Triggering exploration" << endl;
        state=10;
      }
            }
            break;

    case 2:
      {
        int iCo,jCo;
        int Pctrr=0;
        int AcSeq[20][50];
        for(iCo=0; iCo<6; iCo++){
          SeqAc[iCo]=50;
        }
        for(iCo=0; iCo<10; iCo++){
          SeqAcP[iCo]=50;
          //  PastPlan[iCo]=50;
        }
        //===================================================
        fileName = pathPrefix;
        fileName.append("PXper.txt");
        ifstream PlnW(fileName.c_str());
        if(!PlnW)
        { cout << "Error opening plan" << endl; }
        else
        { cout << "Loading plan" << endl;
        Report<< "Loading plan" << endl;}
        for (iCo=0; iCo<1000; iCo++)
        {
          PlnW >> Strata[iCo];
        }
        //==============Just for testing, this will be usally fed from EPIM=======
        for(iCo=0; iCo<20; iCo++)
        {
          for(jCo=0; jCo<50; jCo++)
          {
            AcSeq[iCo][jCo]=Strata[Pctrr];
            Pctrr=Pctrr+1;
          }
        }

        int NObjs=0;
        int ctr=1;
        for(iCo=0; iCo<20; iCo++)
        {
          if(AcSeq[iCo][42]==1)
          {
            NObjs=NObjs+1;
            for(jCo=0; jCo<36; jCo++)
            {
              if(AcSeq[iCo][jCo]==1)
              {
                SeqAc[ctr]=jCo;
                //cout << "found object" << SeqAc[ctr] << endl;
                ctr=ctr+1;
              }
            }
          }
        }
        SeqAc[0]=NObjs;
        //cout<< "there are " << SeqAc[0] << "object shapes to stack: Interpreting.." << endl;
        Interpret();  // Interprets plan  in shapes to plan in FORTH representations
        if(Replan==0){
          state = 5;
        }
        if(Replan==1){
          state = 4;
        }
      }
      break;

    case 4 : {
      //cout<<"Merging the incompletely executed past plan with the new one from EPIM..."<<endl;
      Report<<"Merging the incompletely executed past plan with the new one from EPIM..."<<endl;
      Mergence();
      //cout<<"Mergence synthesized a posisble plan"<<endl;
      Report<<"Mergence synthesized a posisble plan"<<endl;
      state=5;
             }
             break;

    case 11 : {
      cout<<"Exploring possibilities with Objects: Sending Object Hub activations to EPIM"<<endl;
      Report<<"Exploring possibilities with Objects: Sending Object Hub activations to EPIM"<<endl;
      Replan=140;
      OCHub[32]=1;
      state=1;
      UGPush=0;
      PushIntersect=14;
      //this seems to work:Now u need to Micromonitor/Execute this properly....
              }
              break;

    case 5:
      {
        int iCo,jCo;
        int StakSuc=0;
        if(Replan==1){
          Replan=0; //when ever it enters pick and place loop there are chances of failing and replanning
          //so this is implemented such that the system calls itself recurrsively ..... VM1203
          cout<<"Building up on a past plan that was not sucessfull: Trying Again...."<<endl;
          Report<<"Building up on a past plan that was not sucessfull: Trying Again...."<<endl;
        }
        PtOfReplan=0;//***************************************************


        // for(iCo=0; iCo<NPiCs-1; iCo++)
        //{
        //	cout<< "Proposed Plan is as follows:" << endl;
        //	cout << "Picking : " << SeqAcP[iCo+1]  << " and placing it on : " << SeqAcP[iCo] << endl;
        //}
        //
        // Time::delay(20);


        for(iCo=0; iCo<NPiCs-1; iCo++)
        {
          int pickk=50;
          int Placc=50;
          if(GiD==14){
            //cout << "Picking" << SeqAcP[iCo+1]  << "and placing it on" << SeqAcP[iCo] << endl;
            Report<< "Picking" << SeqAcP[iCo+1]  << "and placing it on" << SeqAcP[iCo] << endl;
            pickk=SeqAcP[iCo+1];
            Placc=SeqAcP[iCo];
          }
          if(GiD==32){
            //cout << "Transporting Object " << SeqAcP[iCo] << "in the box" << endl;
            Report << "Transporting Object " << SeqAcP[iCo] << "in the box" << endl;
            pickk=SeqAcP[iCo];
            Placc=SeqAcP[iCo+1];
          }

          int picks = PickandPlace(pickk,Placc,iCo); //**************************************
          //   int picks=1; // loop break just for testing //***********************************
          //cout<<"PICKS Result"<< picks<<endl;
          if(picks==4) //VM Made change here 23/03
          {
            //cout << "PMP target Unreachable or Object slippage: Attention:" << endl;
            Report << "PMP target Unreachable or Object slippage: Attention:" << endl;
            SeqAcP[iCo+1]=SeqAcP[iCo];
          }

          if(picks==0)
          {
            //cout << "Sense failiures: Attention" << endl;
            Report << "Sense failiures: Attention" << endl;
            PtOfReplan=iCo;
            //cout << "Failure sensed when picking" <<SeqAcP[PtOfReplan+1] << "and placing it on"<< SeqAcP[PtOfReplan]<< endl;
            Report << "Failure sensed when picking" <<SeqAcP[PtOfReplan+1] << "and placing it on"<< SeqAcP[PtOfReplan]<< endl;
            Replan=1;; //to exit loop
            iCo=NPiCs;
            //state=3; // go back to refresh and contact reasoning
          }
          if(picks==1){
            StakSuc=StakSuc+picks;
            //cout << "Seems fine: Going ahead with the next micro sequence !!" << endl;
            Report << "Seems fine: Going ahead with the next micro sequence !!" << endl;
          }
        }
        //MAY BE WE MUST TAKE A SNAPSHOT AND MEASURE THE TOP MOST POINT: IN CASE OF STACK DESTRUCTION
        //cout << "Finished the Goal!! Anticipated reward from past experience is" << StakSuc+1 <<endl;
        Report << "Finished the Goal!! Anticipated reward from past experience is" << StakSuc+1 <<endl;
        //have to read the right value from Strata...
        //Time::delay(60);
        if(Replan==1){
          state =3;
        }
        if(Replan==0){
          state = 0;
        }

      }

      break;

    case 3 : {
      //cout<<" Sensed Unexpected hurdles: Monitoring where the past plan failed and contacting EPIM " <<endl;
      Report<<" Sensed Unexpected hurdles: Monitoring where the past plan failed and contacting EPIM " <<endl;
      int iCo,jCo;
      for(iCo=0; iCo<10; iCo++){
        //  SeqAcP[iCo]=50;
        PastPlan[iCo]=50;
      }
      numberpast=0;
      //	findsuccess=0; //********************Need to uncomment this********VM1203
      if((findsuccess==0)||(CannotFind==1)) //or grasp slippage or PMP fialiure
      {
        numberpast=PtOfReplan;
        for(iCo=0; iCo<PtOfReplan+1; iCo++){
          PastPlan[iCo]=SeqAcP[iCo];
          //cout<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
          Report<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
          PPiCo=PastPlan[iCo];
        }
      }
      if((findsuccess2==0)&&(findsuccess!=0)) //failed during palceing operations
      {
        numberpast=PtOfReplan-1;
        for(iCo=0; iCo<PtOfReplan; iCo++){
          PastPlan[iCo]=SeqAcP[iCo];
          //cout<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
          Report<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
        }
      }
      //cout<< "Peceiving the messed up situation and contacting Reasoning" <<PastPlan[numberpast] << endl;

      // A time delay may be needed here
      //Time::delay(12);
      RefreshPlacemap(); //****** Need to Uncomment this for real testing: VM1203
      //***************************************************************************

      //***************************************************************************
      state=1;
             }
             break;

             //====================Decoding Minimal action Plan and Micromonitoring
    case 6:
      {
        int iCo,jCo;
        int Pctrr=0;
        int AcSeqe[20][50];
        if((RootGoalFlag==14)||(Replan==1)) // there is a need to interpret
        {
          for(iCo=0; iCo<20; iCo++){
            SeqAcInterp[iCo]=50;
          }
          for(iCo=0; iCo<20; iCo++){
            SeqAcNum[iCo]=50;
            //  PastPlan[iCo]=50;
          }

          for(iCo=0; iCo<20; iCo++)
          {
            for(jCo=0; jCo<50; jCo++)
            {
              AcSeqe[iCo][jCo]=Strata[Pctrr];
              Pctrr=Pctrr+1;
            }
          }

          NActs=0;
          int ctr=1;
          for(iCo=0; iCo<pointRew; iCo++)// this can be limited to the chunk that is determined by the point of intersection and min energy of the returned plan
          {
            if((AcSeqe[iCo][43]==1)&&(AcSeqe[iCo][49]==0)) //not the goal representation but the action that leads to the goal
            {
              NActs=NActs+1;
              for(jCo=0; jCo<12; jCo++)
              {
                if(AcSeqe[iCo][jCo]==1)// this may be an analog number based on the rememberd memory activity
                {
                  SeqAcInterp[ctr]=jCo;
                  Encapsulate[ctr]=0;
                  SeqAcNum[ctr]=iCo;
                  //NOTE: CHANGED SeqAcNum[iCo] to SeqAcNum[ctr] everywhere..this is to find the correct seq no in case of encapsulation: Replanning seems to work still
                  //cout << "found micro action::" << SeqAcInterp[ctr] << "micro sequnce::"<<SeqAcNum[ctr] << endl;
                  ctr=ctr+1;
                }
              }
            }
            if((AcSeqe[iCo][49]==0)&&(AcSeqe[iCo][48]==1)&&(AcSeqe[iCo][46]==0)) //an encalpulated Goal
            {
              NActs=NActs+1;
              cout << "An encapusalted Goal::" << endl;
              for(jCo=0; jCo<12; jCo++)
              {
                if(AcSeqe[iCo][jCo]==1)// this may be an analog number based on the rememberd memory activity
                {
                  SeqAcInterp[ctr]=jCo;
                  SeqAcNum[ctr]=iCo;
                  Encapsulate[ctr]=1;
                  //cout << "found micro goal::" << SeqAcInterp[ctr] << "micro sequnce::"<<SeqAcNum[ctr] << endl;
                  ctr=ctr+1;
                }
              }

            }
          }
          SeqAcInterp[0]=NActs;
          Encapsulate[0]=NActs;
        }//State7Flag
        int PastplanCtr=0;
        if(Replan==1)
        {
          pointIntersect=PtOfReplann+1;
          //cout<<"Intersecting element is "<<pointIntersect << endl;
        }
        if(MergePlans==14)
        {
          pointIntersect=iterMicro-2;
          //cout<<"Intersecting element is "<<pointIntersect << endl;
        }

        //cout<< "there are " << SeqAcInterp[0] << "Micro sequences leading to goal:" << Goal_AcID+1 << "Interpreting.." << endl;
        for(iCo=pointIntersect; iCo<SeqAcInterp[0]; iCo++)
        {
          if(Encapsulate[iCo+1]==0)
          {
            CMicroSub=MicroMonitor(SeqAcInterp[iCo+1]);  //Am commenting Micromonitor to test encapsulation
            //CMicroSub=50; //Commented out to test online learning
            if((CMicroSub==1)&&(Gres!=0)){
              //cout<<"Top Down and Bottom up activations Do not resonate: retriggereing reasoning/Xploration"<<endl;
              PtOfReplann=SeqAcNum[iCo+1];
              ActionPointer=SeqAcInterp[iCo+1];
              int lastelem=MaintainTrace(PtOfReplann);
              iterMicro=lastelem+1; // this is the place for chunking experience from another memory, iterMicro-1 is the frame of partial cue
              //going back to epim:if no plan recd..u cna retroactivate body hub to Autonomously explore through OPC: this must be a new state
              break;
            }
			if((CMicroSub==1)&&(Gres==0)){
				 //cout<<"Grasping seems to have failed:Need Help"<<endl;
				  //cout<<"::::::Reinitilizing the system::::::::::"<<endl;
				//state=0;
				  Gres=0;
				break;
			}
            if(CMicroSub==50){
              cout<<"Top Down and Bottom up activations resonate: moving to next microsequnce"<<endl;
            }
          }
          if(Encapsulate[iCo+1]==1)
          {
            cout<<"Encapsulated Micro Goal:::::"<< SeqAcInterp[iCo+1] << "in micro sequnce::"<<SeqAcNum[iCo+1] << endl;
            RootGoalFlag=14;
            GoalStack[GoalPointer][0]=Goal_AcID;
            GoalStack[GoalPointer][1]=NActs;
            GoalStack[GoalPointer][2]=iCo+1; //point of encountering an encapsulated goal
            int pCo;
            for(pCo=0; pCo<NActs; pCo++)
            {
              GoalStack[GoalPointer][pCo+3]=SeqAcInterp[pCo+1];
            }
            for(pCo=0; pCo<NActs; pCo++)
            {
              GoalStack[GoalPointer][pCo+3+NActs]=Encapsulate[pCo+1];
            } // I stored the intrepreted plan that was being executed: Now transfering control to the encapsulated goal
            GoalPointer=GoalPointer+1;
            Goal_AcID=SeqAcInterp[iCo+1];
            GContext=14;
            Replan=0;
            pointIntersect=0;
            cout<<" Initializing DARWIN Working memory for new Subgoal " <<endl;
            // InitializeWorkingMemory(GoalPointer);
            //THIS MAY NOT BE NEEDED AS WORKIGN MEMORY IS INITIALIZED THROUGH USER INTERFACE: Comment on 14th Jan
            CMicroSub=14;
            HubID_EPIM=50;
            break;
          }
        }
        if(CMicroSub==50){
          /*if(TerminateFlag==50)
          {
          state = 0;
          }*/
          if(TerminateFlag==1)
          {
            cout<<" MicroGoal with ID::::: " << GoalPointer << "is terminated" <<endl;
            if(GoalPointer==0){
              state = 0;
			  					 if(ifXplore==1)
									   {
										 cout<<" Computing self reward based on energy of action plan::::: " << iterMicroN/2 <<endl;
										 int reWWSelf=iterMicroN/2;
										     Behavior[iterMicroN][reWWSelf+2]=1;
											 Behavior[iterMicroN][44]=1;
												 for(iCo=0; iCo<50; iCo++)
													 {
													 Present<< Behavior[iterMicroN][iCo]<< "    ";
													 }
													 Present << "    " << endl; 
											  iterMicroN=iterMicroN+1;
											  //cout<<" Recording user Goal as context " <<endl;
											  Behavior[iterMicroN][48]=1;
											  Behavior[iterMicroN][49]=1;
											  Behavior[iterMicroN][Goal_AcID]=1;
											  Behavior[iterMicroN][Goal_AcID+12]=1;
											  Behavior[iterMicroN][Goal_AcID+24]=1;
											  for(iCo=0; iCo<50; iCo++)
													 {
													 Present<< Behavior[iterMicroN][iCo]<< "    ";
													 }
													 Present << "    " << endl; 
                                             iterMicroN=iterMicroN+1;
											 Behavior[iterMicroN][47]=1;
											 Behavior[iterMicroN][49]=1;
											 Behavior[iterMicroN][38]=1;
											 Behavior[iterMicroN][39]=1;
											 Behavior[iterMicroN][40]=1;
											 Behavior[iterMicroN][41]=1;
											 for(iCo=0; iCo<50; iCo++)
													 {
													 Present<< Behavior[iterMicroN][iCo]<< "    ";
													 }
													 Present << "    " << endl; 
                                             iterMicroN=iterMicroN+1;
											  cout<<" Run EPIM weight update based on new Episodic memroy: The Present " << iterMicroN/2 <<endl;
									   }
            }
            if(GoalPointer!=0){
              state = 7;
            }
          }
        }
        if((CMicroSub==1)&&(Gres!=0)){
          state = 1;
          Replan=1;
        }
		if((CMicroSub==1)&&(Gres==0))
		{
		state=0;
		}
        if((CMicroSub==1)&&(UGPush==1)&&(NullObj==1)){
          state = 11;
          // Replan=122;
        }
        //change
        if(CMicroSub==14){
          state = 1;
          //Replan=1;
        }
        /*    Interpret();  // Interprets plan  in shapes to plan in FORTH representations
        if(Replan==0){
        state = 5;
        }
        if(Replan==1){
        state = 4;
        }*/

      }
      break;


    case 7 : {
      cout<<" MicroGoal accomplished, Going back to root goal ID:::::"<<GoalStack[GoalPointer-1][0] << endl;
      GoalPointer=GoalPointer-1;
      Goal_AcID= GoalStack[GoalPointer][0];
      NActs=GoalStack[GoalPointer][1];
      pointIntersect= GoalStack[GoalPointer][2]; //point of encountering an encapsulated goal
      if(PushIntersect==14)
      {
        GoalStack[0][2]=0;
      }
      int pCo;
      for(pCo=0; pCo<NActs; pCo++)
      {
        SeqAcInterp[pCo+1]=GoalStack[GoalPointer][pCo+3];
      }
      for(pCo=0; pCo<NActs; pCo++)
      {
        Encapsulate[pCo+1]=GoalStack[GoalPointer][pCo+3+NActs];
      } // I stored the intrepreted plan that was being executed: Now transfering control to the encapsulated goal
      SeqAcInterp[0]=NActs;
      Encapsulate[0]=NActs;
      Replan=0;
      RootGoalFlag=23;
      state=6;//Going back to mincromonitor
             }
             break;

    case 10 : {

			if(Replan==1400)
			{
	             std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
				 std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
		         std::cout<<"ROOT GOAL IN PROGRESS: NO ACTIONS EXECUTED YET"<<std::endl;
				 std::cout<<"No past experience also recalled from EPIM"<<std::endl;
				 std::cout<<"Reinitalizing: To Instruct new assembly goal, Type NewAsm"<<std::endl;
				 std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
				 std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
				 std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
	             state=0;
			}
			else{
      cout<<" Combining past experience with Explorative actions/User guidance" << endl;
      int iCo,jCo;
	  ifXplore=1;
      MergePlans=0;
      NXploreAct=0;
      //   PtOfReplann=SeqAcNum[iCo+1];
      // ActionPointer=SeqAcInterp[iCo+1];  //U already get action pointer becasue u go through Micromonitoring
      int ExplorePt=MaintainTrace(PtOfReplann);
      iterMicro=ExplorePt+1;
      cout<<" Retroactivating Body Hub-Action Hub" << endl;
      int ActPlay=RetroactivateBodyHub(0);
      int ActionPresent =0;
      while(NXploreAct !=0){
        if(ActPlay!=50)
        {
          int PMicroSub=MicroMonitor(ActionHubXplore[ActionPresent]);
          if(PMicroSub==50){
            cout<<"Merging the explorative action with the Previous plan"<<endl;
            NXploreAct=0;
            if(ActionHubXplore[ActionPresent]==6){
              state=0;
            }
            if(ActionHubXplore[ActionPresent]!=6){  //tThe robot did not choose to terminate the goal but tried soemthing else that succeded
              RootGoalFlag=23;
              MergePlans=14;
              Replan=0;
              state=6;
              for(jCo=0; jCo<20; jCo++)
              {
                SeqAcInterp[jCo]=SeqAcInterpR[jCo];
              }
            }
          }

          if(PMicroSub==1){
            cout<<"This did not work out: Exploring again"<<endl;
            // ActionHubXplore[ActPlay]=1;
            ActionPresent=ActionPresent+1;
            NXploreAct=NXploreAct-1;
            // ActPlay=RetroactivateBodyHub(0);
          }
        }
      }
			}
      break;

              }

              //Time::delay(5);
    }
  }
}


void ObserverThread::threadRelease() {
  // nothing
  EpimCtrlPort.close();
  WorABin.close();
  HubA.close();
  MapA.close();

}

int ObserverThread::MaintainTrace(int PtRe)
{
  int iCo,jCo,Pctrr=0;
  for(iCo=iterMicro; iCo<PtRe+1; iCo++)
  {
    for(jCo=0; jCo<50; jCo++)
    {
      BottomUPTrace[iCo][jCo]=Strata[Pctrr];
      Pctrr=Pctrr+1;
    }
  }

  for(jCo=0; jCo<42; jCo++)
  {
    BottomUPTrace[PtRe+1][jCo]=BodyHub[jCo];
  }

  for(jCo=0; jCo<20; jCo++)
  {
    SeqAcInterpR[jCo]=SeqAcInterp[jCo];
  }

  return PtRe+1;
};

void ObserverThread::InsertReachCombo(int robCtrl)
 {
	 int kkPt=numfuseFBoxe*holeCount;
	 invGrspInitFlag=1;
	// if(robCtrl==14){
	          if(RobotMovePMP==1)   // who gets to insert
						  {
							RobotMovePMP=-1;
						  }
						  else if((RobotMovePMP==-1))
						  {
						   RobotMovePMP=1;
						  }				
	  if(RobotMovePMP==-1)  //Tx is watching RX is insertingo
		  {
	      	  ComboCoordinate[0]=SerialHole[HolePointerRX][0];
			  ComboCoordinate[1]=SerialHole[HolePointerRX][1];
			  ComboCoordinate[2]=150;//SerialHole[HolePointerRX][2]
			  //Hole coords for RX

			  ComboCoordinate[3]=FuseePos2D[numfusee-FusePointerTX-1][0];
			  ComboCoordinate[4]=FuseePos2D[numfusee-FusePointerTX-1][1];
			  ComboCoordinate[5]=FuseePos2D[numfusee-FusePointerTX-1][2];
			  FuseFilled[numfusee-FusePointerTX-1]=-1;
			  HoleFilled[HolePointerRX]=-1;
			  //Fuse coordinates for TX
		      HolePointerRX=HolePointerRX+1;
			  FusePointerTX=FusePointerTX+1;
			  int rres=PrimBodySchemaIndustrial(122,0,1,1,0,0); //if no collision Tx reaches its fusee..RX inserts
																//if collision Rx is with its fuse watiing for insertion, TX reaches its fus, making it 140 now	
			 
			  GraspInitCombo(); // TX picks up the fuse, goes to initialize, rx reaches its hole

			  RobotMovePMP=-1;
			  LoCAlign[0]= ComboCoordinate[0];
			  LoCAlign[1]= ComboCoordinate[1];
			  LoCAlign[2]=SerialHole[HolePointerRX][2]+60;
			 // int alex2=PrimBodySchemaIndustrial(111,0,0,1,0,0);
			 // Insert RX
              reorientAndInsert(LoCAlign[0],LoCAlign[1]); //////uses affordance n kinm
			  PrimGraspIndustrial(GRASP_RELEASE,1); // Hardcoded for RX 6o release 
			  Time::delay(0.1);
			  //Relese RX
						
			  LoCAlign[2]=120+LoCAlign[2];  //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................MUST BE pOShANDOCC
			  int alex2=PrimBodySchemaIndustrial(95,0,0,1,0,0);
						
			  // Go Up RX
			  Time::delay(0.1);
			  LoCAlign[1]=-250;
			  LoCAlign[2]=350;		//set to go up
			  rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
			  Time::delay(0.1);
			  //Go side RX

	      }

	  	if(RobotMovePMP==1)  //Rx is watching TX is insertingo
		  {
	      	  ComboCoordinate[0]=FuseePos2D[FusePointerRX][0];
			  ComboCoordinate[1]=FuseePos2D[FusePointerRX][1];
			  ComboCoordinate[2]=FuseePos2D[FusePointerRX][2];
			  ComboCoordinate[3]=SerialHole[kkPt-HolePointerTX-1][0];
			  ComboCoordinate[4]=SerialHole[kkPt-HolePointerTX-1][1];
			  ComboCoordinate[5]=150; //SerialHole[kkPt-HolePointerTX-1][2]
			  FuseFilled[FusePointerRX]=-1;
			  HoleFilled[kkPt-HolePointerTX-1]=-1;
		      HolePointerTX=HolePointerTX+1;
			  FusePointerRX=FusePointerRX+1;
			  int rres=PrimBodySchemaIndustrial(122,0,1,1,0,0);//Tx waiting for insertion, RX  to reach

			  InverseGraspInitCombo();
			  if(invGrspInitFlag!=14){
			  RobotMovePMP=1;
			  LoCAlign[0]= ComboCoordinate[3];
			  LoCAlign[1]= ComboCoordinate[4];
			  LoCAlign[2]=SerialHole[kkPt-HolePointerTX-1][2]+60;
			//  int alex2=PrimBodySchemaIndustrial(111,0,0,1,0,0);
			 // Insert RX
              reorientAndInsert(LoCAlign[0],LoCAlign[1]); //////uses affordance n kinm
			  PrimGraspIndustrial(GRASP_RELEASE,0); // Hardcoded for RX 6o release 
			  Time::delay(0.1);
			  //Relese RX
						
			  LoCAlign[2]=120+LoCAlign[2];  //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................MUST BE pOShANDOCC
			  int alex2=PrimBodySchemaIndustrial(95,0,0,1,0,0);
						
			  // Go Up RX

			  //put the if loop for colavoid
			  Time::delay(3);
			  LoCAlign[1]=400;
			  LoCAlign[2]=350;		//set to go up
			  rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
			  Time::delay(0.1);
			  } 
	      }
	 
	 
	 };
 //};


void ObserverThread::RecursiveAlignment(int robCtrl)
{
		if(robCtrl==14)	{
		                  locHole=0; 
						 if(RobotMovePMP==1)
						  {
							RobotMovePMP=-1;
						  }
						  else if((RobotMovePMP==-1))
						  {
						   RobotMovePMP=1;
						  }						 
						 
						PlaceMapPos=FuseBoXPos;

						if(RobotMovePMP==1)
						  {
							  if(PlaceMap[PlaceMapPos][4] > PlaceMap[PlaceMapPos][7])
								locHole= 3;
							  else
								  locHole= 6;
						  }
						  else if((RobotMovePMP==-1))
						  {
						   if(PlaceMap[PlaceMapPos][4] > PlaceMap[PlaceMapPos][7])
								locHole= 6;
						   else
							   locHole= 3;
						  }
						LoCAlign[0]=PlaceMap[PlaceMapPos][locHole+0]+8;
						LoCAlign[1]=PlaceMap[PlaceMapPos][locHole+1];
						LoCAlign[2]=150;		//set to go up
						int rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
						Time::delay(0.1);

						LoCAlign[2]=PlaceMap[PlaceMapPos][locHole+2]+110;  //was 75                       //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................AS 50 FOR INSERT
						int alex2=PrimBodySchemaIndustrial(111,0,0,1,0,0);
						LoCAlign[2]=PlaceMap[PlaceMapPos][locHole+2]+60;  //was 75                       //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................AS 50 FOR INSERT
						//alex2=PrimBodySchemaIndustrial(111,0,0,1,0,0);
                        //PrimGraspIndustrial(GRASP_RELEASE,0); //hardcoded for TX
						reorientAndInsert(LoCAlign[0],LoCAlign[1]); //////uses affordance n kinm
						PrimGraspIndustrial(GRASP_RELEASE,1); // Hardcoded for RX
						Time::delay(0.1);
						LoCAlign[2]=120+LoCAlign[2];  //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................MUST BE pOShANDOCC
						alex2=PrimBodySchemaIndustrial(95,0,0,1,0,0);
						Time::delay(0.1);
						LoCAlign[0]=PlaceMap[PlaceMapPos][locHole+0];
						LoCAlign[1]=PlaceMap[PlaceMapPos][locHole+1]-150;
						LoCAlign[2]=350;		//set to go up
						rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
						Time::delay(0.1);

						//PrimBodySchemaIndustrial(19,0,1,1,0,0);  //comVM2910
						inDexPMPParalell=23;
					}
	if(robCtrl==23)	{
						 if(FuseePos2D[FFindexRem][1]<85)
						  {
							RobotMovePMP=-1;
						  }
						  else 
						  {
						   RobotMovePMP=1;
						  }

						LoCAlign[0]=FuseePos2D[FFindexRem][0];
						LoCAlign[1]=FuseePos2D[FFindexRem][1];
						LoCAlign[2]=FuseePos2D[FFindexRem][2]+60;		
						int rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
						Time::delay(0.2);
						LoCAlign[0]=FuseePos2D[FFindexRem][0];
						LoCAlign[1]=FuseePos2D[FFindexRem][1];
						LoCAlign[2]=FuseePos2D[FFindexRem][2];		// made change as 36
						rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
						Time::delay(0.2);
						if(RobotMovePMP==1){
						int Gres = PrimGraspIndustrial(GRASP_PINCH,0);  //TX
						}else{
						Gres = PrimGraspIndustrial(GRASP_PINCH,1);  
						}
								 ViaPoint[0]=FuseePos2D[FFindexRem][0];
								 ViaPoint[1]=FuseePos2D[FFindexRem][1];
								 ViaPoint[2]=FuseePos2D[FFindexRem][2]+90;
								 XPosition[0]=FuseePos2D[FFindexRem][0];
								 XPosition[1]=FuseePos2D[FFindexRem][1];
								 XPosition[2]=FuseePos2D[FFindexRem][2];
							    PrimBodySchemaIndustrial(19,0,1,1,0,1);
								Time::delay(0.2);
                        
						PlaceMapPos=FuseBoXPos;

						if(locHole == 3)
						{
							locHole= 6;
						}
						else
						{						
							locHole= 3;
						}
						
						LoCAlign[0]=PlaceMap[PlaceMapPos][locHole+0]+8;
						LoCAlign[1]=PlaceMap[PlaceMapPos][locHole+1];
						LoCAlign[2]=PlaceMap[PlaceMapPos][locHole+2]+110;		
						rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
						Time::delay(0.1);
						LoCAlign[2]=PlaceMap[PlaceMapPos][locHole+2]+60;                 //     was 75   //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................
						//int alex2=PrimBodySchemaIndustrial(111,0,0,1,0,0);
						reorientAndInsert(LoCAlign[0],LoCAlign[1]); //////uses affordance n kinm
						if(RobotMovePMP==1){
						Gres=PrimGraspIndustrial(GRASP_RELEASE,0); //TX
						}else{
						Gres = PrimGraspIndustrial(GRASP_RELEASE,1);  
						}
                        // //hardcoded for TX
						//PrimGraspIndustrial(GRASP_RELEASE,1); // Hardcoded for RX
						Time::delay(0.1);
						LoCAlign[2]=120+LoCAlign[2];  //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................MUST BE pOShANDOCC
						int alex2=PrimBodySchemaIndustrial(95,0,0,1,0,0);
						PrimBodySchemaIndustrial(19,0,1,1,0,0);
						inDexPMPParalell=32;
					}

	
}

void ObserverThread::PPSAdvisor(double target[20][3], int length)

{	//==========================Neural representation of space topology and peripersonal spaces=======================//
	fileName = pathPrefix;
	fileName.append("W_RxN.txt");
	WeightsRX = fopen(fileName.c_str(),"r");

	fileName = pathPrefix;
	fileName.append("Weight_TX.txt");
	WeightsTX = fopen(fileName.c_str(),"r");

	if ((!WeightsRX) || (!WeightsTX)) 
	{
		std::cout << "error loading space topology files"<<std::endl;
	}
	else
	{
		const int neuronNumber = 478;
		int winnerRX[20]={0},winnerTX[20]={0};
		double WeigRX[neuronNumber][3],WeigTX[neuronNumber][3],RevRX[neuronNumber]={0.0},RevTX[neuronNumber]={0.0};
		double maxTX=0,maxRX=0,initmg;
		float s=0;		
		cout << "Loading WeightsRX" << endl;
		for (int m =0; m<neuronNumber; m++)
		{
			for (int n=0; n<3; n++)
			{
				fscanf(WeightsRX,"%f",&s);
				WeigRX[m][n] = s;
				//cout << WeigRX[m][n] <<"\t" << endl;
			}
		}

		cout << "Loading WeightsTX" << endl;
		for (int m =0; m<neuronNumber; m++)
		{
			for (int n=0; n<3; n++)
			{
				fscanf(WeightsTX,"%f",&s);
				WeigTX[m][n] = s;
				//cout << WeigTX[m][n] <<"\t" << endl;
			}
		}
		fclose(WeightsRX);
		fclose(WeightsTX);

		for (int t=0;t<length;t++)
		{
			//for RX
			maxRX=0;
			for (int m =0; m<neuronNumber; m++)
			{
				double dxx=WeigRX[m][0]-target[t][0];
				double dyy=WeigRX[m][1]-target[t][1];
				double dzz=WeigRX[m][2]-target[t][2];			
				double dist1 = sqrt(pow(dxx,2)+ pow(dyy,2)+pow(dzz,2));
				initmg =	(5.64)*exp(-(dist1)/100);
				if(initmg > maxRX)
				{
					maxRX = initmg;
					winnerRX[t] = m;
				}
			}
			/////////////////////////
			//for TX
			maxTX=0;
			for (int m =0; m<neuronNumber; m++)
			{
				double dxx=WeigTX[m][0]-target[t][0];
				double dyy=WeigTX[m][1]-target[t][1];
				double dzz=WeigTX[m][2]-target[t][2];			
				double dist1 = sqrt(pow(dxx,2)+ pow(dyy,2)+pow(dzz,2));
				initmg = (5.64)*exp(-(dist1)/100);
				if(initmg > maxTX)
				{
					maxTX = initmg;
					winnerTX[t] = m;
				}
			}

			/*if (maxRX>maxTX)
			{
			return -1;
			}
			else
			{
			return 1;
			}*/
		}
		//////////Calculating rewards RX
		double maxRewRX=0;
		int chosenObjIndexRX = 0;
		for (int t=0;t<length;t++)
		{
			RevRX[winnerRX[t]]=(1.0/150)*exp(-target[t][1]/100.0);
			
			if (RevRX[winnerRX[t]] > maxRewRX)
			{
				maxRewRX = RevRX[winnerRX[t]];
				chosenObjIndexRX = t;
			}
		}
		FuseIndexRX=chosenObjIndexRX;
		std::cout <<"chosenObjIndex for RX  " <<chosenObjIndexRX<<std::endl;

		//////////Calculating rewards TX
		double maxRewTX=0;
		int chosenObjIndexTX = 0;
		for (int t=0;t<length;t++)
		{
			RevTX[winnerTX[t]]=(1.0/150)*exp(target[t][1]/100.0);
			
			if (RevTX[winnerTX[t]] > maxRewTX)
			{
				maxRewTX = RevTX[winnerTX[t]];
				chosenObjIndexTX = t;
			}
		}
		FuseIndexTX=chosenObjIndexTX;
		std::cout <<"chosenObjIndex for TX  " <<chosenObjIndexTX<<std::endl;
	}
}

void ObserverThread::FusePosession()
{
	int tempX,tempY,tempZ;
	for (int i=0;i<numfusee;i++)
	{
		
		FuseFilled[i]=5;
	}
	for (int i=0;i<(numfuseFBoxe*holeCount);i++)
	{
				HoleFilled[i]=5;
	}

	for (int i=1;i<numfusee;i++)
	{
	 for (int j=0;j<numfusee-1;j++)
	 {
		
		if (FuseePos2D[j][1]>FuseePos2D[j+1][1])
		{
			tempY=FuseePos2D[j][1];
			tempX=FuseePos2D[j][0];
			tempZ=FuseePos2D[j][2];
			FuseePos2D[j][1]=FuseePos2D[j+1][1];
			FuseePos2D[j][0]=FuseePos2D[j+1][0];
			FuseePos2D[j][2]=FuseePos2D[j+1][2];
			FuseePos2D[j+1][1]=tempY;
			FuseePos2D[j+1][0]=tempX;
			FuseePos2D[j+1][2]=tempZ;
		}
	 }
	}

	for (int i=0;i<numfusee;i++)
	{
		cout << FuseePos2D[i][0]<< "\t"<<FuseePos2D[i][1]<<std::endl;
	}

	for (int i=1;i<(numfuseFBoxe*holeCount);i++)
	{
	 for (int j=0;j<(numfuseFBoxe*holeCount)-1;j++)
	 {
		 if(SerialHole[j][1]>SerialHole[j+1][1])
		 {
			tempY=SerialHole[j][1];
			SerialHole[j][1]=SerialHole[j+1][1];
			SerialHole[j+1][1]=tempY;

			tempX=SerialHole[j][0];                  
			SerialHole[j][0]=SerialHole[j+1][0];
			SerialHole[j+1][0]=tempX;

			tempZ=SerialHole[j][2];
			SerialHole[j][2]=SerialHole[j+1][2];
			SerialHole[j+1][2]=tempZ;

		 }
	 }
	
	}

	 for (int i=0;i<numfuseFBoxe*holeCount;i++)
	{
		cout << SerialHole[i][0]<< "\t"<<SerialHole[i][1]<<std::endl;
	}
}





void ObserverThread::FuseAllocatorDyn()
{
	double dist1, dist2, dist3, dxx,dyy, max=0;
	int i;
	if(RealFusee>2)
	{
	  dxx=FuseePos2D[0][0]-FuseePos2D[1][0];
	  dyy=FuseePos2D[0][1]-FuseePos2D[1][1];
	  dist1 = sqrt(pow(dxx,2)+ pow(dyy,2));
	
	  dxx=FuseePos2D[0][0]-FuseePos2D[2][0];
	  dyy=FuseePos2D[0][1]-FuseePos2D[2][1];
	  dist2 = sqrt(pow(dxx,2)+ pow(dyy,2));

	  dxx=FuseePos2D[1][0]-FuseePos2D[2][0];
	  dyy=FuseePos2D[1][1]-FuseePos2D[2][1];
	  dist3 = sqrt(pow(dxx,2)+ pow(dyy,2));

	  max=dist1;
	  FFindex1=0;
	  FFindex2=1;
	  FFindexRem=2;
	  if(dist2>max)
	  {
        max=dist2;
	    FFindex1=0;
	    FFindex2=2;
		FFindexRem=1;

	  }
	  if(dist3>max)
	  {
        max=dist3;
	    FFindex1=1;
	    FFindex2=2;
		FFindexRem=0;
	  }
	  RealFusee=RealFusee-2;
	  RemRemFuse=1;
	}

	if(RealFusee==2)
	{
	  FFindex1=0;
	  FFindex2=1;
      RealFusee=0; 
	}

	if((FuseePos2D[FFindex1][1]>FuseePos2D[FFindex2][1]))//&&(FuseePos2D[FFindex1][1]>20)
	{
	 roboIndexTx=FFindex1;
	 roboIndexRx=FFindex2;
	}
	if((FuseePos2D[FFindex1][1]<FuseePos2D[FFindex2][1])) //&&(FuseePos2D[FFindex2][1]<150)
	{
	 roboIndexTx=FFindex2;
	 roboIndexRx=FFindex1; 
	}
	
  };


int ObserverThread::MicroMonitor(int stateMicormonitor)
	{
		int iCo,jCo,ConsMicro,ActionPlotter[12];
		ConsMicro=0;
      	 stateMM=stateMicormonitor;
					 for(jCo=0; jCo<42; jCo++)
									{
									  BodyHub[jCo]=0;
									}
					  for(jCo=0; jCo<12; jCo++)
									{
									  ActionPlotter[jCo]=0;
									}
				 switch(stateMM)
					 {
						case 2:
							{
							   cout<<"Initialializing search primitive for Goal pointer::"<<  Goal_AcID << endl;
							   double Fres;
							   ActionPlotter[2]=1;
							   if((GoalStack[0][0]==0)||(GoalStack[0][0]==1)) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=0;
							   	   }
							   if(((GoalStack[0][0]==4)&&(GoalPointer==2))||((GoalStack[0][0]==9)&&(GoalPointer==3)))//the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=0;
							   	   }
							    if((GoalStack[0][0]==4)&&(GoalPointer==1)||((GoalStack[0][0]==9)&&(GoalPointer==2))) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=1;
							   	   }
								if((GoalStack[0][0]==3)&&(GoalPointer==1)&&(PushOID==5)) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=0;
							   	   }
								if((GoalStack[0][0]==3)&&(GoalPointer==4)) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=1;
							   	   }
								if((GoalStack[0][0]==3)&&(GoalPointer==3)) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=2;
							   	   }
								

								if(GoalStack[0][0]==9){ //this is for paraleel assembly
								
									if(GoalPointer==3)
									{
										//if(RealFusee>=2){      //VM2011  
										//FuseAllocatorDyn();		
										////inDexPMPParalell=14;
										//}
										//if(RealFusee==1)
										//{
										//  if(RemRemFuse==1)
										//  {
										//    inDexPMPParalell=14;
										//  }
										//  if(RemRemFuse==0)
										//  {
										//   inDexPMPParalell=32;
										//  }

										//}
										FusePosession();  // Just for security
										PPSAdvisor(FuseePos2D,numfusee);
										if(numfuseFBoxe>=1){ 
                                    
      									  Fres=0;
									  }
						           
									}

									if((GoalStack[0][0]==9)&&(GoalPointer==2))
									{
						               Fres=PrimSearch(0,0,95);
									  if(numfuseFBoxe>=1){ ///to be written to reach fuse box and alighn
                                    
      									  Fres=0;
									  }
									}
								
								
								}else   //this is for any other user goal
									{
								   Fres=PrimSearch(0,0,95); //Commented for testing 10/02
							 	}

			 
				 //----------------------------------------------------------------- THIS U NEED TO SEE WHILE REACHING AND INSERTIGN FUSE
    //            int searchLimit=0;
    //             while(Fres>0.1 && searchLimit<4)
    //               {
    //                Fres=PrimSearch(0,0,95);
    //                searchLimit++;
    //                  }
				////Changes June2014
				// if((GWSPtr==1)&&(Fres>0.1)&&(isFBinScene==1))
				// {
    //               PrimBodySchemaIndustrial(19,0,1,1,0,0); //Intialize as there was a FB, you are seraching for a FB, but there is none now
				//   searchLimit=0;
				//    while(Fres>0.1 && searchLimit<4)
    //               {
    //                Fres=PrimSearch(0,0,95);
    //                searchLimit++;
    //                  }
				//	PosHandOcc[0]=PlaceMap[PlaceMapPos][0];
				//	PosHandOcc[1]=PlaceMap[PlaceMapPos][1];
				//	PosHandOcc[2]=PlaceMap[PlaceMapPos][2];
				// } //if it still deosnot find,t he inference is that the user has remeoved the object

				// if(GWSPtr==1){
				//    PosHandOcc[0]=PlaceMap[PlaceMapPos][0];
				//	PosHandOcc[1]=PlaceMap[PlaceMapPos][1];
				//	PosHandOcc[2]=PlaceMap[PlaceMapPos][2];
				// }
				 //-----------------------------------------------------------------------------------
	
							   if(Fres<=0)
								   {
							         //cout<<"Analyzing anticipated consequnce in the body hub"<<endl;
									 BodyHub[18]=1;
									 BodyHub[19]=1;
									 BodyHub[23]=1;
									 ConsMicro=50;
								    }
							    if(Fres>0.1)
								   {
							         //cout<<"Analyzing anticipated consequnce in the body hub"<<endl;
									 initBodyHub();
									 BodyHub[20]=1;
									 BodyHub[21]=1;
									 BodyHub[22]=1;
									 ConsMicro=1;
									 NullObj=1;
								   }

							 }
		                break;

						case 0:
							{
							   cout<<"Initialializing Body Schema primitive for Goal pointer::"<<  Goal_AcID << endl;
							    ActionPlotter[0]=1;
								int rres=0;
								
						        if((GoalStack[0][0]==4)&&(GoalPointer==1)||((GoalStack[0][0]==9)&&(GoalPointer==2))) //the roor goal issued by the user in reference to the global work space
								   {
										if(PeriPersonalFB-PeriPersonalF!=0)
										 {
										     if(PosHandOcc[1]>0)
												 {
											       PrimPush(104,-77); 
												 }
											 if(PosHandOcc[1]<0)
												 {
											      PrimPush(-104,77); // towards Tx 
												 }
											//PrimBodySchema(1,0,1,0,0,1); //and then align and release
										 }
							   	   } 
								
								if((inDexPMPParalell==14)&&(GoalPointer==3)){ //this is for paraleel assembly; at present Tx starts from last
								
									ComboCoordinate[0]=FuseePos2D[0][0];
									ComboCoordinate[1]=FuseePos2D[0][1];
									ComboCoordinate[2]=FuseePos2D[0][2];
									ComboCoordinate[3]=FuseePos2D[numfusee-1][0];
									ComboCoordinate[4]=FuseePos2D[numfusee-1][1];
									ComboCoordinate[5]=FuseePos2D[numfusee-1][2];
									FuseFilled[0]=-1;
									FuseFilled[numfusee-1]=-1;
									rres=PrimBodySchemaIndustrial(122,0,1,1,0,0);
									FusePointer=1;
									FusePointerTX=1;
									FusePointerRX=1;  // this is paraleel reach
									}

								else if ((GoalStack[0][0]==9)&&(GoalPointer==2))
									{
										PlaceMapPos=FuseBoXPos;  //TX goes to insert
										RobotMovePMP=1;
										int kkPt=numfuseFBoxe*holeCount;
										LoCAlign[0]=SerialHole[kkPt-1][0];  
										LoCAlign[1]=SerialHole[kkPt-1][1];  ;
										LoCAlign[2]=150;  
										HoleFilled[kkPt-1]=-1;
						                /*LoCAlign[0]=PlaceMap[PlaceMapPos][0];  //commented 2011 VM 
										LoCAlign[1]=PlaceMap[PlaceMapPos][1];
										LoCAlign[2]=PlaceMap[PlaceMapPos][2]+110;*/
										rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
										HolePointerTX=1;
										HolePointerRX=0;
									}
								else
								{
								 rres=PrimBodySchemaIndustrial(1,0,1,1,0,0); //commented for testing
								}
               
							//  int rres=1;
							   if(rres<=0) // This implies failiure to reach that will be addressed soon, then the if loop must be different
								   {
							         //cout<<"Analyzing anticipated consequnce in the body hub"<<endl;
									 BodyHub[3]=1;
									 BodyHub[4]=1;
									 ConsMicro=50;
									}
							    if(rres>0.1)
								   {
							           //cout<<"Reach Succesfull"<< endl;
									   BodyHub[0]=1;
									   BodyHub[1]=1;
									   BodyHub[24]=1; // 24:Arm Occ R, 25-L, 26-Both: this is based on the 3D location of the target
									   ConsMicro=50;

				  }


                  //=========================================================
				//======================================================

								}
		                break;

						case 1:
							{
							   cout<<"Initialializing Grasp primitive for Goal pointer::"<<  Goal_AcID << endl;
							   ActionPlotter[0]=2;
							
							    if(inDexPMPParalell==14){
								 DoubleGraspCombo();  // this makes paraleel grasping or one with grasped fuse and otehr in reach position
								
									//PrimBodySchemaIndustrial(19,0,1,1,0,0);
									Gres=1;
							   }
							   else {

								 if(PlaceMap[PlaceMapPos][1]>0){
									
									 RobotMovePMP=1;
									 Restorer();
							       Gres = PrimGraspIndustrial(GRASP_PINCH,0);//O for left arm
								 }

								 else
								 {
									 RobotMovePMP=-1;
									 Restorer();
							       Gres = PrimGraspIndustrial(GRASP_PINCH,1);//1 for right arm
								 }
								  Gres=1;
								  int ObstAvoid=ifObstacleMultiFuse();
								  cout<<"Computed ViaPoint"<< ViaPoint[0]<<ViaPoint[1]<<ViaPoint[2]<<endl;
								  PrimBodySchemaIndustrial(19,0,1,1,0,1);
							   }

							  
							     if(Gres==1) // This implies failiure to reach that will be addressed soon, then the if loop must be different
								   {
							         //cout<<"Grasp Done:Analyzing anticipated consequnce in the body hub"<<endl;
									 BodyHub[6]=1;
									 BodyHub[7]=1;
									 BodyHub[11]=1;
									 BodyHub[28]=1;
									 BodyHub[29]=1;
									 BodyHub[30]=1;
									//Finger Occupied must also be represented in the future.
									 ConsMicro=50;
									  
									}
							    if(Gres==0)
								   {
							          //cout<<"Grasping Failed"<< endl;
									   BodyHub[8]=1;
									   BodyHub[9]=1;
									   BodyHub[10]=1; // 24:Arm Occ R, 25-L, 26-Both: this is based on the 3D location of the target
									   ConsMicro=1;
									    if(PlaceMap[PlaceMapPos][1]>0)
														{

														   PrimGraspIndustrial(GRASP_RELEASE,0);
														 }

														 else
														 {
														    PrimGraspIndustrial(GRASP_RELEASE,0);//1 for right arm
														 }
										Time::delay(0.1);
										PrimBodySchemaIndustrial(19,0,1,1,0,0); 
									  
									}
								
									//}
							//	else{
							//	PrimBodySchemaIndustrial(19,0,1,1,0,0); //Change2005: used traj type to control initialize
							//	}
							}
		                break;

						case 6:
							{
							   cout<<"Initialializing Interrupt primitive for Goal pointer::"<<  Goal_AcID << endl;
							    ActionPlotter[6]=1;
							    cout<<"Goal Terminate Sucesfull: Reinitializing working memory and waiting for user goal"<< endl;
								BodyHub[16]=1;
							    BodyHub[17]=1;
								TerminateFlag=1;
							    ConsMicro=50;
							}
		                break;

						case 5:
							{
							   cout<<"Initialializing 'Seek User help and learn new experience' primitive"<< endl;
							   ActionPlotter[5]=1;
							   int ActionUser=UserInterface(32);

							   if(ActionUser==1)
								   { // user is willing to help, puts the object in the scene..., ideally u must refresh placemap
									/*   for(jCo=0; jCo<42; jCo++)
											{
											  GlobalWorkSpace[0][jCo]=PlaceMapHub[0][jCo];
											}
									    */
									   ConsMicro=50;
								   }
							     if(ActionUser==0)
								   {
									   /*BodyHub[16]=1;
									   BodyHub[17]=1;*/
									  ConsMicro=1;
								   }

							}

						break;
						case 11:
							{
							   cout<<"Initialializing Align and Release primitive for Goal pointer::"<<  Goal_AcID << endl;
							    ActionPlotter[11]=1;
								////=========================================================
								// // This block below is now doing the alignment, in future, this will be moved 
								////======================================================
								////   //=========================================================
/*
                  // Variable to store distance to be calculated
                  double AlignF = 99999;

                  // Check if Vision can find the objects after certain trial, check SceneChanged function
                  bool flag_exit = false;

                  // If distance is greater than a threshold, do alignment
                  while(AlignF > 10 && !flag_exit)
                  {
                    // Calculating the distance and moving the robot to do alignment
                    std::cout<<"Going to start the Alignment ......"<<std::endl;
                    flag_exit = Align(AlignF);

                    if(!flag_exit)
                    {
                      std::cout<<"Distance alignment: "<<AlignF<<std::endl;
                      double alex=PrimBodySchemaIndustrial(95,0,0,1,0,0);
                      
                    }

                  }
*/
				// Aligment commented out
				  bool flag_exit = false;
                  if(!flag_exit)
                  {
                    // Changing the height to do the insertion after alignment
                  //  LoCAlign[2]=45+PosHandOcc[2];
					 /* PlaceMapPos=FuseBoXPos;
					  LoCAlign[0]=PlaceMap[PlaceMapPos][0];
					  LoCAlign[1]=PlaceMap[PlaceMapPos][1];
					  LoCAlign[2]=PlaceMap[PlaceMapPos][2]+60;   */                      //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................45+LoCAlign[2]
                    
					  	LoCAlign[0]=SerialHole[(numfuseFBoxe*holeCount)-1][0];  
						LoCAlign[1]=SerialHole[(numfuseFBoxe*holeCount)-1][1];  ;
						LoCAlign[2]=SerialHole[(numfuseFBoxe*holeCount)-1][2]+60;  
					
						reorientAndInsert(LoCAlign[0],LoCAlign[1]); //////uses affordance n kinm

					//	double alex=PrimBodySchemaIndustrial(111,0,0,1,0,0);// made 111 Komment 09Dec
                 
						
						// opening the gripper for the industrial platform
                    // change it to automatic - TX and RX shift
					  //PlaceMapPos=FuseBoXPos;
					  //LoCAlign[0]=PlaceMap[PlaceMapPos][0];
					  //LoCAlign[1]=PlaceMap[PlaceMapPos][1];
					  //LoCAlign[2]=PlaceMap[PlaceMapPos][2]+70;                         //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................45+LoCAlign[2]
       //               alex=PrimBodySchemaIndustrial(111,0,0,1,0,0);

                    PrimGraspIndustrial(GRASP_RELEASE,0); //hardcoded for TX  commented vm2011
                    //PrimGraspIndustrial(GRASP_RELEASE,1); // Hardcoded for RX
					 Time::delay(0.1);
					 LoCAlign[2]=120+LoCAlign[2];  //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................MUST BE pOShANDOCC
                    int alex=PrimBodySchemaIndustrial(95,0,0,1,0,0);
					 LoCAlign[1]=400;
					 LoCAlign[2]=350;		//set to go up
					 int rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
					  Time::delay(0.1);
                  }  //Commented for testing 10/02
								//=========================================================
								//======================================================
								//=========================================================
						
				  for(int c = 0; c < minObjectsCount-2;c++){
					InsertReachCombo(RobotMovePMP);
				  }

				  /*InsertReachCombo(RobotMovePMP);
				  InsertReachCombo(RobotMovePMP);
				  InsertReachCombo(RobotMovePMP);*/
				 

				  //==============================================================
              if(RobotMovePMP==1)   // who gets to insert
						  {
							RobotMovePMP=-1;
						  }
						  else if((RobotMovePMP==-1))
						  {
						   RobotMovePMP=1;
						  }				
		    if(RobotMovePMP==-1)
			{
			  LoCAlign[0]= SerialHole[HolePointerRX][0];
			  LoCAlign[1]= SerialHole[HolePointerRX][1];
			  LoCAlign[2]=150;

			   int alex2=PrimBodySchemaIndustrial(95,0,0,1,0,0);
			 /* LoCAlign[0]= ComboCoordinate[0];
			  LoCAlign[1]= ComboCoordinate[1];*/
			  LoCAlign[2]=86; //SerialHole[HolePointerRX][2]+60 Komment Dec 9
			//  alex2=PrimBodySchemaIndustrial(111,0,0,1,0,0);
			 // Insert RX
              reorientAndInsert(LoCAlign[0],LoCAlign[1]); //////uses affordance n kinm

			  PrimGraspIndustrial(GRASP_RELEASE,1); // Hardcoded for RX 6o release 
			  Time::delay(0.1);
			  //Relese RX
			  LoCAlign[2]=120+LoCAlign[2];  //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................MUST BE pOShANDOCC
			   alex2=PrimBodySchemaIndustrial(95,0,0,1,0,0);
						
			  // Go Up RX
			  Time::delay(0.1);
			  LoCAlign[1]=-250;
			  LoCAlign[2]=350;		//set to go up
			  int rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
			  Time::delay(0.1);
			}

		  
			if(RobotMovePMP==1)
			{
              int kkPt=numfuseFBoxe*holeCount;
			  LoCAlign[0]= SerialHole[kkPt-HolePointerTX-1][0];
			  LoCAlign[1]= SerialHole[kkPt-HolePointerTX-1][1];
			  LoCAlign[2]=150;

			   int alex2=PrimBodySchemaIndustrial(95,0,0,1,0,0);
			 /* LoCAlign[0]= ComboCoordinate[0];
			  LoCAlign[1]= ComboCoordinate[1];*/
			  LoCAlign[2]=86; //SerialHole[HolePointerRX][2]+60
		//	  alex2=PrimBodySchemaIndustrial(111,0,0,1,0,0); kOMMENT tx

			reorientAndInsert(LoCAlign[0],LoCAlign[1]); //////uses affordance n kinm

			 // Insert TX
            
			  PrimGraspIndustrial(GRASP_RELEASE,0); // TX release 
			  Time::delay(0.1);
			  //Relese TX
			  LoCAlign[2]=120+LoCAlign[2];  //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................MUST BE pOShANDOCC
			   alex2=PrimBodySchemaIndustrial(95,0,0,1,0,0);
						
			  // Go Up TX
			  Time::delay(0.1);
			  LoCAlign[1]=400;
			  LoCAlign[2]=350;		//set to go up
			  int rres=PrimBodySchemaIndustrial(95,0,1,1,0,0); 
			  Time::delay(0.1);
			}




		//======================Last insertion==========================
				  
				  /*  PrimBodySchemaIndustrial(19,0,1,1,0,0);  //comment vm 2011
								RecursiveAlignment(inDexPMPParalell);
								RecursiveAlignment(inDexPMPParalell);
								cout<<"Object released"<< endl;
								 BodyHub[28]=0;
								 BodyHub[29]=0;
								 BodyHub[30]=0;
								TerminateFlag=1;
							    ConsMicro=50;*/

							 }
		                break;

						//change
						 case 3:
							{
							   cout<<"Initialializing Push primitive for Goal pointer::"<<  Goal_AcID << endl;
							    ActionPlotter[4]=1;
								if(PosHandOcc[1]>0)
												 {
											       PrimPush(104,-77); 
												 }
											 if(PosHandOcc[1]<0)
												 {
											      PrimPush(-104,77); // towards Tx 
												 }
							    cout<<"Push Sucesfull: Reinitializing working memory and waiting for user goal"<< endl;
								BodyHub[13]=1;
							    BodyHub[14]=1;
								ConsMicro=50;
							 }
		                break;  


						case 10:
							{
							   cout<<"Initialializing Observe primitive for Goal pointer::"<<  Goal_AcID << endl;
							   ConsMicro=50;
							 }
		                break;  
						//change
		 			 }


				 // It will be interesting to plot the comparison between the present behavior in comparison with the plan proposed by reasoning:?????
				 //this has to take into account Encapsulation...to enable reconstruction..
				    Behavior[iterMicroN][stateMicormonitor]=1;
					ActionRecorder[iterMicroN]=stateMicormonitor;
                    Behavior[iterMicroN][43]=1;
					 for(iCo=0; iCo<50; iCo++)
						 {
						 Present<< Behavior[iterMicroN][iCo]<< "    ";
						 }
						 Present << "    " << endl;
				    iterMicroN=iterMicroN+1;
					if(stateMicormonitor!=5)
						{
							for(jCo=0; jCo<42; jCo++)
											{
											  Behavior[iterMicroN][jCo]=BodyHub[jCo];
											}
							Behavior[iterMicroN][46]=1;
								 for(iCo=0; iCo<50; iCo++)
									 {
									 Present<< Behavior[iterMicroN][iCo]<< "    ";
									 }
						 Present << "    " << endl;
						 iterMicroN=iterMicroN+1;
						}

				 //////////////////changes to add visualization ACTION GUI Commented out
				/*Bottle& actionBot =actionPlot.prepare();
				actionBot.clear();
				actionBot.addString("action");
				Bottle& actionBotAll = actionBot.addList();
				actionBotAll.clear();
				for(int i=0; i<12; i++)
					{

                     actionBotAll.addDouble(ActionPlotter[i]);
				}
			    cout<<"Sending Action hub Matrix to DARWIN GUI"<<endl;
             	actionPlot.write();
             	Time::delay(2);
				//Sleep(2000);

				////////////////changes to add visualization BODY
				Bottle& bodyBot =bodyPlot.prepare();
				bodyBot.clear();
				bodyBot.addString("body");
				Bottle& bodyBotAll = bodyBot.addList();
				bodyBotAll.clear();
				for(int i=0; i<42; i++)
					{

                     bodyBotAll.addDouble(BodyHub[i]);
				}
			    cout<<"Sending Body hub Matrix to DARWIN GUI"<<endl;
             	bodyPlot.write();
                Time::delay(3);
				//Sleep(3000);*/


		return ConsMicro;
	};


void ObserverThread::initBodyHub()
{
  int iCo;
  for(iCo=0; iCo<42;iCo++)
  {
    BodyHub[iCo]=0;
  }
};

void ObserverThread::InverseGraspInitCombo()
{
	
                                //Trigger paraleel grasp and Initialization
							     if(PMPresp[0]==1){
									 fuseRX = findFuseInScene( &scene1,ComboCoordinate[0], ComboCoordinate[1],10);
									 RobotMovePMP=-1;
									 Restorer();
							    Gres = PrimGraspIndustrial(GRASP_PINCH,1);  //RX
									 
								/* ViaPoint[0]=FuseePos2D[roboIndexTx][0];
								 ViaPoint[1]=FuseePos2D[roboIndexTx][1];
								 ViaPoint[2]=FuseePos2D[roboIndexTx][2]+90;
								 XPosition[0]=FuseePos2D[roboIndexTx][0];
								 XPosition[1]=FuseePos2D[roboIndexTx][1];
								 XPosition[2]=FuseePos2D[roboIndexTx][2];*/
								 ViaPoint[0]= ComboCoordinate[0];
								 ViaPoint[1]= ComboCoordinate[1];
								 ViaPoint[2]= ComboCoordinate[2]+90;
								 XPosition[0]=ComboCoordinate[0];
								 XPosition[1]=ComboCoordinate[1];
								 XPosition[2]=ComboCoordinate[2];
								 PrimBodySchemaIndustrial(19,0,1,1,0,1);
								Time::delay(0.1);
								ComboCoordinate[1]=-200;
								ComboCoordinate[2]=ComboCoordinate[3]+300;
								    int rress=PrimBodySchemaIndustrial(122,0,1,1,0,0); 
								 }
								 // Rx picks up the fuse goes to initialize, tx reaches the hole
								 if(PMPresp[0]==14){
								      invGrspInitFlag=14;
									  RobotMovePMP=1;
									  LoCAlign[0]= ComboCoordinate[3];
									  LoCAlign[1]= ComboCoordinate[4];
									  LoCAlign[2]=86;
									//  int alex2=PrimBodySchemaIndustrial(111,0,0,1,0,0);
									 // Insert RX
                                       reorientAndInsert(LoCAlign[0],LoCAlign[1]);
									  PrimGraspIndustrial(GRASP_RELEASE,0); // Hardcoded for RX 6o release 
									  Time::delay(3);
									  //Relese RX
						
									  LoCAlign[2]=120+LoCAlign[2];  //MADE CHANGE 22 10 REMOVAL OF ALIGNMENT..................MUST BE pOShANDOCC
									 int alex2=PrimBodySchemaIndustrial(95,0,0,1,0,0);
									  //TX Releases here, RX is waiting...
									  ComboCoordinate[4]=350;
								    ComboCoordinate[5]=ComboCoordinate[5]+300;
							    	int rress=PrimBodySchemaIndustrial(122,0,1,1,0,0); 
									fuseRX = findFuseInScene( &scene1,ComboCoordinate[0], ComboCoordinate[1],10);
									RobotMovePMP=-1;
									Restorer();
									Gres = PrimGraspIndustrial(GRASP_PINCH,1);  //TX
									 
									 /*ViaPoint[0]=FuseePos2D[roboIndexTx][0];
									 ViaPoint[1]=FuseePos2D[roboIndexTx][1];
									 ViaPoint[2]=FuseePos2D[roboIndexTx][2]+90;
									 XPosition[0]=FuseePos2D[roboIndexTx][0];
									 XPosition[1]=FuseePos2D[roboIndexTx][1];
									 XPosition[2]=FuseePos2D[roboIndexTx][2];*/
								 ViaPoint[0]= ComboCoordinate[0];  //Tx moves up
								 ViaPoint[1]= ComboCoordinate[1];
								 ViaPoint[2]= ComboCoordinate[2]+90;
								 XPosition[0]=ComboCoordinate[0];
								 XPosition[1]=ComboCoordinate[1];
								 XPosition[2]=ComboCoordinate[2];
									PrimBodySchemaIndustrial(19,0,1,1,0,1);
									Time::delay(0.1);
									/*FuseePos2D[roboIndexTx][1]=FuseePos2D[roboIndexTx][1]+200;
								    FuseePos2D[roboIndexTx][2]=FuseePos2D[roboIndexTx][2]+300;*/
								//	ComboCoordinate[1]=-200;
								//	ComboCoordinate[2]=ComboCoordinate[3]+300;
								//  int rress=PrimBodySchemaIndustrial(122,0,1,1,0,0); 
													  RobotMovePMP=1;
								 }
				 }; //Here both should have 2 fuses grasped and initialized


void ObserverThread::DoubleGraspCombo()
{
                                //Trigger paraleel grasp and Initialization
							     if(PMPresp[0]==1){
									 fuseTX = findFuseInScene( &scene1,ComboCoordinate[3], ComboCoordinate[4],10);
									 std::cout<<":::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
									 std::cout<<"Printing Fuse "<<fuseTX.toString()<<std::endl;
									 std::cout<<":::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
									 std::cout<<":::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
									 RobotMovePMP=1;
									 Restorer();
        					    Gres = PrimGraspIndustrial(GRASP_PINCH,0);  //TX
									 
								/* ViaPoint[0]=FuseePos2D[roboIndexTx][0];
								 ViaPoint[1]=FuseePos2D[roboIndexTx][1];
								 ViaPoint[2]=FuseePos2D[roboIndexTx][2]+90;
								 XPosition[0]=FuseePos2D[roboIndexTx][0];
								 XPosition[1]=FuseePos2D[roboIndexTx][1];
								 XPosition[2]=FuseePos2D[roboIndexTx][2];*/
								 ViaPoint[0]= ComboCoordinate[3];
								 ViaPoint[1]= ComboCoordinate[4];
								 ViaPoint[2]= ComboCoordinate[5]+90;
								 XPosition[0]=ComboCoordinate[3];
								 XPosition[1]=ComboCoordinate[4];
								 XPosition[2]=ComboCoordinate[5];
								 PrimBodySchemaIndustrial(19,0,1,1,0,1);
								Time::delay(0.1);
								fuseRX = findFuseInScene( &scene1,ComboCoordinate[0], ComboCoordinate[1],10);
								 RobotMovePMP=-1;
								Restorer();
								Gres = PrimGraspIndustrial(GRASP_PINCH,1);
								
							     ViaPoint[0]= ComboCoordinate[0];  //made this change like above vm2011
								 ViaPoint[1]= ComboCoordinate[1];
								 ViaPoint[2]= ComboCoordinate[2]+90;
								 XPosition[0]=ComboCoordinate[0];
								 XPosition[1]=ComboCoordinate[1];
								 XPosition[2]=ComboCoordinate[2];
							    PrimBodySchemaIndustrial(19,0,1,1,0,1);
								Gres=1;
								 }
								 if(PMPresp[0]==14){
								    fuseTX = findFuseInScene( &scene1,ComboCoordinate[3], ComboCoordinate[4],10);
									 RobotMovePMP=1;
									Restorer();
									Gres = PrimGraspIndustrial(GRASP_PINCH,0);  //TX
									
									 /*ViaPoint[0]=FuseePos2D[roboIndexTx][0];
									 ViaPoint[1]=FuseePos2D[roboIndexTx][1];
									 ViaPoint[2]=FuseePos2D[roboIndexTx][2]+90;
									 XPosition[0]=FuseePos2D[roboIndexTx][0];
									 XPosition[1]=FuseePos2D[roboIndexTx][1];
									 XPosition[2]=FuseePos2D[roboIndexTx][2];*/
								 ViaPoint[0]= ComboCoordinate[3];  //Tx moves up
								 ViaPoint[1]= ComboCoordinate[4];
								 ViaPoint[2]= ComboCoordinate[5]+90;
								 XPosition[0]=ComboCoordinate[3];
								 XPosition[1]=ComboCoordinate[4];
								 XPosition[2]=ComboCoordinate[5];
									PrimBodySchemaIndustrial(19,0,1,1,0,1);
									Time::delay(0.1);
									/*FuseePos2D[roboIndexTx][1]=FuseePos2D[roboIndexTx][1]+200;
								    FuseePos2D[roboIndexTx][2]=FuseePos2D[roboIndexTx][2]+300;*/
									ComboCoordinate[4]=ComboCoordinate[4]+250;
									ComboCoordinate[5]=ComboCoordinate[5]+400;
								    int rress=PrimBodySchemaIndustrial(122,0,1,1,0,0);
									fuseRX = findFuseInScene( &scene1,ComboCoordinate[0], ComboCoordinate[1],10);
									RobotMovePMP=-1;
									Restorer();
							        Gres = PrimGraspIndustrial(GRASP_PINCH,1);
									
									LoCAlign[0]=ComboCoordinate[0];
									LoCAlign[1]=ComboCoordinate[1]-150;
									LoCAlign[2]=ComboCoordinate[2]+200;
									PrimBodySchemaIndustrial(95,0,0,1,0,0);
									Time::delay(0.1);
								 
								 }
				 }; //Here both should have 2 fuses grasped and initialized


void ObserverThread::GraspInitCombo()
{                               // // Tx picks up the fuse goes to initialize, Rx reaches the hole
                                //Trigger paraleel grasp and Initialization
							     if(PMPresp[0]==1){
								fuseTX = findFuseInScene( &scene1,ComboCoordinate[3], ComboCoordinate[4],10);
								RobotMovePMP=1;
								Restorer();
							    Gres = PrimGraspIndustrial(GRASP_PINCH,0);  //TX
									 
								/* ViaPoint[0]=FuseePos2D[roboIndexTx][0];
								 ViaPoint[1]=FuseePos2D[roboIndexTx][1];
								 ViaPoint[2]=FuseePos2D[roboIndexTx][2]+90;
								 XPosition[0]=FuseePos2D[roboIndexTx][0];
								 XPosition[1]=FuseePos2D[roboIndexTx][1];
								 XPosition[2]=FuseePos2D[roboIndexTx][2];*/
								 ViaPoint[0]= ComboCoordinate[3];
								 ViaPoint[1]= ComboCoordinate[4];
								 ViaPoint[2]= ComboCoordinate[5]+90;
								 XPosition[0]=ComboCoordinate[3];
								 XPosition[1]=ComboCoordinate[4];
								 XPosition[2]=ComboCoordinate[5];
								 PrimBodySchemaIndustrial(19,0,1,1,0,1);
								Time::delay(0.1);

								ComboCoordinate[4]=350;
								ComboCoordinate[5]=ComboCoordinate[5]+300;
								int rress=PrimBodySchemaIndustrial(122,0,1,1,0,0); 

								 }
								 if(PMPresp[0]==14){
									fuseTX = findFuseInScene( &scene1,ComboCoordinate[3], ComboCoordinate[4],10);
									RobotMovePMP=1;
									Restorer();
									Gres = PrimGraspIndustrial(GRASP_PINCH,0);  //TX
									
									 /*ViaPoint[0]=FuseePos2D[roboIndexTx][0];
									 ViaPoint[1]=FuseePos2D[roboIndexTx][1];
									 ViaPoint[2]=FuseePos2D[roboIndexTx][2]+90;
									 XPosition[0]=FuseePos2D[roboIndexTx][0];
									 XPosition[1]=FuseePos2D[roboIndexTx][1];
									 XPosition[2]=FuseePos2D[roboIndexTx][2];*/
								 ViaPoint[0]= ComboCoordinate[3];  //Tx moves up
								 ViaPoint[1]= ComboCoordinate[4];
								 ViaPoint[2]= ComboCoordinate[5]+90;
								 XPosition[0]=ComboCoordinate[3];
								 XPosition[1]=ComboCoordinate[4];
								 XPosition[2]=ComboCoordinate[5];
									PrimBodySchemaIndustrial(19,0,1,1,0,1);
									Time::delay(0.1);
									/*FuseePos2D[roboIndexTx][1]=FuseePos2D[roboIndexTx][1]+200;
								    FuseePos2D[roboIndexTx][2]=FuseePos2D[roboIndexTx][2]+300;*/
									ComboCoordinate[4]=350;
									ComboCoordinate[5]=ComboCoordinate[5]+300;
								    int rress=PrimBodySchemaIndustrial(122,0,1,1,0,0); 
							
								 
								 }
				 }; //Here both should have 2 fuses grasped and initialized



double ObserverThread::PrimPush(int ReachSide, int PushTarget)
{
  //int Pushres=PrimSearch(0,0,95);
  RefreshPlacemap();

  PlaceMap[PlaceMapPos][0]=PosHandOcc[0];
  PlaceMap[PlaceMapPos][1]=PosHandOcc[1]+ReachSide;
  PlaceMap[PlaceMapPos][2]=PosHandOcc[2]-110;
  int pushreach=PrimBodySchemaIndustrial(1,0,1,1,0.28,0);
  PlaceMap[PlaceMapPos][0]=PosHandOcc[0];
  PlaceMap[PlaceMapPos][1]=PosHandOcc[1]+ReachSide+PushTarget;
  PlaceMap[PlaceMapPos][2]=PosHandOcc[2]-110;
  pushreach=PrimBodySchemaIndustrial(1,0,1,0,0.28,0);
  //cout<<" Predicted object location is    " << XPosition[0] << XPosition[1] <<XPosition[2]<< endl;
  if(pushreach==1)
  {
    //cout<<" Simulation of pushing predicts sucessful spatial reorganization to afford inserting "<< endl;
    pushreach=PrimBodySchemaIndustrial(1,0,1,1,0.28,0);
  }
  pushreach=PrimBodySchemaIndustrial(19,0,1,1,0,0);
  AlignFlag==1;
  //Sleep(5000);
  RefreshPlacemap();
  //cout<<" New object location is    "<<LoCAlign[0] << LoCAlign[1] <<LoCAlign[2]<< endl;
  return pushreach;
};


// This function finds out if the scene has changed or not and returns the stable values
// for the position of the fuse and the hole given a threshold
bool ObserverThread::SceneChanged(double threshold)
{
  // Varaibles to store the previous values of fuse and hole
  double prev_fuse[3];
  double prev_hole[3];

  // Loading the previous values of fuse and hole

  //  if(LoCAlignFusee[0] < -999 || LoCAlignFusee[0] > 999)
  //{

  for(int i=0;i<3;++i)
  {
    prev_fuse[i] = LoCAlignFusee[i];
    prev_hole[i] = LoCAlign[i];
  } 

  //  }
  // Refreshing the placemap to get the current position of the fuse and hole
  ExpectedObjects& myObjs=objport.prepare();
  myObjs.setPast(0);
  myObjs.IDs().clear();
  myObjs.IDs().add(101);  
  myObjs.IDs().add(100);


	if(!(Network::isConnected("/observer/expectedobjs:o"," /vision/expectedobjs:i"))){
		Network::connect("/observer/expectedobjs:o"," /vision/expectedobjs:i");
	}
	if(objport.getOutputCount()){
      objport.write(); // send
	}
  RefreshPlacemap();

  //// Check if atleast two objects are found
  if(NumberofObs>=2)
  {

    bool detected_hole = false;
    bool detected_fuse = false;

    for(int i=0;i<NumberofObs;++i)
    {
      // If hole is found
      if(ObjIDEE[i] == 100)
      {
        detected_hole = true;
      }

      // If fuse is found
      if(ObjIDEE[i] == 101)
      {

        detected_fuse = true;

      }
    }

    // Proceed only if both hole and fuse are found, if not exit the function
    if(!(detected_fuse && detected_hole)){
      std::cout<<"DID NOT find fuse and/or  Hole "<<std::endl;
      return true;
    }
  }

  // If less than two objects are found, exit the function
  else{
    std::cout<<"No.of objects less than 2"<<std::endl;
    return true;
  }

  int chosenFuse= ChooseFuse();
  LoCAlignFusee[0]=LoCAlignFuseeD[chosenFuse][0];
  LoCAlignFusee[1]=LoCAlignFuseeD[chosenFuse][1];
  LoCAlignFusee[2]=LoCAlignFuseeD[chosenFuse][2];
  // Find the distance of the fuse from the previous and the current position

  double xdiff_fuse = (prev_fuse[0]-LoCAlignFusee[0]);
  double ydiff_fuse = (prev_fuse[1]-LoCAlignFusee[1]);
  double zdiff_fuse = (prev_fuse[2]-LoCAlignFusee[2]);
  double distance_fuse = (sqrt(pow(xdiff_fuse,2)+ pow(ydiff_fuse,2)+pow(zdiff_fuse,2)));

  std::cout<<std::endl;
  std::cout<<"Distance of the fuse from the last two snapshots: "<<distance_fuse<<std::endl;

  // Find the distance of the hole from the previous and the current position
  //Changes 2205
  /*double  xdiff_hole = (prev_hole[0]-LoCAlign[0]);
  double ydiff_hole = (prev_hole[1]-LoCAlign[1]);
  double zdiff_hole = (prev_hole[2]-LoCAlign[2]);
  double distance_hole = (sqrt(pow(xdiff_hole,2)+ pow(ydiff_hole,2)+pow(zdiff_hole,2)));*/

  std::cout<<std::endl;
  //std::cout<<"Distance of the hole from the last two snapshots: "<<distance_hole<<std::endl;
  std::cout<<std::endl;

  // If any one of the two distances calculated above are greater than the threshold, 
  // then return true to calculate distance again; //|| distance_hole > threshold
  if(distance_fuse > threshold )
  {
    return true;
  }

  // If both distances are below the threshold, return false and continue to the Align function
  else
  {
    return false;
  }

}

int ObserverThread::ChooseFuse(){
  int chosenIndex=0;
  double xdiff_hole[10],ydiff_hole[10],zdiff_hole[10],distance_hole[10],minDist=0;
  for (int i = 0;i<10;i++){
    xdiff_hole[i] = (XPosition[0]-LoCAlignFuseeD[i][0]);
    ydiff_hole[i] = (XPosition[1]-LoCAlignFuseeD[i][1]);
    zdiff_hole[i] = (XPosition[2]-LoCAlignFuseeD[i][2]);
    distance_hole[i] = (sqrt(pow(xdiff_hole[i],2)+ pow(ydiff_hole[i],2)+pow(zdiff_hole[i],2)));
  }

  minDist=distance_hole[0];
  for (int i=1;i<10;i++){
    if (distance_hole[i]<minDist){
      minDist=distance_hole[i];
      chosenIndex=i;
    }

  }
  return chosenIndex;
}

// This function Aligns the fuse to the opening and facilitates insertion into the hole
bool ObserverThread::Align(double& AlignF)
{
  AlignFlag=1;
  //Sleep(30000); // Static Sleep - not used any more

  std::cout<<"Entered Align Function"<<std::endl;

  bool Scene_flag = true;
  int counter = 0;

  //Find the right Fuse

  while(Scene_flag)
  {
    //Distance threshold is set to 10
    ++counter;
    std::cout<<"Calling the SceneChanged Function for  "<<counter<<" time"<<std::endl;
    Scene_flag =  SceneChanged(10);

    // If vision cannot find the required objects after number of tries, this case 10, then exit
    if(counter >= 10 && Scene_flag)
    {
      std::cout<<"Vision cannot find required objects in the scene, Please check"<<std::endl;
      AlignF = -1;
      return true;
    }

  }
  //Add new function to check the distance between current and previous fuse cap position

  //RefreshPlacemap();

  // Variables to store difference
  double xdiff = 0;
  double ydiff = 0;

  /*LoCAlign[0]=(XPosition[0]+LoCAlign[0])/2;
  LoCAlign[1]=(XPosition[1]+LoCAlign[1])/2;
  LoCAlign[2]=XPosition[2]-40;*/
  //LoCAlign[0]=((LoCAlignFusee[0]+LoCAlign[0])/2);
  //LoCAlign[1]=(LoCAlignFusee[1]+LoCAlign[1])/2;
  //LoCAlign[2]=205;
  // Finding the distance between positions of the fuse and the hole given by vision
  //Changes2205
  xdiff = (LoCAlign[0]-LoCAlignFusee[0]);
  ydiff = (LoCAlign[1]-LoCAlignFusee[1]);
  AlignF = (sqrt(pow(xdiff,2)+ pow(ydiff,2)));

  std::cout<<"Distance : "<<AlignF<<std::endl;

  // This difference in X and Y are added to the current position of the robot (calculated from the encoder values)
  // to get the next position to which the robot should move
  LoCAlign[0]=XPosition[0]+xdiff;
  LoCAlign[1]=XPosition[1]+ydiff;
  LoCAlign[2]=PosHandOcc[2]+45;

  return false;
};

double ObserverThread::AlignHand()
{
  AlignHandFlag=1;
  //Time::delay(30);
  //Sleep(30000);
  RefreshPlacemap();

  double xdiff = (XPosition[0]-LoCAlignFusee[0]);
  double ydiff = (XPosition[1]-LoCAlignFusee[1]);
  double distance = (sqrt(pow(xdiff,2)+ pow(ydiff,2)));
  std::cout<<"Distance : "<<distance<<std::endl;
  LoCAlignHand[0]=XPosition[0]+xdiff;
  LoCAlignHand[1]=XPosition[1]+ydiff;
  LoCAlign[2]=145;
  double alex=PrimBodySchemaIndustrial(95,0,0,0,0,0);
  return distance;
};

void ObserverThread::LoadGWSArgument()
{
  UserInterface(41);
  GetLocalAct(NumWords);
  int PropWeightC=1; // this can come as a result of the elimination/growth rule
  Retroactivate(PropWeightC);
  cout<<" Initializing DARWIN Working memory " <<endl;
  InitializeWorkingMemory(1);
}

void ObserverThread::Mergence(){
  int iCo, jCo, NoBReplan=0;
  int TempSeqAcP[10];
  //	if(CannotFind==1){
  //		NPiCs=NPiCs-1;
  //		}
  for(iCo=0; iCo<NPiCs; iCo++)
  {
    TempSeqAcP[iCo]=SeqAcP[iCo];
    //cout<< "Interpreting New plan"<< TempSeqAcP[iCo] <<endl;
    Report<< "Mergence Interpreting New plan"<< TempSeqAcP[iCo] <<endl;
    SeqAcP[iCo]=50;
  }
  SeqAcP[0]=PPiCo;
  NoBReplan=NoBReplan+1;
  //cout<< "Connecting element of past plan"<< SeqAcP[0] <<endl;
  Report<< "Connecting element of past plan"<< SeqAcP[0] <<endl;
  int present;
  for(iCo=0; iCo<NPiCs; iCo++)
  {
    present=0;
    for(jCo=0; jCo<numberpast+1; jCo++){
      if(PastPlan[jCo]==TempSeqAcP[iCo])
      {
        //cout<<"Object" << TempSeqAcP[iCo] << "is common"<< endl;
        Report<<"Object" << TempSeqAcP[iCo] << "is common"<< endl;
        present=1;
      }
    }
    if((present==0)&&(TempSeqAcP[iCo]!=SeqAcP[0])){
      //cout<<"Using Object"<<TempSeqAcP[iCo]<<"in the new plan" <<endl;
      SeqAcP[NoBReplan]=TempSeqAcP[iCo];
      NoBReplan=NoBReplan+1;
    }
  }
  NPiCs=NoBReplan;

}

int ObserverThread::PickandPlace(int pick, int place, int seqNumber) {

  int resPandP=0;
  double PMPRepl=0, PMPReplPlace=0,GraspO=-1, GraspC=-1, GraspR=-1,PMPReplInit=0,PMPReplMicro=0;
  CannotFind=-1, Cumulate=0;
  findsuccess=0;
  findsuccess2=0;
  if(GiD==14){
    /*findsuccess=PrimSearch(pick,place,5);
    if(findsuccess == 0)
    {
    findsuccess=PrimSearch(pick,place,5);
    if(findsuccess == 0)
    {
    findsuccess=PrimSearch(pick,place,5);
    }
    }*/
    //cout<<" Waiting for findsuccess " <<endl;
    cin >> findsuccess;  //VM extended test 0104

  }
  if(GiD==32){
    findsuccess=PrimSearch(pick,place,68);
  }
  //cout<<"FIND SUCCESS IS : "<<findsuccess<<endl;
  Report<<"FIND SUCCESS IS : "<<findsuccess<<endl;

  //    int findsuccess=1;
  if (findsuccess==1)
  {
    //
    if(GiD==14){
      //cout<< "Both Objects involved in the present micro sequnce are there " << endl;
      Report<< "Both Objects involved in the present micro sequnce are there "<<pick << place << endl;
    }
    //cout<< "Contacting Grasp server to open gripper " << endl; //open
    Report<< "Contacting Grasp server to open gripper " << endl;
    Cumulate=Cumulate+1	;

    /*        GraspO=PrimGrasp(2); //commented 0603 // opening of gripper */

    GraspO = 5;//commented 0603 *************************VM01/04
    if(GraspO==5)
    {
      //cout<< "Contacting PMP server ........VOCAB REA " << endl;
      Report<< "Contacting PMP server ........VOCAB REA " << endl;
      Cumulate=Cumulate+1	;
      //cout<<"THE OBJECT ID:  "<<GetObjIDs[0]<<std::endl;

      //	PMPRepl=PrimBodySchema(1,GetObjIDs[0],1,pick);// contains arguements for reach object pick;
      //cout<<" Waiting for PMPRepl " <<endl;
      cin >>PMPRepl;

      //cout<<"PMPRepl: "<<PMPRepl<<std::endl;
      Report<<"PMPRepl received from Prim Body schema is: "<<PMPRepl<<std::endl;
      //if this is sucessful, grasp the object, init the arm, find if the object is still there in the scene
      //if all these actions are sucessful
    }
    if(PMPRepl==1)
    {
      //cout<< "Contacting Grasp server to Grip the object " << endl; //open
      Report<< "Contacting Grasp server to Grip the object " << endl; //open
      Cumulate=Cumulate+1;
      /*     GraspC=PrimGrasp(1); //commented 0603 */
      //cout<< "Waiting for GraspC " << endl;
      cin>>GraspC;
    }
    ////commented 0603
    //GraspC = 5;
    if(GraspC==5)
    {
      GraspC=4;// added VM 23/03
      //cout<< "Contacting PMP server to init and Check sucess of Pick " << endl; //open
      Report<< "Contacting PMP server to init and Check sucess of Pick " << endl; //open
      Cumulate=Cumulate+1;
      //Time::delay(10);

      /*    PMPReplInit=PrimBodySchema(19,GetObjIDs[0],1,pick); */ //**********VM01/04

      //Time::delay(10);
      //*****************************23/03/: Modified this part so as not not route through vision..but Gripper instead.......
      //**My also have to add one line in PMP Module..if necessary...
      /*			 PrimSearch(pick,place,68);
      int checkDisp=GetObjIDs[0];
      if ((checkDisp != 50) && (PlaceMap[checkDisp][cannotfindXLoc]!=cannotfindX)&&(GraspC==5))
      {
      CannotFind=0;
      }
      if(checkDisp==50)
      {
      CannotFind=0;
      } */

      //	 GraspC=PrimGrasp(1);  //checking closing again after arm has initialized..VM 23/03

      GraspC=5; //VM01/04
      if(GraspC==5){
        CannotFind=0;
        //cout<<"Object is coupled to the gripper : "<<CannotFind<<endl;
        Report<<"Object is coupled to the gripper : "<<CannotFind<<endl;
      }

      //you need to refresh placemap here and check if the picked object ID is no longer there
    }

    //============================bug correct
    if(GraspC==4){
      CannotFind=1;
      //cout<<"Object has slipped from the fingers : "<<CannotFind<<endl;
      Report<<"Object has slipped from the fingers : "<<CannotFind<<endl;
    }
    //===========================================================

    //CannotFind = 0;
    if (CannotFind==0){
      //RefreshPlacemap();
      Cumulate=Cumulate+1	;
      if(GiD==14){
        //Time::delay(10);
        /*	findsuccess2=PrimSearch(place,pick,68);
        if(findsuccess2 == 0)
        {
        findsuccess2=PrimSearch(place,pick,68);
        if(findsuccess2 == 0)
        {
        findsuccess2=PrimSearch(place,pick,68);
        }
        }*/
        //cout<<" Waiting for findsucess2 " <<endl;
        cin>>findsuccess2; //VM04/01
        //cout<< "reaching object " << GetObjIDs[1]<<" , " << GetObjIDs[0] << endl;
        Report<< "reaching object to place " << GetObjIDs[1]<<" , " << GetObjIDs[0] << endl;
      }
      if(GiD==32){
        findsuccess2=1;
        //cout<< "Placing object "<< endl;
        Report<< "Placing object "<< endl;
      }
      if(findsuccess2==1){
        if(GiD==14){

          //	PMPReplPlace=PrimBodySchema(1,GetObjIDs[0],3,pick);// contains arguements for reach object Place;	//was place before
          //cout<<" Waiting for PMPReplPlace " <<endl;
          cin>>PMPReplPlace;
          //VM01/04
          //if this is sucessful, Release the object, init the arm, find if the object with ID pick is now there in the scene
          //and approximately allined in the z-dimension with object place
        }
        if(GiD==32){

          StaticLoc[0]=550;  //Check the start position to place
          StaticLoc[1]= 450;
          if(pick == 2)
          {
            StaticLoc[2]=125+90;
          }
          else
          {
            StaticLoc[2]=95+90;
          }

          PMPReplPlace=PrimBodySchema(21,GetObjIDs[0],1,GetObjIDs[0],0,1);// contains arguements for reach object Place;
          //if this is sucessful, Release the object, init the arm, find if the object with ID pick is now there in the scene
          //and approximately allined in the z-dimension with object place


          ////cout<<"LOCATION where I am placing object "<<pick<<
          //"is"<<StaticLoc[0]<<StaticLoc[1]<<StaticLoc[2]<<endl;
        }
      }
    }

    if(PMPReplPlace==1)
    {
      //cout<< "Contacting Grasp server to stack/release " << endl; //open
      Report<< "Contacting Grasp server to stack/release " << endl; //open
      Cumulate=Cumulate+1	;
      // GraspR=PrimGrasp(2); // releasing opening gripper
      GraspR=5;//
    }
    //GraspR = 1;
    if(GraspR==5)
    {

      Cumulate=Cumulate+1;
      Report<<"Entered initializing loop : "<<GraspR<<endl;
      //cout<<"Entered initializing loop : "<<GraspR<<endl;
      //	  PMPReplMicro=PrimBodySchema(19,GetObjIDs[1],1,GetObjIDs[1]); //init the arm and go back to the master to see what next
      //VM01/04

      Report<<"out of PMPREPLMICRO : "<<PMPReplMicro<<endl;
      //cout<<"out of PMPREPLMICRO : "<<PMPReplMicro<<endl;
      //Time::delay(10);
    }
    // here you get a cumulative score of sucess of vision, PMP and Grasp,
    //this has to be fed back to the Observer to execute the next Usequnce
  }

  if (findsuccess==0)
  {
    //cout<< "Sense failure in finding object to pick up" << endl;
    Report<< "Sense failure in finding object to pick up" << endl;
    Cumulate=0;
  }

  if ((findsuccess2==0)&&(findsuccess!=0))
  {
    //cout<<"Sense failure in finding object on which I need to place the picked object"<<endl;
    Report<<"Sense failure in finding object on which I need to place the picked object"<<endl;
    Cumulate=0;
  }

  if ((PMPRepl==2)&&(findsuccess==1))
  {
    //cout<<"Sense failure in reaching the object I need to pick up"<<endl;
    Report<<"Sense failure in reaching the object I need to pick up"<<endl;
    Cumulate=4;  // object to be picked is not reachable: Comment VM 23/03^
  }

  if ((PMPReplPlace==2)&&(PMPRepl==1)&&((CannotFind!=1)))
  {
    //cout<<"Sense failure in Placing"<<endl;
    Report<<"Sense failure in Placing"<<endl;
    Cumulate=0;
  }

  if ((CannotFind==1)&&(PMPRepl==1))
  {
    //cout<<"Sense failure: The scene is not refreshed/Object has slipped our: CannotFind"<<CannotFind<<endl;
    Report<<"Sense failure: The scene is not refreshed/Object has slipped our"<<endl;
    Cumulate=0; // object picked up has slipped  Comment VM/23/03
  }

  if ((PMPRepl==1)&&(PMPReplPlace==1)&&(CannotFind==0)&&(findsuccess==1)&&(findsuccess2==1)){
    Cumulate=1;
  }
  //cout<<"CUMULATE SCORE"<<Cumulate<<endl;

  if(GiD==32){
    Cumulate=findsuccess;
  }

  return Cumulate; // VM transformed the findsucess score into a cumualtive score that relates to all micro events
}


int ObserverThread::ifObstacle(){

	double DistObst,xdiff_Obst,ydiff_Obst;
	int NeedObst=0;
	double Slopee=0;
	xdiff_Obst = (PosHandOcc[0]-FuseObst[0]);
	if(xdiff_Obst==0)
	{
      xdiff_Obst=xdiff_Obst+0.1;
	}
	ydiff_Obst = (PosHandOcc[1]-FuseObst[1]);
    DistObst = (sqrt(pow(xdiff_Obst,2)+ pow(ydiff_Obst,2)));
	if(DistObst<=41)
		{
		  NeedObst=1;
		  Slopee=ydiff_Obst/xdiff_Obst;
			  if(xdiff_Obst>0)
			{
			   ViaPoint[0]=FuseObst[0]-32;
			   ViaPoint[1]=(Slopee*ViaPoint[0])+FuseObst[1];
			}
			if(xdiff_Obst<0)
			{
			   ViaPoint[0]=FuseObst[0]+32;
			   ViaPoint[1]=(Slopee*ViaPoint[0])+FuseObst[1];
			}
			ViaPoint[2]=PosHandOcc[2]+95;
		}
		
return NeedObst;
    }

int ObserverThread::ifObstacleMultiFuse(){

	double DistObstMF,xdiff_ObstMF,ydiff_ObstMF;
	int NeedObstMF=0;
	//double Slopee=0;
	for (int i=0;i<numfuseFBoxe;i++)
	xdiff_ObstMF = (PosHandOcc[0]-FuseObst[0]);
	if(xdiff_ObstMF==0)
	{
      xdiff_ObstMF=xdiff_ObstMF+0.1;
	}
	ydiff_ObstMF = (PosHandOcc[1]-FuseObst[1]);
    DistObstMF = (sqrt(pow(xdiff_ObstMF,2)+ pow(ydiff_ObstMF,2)));
	//if(DistObstMF<=41)
		//{
		  //cout<<"Seems one of the obejcts in the scene is an active Obstace: Initiating ViaPoint estimation"<<endl;
		  NeedObstMF=1;
		  ViaPoint[0]=FuseObst[0];
		  ViaPoint[1]=FuseObst[1];
		  ViaPoint[2]=PosHandOcc[2]+90;
		//}
		
return NeedObstMF;
    }


void ObserverThread::ComputeViaPoint(){


}

//This is for the industrial robot
//int PMPGoalCode, int OIDinPM,int PIdentifier,int ObjectIDPMP

double ObserverThread::PrimBodySchemaIndustrial(int PMPGoalCode,int OIDinPM,int PIdentifier, int MsimFlag, int WristOrient, int TrajType){

	double returnval = 0.0;
	try {
  Network::connect("/BodySchemaSim:io", "/pmpRX/PMPreply:io");
  //cout << "Inside the PrimBodySchema : " << endl;
  Report << "Inside the PrimBodySchema : "  << endl;
  int iCo=0, iCocord=-1,cannotfindXLoc=100;
  cannotfindX=0;
  for(int i=2;i<18;i=i+3)
  {
    if((PlaceMap[OIDinPM][i])>iCocord) //checks 2-5-8-11-14..
    {
      iCocord=PlaceMap[OIDinPM][i]; //z coordinate
      iCo=i;
    }
  }
  PMPCommand BodySchema_cmd;
  PMPResult BodySchema_response;
  //Bottle BodySchema_cmd, BodySchema_response;
  //BodySchema_cmd.addVocab(COMMAND_VOCAB_REA);
  BodySchema_cmd.setGoalCodePMP(PMPGoalCode);

  BodySchema_cmd.setMSFlag(MsimFlag);
  BodySchema_cmd.setWr_Iorient(WristOrient);
  BodySchema_cmd.setTRAJTYPE(TrajType);
  BodySchema_cmd.setoidpmap(ObjIDEE[PlaceMapPos]);
  BodySchema_cmd.setrmPMP( RobotMovePMP);
  BodySchema_cmd.VTGSIN() = DoubleVector(6);

 
  //the rest of information will be replaced by correct numbers coming from Place Map (that is a event driven
  //working memory keeping track of what thngs are there and where they are in the world)

 /*  if(PMPGoalCode==122){
    BodySchema_cmd.VTGSIN().set(0,FuseePos2D[roboIndexRx][0]);
	BodySchema_cmd.VTGSIN().set(1,FuseePos2D[roboIndexRx][1]);
	BodySchema_cmd.VTGSIN().set(2,FuseePos2D[roboIndexRx][2]);
	BodySchema_cmd.VTGSIN().set(3,FuseePos2D[roboIndexTx][0]);
	BodySchema_cmd.VTGSIN().set(4,FuseePos2D[roboIndexTx][1]);
	BodySchema_cmd.VTGSIN().set(5,FuseePos2D[roboIndexTx][2]);
   }*/

    if(PMPGoalCode==122){
    BodySchema_cmd.VTGSIN().set(0,ComboCoordinate[0]);
	BodySchema_cmd.VTGSIN().set(1,ComboCoordinate[1]);
	BodySchema_cmd.VTGSIN().set(2,ComboCoordinate[2]);
	BodySchema_cmd.VTGSIN().set(3,ComboCoordinate[3]);
	BodySchema_cmd.VTGSIN().set(4,ComboCoordinate[4]);
	BodySchema_cmd.VTGSIN().set(5,ComboCoordinate[5]);
   }

   if(PMPGoalCode==140){
    BodySchema_cmd.VTGSIN().set(0,ComboCoordinate[0]);
	BodySchema_cmd.VTGSIN().set(1,ComboCoordinate[1]);
	BodySchema_cmd.VTGSIN().set(2,ComboCoordinate[2]);
	BodySchema_cmd.VTGSIN().set(3,ComboCoordinate[3]);
	BodySchema_cmd.VTGSIN().set(4,ComboCoordinate[4]);
	BodySchema_cmd.VTGSIN().set(5,ComboCoordinate[5]);
   }

  if(PMPGoalCode==1){
    BodySchema_cmd.VTGSIN().set(0,PlaceMap[PlaceMapPos][0]);
    cannotfindX=PlaceMap[PlaceMapPos][1];
    cannotfindXLoc=iCo-2;
    BodySchema_cmd.VTGSIN().set(1,PlaceMap[PlaceMapPos][1]);
    if(ObjIDEE[PlaceMapPos]==101)
    {
      //BodySchema_cmd.addDouble(PlaceMap[PlaceMapPos][2]+41);
      // Added a constant value for Z-Axis
      BodySchema_cmd.VTGSIN().set(2,38);
    }

    if(ObjIDEE[PlaceMapPos]==100)
    {
      BodySchema_cmd.VTGSIN().set(2,PlaceMap[PlaceMapPos][2]+95);//remember this
    }
    /*if(PIdentifier==3)
    {
    if(OIDinPM == 2)
    {
    BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+123);
    }
    else
    {
    BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+93);
    }
    }*/
  }
  if(PMPGoalCode==95){
    BodySchema_cmd.VTGSIN().set(0,LoCAlign[0]);
    BodySchema_cmd.VTGSIN().set(1,LoCAlign[1]);
    BodySchema_cmd.VTGSIN().set(2,LoCAlign[2]);
  }

   if(PMPGoalCode==111){
    BodySchema_cmd.VTGSIN().set(0,LoCAlign[0]);
    BodySchema_cmd.VTGSIN().set(1,LoCAlign[1]);
    BodySchema_cmd.VTGSIN().set(2,LoCAlign[2]);// height for insertion set constant to 85 from LoCAlign[2]
  }

  //     if(PMPGoalCode == 93){
  //BodySchema_cmd.addDouble(PlaceMap[PlaceMapPos][0]);
  //       std:://cout<<"LocaAlignFuse[0]: "<<PlaceMap[PlaceMapPos][0]<<std::endl;
  //BodySchema_cmd.addDouble(PlaceMap[PlaceMapPos][1]+5);
  //       std:://cout<<"LocaAlignFuse[1]: "<<PlaceMap[PlaceMapPos][1]+5<<std::endl;
  //BodySchema_cmd.addDouble(80);
  //       std:://cout<<"LocaAlignFuse[2]: "<<LoCAlignFusee[2]<<std::endl;
  //				}
  if(PMPGoalCode==19){
	  if(TrajType==1) //Change2005
	  {
		    cout<<"Entered Goal of Initialization with Trajectory Via Point " <<endl;
			Report<<"Entered Goal of Initialization with Trajectory Via Point " <<endl;
			BodySchema_cmd.VTGSIN().set(0,ViaPoint[0]);
			BodySchema_cmd.VTGSIN().set(1,ViaPoint[1]);
			BodySchema_cmd.VTGSIN().set(2,ViaPoint[2]);
			BodySchema_cmd.VTGSIN().set(3,XPosition[0]);
			BodySchema_cmd.VTGSIN().set(4,XPosition[1]);
			BodySchema_cmd.VTGSIN().set(5,XPosition[2]);
			cout<<"Entered Goal of Initialization X1,X2,X3 : "<<ViaPoint[0]<<" , "<<ViaPoint[1]<<" , "<<ViaPoint[2] <<endl;
		//	Report<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;
	  }
	  else{
    cout<<"Entered Goal of Initialization " <<endl;
    Report<<"Entered Goal of Initialization " <<endl;
    BodySchema_cmd.VTGSIN().set(0,XPosition[0]);
    BodySchema_cmd.VTGSIN().set(1,XPosition[1]);
    BodySchema_cmd.VTGSIN().set(2,XPosition[2]);
    cout<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;
  //  Report<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;
	  }
  }
  if(PMPGoalCode==21){
    BodySchema_cmd.VTGSIN().set(0,StaticLoc[0]);
    BodySchema_cmd.VTGSIN().set(1,StaticLoc[1]);
    BodySchema_cmd.VTGSIN().set(2,StaticLoc[2]);
  }
  //BodySchema_cmd.addDouble(0);
  //BodySchema_cmd.addDouble(0);
  //BodySchema_cmd.addDouble(0);
  // for ComputeTheta
  /*BodySchema_cmd.addDouble(PlaceMap[OIDinPM][0]);
  BodySchema_cmd.addDouble(PlaceMap[OIDinPM][1]);
  BodySchema_cmd.addDouble(PlaceMap[OIDinPM][3]);
  BodySchema_cmd.addDouble(PlaceMap[OIDinPM][4]);
  BodySchema_cmd.addDouble(PlaceMap[OIDinPM][6]);
  BodySchema_cmd.addDouble(PlaceMap[OIDinPM][7]);
  BodySchema_cmd.addDouble(PlaceMap[OIDinPM][9]);
  BodySchema_cmd.addDouble(PlaceMap[OIDinPM][10]);*/
  PMPKINCommand BodySchemaWrap;
  BodySchemaWrap.pmp(BodySchema_cmd);
  BodySchemaCtrlPort.write(BodySchemaWrap,BodySchema_response);

  cout<<"Sent a request to PMP Server with PMPGoalCode : "<<PMPGoalCode<<endl;
  Report<<"Sent a request to PMP Server with PMPGoal Code : "<<PMPGoalCode<<endl;
  printf("%s \n",BodySchema_response.toString().c_str());

  double Bresponsecode = BodySchema_response.flag();
  cout<<Bresponsecode<<endl;

  if(Bresponsecode == 221 /*COMMAND_VOCAB_REACH*/) {
    //cout << "Receiving status from Body Schema PMP server" << endl;
  //  Report << "Receiving status from Body Schema PMP server" << endl;
    //for(int i=0;i<10;i++)
    //{
    //  PMPresp[i]= BodySchema_response.get(i+1).asDouble();
    //  //cout << "Resp from PMP server" << PMPresp[i]<< endl;
    //}
    XPosition[0]=BodySchema_response.location().x();
    XPosition[1]=BodySchema_response.location().y();
    XPosition[2]=BodySchema_response.location().z();
    //cout<<" Forward model output of arm position"<< endl;
    //cout<< XPosition[0] << XPosition[1] << XPosition[2] << endl;
    if(BodySchema_response.result()==1){
      //cout << "Reached goal object sucessfully: wait for next goal from client" << endl;
      Report << "Reached goal object sucessfully: wait for next goal from client" << endl;
    }
	PMPresp[0]=BodySchema_response.result();
    if(BodySchema_response.result()==0){
      //cout << "Goal is not doable: need to form a updated plan with help of EPIM" << endl;
    }
  }
  returnval = BodySchema_response.result();
  Network::disconnect("/BodySchemaSim:io", "/pmpRX/PMPreply:io");
  }
  catch (std::exception& e) {
	  cout << e.what();
	  cout << "error in PrimBodySchema";
  }
  
  return returnval;
}

double ObserverThread::PrimBodySchema(int PMPGoalCode, int OIDinPM,int PIdentifier,int MsimFlag, int WristOrient, int TrajType){

  Network::connect("/BodySchemaSim:io", "/PMP/PMPreply:io");
  //cout << "Inside the PrimBodySchema : " << endl;
  Report << "Inside the PrimBodySchema : "  << endl;

  int iCo=0, iCocord=-1,cannotfindXLoc=100;
  cannotfindX=0;
  /*             for(int i=2;i<18;i=i+3)
  {
  if((PlaceMap[OIDinPM][i])>iCocord) //checks 2-5-8-11-14..
  {
  iCocord=PlaceMap[OIDinPM][i]; //z coordinate
  iCo=i;
  }
  }  */  //This is commented as I dont have 18 3D positions

  Bottle BodySchema_cmd, BodySchema_response;
  BodySchema_cmd.addVocab(COMMAND_VOCAB_CACT);
  BodySchema_cmd.addInt(PMPGoalCode); // Reach 1 or Initialize 19 or 21 Conitnuous reaching
  BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);
  //the rest of information will be replaced by correct numbers coming from Place Map (that is a event driven
  //working memory keeping track of what thngs are there and where they are in the world)
  BodySchema_cmd.clear();
  BodySchema_response.clear();
  if(PMPGoalCode==1){
    BodySchema_cmd.addVocab(COMMAND_VOCAB_GRIG);
    Bottle& Coordinates = BodySchema_cmd.addList();
    Coordinates.addDouble(PlaceMap[OIDinPM][0]);
    //cannotfindX=PlaceMap[OIDinPM][iCo-2];  This was earlier
    //cannotfindXLoc=iCo-2;
    Coordinates.addDouble(PlaceMap[OIDinPM][1]);
    Coordinates.addDouble(PlaceMap[OIDinPM][2]);
    BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

    BodySchema_cmd.clear();
    BodySchema_response.clear();
    BodySchema_cmd.addVocab(COMMAND_VOCAB_MSIM);
    BodySchema_cmd.addInt(MsimFlag);
    BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

    BodySchema_cmd.clear();
    BodySchema_response.clear();
    BodySchema_cmd.addVocab(COMMAND_VOCAB_WRIO);
    BodySchema_cmd.addInt(WristOrient);
    BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

    BodySchema_cmd.clear();
    BodySchema_response.clear();
    BodySchema_cmd.addVocab(COMMAND_VOCAB_TRAT);
    BodySchema_cmd.addInt(TrajType);
    BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);


    BodySchema_cmd.clear();
    BodySchema_response.clear();
    BodySchema_cmd.addVocab(COMMAND_VOCAB_REA);
    BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);
    /*	if(PIdentifier==3)
    {
    if(ObjectIDPMP == 2)
    {
    BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+123);
    }
    else
    {
    BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+93);
    }
    }*/ //This was for the mushroom Offset

  }

  if(PMPGoalCode==19){

    //cout<<"Entered Goal of Initialization " <<endl;
    Report<<"Entered Goal of Initialization " <<endl;
    /*BodySchema_cmd.addDouble(XPosition[0]);
    BodySchema_cmd.addDouble(XPosition[1]);
    BodySchema_cmd.addDouble(XPosition[2]);
    //cout<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;
    Report<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;*/
  }

  //continuous reachign through a via point
  if(PMPGoalCode==21){
    BodySchema_cmd.addDouble(StaticLoc[0]);
    BodySchema_cmd.addDouble(StaticLoc[1]);
    BodySchema_cmd.addDouble(StaticLoc[2]);
    BodySchema_cmd.addDouble(StaticLoc[3]);
    BodySchema_cmd.addDouble(StaticLoc[4]);
    BodySchema_cmd.addDouble(StaticLoc[5]);
  }
  //BodySchema_cmd.addDouble(0);
  //BodySchema_cmd.addDouble(0);
  //BodySchema_cmd.addDouble(0);
  //// for ComputeTheta
  //BodySchema_cmd.addDouble(PlaceMap[OIDinPM][0]);
  //BodySchema_cmd.addDouble(PlaceMap[OIDinPM][1]);
  //BodySchema_cmd.addDouble(PlaceMap[OIDinPM][3]);
  //BodySchema_cmd.addDouble(PlaceMap[OIDinPM][4]);
  //BodySchema_cmd.addDouble(PlaceMap[OIDinPM][6]);
  //BodySchema_cmd.addDouble(PlaceMap[OIDinPM][7]);
  //BodySchema_cmd.addDouble(PlaceMap[OIDinPM][9]);
  //BodySchema_cmd.addDouble(PlaceMap[OIDinPM][10]);
  BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

  //cout<<"Sent a request to PMP Server with PMPGoalCode : "<<PMPGoalCode<<endl;
  Report<<"Sent a request to PMP Server with PMPGoal Code : "<<PMPGoalCode<<endl;
  printf("%s \n",BodySchema_response.toString().c_str());

  double Bresponsecode = BodySchema_response.get(0).asDouble();
  //cout<<Bresponsecode<<endl;

  if(Bresponsecode == 221 /*COMMAND_VOCAB_REACH*/) {
    //cout << "Receiving status from Body Schema PMP server" << endl;
    Report << "Receiving status from Body Schema PMP server" << endl;
    for(int i=0;i<14;i++)
    {
      PMPresp[i]= BodySchema_response.get(i+1).asDouble();
      //cout << "Resp from PMP server" << PMPresp[i]<< endl;
    }
    XPosition[0]=PMPresp[1];
    XPosition[1]=PMPresp[2];
    XPosition[2]=PMPresp[3];
    //cout<<" Forward model output of arm position"<< endl;
    //cout<< XPosition[0] << XPosition[1] << XPosition[2] << endl;
    if(PMPresp[0]==1){
      //cout << "Reached goal object sucessfully: wait for next goal from client" << endl;
      Report << "Reached goal object sucessfully: wait for next goal from client" << endl;
    }

    if(PMPresp[0]==0){
      //cout << "Goal is not doable: need to form a updated plan with help of EPIM" << endl;
    }
  }
  return PMPresp[0];
}


double ObserverThread::PrimSearch(int obj1, int obj2, int goalidentity){

 ExpectedObjects& myObjs=objport.prepare();
 myObjs.setPast(0);
 myObjs.IDs().clear();
 if(GlobalWorkSpace[GWSPtr][8]==1)
 { 	
	 ////cout<<"sending 101"<<endl;
	myObjs.IDs().add(101);
 }
  if(GlobalWorkSpace[GWSPtr][9]==1)
 {
	 ////cout<<"sending 100"<<endl;
	myObjs.IDs().add(100);
 }

	if(!(Network::isConnected("/observer/expectedobjs:o"," /vision/expectedobjs:i"))){
		Network::connect("/observer/expectedobjs:o"," /vision/expectedobjs:i");
	}
	if(objport.getOutputCount()){
      objport.write(); // send
	}

 // RefreshPlacemap(); // connect to the bottle that brings in visual information and refresh the place map
  int iCo, jCo, niterFind=1, goalFind[2];
  double FindResp=0;
  if(goalidentity==68){
    goalFind[0]=obj1;
  }
  if (goalidentity==5){
    niterFind=2;
    goalFind[0]=obj1;
    goalFind[1]=obj2;
  }

  if((goalidentity==5)||(goalidentity==68)){
    for(iCo=0; iCo<niterFind; iCo++)
    {
      GetObjIDs[iCo]=50;
      for(jCo=0; jCo<NumObjectsinScene; jCo++)
      {
        if(ObjIDEE[jCo]==goalFind[iCo])
          GetObjIDs[iCo]=jCo;
      }
      if(niterFind==1)
      {
        if(GetObjIDs[iCo]!= 50)
        {
          FindResp=1;
          //cout <<"object is there" << endl;
        }
      }

      if(niterFind==2)
      {
        if(GetObjIDs[iCo]!= 50)
        {
          FindResp=FindResp+0.5;
          //cout <<"object" << iCo << "is there" << endl;
        }
      }

    }
  }

  if(goalidentity==95)
  {
    // u now need to transform all object ID's into learnt object hub activations to bring compositionlity and flexibility
    PlaceMapPos=10;
    //PlaceMapHub[0][0]=1; //this must come due to top down retroactivation from user goal given in proto language: now is blue driver
    //PlaceMapHub[0][6]=1;
    int SumOfGWS=0;
    double sumPh;
    for(iCo=0; iCo<NumObjectsinScene; iCo++)
    {
      sumPh=0;
      for(jCo=0; jCo<42; jCo++)
      {
        sumPh=sumPh+((GlobalWorkSpace[GWSPtr][jCo]-PlaceMapHub[iCo][jCo])*(GlobalWorkSpace[GWSPtr][jCo]-PlaceMapHub[iCo][jCo]));
      }
      if(sumPh==0)
      {
        //cout <<"object is there at PlaceMap Location" << iCo << endl;
        PlaceMapPos=iCo;
        if((ObjIDEE[iCo]==101))
        {
          //InitializeWorkingMemory(0);
          if(PlaceMap[PlaceMapPos][1]>0){
            PeriPersonalF=4;
          }
          if(PlaceMap[PlaceMapPos][1]<=0){
            PeriPersonalF=5;  //trigger RX for fuse
          }
			 /* if(numfuseFBoxe>1)
			  {*/
			     ChooseFuseBox();
			  /*}*/
        }

		if((ObjIDEE[iCo]==101)) //Changes2005...get the 3Dlocation of the Chosen Fuse
        {
          FuseObst[0]=PlaceMap[PlaceMapPos][0];
		  FuseObst[1]=PlaceMap[PlaceMapPos][1];
		  FuseObst[2]=PlaceMap[PlaceMapPos][2];
        }

		if((ObjIDEE[iCo]==100)) //Changes2005...get the 3Dlocation of the Chosen FuseBox
        {
          if(numfuseFBoxe>1)
		  {
			int chosenINDFB=VerifyFuseBox();
			PlaceMap[PlaceMapPos][0]=FBoxPos2D[chosenINDFB][0];
			PlaceMap[PlaceMapPos][1]=FBoxPos2D[chosenINDFB][1];
			PlaceMap[PlaceMapPos][2]=FBoxPos2D[chosenINDFB][2];
			PosHandOcc[0]=FBoxPos2D[chosenINDFB][0];
			PosHandOcc[1]=FBoxPos2D[chosenINDFB][1];
			PosHandOcc[2]=FBoxPos2D[chosenINDFB][2];
		  }
		}

        break;
      }
    }
    if(NumObjectsinScene==0)
    {
      for(jCo=0; jCo<42; jCo++)
      {
        SumOfGWS=SumOfGWS+(GlobalWorkSpace[GWSPtr][jCo]);
      }
      if(SumOfGWS>0)
      {
        sumPh=1;
      }
    }

    FindResp=0;

  }
  return FindResp;
}

int ObserverThread::VerifyFuseBox(){
  int chosenIndexFB=0;
  double xdiff_holeFB[10],ydiff_holeFB[10],zdiff_holeFB[10],distance_holeFB[10],minDistFB=0;
  for (int i = 0;i<numfuseFBoxe;i++){
    xdiff_holeFB[i] = (PosHandOcc[0]-FBoxPos2D[i][0]);
    ydiff_holeFB[i] = (PosHandOcc[1]-FBoxPos2D[i][1]);
    zdiff_holeFB[i] = (PosHandOcc[2]-FBoxPos2D[i][2]);
    distance_holeFB[i] = (sqrt(pow(xdiff_holeFB[i],2)+ pow(ydiff_holeFB[i],2)+pow(zdiff_holeFB[i],2)));
  }

  minDistFB=distance_holeFB[0];
  for (int i=1;i<numfuseFBoxe;i++){
    if (distance_holeFB[i]<minDistFB){
      minDistFB=distance_holeFB[i];
      chosenIndexFB=i;
    }

  }
  //cout<<"chosenIndexFB latest"<<"      "<< PosHandOcc[0]<<"latest update    "<<FBoxPos2D[chosenIndexFB][0]<<endl;
  return chosenIndexFB;
}

void ObserverThread::ChooseFuseBox()
	{
       int iCo,PPFB=0,fuseflag=-1;
	   int nfbx=numfuseFBoxe;
	     for(iCo=0; iCo<nfbx; iCo++)
			 {
				 PPFB=0;
				 if(FBoxPos2D[iCo][1]>0)
					 {
						PPFB=4;
					 }
				  if(FBoxPos2D[iCo][1]<=0)
					 {
						PPFB=5;
					 }
				  if(PPFB-PeriPersonalF==0)//chosse the fuse box that is in the same peripersonal side
					  {
				         //cout<<"Desired fuse box Chosen"<<iCo<<endl;
						 PeriPersonalFB=PPFB;
						 PosHandOcc[0]=FBoxPos2D[iCo][0];
						 PosHandOcc[1]=FBoxPos2D[iCo][1];
						 PosHandOcc[2]=FBoxPos2D[iCo][2];
						 fuseflag=5;
						 iCo=iCo+10;
					  }
				 }
		 if((fuseflag<0)&&(numfuseFBoxe>0))
		 {
		   //cout<<"No fuse box in the same peripersonal space"<<iCo<<endl;
		   PosHandOcc[0]=FBoxPos2D[0][0];
		   PosHandOcc[1]=FBoxPos2D[0][1];
		   PosHandOcc[2]=FBoxPos2D[0][2];
		   if(FBoxPos2D[0][1]>0)
					 {
						PeriPersonalFB=4;
					 }
				  if(FBoxPos2D[0][1]<=0)
					 {
						PeriPersonalFB=5;
					 }
		 }

	};


int ObserverThread::PrimGrasp(GraspTypeType GraspReq,int BodyChain){
  Network::connect("/observer/GraspCtrl:io","/grasp/command:i");
  GraspCommand Grasp_cmd; //sending and receiving message formats changed by Kris
  switch (BodyChain) {
  case 0://left arm
    Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_LEFT).settype(GraspReq));
    break;
  case 1://right arm
    Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_RIGHT).settype(GraspReq));
    break;
  case 2://whole body
    Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_LEFT).settype(GraspReq));
    Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_RIGHT).settype(GraspReq));
    break;
  default:
    //cout<< "PrPrimGraspimGrasp: expecting other value for body side - use BodyChain" <<BodyChain <<endl;
    Report<< "PrimGrasp: expecting other value for body side - use BodyChain" <<BodyChain<<endl;
  }
  //Bottle Grasp_cmd, Grasp_response; //commented by Kris
  //Grasp_cmd.addInt(RobID);
  //Grasp_cmd.addInt(GraspReq);
  GraspCommand& Grasp_write = GraspPort.prepare();
  Grasp_write = Grasp_cmd;
  GraspPort.write();

  int graspcommandssent = Grasp_cmd.size();
  VectorBottle<GraspResult> tempgraspresponse;

  while (graspcommandssent > 0) {
	  if (GraspResultPort.getPendingReads()) {
		  tempgraspresponse.add(*(GraspResultPort.read()));
		  --graspcommandssent;
	  }
  }

  printf("%s \n",tempgraspresponse.toString().c_str());

  //int Gresponsecode = Grasp_response.get(0).asInt();
  GraspResultType Gresponsecode = static_cast<GraspResultType>(tempgraspresponse[0].result());
  //cout<< "Reply recd from Grasp server" << Gresponsecode<<endl;
  Report<< "Reply recd from Grasp server" << Gresponsecode<<endl;
  return Gresponsecode;
}

int ObserverThread::PrimGraspIndustrial(GraspTypeType GraspReq,int BodyChain){
	int returnval = 0;
	try {
						if(!(Network::isConnected(GraspPort.getName().c_str(),"/grasp/command:i"))){
							Network::connect(GraspPort.getName().c_str(),"/grasp/command:i");
						}
						if(!(Network::isConnected("/grasp/outcome:o",GraspResultPort.getName().c_str()))){
							Network::connect("/grasp/outcome:o",GraspResultPort.getName().c_str());
						}

						GraspCommand Grasp_cmd; //sending and receiving message formats changed by Kris
						switch (BodyChain) {
						case 0://left arm
							Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_LEFT).settype(GraspReq));
							break;
						case 1://right arm
							Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_RIGHT).settype(GraspReq));
							break;
						case 2://whole body
							Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_LEFT).settype(GraspReq));
							Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_RIGHT).settype(GraspReq));
							break;
						default:
							//cout<< "PrimGrasp: expecting other value for body side - use BodyChain" <<BodyChain <<endl;
							cout << "PrimGrasp: expecting other value for body side - use BodyChain" <<BodyChain<<endl;
						}
						//Bottle Grasp_cmd, Grasp_response; //commented by Kris
						//Grasp_cmd.addInt(RobID);
						//Grasp_cmd.addInt(GraspReq);
						GraspTimeStamp.update();
						GraspCommand& Grasp_write = GraspPort.prepare();
						  Grasp_write = Grasp_cmd;
						  GraspPort.setEnvelope(GraspTimeStamp);
						  GraspPort.write();

						  int graspcommandssent = Grasp_cmd.size();
						  VectorBottle<GraspResult> tempgraspresponse;

						  while (graspcommandssent > 0) {
							  if (GraspResultPort.getPendingReads()) {
								  //make sure to read the timestamp
								  tempgraspresponse.add(*(GraspResultPort.read()));
								  --graspcommandssent;
							  }
						  }
						printf("%s \n",tempgraspresponse.toString().c_str());

  //int Gresponsecode = Grasp_response.get(0).asInt();
  GraspResultType Gresponsecode = static_cast<GraspResultType>(tempgraspresponse[0].result());
  //cout<< "Reply recd from Grasp server" << Gresponsecode<<endl;
  Report<< "Reply recd from Grasp server" << Gresponsecode<<endl;
  
   int GrRead=-1; //TODO: Deal with multiple response from grasp module
  switch(Gresponsecode)
          {
			  case GRASP_SUCC:
				{
				  GrRead = 1;
				}
				break;
			  case GRASP_FAIL:
			 {
				 GrRead=0;
			 //Do something
			 }
			 break;
			                  
          }
	returnval = GrRead;
	}
	 catch (std::exception& e) {
	  cout << e.what();
	  cout << "error in PrimGraspIndustrial";
  }
//int returnval=1;
return returnval;

}

//int ObserverThread::PrimGraspIndustrial(GraspTypeType GraspReq,int BodyChain){
//   
//						if(!(Network::isConnected("/GraspCtrl:io","/grasp/command:i"))){
//							Network::connect("/GraspCtrl:io","/grasp/command:i");
//						}
//						GraspCommand Grasp_cmd; //sending and receiving message formats changed by Kris
//						GraspResult Grasp_response;
//						switch (BodyChain) {
//						case 0://left arm
//							Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_LEFT).settype(GraspReq));
//							break;
//						case 1://right arm
//							Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_RIGHT).settype(GraspReq));
//							break;
//						case 2://whole body
//							Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_LEFT).settype(GraspReq));
//							Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_RIGHT).settype(GraspReq));
//							break;
//						default:
//							//cout<< "PrimGrasp: expecting other value for body side - use BodyChain" <<BodyChain <<endl;
//							Report<< "PrimGrasp: expecting other value for body side - use BodyChain" <<BodyChain<<endl;
//						}
//						//Bottle Grasp_cmd, Grasp_response; //commented by Kris
//						//Grasp_cmd.addInt(RobID);
//						//Grasp_cmd.addInt(GraspReq);
//						GraspPort.write(Grasp_cmd,Grasp_response);
//						printf("%s \n",Grasp_response.toString().c_str());
//
//  if(!(Network::isConnected("/GraspCtrl:io","/grasp/command:i")))
//  {
//    Network::connect("/GraspCtrl:io","/grasp/command:i");
//  }
////  GraspCommand Grasp_cmd; //sending and receiving message formats changed by Kris
//  //GraspResult Grasp_response;
//  switch (BodyChain) {
//  case 0://left arm
//    Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_LEFT).settype(GraspReq));
//    break;
//  case 1://right arm
//    Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_RIGHT).settype(GraspReq));
//    break;
//  case 2://whole body
//    Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_LEFT).settype(GraspReq));
//    Grasp_cmd.add(GraspDescriptor().seteffector(GRASP_RIGHT).settype(GraspReq));
//    break;
//  default:
//    //cout<< "PrimGrasp: expecting other value for body side - use BodyChain" <<BodyChain <<endl;
//    Report<< "PrimGrasp: expecting other value for body side - use BodyChain" <<BodyChain<<endl;
//  }
//  //Bottle Grasp_cmd, Grasp_response; //commented by Kris
//  //Grasp_cmd.addInt(RobID);
//  //Grasp_cmd.addInt(GraspReq);
//  GraspPort.write(Grasp_cmd,Grasp_response);
//  printf("%s \n",Grasp_response.toString().c_str());
//
//  //int Gresponsecode = Grasp_response.get(0).asInt();
//  GraspResultType Gresponsecode = static_cast<GraspResultType>(Grasp_response[0]);
//  //cout<< "Reply recd from Grasp server" << Gresponsecode<<endl;
//  Report<< "Reply recd from Grasp server" << Gresponsecode<<endl;
//   int GrRead=-1;
//  switch(Gresponsecode)
//          {
//			  case GRASP_SUCC:
//				{
//				  GrRead = 1;
//				}
//				break;
//			  case GRASP_FAIL:
//			 {
//				 GrRead=0;
//			 //Do something
//			 }
//			 break;
//			                  
//          }
//  
//  return GrRead;
//
//}


double ObserverThread::RefreshPlacemap(){
  //bool flag = true;
  fuseBoxPos=99;
  //while(flag)
  //{
  for(int i=0;i<10;i++){
    LoCAlignFuseeD[i][0]=9999;
    LoCAlignFuseeD[i][1]=9999;
    LoCAlignFuseeD[i][2]=9999;
	FBoxPos2D[i][0]=9999;
    FBoxPos2D[i][1]=9999;
    FBoxPos2D[i][2]=9999;
	FuseePos2D[i][0]=9999;
    FuseePos2D[i][1]=9999;
    FuseePos2D[i][2]=9999;
	}


  //Network::connect("/SmallWorldsOPC:io", "/OPCServer:io");
  //

  ////Network::connect("/observer/taskAffordance:o","/affordance/task:i"); FROM KRIS: DON'T DO THIS, AFFORDANCEACCESS TAKES CARE OF THIS
  //Bottle OPC_cmd, OPC_Response;

  //OPC_cmd.addVocab(COMMAND_VOCAB_FIND);
  //OPCCtrlPort.write(OPC_cmd,OPC_Response);
  

 // IntVector filled;
 // filled.clear();
 // filled.add(0);
//  ActionSequence sequence = affordanceAccess->insert(GRASP_LEFT,fuse, fusebox,hole,filled);

 if(! Network::connect("/vision/objects:o", "/observer/VisionScene:i"))
 {
	 Network::connect("/vision/objects:o", "/observer/VisionScene:i");
 }
  
  if (VScene.getInputCount())
	{
		
		scene=VScene.read(true);
		std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
		std::cout<<scene->toString()<<std::endl;
		std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
		scene1=*scene;
	}
  //printf("%s \n",OPC_Response.toString().c_str());




  /*TaskAffordance task; //NOTE FROM KRIS: DON'T DO THIS HERE, USE THE AFFORDANCEACCESS FUNTION "INSERT"

	InsertAffordance insert;
	std::vector<int> filled (3,0);
	IntVector test;
	test.clear();
test.add(1);
	insert.seteffector(GRASP_LEFT);
	insert.Object() = (*scene)[1];
	insert.Into() = (*scene)[0];
	insert.setHole(0);
	insert.Filled() = test;
	task.inst(insert);

	TaskAffordance& sendtask = taskPort.prepare();
					sendtask = task;
					taskPort.write(true);*/
				//	Time::delay(10);


/////////////////////////////////////////
	NumberofObs = (int) scene->size();
    NumObjectsinScene=NumberofObs;
	int ctrr;

	for(int i=0; i<scene->size(); ++i)
	{
		ctrr=0;
		VisualObject& obj = (*scene)[i];
		 ObjIDEE[i]=obj.ID();
		//std::cout << obj.toString() << std::endl; 
		if(ObjIDEE[i]==101){
			int nInsert=obj.InsertTip().size();
			Graspability[i]=nInsert;
			for(int j=0; j<nInsert; j++)
			{
				Eighteens[i][ctrr]=obj.InsertTip()[j].x();
				Eighteens[i][ctrr+1]=obj.InsertTip()[j].y();
				Eighteens[i][ctrr+2]=obj.InsertTip()[j].z();
				ctrr=ctrr+3;
			}
		}
		if(ObjIDEE[i]==201){
			int nInsert=obj.InsertTip().size();
			Graspability[i]=nInsert;
			for(int j=0; j<nInsert; j++)
			{
				Eighteens[i][ctrr]=obj.InsertTip()[j].x();
				Eighteens[i][ctrr+1]=obj.InsertTip()[j].y();
				Eighteens[i][ctrr+2]=obj.InsertTip()[j].z();
				ctrr=ctrr+3;
			}
		}
		if(ObjIDEE[i]==100){
			//int nInsert=0;
			Graspability[i]=1;

			Eighteens[i][ctrr]=obj.Hole()[0].Center().x();
			Eighteens[i][ctrr+1]=obj.Hole()[0].Center().y();
			Eighteens[i][ctrr+2]=obj.Hole()[0].Center().z();
			Eighteens[i][ctrr+3]=obj.Hole()[1].Center().x();
			Eighteens[i][ctrr+4]=obj.Hole()[1].Center().y();
			Eighteens[i][ctrr+5]=obj.Hole()[1].Center().z();
			Eighteens[i][ctrr+6]=obj.Hole()[2].Center().x();
			Eighteens[i][ctrr+7]=obj.Hole()[2].Center().y();
			Eighteens[i][ctrr+8]=obj.Hole()[2].Center().z();
			ctrr=ctrr+9;

		}
		if(ObjIDEE[i]==200){
			//int nInsert=0;
			Graspability[i]=1;

			Eighteens[i][ctrr]=obj.Hole()[0].Center().x();
			Eighteens[i][ctrr+1]=obj.Hole()[0].Center().y();
			Eighteens[i][ctrr+2]=obj.Hole()[0].Center().z();
			Eighteens[i][ctrr+3]=obj.Hole()[1].Center().x();
			Eighteens[i][ctrr+4]=obj.Hole()[1].Center().y();
			Eighteens[i][ctrr+5]=obj.Hole()[1].Center().z();
			Eighteens[i][ctrr+6]=obj.Hole()[2].Center().x();
			Eighteens[i][ctrr+7]=obj.Hole()[2].Center().y();
			Eighteens[i][ctrr+8]=obj.Hole()[2].Center().z();
			Eighteens[i][ctrr+9]=obj.Hole()[3].Center().x();
			Eighteens[i][ctrr+10]=obj.Hole()[3].Center().y();
			Eighteens[i][ctrr+11]=obj.Hole()[3].Center().z();
			ctrr=ctrr+12;

		}

		XlatTrackP[i]=ObjIDEE[i];
      XlatTrackPl[i]=ObjIDEE[i];

	}


	int SerialHH=0;
    numfusee=0;
	RealFusee=0;
	numfuseFBoxe=0;
    //=============================================
    for(int i=0;i<NumberofObs;i++)
    {
      Report << "place map coordinates of object " << i << endl;
      for(int j=0;j<Graspability[i]*12;j++)
      {
        PlaceMap[i][j]=Eighteens[i][j];
        Report << PlaceMap[i][j]<< endl;

      }
     
   if(ProvHub[32]>=1){
	  holeCount=3;
	 if((ObjIDEE[i]==101))
        	{
              FuseePos2D[RealFusee][0]=PlaceMap[i][0]+8;
			  FuseePos2D[RealFusee][1]=PlaceMap[i][1];
			  FuseePos2D[RealFusee][2]=38;//PlaceMap[i][2]
        	  RealFusee=RealFusee+1;
			  numfusee=numfusee+1; //vm2011
        	}
	 
	 if((ObjIDEE[i]==100))  //at present we are only taking care of pushing while reaching and not aligning: this can be added by making some changes to scene change function
        {
          FBoxPos2D[numfuseFBoxe][0]=PlaceMap[i][0]+8;
		  FBoxPos2D[numfuseFBoxe][1]=PlaceMap[i][1];
		  FBoxPos2D[numfuseFBoxe][2]=PlaceMap[i][2];
		  FBoxPos2D[numfuseFBoxe][3]=PlaceMap[i][3]+8;
		  FBoxPos2D[numfuseFBoxe][4]=PlaceMap[i][4];
		  FBoxPos2D[numfuseFBoxe][5]=PlaceMap[i][5];
		  FBoxPos2D[numfuseFBoxe][6]=PlaceMap[i][6]+8;
		  FBoxPos2D[numfuseFBoxe][7]=PlaceMap[i][7];
		  FBoxPos2D[numfuseFBoxe][8]=PlaceMap[i][8];

		  if(numfuseFBoxe==0)
			  {
				  FuseBoXPos=i;
		  }
		  numfuseFBoxe=numfuseFBoxe+1; //Changes2205
		  if(isFBinScene==14)
		  {
		    isFBinScene=1; //if this is 1 means there was a FB in the scene in the past
		  }
        }
	JJSh=0;
	for (int i = 0; i<numfuseFBoxe;i++)
	{
		SerialHole[JJSh][0]=FBoxPos2D[i][0];
		SerialHole[JJSh][1]=FBoxPos2D[i][1];
		SerialHole[JJSh][2]=FBoxPos2D[i][2];
		SerialHole[JJSh+1][0]=FBoxPos2D[i][3];
		SerialHole[JJSh+1][1]=FBoxPos2D[i][4];
		SerialHole[JJSh+1][2]=FBoxPos2D[i][5];
		SerialHole[JJSh+2][0]=FBoxPos2D[i][6];
		SerialHole[JJSh+2][1]=FBoxPos2D[i][7];
		SerialHole[JJSh+2][2]=FBoxPos2D[i][8];
		JJSh=JJSh+3;
	}
   }

    if(ProvHub[23]>=1){
	holeCount=4;
	 if((ObjIDEE[i]==201))
        	{
              FuseePos2D[RealFusee][0]=PlaceMap[i][0]+8;
			  FuseePos2D[RealFusee][1]=PlaceMap[i][1];
			  FuseePos2D[RealFusee][2]=38;//PlaceMap[i][2]
        	  RealFusee=RealFusee+1;
			  numfusee=numfusee+1; //vm2011
        	}
	 
	 if((ObjIDEE[i]==200))  //at present we are only taking care of pushing while reaching and not aligning: this can be added by making some changes to scene change function
        {
          FBoxPos2D[numfuseFBoxe][0]=PlaceMap[i][0]+8;
		  FBoxPos2D[numfuseFBoxe][1]=PlaceMap[i][1];
		  FBoxPos2D[numfuseFBoxe][2]=PlaceMap[i][2];
		  FBoxPos2D[numfuseFBoxe][3]=PlaceMap[i][3]+8;
		  FBoxPos2D[numfuseFBoxe][4]=PlaceMap[i][4];
		  FBoxPos2D[numfuseFBoxe][5]=PlaceMap[i][5];
		  FBoxPos2D[numfuseFBoxe][6]=PlaceMap[i][6]+8;
		  FBoxPos2D[numfuseFBoxe][7]=PlaceMap[i][7];
		  FBoxPos2D[numfuseFBoxe][8]=PlaceMap[i][8];
		  FBoxPos2D[numfuseFBoxe][9]=PlaceMap[i][9]+8;
		  FBoxPos2D[numfuseFBoxe][10]=PlaceMap[i][10];
		  FBoxPos2D[numfuseFBoxe][11]=PlaceMap[i][11];

		  if(numfuseFBoxe==0)
			  {
				  FuseBoXPos=i;
		  }
		  numfuseFBoxe=numfuseFBoxe+1; //Changes2205
		  if(isFBinScene==14)
		  {
		    isFBinScene=1; //if this is 1 means there was a FB in the scene in the past
		  }
        }
	JJSh=0;
	for (int i = 0; i<numfuseFBoxe;i++)
	{
		SerialHole[JJSh][0]=FBoxPos2D[i][0];
		SerialHole[JJSh][1]=FBoxPos2D[i][1];
		SerialHole[JJSh][2]=FBoxPos2D[i][2];
		SerialHole[JJSh+1][0]=FBoxPos2D[i][3];
		SerialHole[JJSh+1][1]=FBoxPos2D[i][4];
		SerialHole[JJSh+1][2]=FBoxPos2D[i][5];
		SerialHole[JJSh+2][0]=FBoxPos2D[i][6];
		SerialHole[JJSh+2][1]=FBoxPos2D[i][7];
		SerialHole[JJSh+2][2]=FBoxPos2D[i][8];
		SerialHole[JJSh+3][0]=FBoxPos2D[i][9];
		SerialHole[JJSh+3][1]=FBoxPos2D[i][10];
		SerialHole[JJSh+3][2]=FBoxPos2D[i][11];
		JJSh=JJSh+4;
	}
   }
	  //changes2205..
   /*   if((AlignFlag==1)&&(ObjIDEE[i]==100))
      {
        LoCAlign[0]=PlaceMap[i][0];
        LoCAlign[1]=PlaceMap[i][1];
        LoCAlign[2]=PlaceMap[i][2];
      }*/ // FuseBox is the chosen one
	      if((AlignFlag==1)&&(ObjIDEE[i]==100))
		  {
			LoCAlign[0]=PosHandOcc[0];
			LoCAlign[1]=PosHandOcc[1];
			LoCAlign[2]=PosHandOcc[2];
		  }
	  
      if((AlignFlag==1)&&(ObjIDEE[i]==101))
      {
        LoCAlignFuseeD[numfusee][0]=PlaceMap[i][0];
        LoCAlignFuseeD[numfusee][1]=PlaceMap[i][1];
        LoCAlignFuseeD[numfusee][2]=PlaceMap[i][2];
       // numfusee=numfusee+1;
      }

    }
	
    //==============================================================================================

 
  if (numfusee>JJSh){
		minObjectsCount=JJSh;
  }
  else{
	  minObjectsCount=numfusee;
  }
  Xlator();
  FusePosession();
  //}
 // Network::disconnect("/SmallWorldsOPC:io", "/OPCServer:io");
  return NumberofObs;

}

VisualObject ObserverThread::findFuseInScene( VisualScene* sceneIn,double x, double y,double threshold)
{
	bool found = false;
	std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
	std::cout<<"Finding Fuse"<<std::endl;
	std::cout<<sceneIn->toString()<<std::endl;
	std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;

	VisualObject testfuse;// = new darwin::msg::VisualObject();

	if(sceneIn->size() > 0)
	{
		for(int i=0; i< sceneIn->size(); ++i){

			VisualObject& obj = (*sceneIn)[i];
			if ((obj.ID()==101)&& (holeCount==3)){
				double tempx = obj.InsertTip()[0].x()+8;
				double tempy = obj.InsertTip()[0].y();

				if( (abs(x-tempx)<threshold) && (abs(y-tempy)<threshold) ) {
					found=true;
					
					std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
					std::cout<<"Empty  Fuse"<<std::endl;
					std::cout<<testfuse.toString()<<std::endl;
					std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
					testfuse = obj;
					std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
					std::cout<<"Filled Fuse"<<std::endl;
					std::cout<<testfuse.toString()<<std::endl;
					std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
					return (testfuse);
				}
			}
			if ((obj.ID()==201) && (holeCount==4)){
				double tempx = obj.InsertTip()[0].x()+8;
				double tempy = obj.InsertTip()[0].y();

				if( (abs(x-tempx)<threshold) && (abs(y-tempy)<threshold) ) {
					found=true;
					
					std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
					std::cout<<"Empty  Fuse"<<std::endl;
					std::cout<<testfuse.toString()<<std::endl;
					std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
					testfuse =obj;
					std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
					std::cout<<"Filled Fuse"<<std::endl;
					std::cout<<testfuse.toString()<<std::endl;
					std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
					return (testfuse);
				}
			}
		}
	}

	else
	{
		found = false;
	}
	if (!found)
	{

		
		std::cout<<"Fuse needed for affordance not found"<<std::endl;		
		std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
		std::cout<<"TEMP Fuse"<<std::endl;
		Coordinates3D pt;
		testfuse.setID(9999);
		pt.setx(9999)
		  .sety(9999)
		  .setz(9999);
		testfuse.InsertTip().add(pt);
		std::cout<<testfuse.toString()<<std::endl;
		std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
		return(testfuse);
	}
}
VisualObject ObserverThread::findFuseboxInScene( VisualScene * sceneIn,double x, double y)
{
	activeHole = -1;
	bool found = false;
	std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
	std::cout<<"Finding FuseBox"<<std::endl;
	std::cout<<sceneIn->toString()<<std::endl;
	std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
	VisualObject testfusebox;// = new darwin::msg::VisualObject();
	if(sceneIn->size() > 0)
	{
		for(int i=0; i< sceneIn->size(); ++i){


			VisualObject& obj = (*sceneIn)[i];
			if ((obj.ID()==100) && (holeCount==3)){
				for(int j=0;j<3;j++){
					double tempx = obj.Hole()[j].Center().x()+8;
					double tempy = obj.Hole()[j].Center().y();
					if( (abs(x-tempx)<15) && (abs(y-tempy)<15) ) 
					{
						activeHole=j;
						found=true;
						std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
						std::cout<<"Empty  testfusebox"<<std::endl;
						std::cout<<testfusebox.toString()<<std::endl;
						std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
						testfusebox = obj;
						std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
						std::cout<<"Filled testfusebox"<<std::endl;
						std::cout<<testfusebox.toString()<<std::endl;
						std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
						return(testfusebox);
					}
				}
			}
			if ((obj.ID()==200) && (holeCount==4)){
				for(int j=0;j<4;j++){
					double tempx = obj.Hole()[j].Center().x()+8;
					double tempy = obj.Hole()[j].Center().y();
					if( (abs(x-tempx)<15) && (abs(y-tempy)<15) ) 
					{
						activeHole=j;
						found=true;
						std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
						std::cout<<"Empty  testfusebox"<<std::endl;
						std::cout<<testfusebox.toString()<<std::endl;
						std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
						testfusebox = obj;
						std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
						std::cout<<"Filled testfusebox"<<std::endl;
						std::cout<<testfusebox.toString()<<std::endl;
						std::cout<<":::::::::::::::::::::::::::::::::::::::::"<<std::endl;
						return(testfusebox);
					}
				}
			}



		}
	}

	else
	{
		found = false;
	}
	if (!found)
	{
		std::cout<<"Fusebox needed for affordance not found"<<std::endl;
		Coordinates3D pt;
		testfusebox.setID(9999);
		pt.setx(9999)
		  .sety(9999)
		  .setz(9999);
		testfusebox.InsertTip().add(pt);
		return(testfusebox);
	}
}

int ObserverThread::chooseFromSequence(ActionSequence sequence)
{
	std::cout<<":::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
   std::cout<<sequence.toString()<<std::endl;
   std::cout<<":::::::::::::::::::::::::::::::::::::::::::"<<std::endl;

   int retrunindex = -1;
   for(int i=0;i<sequence.size();i++)
   {

	   if (sequence[i].isrch() && sequence[i].rch().iskin() && sequence[i].rch().kin().size() && sequence[i].rch().kin()[0].context() == CONTEXT_INSERT) 
	   {
		   retrunindex = i;

		   std::cout<<":::::::::::::::::::::::::::"<<std::endl;
		   std::cout<<"Found the insertion context in reach type"<<std::endl;
		   std::cout<<"THe index is "<<retrunindex<<std::endl;
		   std::cout<<":::::::::::::::::::::::::::"<<std::endl;

	   }
    
   }
   std::cout<<"Sequence size is "<<sequence.size()<<std::endl;
   return retrunindex;
}

int ObserverThread::writeToKinematics(ReachCommand &kinreach)
{
	int returnval = 0;
	try {
		Network::connect("/BodySchemaSim:io", "/pmpRX/PMPreply:io");
		PMPResult kin_response;
		ReachCommand rchWrite = kinreach;

		PMPKINCommand KinSchemaWrap;
		KinSchemaWrap.kin(rchWrite);

		std::cout<<"Writing this from kinematics : "<<std::endl;
		std::cout<<"::::::::::::::::::::::::::::::::::::::::"<<std::endl;
		std::cout<<"::::::::::::::::::::::::::::::::::::::::"<<std::endl;
		std::cout<<KinSchemaWrap.toString()<<std::endl;
		std::cout<<"::::::::::::::::::::::::::::::::::::::::"<<std::endl;
		std::cout<<"::::::::::::::::::::::::::::::::::::::::"<<std::endl;
		BodySchemaCtrlPort.write(KinSchemaWrap,kin_response);


		printf("%s \n",kin_response.toString().c_str());

		double Bresponsecode = kin_response.flag();
		cout<<Bresponsecode<<endl;

		if(Bresponsecode == 221 /*COMMAND_VOCAB_REACH*/) {
			XPosition[0]=kin_response.location().x();
			XPosition[1]=kin_response.location().y();
			XPosition[2]=kin_response.location().z();
			//cout<<" Forward model output of arm position"<< endl;
			//cout<< XPosition[0] << XPosition[1] << XPosition[2] << endl;
			if(kin_response.result()==1){
				//cout << "Reached goal object sucessfully: wait for next goal from client" << endl;
				Report << "Reached goal object sucessfully: wait for next goal from client" << endl;
			}
			PMPresp[0]=kin_response.result();
			if(kin_response.result()==0){
				//cout << "Goal is not doable: need to form a updated plan with help of EPIM" << endl;
			}
		}
		returnval = kin_response.result();
		Network::disconnect("/BodySchemaSim:io", "/pmpRX/PMPreply:io");
	}
	catch (std::exception& e) {
		cout << e.what();
		cout << "error in PrimBodySchema";
	}

	return returnval;
}

void ObserverThread::Restorer()
{
	int index=-1;
	int sceneCalled=0;
	int holeNum=0;
	scene2=&scene1;

	std::cout<<"Printing the scene2 "<<std::endl;
	std::cout<<":::::::::::::::::::::::::::::::::::::::: "<<std::endl;
	std::cout<<scene2->toString()<<std::endl;
	std::cout<<":::::::::::::::::::::::::::::::::::::::: "<<std::endl;

	/*while(index==-1)
	{
		std::cout<<"SIZE IS "<<scene2->size()<<std::endl;


		if(scene2->size() <= 0)
		{
			index = -1;
		}

		else
		{
			for(int i=0; i< scene2->size(); ++i)
			{
				
				VisualObject& obj = (*scene2)[i];
				std::cout<<"OBJECT IS  "<<obj.toString()<<std::endl;
				if (obj.ID()==100)
				{
					index = i;
				}
			}
		}

		if(sceneCalled>3)
			{
				break;
			}

		if (index==-1){
			if (VScene.getInputCount())
			{	
				Time::delay(2);
				scene2=VScene.read(true);
				//std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
				//std::cout<<scene2->toString()<<std::endl;
				//std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
			}
			else
			{
				std::cout<<"No Vision data found; Is the vision running?"<<std::endl;
			}
			sceneCalled++;
		}
	}*/


	if(scene2->size() <= 0)
	{
		index = -1;
	}

	else
	{
		for(int i=0; i< scene2->size(); ++i)
		{
				
			VisualObject& obj = (*scene2)[i];
			std::cout<<"OBJECT IS  "<<obj.toString()<<std::endl;
			if ((obj.ID()==100) && (holeCount==3))
			{
				index = i;
			}
			if ((obj.ID()==200) && (holeCount==4))
			{
				index = i;
			}
		}
	}
	///////////
	if(index == -1)
	{
			std::cout<<"Could not find a valid hole to send to affordance module"<<std::endl;
			std::cout<<"Nothing to insert into; exiting function"<<std::endl;
			return;
	}
	////////////

	IntVector filled;
	filled.clear();
	if(holeCount==3)
	{

		if(holeNum==0)
		{
			filled.add(1);
			filled.add(2);
		}
		if(holeNum==1)
		{
			filled.add(0);
			filled.add(2);
		}
		if(holeNum==2)
		{
			filled.add(0);
			filled.add(1);
		}
	}

	if(holeCount==4)
	{
		if(holeNum==0)
		{
			filled.add(1);
			filled.add(2);
			filled.add(3);
		}
		if(holeNum==1)
		{
			filled.add(0);
			filled.add(2);
			filled.add(3);
		}
		if(holeNum==2)
		{
			filled.add(0);
			filled.add(1);
			filled.add(3);
		}
		if(holeNum==3)
		{
			filled.add(0);
			filled.add(1);
			filled.add(2);
		}
	}

	ActionSequence sequence;
	
	if (RobotMovePMP==1)
	{

		VisualObject fuseTXtemp;// = new VisualObject();
		std::cout<<"fuseTXtemp Empty"<<std::endl;
		std::cout<<fuseTXtemp.toString()<<std::endl;

		fuseTXtemp = findFuseInScene( &scene1,ComboCoordinate[3], ComboCoordinate[4],20);

		std::cout<<"fuseTXtemp After function"<<std::endl;
		std::cout<<fuseTXtemp.toString()<<std::endl;

		if(!(fuseTXtemp.ID() == 9999))
			sequence = affordanceAccess->insert(GRASP_LEFT,fuseTXtemp,(*scene2)[index],holeNum,filled);
		else
			std::cout<<"Could not generate valid insert affordance"<<std::endl;
	}
	if (RobotMovePMP==-1)
	{
		VisualObject fuseRXtemp;// = new VisualObject();
		std::cout<<"fuseRXtemp Empty"<<std::endl;
		std::cout<<fuseRXtemp.toString()<<std::endl;

		fuseRXtemp = findFuseInScene( &scene1,ComboCoordinate[0], ComboCoordinate[1],20);

		std::cout<<"fuseRXtemp After function"<<std::endl;
		std::cout<<fuseRXtemp.toString()<<std::endl;

		std::cout<<fuseRXtemp.toString()<<std::endl;
		if(!(fuseRXtemp.ID() == 9999))
			sequence = affordanceAccess->insert(GRASP_RIGHT,fuseRXtemp,(*scene2)[index],holeNum,filled);
		else
			std::cout<<"Could not generate valid insert affordance"<<std::endl;
	}
	bool testlying =false;
	if(!(sequence.size() == 0)){
		FuseOrient=sequence.size();
		for(int i=0;i<FuseOrient;++i)
		{
			if(sequence[i].isrch() && sequence[i].rch().iskin() && sequence[i].rch().kin()[0].approach() == REACH_CAMERA_SIDE)
			{
				testlying = true;
			}
		}
	}
	
	if(testlying)
	{
		for (int i=2;i<FuseOrient;i++)
		{

			 if (sequence[i].isrch() && sequence[i].rch().iskin() && sequence[i].rch().kin().size()) 
			{
				int res = writeToKinematics(sequence[i].rch().kin());
			}
			 if (sequence[i].isgrp() && sequence[i].grp()[0].type() == GRASP_POWER && sequence[i].grp().size()) 
			{
				if (RobotMovePMP==1){
					int res =PrimGraspIndustrial(GRASP_PINCH,0);
				}
				if (RobotMovePMP==-1){
					int res =PrimGraspIndustrial(GRASP_PINCH,1);
				}
					
			}
			else if (sequence[i].isgrp() && sequence[i].grp()[0].type() == GRASP_RELEASE && sequence[i].grp().size()) 
			{
				if (RobotMovePMP==1){
					int res =PrimGraspIndustrial(GRASP_RELEASE,0);
				}
				if (RobotMovePMP==-1){
					int res =PrimGraspIndustrial(GRASP_RELEASE,1);
				}
					
			}
		}
		/// shown the object to camera and getting new sequence.
		bool properlyGripped =false;
		sceneCalled=0;
		while (!properlyGripped)
		{			
			if(sceneCalled>10)
			{
				std::cout<<"::::::::::::::::::::::Tried many times::Vision says Object in gripper not held upright::::::::::::::::"<<std::endl;
				break;
			}
			if (VScene.getInputCount())
			{	
				std::cout<<"::::::::::::::::::::::::Waiting for a new estimate from Vision::::::::::::::::"<<std::endl;
				Time::delay(4);
				scene2=VScene.read(true);
			}
			sceneCalled++;

			if (RobotMovePMP==1)
			{
				VisualObject fuseTXtemp;// = new VisualObject();
				std::cout<<"fuseTXtemp Empty"<<std::endl;
				std::cout<<fuseTXtemp.toString()<<std::endl;

				fuseTXtemp = findFuseInScene( scene2,250,300,60);

				std::cout<<"fuseTXtemp After function"<<std::endl;
				std::cout<<fuseTXtemp.toString()<<std::endl;

				if(!(fuseTXtemp.ID() == 9999))
					sequence = affordanceAccess->insert(GRASP_LEFT,fuseTXtemp,(*scene2)[index],holeNum,filled);
				else
					std::cout<<"Could not generate valid insert affordance"<<std::endl;
			}
			if (RobotMovePMP==-1)
			{

				VisualObject fuseRXtemp;// = new VisualObject();
				std::cout<<"fuseRXtemp Empty"<<std::endl;
				std::cout<<fuseRXtemp.toString()<<std::endl;

				fuseRXtemp = findFuseInScene( scene2,250,0,60);

				std::cout<<"fuseRXtemp After function"<<std::endl;
				std::cout<<fuseRXtemp.toString()<<std::endl;

				if(!(fuseRXtemp.ID() == 9999))

					sequence = affordanceAccess->insert(GRASP_RIGHT,fuseRXtemp,(*scene2)[index],holeNum,filled);
				else
					std::cout<<"Could not generate valid insert affordance"<<std::endl;
			}

			if(!(sequence.size() == 0))
			{			
				FuseOrient=sequence.size();
				for(int i=0;i<FuseOrient;++i)
				{
					if(sequence[i].isrch() && sequence[i].rch().iskin() && sequence[i].rch().kin()[0].approach() == REACH_CAMERA_SIDE)
					{
						properlyGripped = true;
					}
				}
			}

			else
			{
				std::cout<<":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
				std::cout<<"Could not generate valid insert affordance"<<std::endl;
				std::cout<<":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
				std::cout<<":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
			}
		}

		if(properlyGripped)
		{
			for (int i=0;i<FuseOrient;i++)
			{

				 if (sequence[i].isrch() && sequence[i].rch().iskin() && sequence[i].rch().kin().size()) 
				{
					int res = writeToKinematics(sequence[i].rch().kin());
				}
				 if (sequence[i].isgrp() && sequence[i].grp()[0].type() == GRASP_POWER && sequence[i].grp().size()) 
				{
					if (RobotMovePMP==1){
						int res =PrimGraspIndustrial(GRASP_PINCH,0);
					}
					if (RobotMovePMP==-1){
						int res =PrimGraspIndustrial(GRASP_PINCH,1);
					}
					
				}
				else if (sequence[i].isgrp() && sequence[i].grp()[0].type() == GRASP_RELEASE_SLOW  && sequence[i].grp().size()) 
				{
					if (RobotMovePMP==1){
						int res =PrimGraspIndustrial(GRASP_RELEASE_SLOW,0);
					}
					if (RobotMovePMP==-1){
						int res =PrimGraspIndustrial(GRASP_RELEASE_SLOW,1);
					}
					
				}
			}
			///go to the fixed location with current robot using PMP and reach  above the standing fuse
			if (RobotMovePMP==1)
			{
				sceneCalled=0;
				bool fusefound=false;
				if (VScene.getInputCount())
				{	
					std::cout<<"::::::::::::::::::::::::New Estimate for Fuse::::::::::::::::"<<std::endl;
					scene2=VScene.read(true);
				}
				while(!fusefound)
				{
					if (sceneCalled>10)
					{
						std::cout<<"::::::::Could not find the chosen fuse:::::"<<std::endl;
						break;
					}


					fuseTXg = findFuseInScene( scene2,260,330,60);//z =200
					if(abs(fuseTXg.InsertTip()[0].x()-9999) > 1)
					{
						fusefound=true;
						fuseTX=fuseTXg;
						StaticLoc[0] = fuseTXg.InsertTip()[0].x()+8; 
						StaticLoc[1] = fuseTXg.InsertTip()[0].y();
						StaticLoc[2] = fuseTXg.InsertTip()[0].z()+90;
						int rres=PrimBodySchemaIndustrial(21,0,1,1,0,0);
						StaticLoc[2] =  fuseTXg.InsertTip()[0].z()+28;
						rres=PrimBodySchemaIndustrial(21,0,1,1,0,0);
						ComboCoordinate[3]=fuseTXg.InsertTip()[0].x()+8;
						ComboCoordinate[4]=fuseTXg.InsertTip()[0].y();
						ComboCoordinate[5]=fuseTXg.InsertTip()[0].z()+28;
					}
					else if (VScene.getInputCount())
					{	
						std::cout<<"::::::::::::::::::::::::Asking Vision for fuse again::::::::::::::::"<<std::endl;
						Time::delay(1);
						scene2=VScene.read(true);
					}
					sceneCalled++;

				}

			}
			if (RobotMovePMP==-1)
			{	
				sceneCalled=0;
				bool fusefound=false;
				if (VScene.getInputCount())
				{	
					std::cout<<"::::::::::::::::::::::::New Estimate for Fuse::::::::::::::::"<<std::endl;
					scene2=VScene.read(true);
				}
				while(!fusefound)
				{
					if (sceneCalled>10)
					{
						std::cout<<"::::::::Could not find the chosen fuse:::::"<<std::endl;
						break;
					}
					fuseRXg = findFuseInScene( scene2,260,-100,60);//z =200
					if(abs(fuseRXg.InsertTip()[0].x()-9999) > 1)
					{
						fusefound=true;
						fuseRX=fuseRXg;
						StaticLoc[0] = fuseRXg.InsertTip()[0].x()+8; 
						StaticLoc[1] = fuseRXg.InsertTip()[0].y();
						StaticLoc[2] = fuseRXg.InsertTip()[0].z()+90;
						int rres=PrimBodySchemaIndustrial(21,0,1,1,0,0);
						StaticLoc[2] =  fuseRXg.InsertTip()[0].z()+28;
						rres=PrimBodySchemaIndustrial(21,0,1,1,0,0);
						ComboCoordinate[0]= fuseRXg.InsertTip()[0].x()+8;
						ComboCoordinate[1]= fuseRXg.InsertTip()[0].y();
						ComboCoordinate[2]= fuseRXg.InsertTip()[0].z()+28;
					}
					else if (VScene.getInputCount())
					{	
						std::cout<<"::::::::::::::::::::::::Asking Vision for fuse again::::::::::::::::"<<std::endl;
						Time::delay(1);
						scene2=VScene.read(true);
					}
					sceneCalled++;
				}
			}
		}
		else
		{
			std::cout<<"ASK KRIS/SHARATH"<<std::endl;
		}
	}
	else
	{
		std::cout<<"Fuse is not lying down"<<std::endl;
	}
}
void ObserverThread::reorientAndInsert(double x, double y)
{
	currentHole=findFuseboxInScene( &scene1,x, y);
	IntVector filled;
	filled.clear();
	if((currentHole.ID())==100)
	{
		if(activeHole==0)
		{
			filled.add(1);
			filled.add(2);
		}
		if(activeHole==1)
		{
			filled.add(0);
			filled.add(2);
		}
		if(activeHole==2)
		{
			filled.add(0);
			filled.add(1);
		}
	}
	if((currentHole.ID())==200)
	{
		if(activeHole==0)
		{
			filled.add(1);
			filled.add(2);
			filled.add(3);
		}
		if(activeHole==1)
		{
			filled.add(0);
			filled.add(2);
			filled.add(3);
		}
		if(activeHole==2)
		{
			filled.add(0);
			filled.add(1);
			filled.add(3);
		}
		if(activeHole==3)
		{
			filled.add(0);
			filled.add(1);
			filled.add(2);
		}
	}


	ActionSequence sequence;
	if (RobotMovePMP==1)
	{
		std::cout<<"fuseTX is sent in this way "<<fuseTX.toString()<<std::endl;
		std::cout<<"currentHole is sent in this way "<<currentHole.toString()<<std::endl;
		sequence = affordanceAccess->insert(GRASP_LEFT,fuseTX,currentHole,activeHole,filled);
	}
	if (RobotMovePMP==-1)
	{
		std::cout<<"fuseRX is sent in this way "<<fuseRX.toString()<<std::endl;
		std::cout<<"currentHole is sent in this way "<<currentHole.toString()<<std::endl;
		sequence = affordanceAccess->insert(GRASP_RIGHT,fuseRX,currentHole,activeHole,filled);
	}
	if(!(sequence.size() == 0)){

		int chs=chooseFromSequence(sequence);
		if(sequence[chs-2].subID() == Action::rchID::value)
		{
			int res = writeToKinematics(sequence[chs-2].rch().kin());
		}

		if(sequence[chs].subID() == Action::rchID::value)
		{	
		
			int res = writeToKinematics(sequence[chs].rch().kin());
		}
	}
	else
	{
		std::cout<<"No sequence returned from affordances"<<std::endl;
	}

}


void ObserverThread::Interpret(){

  NPiCs=0;
  for(int iCo=1; iCo<SeqAc[0]+1; iCo++)	{

    if((SeqAc[iCo]==20))
    {
      for(int jCo=0; jCo<numcu; jCo++){
        SeqAcP[NPiCs]=NumCubID[jCo];
        //cout<< "found cube" <<SeqAcP[NPiCs] <<endl;
        NPiCs=NPiCs+1;
      }
    }
    if((SeqAc[iCo]==13))
    {
      for(int jCo=0; jCo<numcy; jCo++){
        SeqAcP[NPiCs]=NumCylID[jCo];
        //cout<< "found cyli" <<SeqAcP[NPiCs] <<endl;
        NPiCs=NPiCs+1;
      }
    }
    if((SeqAc[iCo]==18))
    {
      SeqAcP[NPiCs]=7;
      NPiCs=NPiCs+1;
    }
    if((SeqAc[iCo]==35))
    {
      SeqAcP[NPiCs]=NumMushID[0];
      NPiCs=NPiCs+1;
    }

  }
  //cout<<"NPiCs Interpreter"<<NPiCs<<endl;

}



void ObserverThread::Xlator(){
  int m,n,
    largeness=0;
  numcu=0;
  numcy=0;
  nummu=0;
  NumCylID[0]=50;
  NumCylID[1]=50;
  NumCubID[0]=50;
  NumCubID[1]=50;
  NumCubID[2]=50;
  NumberofObsE=0;
  for (m =0; m<10; m++)
  {
    for (n=0; n<42; n++)
    {
      PlaceMapHub[m][n]=0;
    }
  }

  int delnumcy=0,delnumcu=0;
  int CountEP=0;
  for(int i=0;i<NumberofObs;i++)
  {
    if ((ObjIDEE[i]==2)||(ObjIDEE[i]==5))
    {
      if(nummu==0){
        ObjIDEEEpim[CountEP]=35; //Mushroom
        CountEP=CountEP+1;
      }
      XlatTrackP[i]=ObjIDEE[i];
      XlatTrackPl[i]=ObjIDEE[i];
      NumMushID[nummu]=ObjIDEE[i];
      nummu=nummu+1;
      if(nummu==1)
      {
        NumberofObsE=NumberofObsE+1;
      }
    }
    if ((ObjIDEE[i]==0))
    {
      ObjIDEEEpim[CountEP]=7; //fuse
      //ObjIDEEEpim[NumberofObs+1]=37;
      CountEP=CountEP+1;
      XlatTrackP[i]=ObjIDEE[i];
      XlatTrackPl[i]=ObjIDEE[i];
      largeness=1;
      NumberofObsE=NumberofObsE+1;
      //cout<<"Large object"<<endl;
    }
    if ((ObjIDEE[i]==3)||(ObjIDEE[i]==6))
    {
      if(numcy==0){
        ObjIDEEEpim[CountEP]=13; //Cylinder
        CountEP=CountEP+1;
      }
      XlatTrackP[i]=ObjIDEE[i];
      XlatTrackPl[i]=ObjIDEE[i];
      NumCylID[numcy]=ObjIDEE[i];
      numcy=numcy+1;
      if(numcy==1)
      {
        NumberofObsE=NumberofObsE+1;
      }
    }

    if ((ObjIDEE[i]==15)||(ObjIDEE[i]==1)||(ObjIDEE[i]==4))
    {
      if(numcu==0){
        ObjIDEEEpim[CountEP]=20; //Cube
        CountEP=CountEP+1;
      }
      XlatTrackP[i]=ObjIDEE[i];
      XlatTrackPl[i]=ObjIDEE[i];
      NumCubID[numcu]=ObjIDEE[i];
      numcu=numcu+1;
      if(numcu==1)
      {
        NumberofObsE=NumberofObsE+1;
      }
      //cout<<"Numcu"<< numcu << "NumCubID[0]" << NumCubID[0]<< endl;
    }

    if ((ObjIDEE[i]==100))
    {
      PlaceMapHub[i][9]=1;
      NumberofObsE=NumberofObsE+1;
      //cout<<"Fuse box found"<<endl;
    }

    if ((ObjIDEE[i]==101))
    {
      PlaceMapHub[i][8]=1;
      NumberofObsE=NumberofObsE+1;
      //cout<<"Fuse found"<<endl;
    }

    if ((ObjIDEE[i]==102))
    {
      PlaceMapHub[i][32]=1;
      NumberofObsE=NumberofObsE+1;
      //cout<<"Composite Object: FUSE BOX SET UP found"<<endl;
    }

    if ((ObjIDEE[i]==120))
    {
      PlaceMapHub[i][14]=1;
      NumberofObsE=NumberofObsE+1;
      //cout<<"MECCANO Block found"<<endl;
    }

    if ((ObjIDEE[i]==121))
    {
      PlaceMapHub[i][6]=1;
      NumberofObsE=NumberofObsE+1;
      //cout<<"MECCANO screwdriver found"<<endl;
    }

    if ((ObjIDEE[i]==122))
    {
      PlaceMapHub[i][33]=1;
      NumberofObsE=NumberofObsE+1;
      //cout<<"MECCANO Composite object found"<<endl;
    }

  }
  //cout<<"No of objects in the abstract neural representation:" << NumberofObsE << endl;

}

void ObserverThread::InvXlator(int pi, int pl){

  PickMicro=50;
  PlaceMicro=50;
  if(pi==35){
    for(int i=0;i<NumberofObs;i++)
    {
      if (XlatTrackP[i]==2)
      {
        PickMicro=2;
        XlatTrackP[i]=50;
      }
    }
  }

  if(pi==18){
    {
      PickMicro=6;
    }
  }

  if(pi==13){
    for(int i=0;i<NumberofObs;i++)
    {
      if ((XlatTrackP[i]==3)||(XlatTrackP[i]==5))
      {
        PickMicro=XlatTrackP[i];
        XlatTrackP[i]=50;
        i=10;
      }
    }
  }

  if(pi==20){
    for(int i=0;i<NumberofObs;i++)
    {
      if ((XlatTrackP[i]==0)||(XlatTrackP[i]==1)||(XlatTrackP[i]==4))
      {
        PickMicro=XlatTrackP[i];
        XlatTrackP[i]=50;
        i=10;
      }

    }
  }
  //================================================================================
  if(pl==35){
    for(int i=0;i<NumberofObs;i++)
    {
      if (XlatTrackPl[i]==2)
      {
        PlaceMicro=2;
        XlatTrackPl[i]=50;
      }
    }
  }

  if(pl==18){
    {
      PlaceMicro=6;
    }
  }

  if(pl==13){
    for(int i=0;i<NumberofObs;i++)
    {
      if ((XlatTrackPl[i]==3)||(XlatTrackPl[i]==5))
      {
        PlaceMicro=XlatTrackPl[i];
        XlatTrackPl[i]=50;
        i=10;
      }
    }
  }

  if(pl==20){
    for(int i=0;i<NumberofObs;i++)
    {
      if ((XlatTrackPl[i]==0)||(XlatTrackPl[i]==1)||(XlatTrackPl[i]==4))
      {
        PlaceMicro=XlatTrackPl[i];
        XlatTrackPl[i]=50;
        i=10;
      }

    }
  }

}

int ObserverThread::RetroactivateBodyHub(int PropWB)
{
  //BodyHub[34]=1; //testing
  //BodyHub[35]=1;
  int iCo,jCo,pr_Bo;
  ActionChoice=50;
  for(iCo=0; iCo<12; iCo++)
  {
    pr_Bo=0;
    for(jCo=0; jCo<42; jCo++)
    {
      pr_Bo = pr_Bo + BodyHub2Acn[jCo][iCo]*BodyHub[jCo];
    }
    ActionHub[iCo]=pr_Bo/(pr_Bo+0.001);
    if(ActionHub[iCo]>0.50){
      //cout << "Xplorative Action:::::" << ActionHub[iCo] << "possible in relation to the present situation" <<endl;
      ActionHubXplore[NXploreAct]=iCo;
      NXploreAct=NXploreAct+1;
    }
  }

  for(iCo=0; iCo<12; iCo++)
  {
    if(ActionHub[iCo]>0.5){
      if(ActionHub[iCo]-ActionHubXplore[iCo]>0.5){

        ActionChoice=iCo;
        break;
      }
    }
  }
  return ActionChoice;
  //	Sleep(15000);
};

void ObserverThread::Retroactivate(int PropWC)
{
  int iCo, jCo, mCo, sumWHM;

  if(PropWC==0)
  {
    for(iCo=0; iCo<36; iCo++)
    {
      sumWHM=0;
      for(jCo=0; jCo<60; jCo++)
      {
        sumWHM=sumWHM+MapstoProvHub[iCo][jCo];
      }
      if(sumWHM==2)
      {
        for(mCo=0; mCo<60; mCo++)
        {
          MapstoProvHub[iCo][mCo]=0;
        }
      }
    }

  }

  for(iCo=0; iCo<500; iCo++)
  {
    //WProvColShapRev*LocalAct': MapstoProvHub[36][90] Bottom up====================================
    double maxlocal=0;
    for(jCo=0; jCo<36; jCo++)
    {
      double pr_Co=0;
      for(mCo=0; mCo<90; mCo++)
      {
        pr_Co = pr_Co + MapstoProvHub[jCo][mCo]*LocalMapAct[mCo];
      }

      ProvHub[jCo]=ProvHub[jCo]+0.015*((-0.01*ProvHub[jCo])+pr_Co);
      if(ProvHub[jCo]<0.0001)
      {
        ProvHub[jCo]=0;
      }


    }

    //==================================== WProvColShapRev'*u ===Top Down==============;

    for(jCo=0; jCo<90; jCo++)
    {
      double pr_CoT=0;
      for(mCo=0; mCo<36; mCo++)
      {
        pr_CoT = pr_CoT + ProvHubtoMaps[jCo][mCo]*ProvHub[mCo];
      }
      LocalMapAct[jCo]=LocalMapAct[jCo]+1.3*((-0.01*LocalMapAct[jCo])+pr_CoT);
      if(LocalMapAct[jCo]<0.0001)
      {
        LocalMapAct[jCo]=0;
      }if(LocalMapAct[jCo]>maxlocal)
      {
        maxlocal=LocalMapAct[jCo];
      }

    }

    //GUI Commented out
    /*if (iCo%50 == 0) {
      Bottle& cwsBot =cwsPlot.prepare();
      cwsBot.clear();
      cwsBot.addString("CWS");
      Bottle& cwsBotAll = cwsBot.addList();
      cwsBotAll.clear();
      for(int i=0; i<90; i++)
      {
        cwsBotAll.addDouble(LocalMapAct[i]/maxlocal);
      }
      //	//cout<<"Sending  Color Word Shape hubs to DARWIN GUI"<<endl;
      cwsPlot.write();
      Time::delay(1);
      //Sleep(1000);

    }*/

  }

  for(iCo=0; iCo<42; iCo++)
  {
    HubA << ProvHub[iCo]<< endl;
  }
  for(iCo=0; iCo<90; iCo++)
  {
    MapA << LocalMapAct[iCo]<< endl;
  }
  PushOID=0;
  if(ProvHub[32]>=0.9)
  {
    PushOID=5;
  }
  /*Bottle& bodyBot =hubPlot.prepare();
  bodyBot.clear();
  bodyBot.addString("object");
  Bottle& bodyBotAll = bodyBot.addList();
  bodyBotAll.clear();
  for(int i=0; i<42; i++)
  {
  bodyBotAll.addDouble(ProvHub[i]);
  }
  cout<<"Sending Weight Matrix to DARWIN GUI"<<endl;
  hubPlot.write();*/



};

void ObserverThread::InitializeWorkingMemory(int GoalPointer)
{
  int jCo;
  for(jCo=0; jCo<42; jCo++)
  {
    if(ProvHub[jCo]>0){
      GlobalWorkSpace[GoalPointer][jCo]=1;
    }
    if(ProvHub[jCo]<=0){
      GlobalWorkSpace[GoalPointer][jCo]=0;
    }

  }
  if((Goal_AcID==9))
  {
    for(jCo=0; jCo<42; jCo++)
    {
      GlobalWorkSpace[0][jCo]=0;
      GlobalWorkSpace[1][jCo]=0;
    }
    GlobalWorkSpace[0][8]=1;
    GlobalWorkSpace[1][9]=1;
  }

  if((Goal_AcID==3)&&(PushOID==5))
  {
    for(jCo=0; jCo<42; jCo++)
    {
      GlobalWorkSpace[0][jCo]=0;
      GlobalWorkSpace[1][jCo]=0;
      GlobalWorkSpace[1][jCo]=0;
    }
    GlobalWorkSpace[0][32]=1;
    GlobalWorkSpace[1][8]=1;
    GlobalWorkSpace[2][9]=1;
  }
  //this stores the anticipated object activation, placemap is initialized through refresh placemap-opc connection
};

void ObserverThread:: GetLocalAct(int numW)
{
  int iCo, jCo, m,n, clocal=0;
  fileName = pathPrefix;
  fileName.append("MapI.txt");
  ofstream MapAini(fileName.c_str());
  MaxiAct=0.0001;
  MaxiActS=0.0001;
  MaxiActW=0.0001;
  /*for (n=0; n<30; n++)
  {
  Col[n]=0;
  Word[n]=0;
  Shape[n]=0;
  }*/
  for (n=0; n<90; n++)
  {
    LocalMapAct[n]=0;
  }
  for (n=0; n<42; n++)
  {
    ProvHub[n]=0;
  }

  for(iCo=0; iCo<30; iCo++)
  {
    Rdis=sqrt(pow(ColW[iCo][0]-ipCol[0],2)+pow(ColW[iCo][1]-ipCol[1],2)+pow(ColW[iCo][2]-ipCol[2],2));
    RdisS=sqrt(pow(ShapW[iCo][0]-ipShap[0],2)+pow(ShapW[iCo][1]-ipShap[1],2)+pow(ShapW[iCo][2]-ipShap[2],2));
    RdisAct[iCo]=1*exp(-(Rdis)/40);
    RdisActS[iCo]=1*exp(-(RdisS)/40);
    if(RdisAct[iCo]>MaxiAct)
    {
      MaxiAct=RdisAct[iCo];
      // u can also have a indicatior here of which neuron is most active in col som
    }
    if(RdisActS[iCo]>MaxiActS)
    {
      MaxiActS=RdisActS[iCo];
      // u can also have a indicatior here of which neuron is most active in col som
    }
  }

  for(iCo=0; iCo<30; iCo++)
  {
    pr_Co=0;
    pr_CoS=0;
    for(jCo=0; jCo<30; jCo++)
    {
      pr_Co = pr_Co + inhib[iCo][jCo]*RdisAct[jCo];
      pr_CoS = pr_CoS + inhib[iCo][jCo]*RdisActS[jCo];
    }

    Col[iCo]=(RdisAct[iCo]-(0.035*pr_Co))/MaxiAct;
    Shape[iCo]=(RdisActS[iCo]-(0.035*pr_CoS))/MaxiActS;
    LocalMapAct[clocal]=Col[iCo];
    LocalMapAct[60+clocal]=Shape[iCo];
    clocal=clocal+1;
  }

  //========================================================
  //Words act
  for(m=0; m<numW; m++)
  {
    //RdistempW=0;
    MaxiActW=0.0001;
    for(iCo=0; iCo<30; iCo++)
    {
      RdistempW=0;
      RdisW=0;
      for(jCo=0; jCo<120; jCo++)
      {
        RdistempW = RdistempW + ((WorW[iCo][jCo]-WordIn[m][jCo])*(WorW[iCo][jCo]-WordIn[m][jCo])); //pow(WorW[iCo][jCo]-WordIn[m][jCo],2)
      }
      RdisW=sqrt(RdistempW);
      RdisActW[m][iCo]=5.64*exp(-1*(RdisW/0.25));
      if(RdisActW[m][iCo]>MaxiActW)
      {
        MaxiActW=RdisActW[m][iCo];
      }
    }
    cout << "Word to Object Hub activations"<< endl;
    //Time::delay(20);
    //Sleep(20000);
    //===============================================================
    for(iCo=0; iCo<30; iCo++)
    {
      pr_CoW=0;
      for(jCo=0; jCo<30; jCo++)
      {
        pr_CoW = pr_CoW + inhib[iCo][jCo]*RdisActW[m][jCo];
      }
      WorActiv[m][iCo]=(RdisActW[m][iCo]-(0.03*pr_CoW))/MaxiActW;
      //WorActiv[m][iCo]=RdisActW[m][iCo]*10;
    }

  } // m loop of words to be projected to the word SOM

  for(iCo=0; iCo<30; iCo++)
  {
    if(numW==1)
    {
      LocalMapAct[30+iCo]=WorActiv[0][iCo];
    }
    if(numW==2)
    {
      LocalMapAct[30+iCo]=WorActiv[0][iCo]+WorActiv[1][iCo];
    }
  }

  //===== here u get the net initial bottom up neural activity: col, shap: by vision and word through keyboard ========
  for(iCo=0; iCo<90; iCo++)
  {
    //MapAini <<WorActiv[0][iCo]<<endl;
    MapAini << LocalMapAct[iCo]<< endl;
  }
};

int ObserverThread::RetroactivateAcHub()
{

  return 0;
};


void ObserverThread::InitializeSW()
{
  int m=0,n=0;
  //================== Initialize connectivity ==========================
  fileName = pathPrefix;
  fileName.append("WProvCSWRevInd.txt");
  ifstream WCSW(fileName.c_str());
  printf("%s\n",fileName.c_str());

  fileName = pathPrefix;
  fileName.append("wordWRevInd.txt");
  ifstream WorN(fileName.c_str());
  printf("%s\n",fileName.c_str());

  fileName = pathPrefix;
  fileName.append("colNeuronsW.txt");
  ifstream ColN(fileName.c_str());

  fileName = pathPrefix;
  fileName.append("WHTM.txt");
  ifstream WHMW(fileName.c_str());

  fileName = pathPrefix;
  fileName.append("ShapeNeuronsWRev.txt");
  ifstream SHWN(fileName.c_str());

  fileName = pathPrefix;
  fileName.append("WBHubA.txt");
  ifstream WBHA(fileName.c_str());

  fileName = pathPrefix;
  fileName.append("WAP.txt");
  ifstream WAPR(fileName.c_str());

  for (m =0; m<36; m++)
  {
    for (n=0; n<90; n++)
    {
      WCSW >> MapstoProvHub[m][n];
    }
  }

  for (m =0; m<42; m++)
  {
    for (n=0; n<12; n++)
    {
      WBHA >> BodyHub2Acn[m][n];
    }
  }

  for (m =0; m<90; m++)
  {
    for (n=0; n<36; n++)
    {
      WHMW >> ProvHubtoMaps[m][n];
    }
  }

  for (m =0; m<30; m++)
  {
    for (n=0; n<120; n++)
    {
      WorN >> WorW[m][n];
    }
  }
  //cout << WorW[0][8]<< endl;

  for (m =0; m<12; m++)
  {
    for (n=0; n<120; n++)
    {
      WAPR >> WActionPrim[m][n];
    }
  }

  //cout << WActionPrim[0][8]<< endl;

  for (m =0; m<30; m++)
  {
    for (n=0; n<3; n++)
    {
      ColN >> ColW[m][n];
    }
  }

  for (m =0; m<30; m++)
  {
    for (n=0; n<3; n++)
    {
      SHWN >>	ShapW[m][n];
    }
  }

  for (m =0; m<10; m++)
  {
    for (n=0; n<42; n++)
    {
      PlaceMapHub[m][n]=0;
    }
  }

  for (m =0; m<10; m++)
  {
    for (n=0; n<50; n++)
    {
      GlobalWorkSpace[m][n]=0;
    }
  }
  for (m =0; m<10; m++)
  {
    for (n=0; n<50; n++)
    {
      GoalStack[m][n]=50;
    }
  }
  for (n=0; n<20; n++)
  {
    Encapsulate[n]=50;
	ActionRecorder[n]=50;
  }

  for (m =0; m<20; m++)
  {
    for (n=0; n<50; n++)
    {
      BottomUPTrace[m][n]=0;
      Behavior[m][n]=0;
	  ProtoPlan[m][n]=0;
    }
  }


  for (n=0; n<120; n++)
  {
    WordAIn[n]=0;
  }
  for (m =0; m<30; m++)
  {
    for (n=0; n<30; n++)
    {
      inhib[m][n]=1;
      if(m==n)
      {
        inhib[m][n]=0;
      }
    }
  }
  for (n=0; n<12; n++)
  {
    ActionHubXplore[n]==0;
  }
  for (n=0; n<30; n++)
  {
    Col[n]=0;
    Word[n]=0;
    Shape[n]=0;
  }
  ipCol[0]=0;
  ipCol[1]=0;
  ipCol[2]=0;
  ipShap[0]=0;
  ipShap[1]=0;
  ipShap[2]=0;


  for (n=0; n<42; n++)
  {
    ProvHub[n]=0;
    BodyHub[n]=0;
    OCHub[n]=0;
    BodyHubTD[n]=0;
    BodyHubBU[n]=0;
  }
};

void ObserverThread::ProtoLtoEpim()
{
    int m,n,iCo;
    cout << "Enter the Root Goal of this new assembly" << endl;
	int iterProto=0;
	int AcVerb=WordEncodeA(2,2);
	cout<<" Recording user Goal as context " <<endl;
	ProtoPlan[iterProto][48]=1;
	ProtoPlan[iterProto][49]=1;
	ProtoPlan[iterProto][AcVerb]=1;
	ProtoPlan[iterProto][AcVerb+12]=1;
	ProtoPlan[iterProto][AcVerb+24]=1;
	for(iCo=0; iCo<50; iCo++)
		{
		 Present<< ProtoPlan[iterProto][iCo]<< "    ";
		}
    Present << "    " << endl; 
	iterProto=iterProto+1;

	NumWords=1;
    fileName = pathPrefix;
    fileName.append("WorBin.txt");
    ofstream WorBin(fileName.c_str());
	cout<<" Transforming goal to neural activations..... recording sequence" <<endl;
    WordEncode(0,2);
   	GetLocalAct(1);   //word to neural activations
	Retroactivate(1);
	int MaxAct=GetMaxProv();
	cout<<"provH"<<ProvHub[0]<<endl;
	ProtoPlan[iterProto][47]=1;
	ProtoPlan[iterProto][49]=1;
	ProtoPlan[iterProto][MaxAct]=1;
		for(iCo=0; iCo<50; iCo++)
		{
		 Present<< ProtoPlan[iterProto][iCo]<< "    ";
		}
    Present << "    " << endl; 
	iterProto=iterProto+1;

	cout<<" Instruct the desired actions on objects for the new assembly " <<endl;
	AcVerb=WordEncodeA(2,2);
	ProtoPlan[iterProto][48]=1;
	ProtoPlan[iterProto][AcVerb]=1;
		for(iCo=0; iCo<50; iCo++)
		{
		 Present<< ProtoPlan[iterProto][iCo]<< "    ";
		}
    Present << "    " << endl; 
	iterProto=iterProto+1;
	WordEncode(0,2);
   	GetLocalAct(1);   //word to neural activations
	cout<<" Transforming object to neural activations..... recording sequence" <<endl;
	Retroactivate(1);
	MaxAct=GetMaxProv();
	cout<<"provH"<<ProvHub[0]<<endl;
	ProtoPlan[iterProto][47]=1;
	ProtoPlan[iterProto][MaxAct]=1;
		for(iCo=0; iCo<50; iCo++)
		{
		 Present<< ProtoPlan[iterProto][iCo]<< "    ";
		}
    Present << "    " << endl; 
	iterProto=iterProto+1;
	WordEncode(0,2);
   	GetLocalAct(1);   //word to neural activations
	Retroactivate(1);
	MaxAct=GetMaxProv();
	cout<<"provH"<<ProvHub[0]<<endl;
	ProtoPlan[iterProto][47]=1;
	ProtoPlan[iterProto][MaxAct]=1;
		for(iCo=0; iCo<50; iCo++)
		{
		 Present<< ProtoPlan[iterProto][iCo]<< "    ";
		}
    Present << "    " << endl; 
	iterProto=iterProto+1;
	cout<<" Input fetched reward" <<endl;
	int rewProto;
	cin>>rewProto;
	cout<<" From neural activations to episodic memory trace" <<endl;
	ProtoPlan[iterProto][45]=1;  //this is taken care of in the epim for now
	ProtoPlan[iterProto][rewProto]=1;
		for(iCo=0; iCo<50; iCo++)
		{
		 Present<< ProtoPlan[iterProto][iCo]<< "    ";
		}
    Present << "    " << endl; 
	iterProto=iterProto+1;
	cout<<" NEW MEMORY ENCODED" <<endl;
	for(iCo=0; iCo<20; iCo++)
	{
       for(n=0; n<50; n++)
		{
		 cout<< ProtoPlan[iCo][n]<< "    ";
		}
	}

};

int ObserverThread::GetMaxProv()
{
	int iCo,jCo;
	int maxP=0.01;
	int Maxativ=0;
	for(iCo=0; iCo<42; iCo++)
      {
		   if(ProvHub[iCo]>maxP)
        {
            Maxativ=iCo;
        }

      }
return Maxativ;
}

void ObserverThread::WordEncode(int numu, int hubenc)
{

  string mycol;
  int sizz,m,n;
  for (n=0; n<120; n++)
  {
    WordIn[numu][n]=0;
  }

  cout << "Input Noun " << endl;
  cin >> mycol;
  sizz=mycol.size();
  //cout << sizz << endl;
  for (n=0; n<sizz; n++)
  {
    //cout << mycol[n] << endl;
    if (mycol[n] ==  'r') { WordIn[numu][(8+(n*20))]=1;}
    if (mycol[n] ==  'e') { WordIn[numu][(0+(n*20))]=1;}
    if (mycol[n] ==  'd') { WordIn[numu][(9+(n*20))]=1;}
    if (mycol[n] ==  't') { WordIn[numu][(1+(n*20))]=1;}
    if (mycol[n] ==  'a') { WordIn[numu][(2+(n*20))]=1;}
    if (mycol[n] ==  'o') { WordIn[numu][(3+(n*20))]=1;}
    if (mycol[n] ==  'i') { WordIn[numu][(4+(n*20))]=1;}
    if (mycol[n] ==  'n') { WordIn[numu][(5+(n*20))]=1;}
    if (mycol[n] ==  's') { WordIn[numu][(6+(n*20))]=1;}
    if (mycol[n] ==  'h') { WordIn[numu][(7+(n*20))]=1;}
    if (mycol[n] ==  'l') { WordIn[numu][(10+(n*20))]=1;}
    if (mycol[n] ==  'c') { WordIn[numu][(11+(n*20))]=1;}
    if (mycol[n] ==  'u') { WordIn[numu][(12+(n*20))]=1;}
    if (mycol[n] ==  'm') { WordIn[numu][(13+(n*20))]=1;}
    if (mycol[n] ==  'w') { WordIn[numu][(14+(n*20))]=1;}
    if (mycol[n] ==  'f') { WordIn[numu][(15+(n*20))]=1;}
    if (mycol[n] ==  'g') { WordIn[numu][(16+(n*20))]=1;}
    if (mycol[n] ==  'y') { WordIn[numu][(17+(n*20))]=1;}
    if (mycol[n] ==  'p') { WordIn[numu][(18+(n*20))]=1;}
    if (mycol[n] ==  'j') { WordIn[numu][(19+(n*20))]=1;}

  }
};


int ObserverThread::WordEncodeA(int numu, int hubenc)
{
  string mycol;
  int sizz,n;
 
  // cout << "Input Action verb " << endl;
      //==============================================================
   //   cout << "Input word " << endl;
      string mycoll;
      int sizze;
      cin >> mycoll;
      sizze=mycoll.size();
     // cout << sizze << endl;
      for (n=0; n<120; n++)
      {
        WordAIn[n]=0;
      }
  ///
     for (n=0; n<sizze; n++)
      {
       // cout << mycoll[n] << endl;
        if (mycoll[n] ==  'r') { WordAIn[(8+(n*20))]=1;}
        if (mycoll[n] ==  'e') { WordAIn[(0+(n*20))]=1;}
        if (mycoll[n] ==  'd') { WordAIn[(9+(n*20))]=1;}
        if (mycoll[n] ==  't') { WordAIn[(1+(n*20))]=1;}
        if (mycoll[n] ==  'a') { WordAIn[(2+(n*20))]=1;}
        if (mycoll[n] ==  'o') { WordAIn[(3+(n*20))]=1;}
        if (mycoll[n] ==  'i') { WordAIn[(4+(n*20))]=1;}
        if (mycoll[n] ==  'n') { WordAIn[(5+(n*20))]=1;}
        if (mycoll[n] ==  's') { WordAIn[(6+(n*20))]=1;}
        if (mycoll[n] ==  'h') { WordAIn[(7+(n*20))]=1;}
        if (mycoll[n] ==  'l') { WordAIn[(10+(n*20))]=1;}
        if (mycoll[n] ==  'c') { WordAIn[(11+(n*20))]=1;}
        if (mycoll[n] ==  'u') { WordAIn[(12+(n*20))]=1;}
        if (mycoll[n] ==  'm') { WordAIn[(13+(n*20))]=1;}
        if (mycoll[n] ==  'w') { WordAIn[(14+(n*20))]=1;}
        if (mycoll[n] ==  'f') { WordAIn[(15+(n*20))]=1;}
        if (mycoll[n] ==  'g') { WordAIn[(16+(n*20))]=1;}
        if (mycoll[n] ==  'y') { WordAIn[(17+(n*20))]=1;}
        if (mycoll[n] ==  'p') { WordAIn[(18+(n*20))]=1;}
        if (mycoll[n] ==  'j') { WordAIn[(19+(n*20))]=1;}
      }

      int iCo,jCo,ahactiv=0;
      int maxA=0.01;
      for(iCo=0; iCo<12; iCo++)
      {
        int pr_Coo=0;
        for(jCo=0; jCo<120; jCo++)
        {
          pr_Coo = pr_Coo + WActionPrim[iCo][jCo]*WordAIn[jCo];
        }

        ActH[iCo]=pr_Coo;
        if(ActH[iCo]>maxA)
        {
          maxA=ActH[iCo];
          ahactiv=iCo;
        }
      }
      for(iCo=0; iCo<12; iCo++)
      {
        ActH[iCo]=ActH[iCo]/maxA;
        WorABin <<ActH[iCo] << endl;
        // cout<<ActH[iCo]<<endl;
      }
      cout<<"ahaactive"<<ahactiv<<endl;
	  return ahactiv;
    };

  



int ObserverThread::UserInterface(int GoalLearn)
{
  int m,n,userIntervention;
  if((GoalLearn==14)||(GoalLearn==41))
  {
    if(GoalLearn==14)
    {
      cout << "Input Root Goal " << endl;
      //==============================================================
      cout << "Input word " << endl;
      string mycoll;
      int sizze;
      cin >> mycoll;
      sizze=mycoll.size();
     // cout << sizze << endl;
      for (n=0; n<120; n++)
      {
        WordAIn[n]=0;
      }
	  if(sizze!=6){
      for (n=0; n<sizze; n++)
      {
        cout << mycoll[n] << endl;
        if (mycoll[n] ==  'r') { WordAIn[(8+(n*20))]=1;}
        if (mycoll[n] ==  'e') { WordAIn[(0+(n*20))]=1;}
        if (mycoll[n] ==  'd') { WordAIn[(9+(n*20))]=1;}
        if (mycoll[n] ==  't') { WordAIn[(1+(n*20))]=1;}
        if (mycoll[n] ==  'a') { WordAIn[(2+(n*20))]=1;}
        if (mycoll[n] ==  'o') { WordAIn[(3+(n*20))]=1;}
        if (mycoll[n] ==  'i') { WordAIn[(4+(n*20))]=1;}
        if (mycoll[n] ==  'n') { WordAIn[(5+(n*20))]=1;}
        if (mycoll[n] ==  's') { WordAIn[(6+(n*20))]=1;}
        if (mycoll[n] ==  'h') { WordAIn[(7+(n*20))]=1;}
        if (mycoll[n] ==  'l') { WordAIn[(10+(n*20))]=1;}
        if (mycoll[n] ==  'c') { WordAIn[(11+(n*20))]=1;}
        if (mycoll[n] ==  'u') { WordAIn[(12+(n*20))]=1;}
        if (mycoll[n] ==  'm') { WordAIn[(13+(n*20))]=1;}
        if (mycoll[n] ==  'w') { WordAIn[(14+(n*20))]=1;}
        if (mycoll[n] ==  'f') { WordAIn[(15+(n*20))]=1;}
        if (mycoll[n] ==  'g') { WordAIn[(16+(n*20))]=1;}
        if (mycoll[n] ==  'y') { WordAIn[(17+(n*20))]=1;}
        if (mycoll[n] ==  'p') { WordAIn[(18+(n*20))]=1;}
        if (mycoll[n] ==  'j') { WordAIn[(19+(n*20))]=1;}
      }

      int iCo,jCo,ahactiv=0;
      int maxA=0.01;
      for(iCo=0; iCo<12; iCo++)
      {
        int pr_Coo=0;
        for(jCo=0; jCo<120; jCo++)
        {
          pr_Coo = pr_Coo + WActionPrim[iCo][jCo]*WordAIn[jCo];
        }

        ActH[iCo]=pr_Coo;
        if(ActH[iCo]>maxA)
        {
          maxA=ActH[iCo];
          ahactiv=iCo;
        }
      }
      for(iCo=0; iCo<12; iCo++)
      {
        ActH[iCo]=ActH[iCo]/maxA;
        WorABin <<ActH[iCo] << endl;
        // cout<<ActH[iCo]<<endl;
      }
      cout<<"ahaactive"<<ahactiv<<endl;
      Goal_AcID=ahactiv;
      GoalStack[GoalPointer][0]=Goal_AcID;
	  }
	  if(sizze==6)
	  {
	    cout<<"Initializing formation of new memory"<<endl;
		Protol=14;
	    ProtoLtoEpim(); 
		userIntervention=1;
	  }
    }
    //========================================================
	if(Protol!=14)
    {
		int sizz;
    /*cout << "Input number goal arguement words " << endl;
    cin >> sizz;*/
    sizz=1;
    NumWords=sizz;
    fileName = pathPrefix;
    fileName.append("WorBin.txt");
    ofstream WorBin(fileName.c_str());
    for (n=0; n<sizz; n++)
    {
      WordEncode(n,2);
    }

    for (m =0; m<sizz; m++)
    {
      for (n=0; n<120; n++)
      {
        WorBin << WordIn[m][n] << "    ";
      }
      WorBin << "    " << endl;
    }
    userIntervention=1;
  }
  }


  if(GoalLearn==32) // is for the purpose oc communicating with the user in the online Xperience gaining mechanism
  {
    cout << "Error ID " << "cannot find the object requested: need help " <<  endl;
    cout << "Reinitate and Retry or Learn experience directly " << endl;
    cin >> userIntervention;
  }

  return userIntervention;
};


void ObserverThread::onStop() {

  //    outputPort.interrupt();
  //  outputPort.close()
};




